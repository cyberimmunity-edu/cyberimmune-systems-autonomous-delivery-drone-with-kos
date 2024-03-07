#include "../include/navigation_system.h"
#include "../include/compass_qmc5883.h"
#include "../include/mpu_6050.h"
#include "../../ipc_messages/include/initialization_interface.h"

#include <coresrv/hal/hal_api.h>
#include <rtl/retcode_hr.h>
#include <i2c/i2c.h>
#include <bsp/bsp.h>

#include <stdio.h>
#include <unistd.h>

#define NAME_MAX_LENGTH 64
#define RETRY_DELAY_SEC 1

char compassI2C[] = "i2c0";
char mpuI2C[] = "i2c1";
char compassConfigSuffix[] = "p0-1";
char mpuConfigSuffix[] = "p2-3";

int initNavigationSystem() {
    if (!waitForInit("ns_pc_connection", "PeripheryController")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }

    char boardName[NAME_MAX_LENGTH] = {0};
    if (KnHalGetEnv("board", boardName, sizeof(boardName)) != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to get board name\n", ENTITY_NAME);
        return 0;
    }

    Retcode rc = BspInit(NULL);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to initialize BSP ("RETCODE_HR_FMT")\n", ENTITY_NAME, RETCODE_HR_PARAMS(rc));
        return 0;
    }

    char* i2c[] = { compassI2C, mpuI2C };
    char* suffix[] = { compassConfigSuffix, mpuConfigSuffix };
    for (int i = 0; i < 2; i++) {
        char config[NAME_MAX_LENGTH];
        if (snprintf(config, NAME_MAX_LENGTH, "%s.%s", boardName, suffix[i]) < 0) {
            fprintf(stderr, "[%s] Error: Failed to generate I2C compass config name\n", ENTITY_NAME);
            return 0;
        }

        rc = BspEnableModule(i2c[i]);
        if (rc != rcOk) {
            fprintf(stderr, "[%s] Error: Failed to enable I2C %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, i2c[i], RETCODE_HR_PARAMS(rc));
            return 0;
        }
        rc = BspSetConfig(i2c[i], config);
        if (rc != rcOk) {
            fprintf(stderr, "[%s] Error: Failed to set BSP config for I2C %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, i2c[i], RETCODE_HR_PARAMS(rc));
            return 0;
        }
    }

    rc = I2cInit();
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to initialize I2C ("RETCODE_HR_FMT")\n", ENTITY_NAME, RETCODE_HR_PARAMS(rc));
        return 0;
    }

    return 1;
}

int initSensors() {
    if (!startCompass(compassI2C))
        return 0;

    if (!startMpu(mpuI2C))
        return 0;

    return 1;
}

int calibrateCompass() {
    return compassCalibration();
}

int calibrateMpu() {
    return gyroscopeCalibration();
}

int getAzimuth(float &azimuth) {
    return readAzimuth(azimuth);
}

int getAcceleration(float &x, float &y, float &z) {
    return readAcceleration(x, y, z);
}

int getGyroscope(float &x, float &y, float &z) {
    return readGyroscope(x, y, z);
}

int getTemperature(float &temperature) {
    return readTemperature(temperature);
}