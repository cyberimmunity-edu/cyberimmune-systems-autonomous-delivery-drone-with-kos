#include "../include/navigation_system.h"
#include "../../shared/include/ipc_messages_initialization.h"

#include <coresrv/hal/hal_api.h>
#include <rtl/retcode_hr.h>
#include <uart/uart.h>
#include <i2c/i2c.h>
#include <bsp/bsp.h>

#include <thread>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#define NAME_MAX_LENGTH 64

std::thread barometerThread;

char gpsUart[] = "uart3";
char gpsConfigSuffix[] = "default";
UartHandle gpsUartHandler = NULL;

char barometerI2C[] = "i2c1";
char barometerConfigSuffix[] = "p2-3";
I2cHandle barometerHandler = NULL;

int32_t tempCoefs[3];
int32_t pressCoefs[9];

float temperatureFine;

int writeRegister(uint8_t reg, uint8_t val) {
    I2cMsg messages[1];
    uint8_t buf[2] = { reg, val };

    messages[0].addr = 0x76;
    messages[0].flags = 0;
    messages[0].buf = buf;
    messages[0].len = 2;

    Retcode rc = I2cXfer(barometerHandler, 400000, messages, 1);
    if (rc != rcOk)
        return 0;

    return 1;
}

int readRegister16(uint8_t reg, uint8_t* val) {
    I2cMsg messages[2];
    uint8_t writeBuffer[1] = { reg };
    uint8_t readBuffer[2];

    messages[0].addr = 0x76;
    messages[0].flags = 0;
    messages[0].buf = writeBuffer;
    messages[0].len = 1;

    messages[1].addr = 0x76;
    messages[1].flags = I2C_FLAG_RD;
    messages[1].buf = readBuffer;
    messages[1].len = 2;

    I2cError rc = I2cXfer(barometerHandler, 400000, messages, 2);
    if (rc != rcOk)
        return 0;

    val[0] = readBuffer[0];
    val[1] = readBuffer[1];

    return 1;
}

int readRegister24(uint8_t reg, int32_t &val) {
    I2cMsg messages[2];
    uint8_t writeBuffer[1] = { reg };
    uint8_t readBuffer[3];

    messages[0].addr = 0x76;
    messages[0].flags = 0;
    messages[0].buf = writeBuffer;
    messages[0].len = 1;

    messages[1].addr = 0x76;
    messages[1].flags = I2C_FLAG_RD;
    messages[1].buf = readBuffer;
    messages[1].len = 3;

    I2cError rc = I2cXfer(barometerHandler, 400000, messages, 2);
    if (rc != rcOk)
        return 0;

    val = ((int32_t)(readBuffer[0]) << 12) | ((int32_t)(readBuffer[1]) << 4) | ((int32_t)(readBuffer[2]) >> 4);
    return 1;
}

int getTemperature(float &temperature) {
    int32_t temp;
    if (!readRegister24(0xFA, temp))
        return 0;

    double tempFst = (temp / 16384.0 - tempCoefs[0] / 1024.0) * tempCoefs[1];
    double tempSnd = pow(temp / 131072.0 - tempCoefs[0] / 8192.0, 2) * tempCoefs[2];

    temperatureFine = tempFst + tempSnd;
    temperature = temperatureFine / 5120.0;

    return 1;
}

int getPressure(float &pressure) {
    int32_t press;
    if (!readRegister24(0xF7, press))
        return 0;

    double pressFst = (temperatureFine / 2.0) - 64000.0;
    double pressSnd = pow(pressFst, 2) * pressCoefs[5] / 32768.0;
    pressSnd += pressFst * pressCoefs[4] * 2.0;
    pressSnd = (pressSnd / 4.0) + (pressCoefs[3] * 65536.0);
    pressFst = (pressCoefs[2] * pow(pressFst, 2) / 524288.0 + pressFst * pressCoefs[1]) / 524288.0;
    pressFst = (1.0 + pressFst / 32768.0) * pressCoefs[0];

    double pressTrd = 1048576.0 - press;
    pressTrd = (pressTrd - (pressSnd / 4096.0)) * 6250.0 / pressFst;
    pressFst = pressCoefs[8] * pow(pressTrd, 2) / 2147483648.0;
    pressSnd = pressTrd * pressCoefs[7] / 32768.0;

    pressure = pressTrd + (pressFst + pressSnd + pressCoefs[6]) / 16.0;

    return 1;
}

void getBarometer() {
    while (true) {
        float temp, press;
        getTemperature(temp);
        getPressure(press);
        float alt = 44330.0 * (1.0 - pow(press / 101325.0, 0.1903));
        int32_t altitude = 100 * alt;
        setAltitude(altitude);
        usleep(500000);
    }
}

void getSensors() {
    bool read, update;
    uint8_t value;
    int mode, idx, latSign, lngSign;
    int32_t latitude, longitude;
    char head[8], satsStr[8], dopStr[8], latStr[16], lngStr[16];

    while (true) {
        update = true;
        read = true;
        mode = 0;
        idx = 0;

        while (read) {
            Retcode rc = UartReadByte(gpsUartHandler, &value);
            if (rc != rcOk)
                continue;
            switch (mode) {
            case 0:
                if (value == '$')
                    mode = 1;
                break;
            case 1: //Header
                if (idx >= 8) {
                    read = false;
                    update = false;
                }
                else if (value == ',') {
                    head[idx] = '\0';
                    if ((head[2] == 'G') && (head[3] == 'G') && (head[4] == 'A'))
                        mode = 2;
                    else
                        mode = 0;
                    idx = 0;
                }
                else {
                    head[idx] = value;
                    idx++;
                }
                break;
            case 2: //UTC time
            case 7: //Quality
                if (value == ',')
                    mode++;
                break;
            case 3: //Lat
                if (idx >= 16) {
                    read = false;
                    update = false;
                }
                else if (value == ',') {
                    latStr[idx] = '\0';
                    idx = 0;
                    mode = 4;
                }
                else {
                    latStr[idx] = value;
                    idx++;
                }
                break;
            case 4: //Lat dir
                if (value == 'N')
                    latSign = 1;
                else if (value == 'S')
                    latSign = -1;
                else if (value == ',')
                    mode = 5;
                else {
                    read = false;
                    update = false;
                }
                break;
            case 5: //Lng
                if (idx >= 16) {
                    read = false;
                    update = false;
                }
                else if (value == ',') {
                    lngStr[idx] = '\0';
                    idx = 0;
                    mode = 6;
                }
                else {
                    lngStr[idx] = value;
                    idx++;
                }
                break;
            case 6: //Lng dir
                if (value == 'E')
                    lngSign = 1;
                else if (value == 'W')
                    lngSign = -1;
                else if (value == ',')
                    mode = 7;
                else {
                    read = false;
                    update = false;
                }
                break;
            case 8: //Sats
                if (idx >= 8) {
                    read = false;
                    update = false;
                }
                else if (value == ',') {
                    satsStr[idx] = '\0';
                    idx = 0;
                    mode = 9;
                }
                else {
                    satsStr[idx] = value;
                    idx++;
                }
                break;
            case 9: //Hdop
                if (idx >= 8) {
                    read = false;
                    update = false;
                }
                else if (value == ',') {
                    dopStr[idx] = '\0';
                    idx = 0;
                    read = false;
                }
                else {
                    dopStr[idx] = value;
                    idx++;
                }
                break;
            }
        }

        if (update) {
            longitude = round(10000000 * atof(lngStr + 3) / 60.0f);
            latitude = round(10000000 * atof(latStr + 2) / 60.0f);
            lngStr[3] = '\0';
            latStr[2] = '\0';
            longitude += 10000000 * atoi(lngStr);
            latitude += 10000000 * atoi(latStr);

            setCoords(latitude, longitude);
            setGpsInfo(atof(dopStr), atoi(satsStr));
        }
        else
            fprintf(stderr, "[%s] Warning: Failed to parse NMEA string from GPS\n", ENTITY_NAME);
    }
}

int initNavigationSystem() {
    while (!waitForInit("periphery_controller_connection", "PeripheryController")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Periphery Controller. Trying again in 1s\n", ENTITY_NAME);
        sleep(1);
    }

    char boardName[NAME_MAX_LENGTH] = {0};
    if (KnHalGetEnv("board", boardName, sizeof(boardName)) != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to get board name\n", ENTITY_NAME);
        return 0;
    }

    char gpsConfig[NAME_MAX_LENGTH];
    if (snprintf(gpsConfig, NAME_MAX_LENGTH, "%s.%s", boardName, gpsConfigSuffix) < 0) {
        fprintf(stderr, "[%s] Error: Failed to generate UART config name\n", ENTITY_NAME);
        return 0;
    }
    char barometerConfig[NAME_MAX_LENGTH];
    if (snprintf(barometerConfig, NAME_MAX_LENGTH, "%s.%s", boardName, barometerConfigSuffix) < 0) {
        fprintf(stderr, "[%s] Error: Failed to generate I2C config name\n", ENTITY_NAME);
        return 0;
    }

    Retcode rc = BspInit(NULL);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to initialize BSP ("RETCODE_HR_FMT")\n", ENTITY_NAME, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    rc = BspEnableModule(gpsUart);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to enable UART %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, gpsUart, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    rc = BspSetConfig(gpsUart, gpsConfig);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to set BSP config for UART %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, gpsUart, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    rc = UartInit();
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to initialize UART ("RETCODE_HR_FMT")\n", ENTITY_NAME, RETCODE_HR_PARAMS(rc));
        return 0;
    }

    rc = BspEnableModule(barometerI2C);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to enable I2C %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, barometerI2C, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    rc = BspSetConfig(barometerI2C, barometerConfig);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to set BSP config for I2C %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, barometerI2C, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    rc = I2cInit();
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to initialize I2C ("RETCODE_HR_FMT")\n", ENTITY_NAME, RETCODE_HR_PARAMS(rc));
        return 0;
    }

    return 1;
}

int initSensors() {
    Retcode rc = UartOpenPort(gpsUart, &gpsUartHandler);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Warning: Failed to open UART %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, gpsUart, RETCODE_HR_PARAMS(rc));
        return 0;
    }

    rtl_size_t writtenBytes;
    uint8_t gnssNmea[] = { 0xb5, 0x62,
        0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00,
        0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbb, 0x58 };
    ssize_t expectedSize = sizeof(gnssNmea);
    rc = UartWrite(gpsUartHandler, gnssNmea, expectedSize, NULL, &writtenBytes);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Warning: Failed to write configuration message to GPS ("RETCODE_HR_FMT")\n", ENTITY_NAME, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    else if (writtenBytes != expectedSize) {
        fprintf(stderr, "[%s] Warning: Failed to write configuration message to GPS: %ld bytes were expected, %ld bytes were sent\n", ENTITY_NAME, expectedSize, writtenBytes);
        return 0;
    }
    uint8_t gnssSystems[] =  { 0xb5, 0x62,
        0x06, 0x8a, 0x4a, 0x00, 0x00, 0x07, 0x00, 0x00, 0x1f, 0x00, 0x31, 0x10, 0x01, 0x01, 0x00, 0x31,
        0x10, 0x01, 0x20, 0x00, 0x31, 0x10, 0x00, 0x05, 0x00, 0x31, 0x10, 0x00, 0x21, 0x00, 0x31, 0x10,
        0x01, 0x07, 0x00, 0x31, 0x10, 0x01, 0x22, 0x00, 0x31, 0x10, 0x00, 0x0d, 0x00, 0x31, 0x10, 0x00,
        0x0f, 0x00, 0x31, 0x10, 0x00, 0x24, 0x00, 0x31, 0x10, 0x00, 0x12, 0x00, 0x31, 0x10, 0x00, 0x14,
        0x00, 0x31, 0x10, 0x00, 0x25, 0x00, 0x31, 0x10, 0x01, 0x18, 0x00, 0x31, 0x10, 0x01, 0xa7, 0x51 };
    expectedSize = sizeof(gnssSystems);
    rc = UartWrite(gpsUartHandler, gnssSystems, expectedSize, NULL, &writtenBytes);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Warning: Failed to write configuration message to GPS ("RETCODE_HR_FMT")\n", ENTITY_NAME, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    else if (writtenBytes != expectedSize) {
        fprintf(stderr, "[%s] Warning: Failed to write configuration message to GPS: %ld bytes were expected, %ld bytes were sent\n", ENTITY_NAME, expectedSize, writtenBytes);
        return 0;
    }

    rc = I2cOpenChannel(barometerI2C, &barometerHandler);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Warning: Failed to open I2C %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, barometerI2C, RETCODE_HR_PARAMS(rc));
        return 0;
    }

    if (!writeRegister(0xF5, 0x90)) {
        fprintf(stderr, "[%s] Warning: Failed to set barometer filter\n", ENTITY_NAME);
        return 0;
    }
    if (!writeRegister(0xF4, 0x57)) {
        fprintf(stderr, "[%s] Warning: Failed to set barometer sampling rates\n", ENTITY_NAME);
        return 0;
    }

    for (int i = 0; i < 3; i++) {
        uint8_t value[2];
        if (!readRegister16(0x88 + i * 2, value)) {
            fprintf(stderr, "[%s] Warning: Failed to read barometer temperature coefficient\n", ENTITY_NAME);
            return 0;
        }
        if (i) {
            int16_t signedValue;
            memcpy(&signedValue, &value, sizeof(int16_t));
            tempCoefs[i] = signedValue;
        }
        else {
            uint16_t unsignedValue;
            memcpy(&unsignedValue, &value, sizeof(uint16_t));
            tempCoefs[i] = unsignedValue;
        }
    }

    for (int i = 0; i < 9; i++) {
        uint8_t value[2];
        if (!readRegister16(0x8E + i * 2, value)) {
            fprintf(stderr, "[%s] Warning: Failed to read barometer pressure coefficient\n", ENTITY_NAME);
            return 0;
        }
        if (i) {
            int16_t signedValue;
            memcpy(&signedValue, &value, sizeof(int16_t));
            pressCoefs[i] = signedValue;
        }
        else {
            uint16_t unsignedValue;
            memcpy(&unsignedValue, &value, sizeof(uint16_t));
            pressCoefs[i] = unsignedValue;
        }
    }

    barometerThread = std::thread(getBarometer);

    return 1;
}