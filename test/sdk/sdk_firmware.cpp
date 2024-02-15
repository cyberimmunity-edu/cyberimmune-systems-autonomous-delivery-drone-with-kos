#include "sdk_firmware.h"
#include "sdk_gpio.h"
#include "sdk_light.h"
#include "sdk_kill_switch.h"

#ifdef FOR_SITL
#include "sdk_autopilot_communication.h"
#else
#include <coresrv/hal/hal_api.h>
#include <rtl/retcode_hr.h>
#include <bsp/bsp.h>
#include <gpio/gpio.h>
#include <uart/uart.h>
#include <i2c/i2c.h>
#endif

#include <kos_net.h>
#include <stdio.h>

char gpio_name[] = "gpio0";
char uart_name[] = "uart3";
char i2c_compass_name[] = "i2c0";
char i2c_mpu_name[] = "i2c1";
char gpio_config_suffix[] = "default";
char uart_config_suffix[] = "default";
char i2c_compass_config_suffix[] = "p0-1";
char i2c_mpu_config_suffix[] = "p2-3";

int initializeFirmware() {
#ifndef FOR_SITL
    char boardName[64] = {0};
    if (KnHalGetEnv("board", boardName, sizeof(boardName)) != rcOk)
    {
        fprintf(stderr, "Erro: failed to get board name\n");
        return 0;
    }
    char gpioConfig[128], uartConfig[128], compassConfig[128], mpuConfig[128];
    int ret = snprintf(gpioConfig, 128, "%s.%s", boardName, gpio_config_suffix);
    if (ret < 0) {
        fprintf(stderr, "Error: failed to generate GPIO config name\n");
        return 0;
    }
    ret = snprintf(uartConfig, 128, "%s.%s", boardName, uart_config_suffix);
    if (ret < 0) {
        fprintf(stderr, "Error: failed to generate UART config name\n");
        return 0;
    }
    ret = snprintf(compassConfig, 128, "%s.%s", boardName, i2c_compass_config_suffix);
    if (ret < 0) {
        fprintf(stderr, "Error: failed to generate I2C compass config name\n");
        return 0;
    }
    ret = snprintf(mpuConfig, 128, "%s.%s", boardName, i2c_mpu_config_suffix);
    if (ret < 0) {
        fprintf(stderr, "Error: failed to generate I2C mpu config name\n");
        return 0;
    }

    Retcode rc = BspInit(NULL);
    fprintf(stderr, "Info: initializing BSP\n");
    if (rc != rcOk) {
        fprintf(stderr, "Error: failed to initialize BSP ("RETCODE_HR_FMT")\n", RETCODE_HR_PARAMS(rc));
        return 0;
    }
    fprintf(stderr, "Info: BSP is initialized\n");

    fprintf(stderr, "Info: initializing GPIO\n");
    rc = BspSetConfig(gpio_name, gpioConfig);
    if (rc != rcOk) {
        fprintf(stderr, "Error: failed to set BSP config for GPIO ("RETCODE_HR_FMT")\n", gpio_name, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    rc = GpioInit();
    if (rc != rcOk) {
        fprintf(stderr, "Error: failed to initialize GPIO ("RETCODE_HR_FMT")\n", RC_GET_CODE(rc));
        return 0;
    }
    fprintf(stderr, "Info: GPIO is initialized\n");
    setKillSwitch(0);
    setLight(1);

    fprintf(stderr, "Info: initializing UART\n");
    rc = BspEnableModule(uart_name);
    if (rc != rcOk) {
        fprintf(stderr, "Error: failed to enable UART module ("RETCODE_HR_FMT")\n", RETCODE_HR_PARAMS(rc));
        return 0;
    }
    rc = BspSetConfig(uart_name, uartConfig);
    if (rc != rcOk) {
        fprintf(stderr, "Error: failed to set BSP config for UART ("RETCODE_HR_FMT")\n", RETCODE_HR_PARAMS(rc));
        return 0;
    }
    rc = UartInit();
    if (rc != rcOk) {
        fprintf(stderr, "Error: failed to initialize UART ("RETCODE_HR_FMT")\n", RETCODE_HR_PARAMS(rc));
        return 0;
    }
    fprintf(stderr, "Info: UART is initialized\n");

    fprintf(stderr, "Info: initializing I2C\n");
    rc = I2cInit();
    if (rc != rcOk) {
        fprintf(stderr, "Error: failed to initialize I2C\n");
        return 0;
    }
    fprintf(stderr, "Info: I2C is initialized\n");

    fprintf(stderr, "Info: initializing QMC5883 compass\n");
    rc = BspEnableModule(i2c_compass_name);
    if (rc != rcOk) {
        fprintf(stderr, "Error: failed to enable I2C module on channel '%s'\n", i2c_compass_name);
        return 0;
    }
    rc = BspSetConfig(i2c_compass_name, compassConfig);
    if (rc != rcOk) {
        fprintf(stderr, "Error: failed to set BSP config for I2C channel '%s'\n", i2c_compass_name);
        return 0;
    }
    fprintf(stderr, "Info: QMC5883 compass is initialized\n");

    fprintf(stderr, "Info: initializing MPU6050\n");
    rc = BspEnableModule(i2c_mpu_name);
    if (rc != rcOk) {
        fprintf(stderr, "Error: failed to enable I2C module on channel '%s'\n", i2c_mpu_name);
        return 0;
    }
    rc = BspSetConfig(i2c_mpu_name, mpuConfig);
    if (rc != rcOk) {
        fprintf(stderr, "Error: failed to set BSP config for I2C channel '%s'\n", i2c_mpu_name);
        return 0;
    }
    fprintf(stderr, "Info: MPU6050 is initialized\n");
#endif

    fprintf(stderr, "Info: all modules are initialized\n");
    setLight(0);

    fprintf(stderr, "Info: initializing network connection\n");
    if (!wait_for_network()) {
        fprintf(stderr, "Error: failed to connect to network\n");
        return 0;
    }
    fprintf(stderr, "Info: network connection is established\n");

#ifdef FOR_SITL
    if (!initializeSitlUart()) {
        fprintf(stderr, "Error: failed to open SITL socket\n");
        return 0;
    }
#endif

    return 1;
}

char* getChannelName(firmwareChannel channel) {
    switch (channel) {
    case GPIO:
        return gpio_name;
    case UART:
        return uart_name;
    case COMPASS:
        return i2c_compass_name;
    case MPU:
        return i2c_mpu_name;
    }

    return NULL;
}