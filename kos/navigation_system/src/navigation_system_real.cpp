#include "../include/navigation_system.h"
#include "../../shared/include/ipc_messages_initialization.h"

#include <coresrv/hal/hal_api.h>
#include <rtl/retcode_hr.h>
#include <uart/uart.h>
#include <bsp/bsp.h>

#include <stdio.h>
#include <unistd.h>
#include <math.h>

#define NAME_MAX_LENGTH 64

char gpsUart[] = "uart3";
char gpsConfigSuffix[] = "default";
UartHandle gpsUartHandler = NULL;

void getSensors() {
    bool read;
    uint8_t value;
    int mode, idx, latSign, lngSign;
    int32_t latitude, longitude, altitude;
    char head[6], dopStr[9], latStr[33], lngStr[33], altStr[33];

    while (true) {
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
            case 1:
                //Header
                if (value == ',') {
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
            case 8: //Sats
                if (value == ',')
                    mode++;
                break;
            case 3:
                //Lat
                if (value == ',') {
                    latStr[idx] = '\0';
                    idx = 0;
                    mode = 4;
                }
                else {
                    latStr[idx] = value;
                    idx++;
                }
                break;
            case 4:
                //Lat dir
                if (value == 'N')
                    latSign = 1;
                else if (value == 'S')
                    latSign = -1;
                else if (value == ',')
                    mode = 5;
                break;
            case 5:
                //Lng
                if (value == ',') {
                    lngStr[idx] = '\0';
                    idx = 0;
                    mode = 6;
                }
                else {
                    lngStr[idx] = value;
                    idx++;
                }
                break;
            case 6:
                //Lng dir
                if (value == 'E')
                    lngSign = 1;
                else if (value == 'W')
                    lngSign = -1;
                else if (value == ',')
                    mode = 7;
                break;
            case 9:
                //Hdop
                if (value == ',') {
                    dopStr[idx] = '\0';
                    idx = 0;
                    mode = 10;
                }
                else {
                    dopStr[idx] = value;
                    idx++;
                }
                break;
            case 10:
                //Alt
                if (value == ',') {
                    altStr[idx] = '\0';
                    idx = 0;
                    read = false;
                }
                else {
                    altStr[idx] = value;
                    idx++;
                }
                break;
            }
        }

        altitude = round(100 * atof(altStr));
        longitude = round(10000000 * atof(lngStr + 3) / 60.0f);
        latitude = round(10000000 * atof(latStr + 2) / 60.0f);
        lngStr[3] = '\0';
        latStr[2] = '\0';
        longitude += 10000000 * atoi(lngStr);
        latitude += 10000000 * atoi(latStr);

        setCoords(latitude, longitude, altitude);
        setDop(atof(dopStr));
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

    Retcode rc = BspInit(NULL);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to initialize BSP ("RETCODE_HR_FMT")\n", ENTITY_NAME, RETCODE_HR_PARAMS(rc));
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

    return 1;
}

int initSensors() {
    Retcode rc = UartOpenPort(gpsUart, &gpsUartHandler);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Warning: Failed to open UART %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, gpsUart, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    UartConfig uartPortConfig;
    rc = UartGetConfig(gpsUartHandler, &uartPortConfig);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Warning: Failed to get config from UART %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, gpsUart, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    uartPortConfig.baudrate = UartBaudRate::UART_BR_9600;
    rc = UartSetConfig(gpsUartHandler, &uartPortConfig);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Warning: Failed to set config to UART %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, gpsUart, RETCODE_HR_PARAMS(rc));
        return 0;
    }

    return 1;
}