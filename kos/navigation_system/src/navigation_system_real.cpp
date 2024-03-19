#include "../include/navigation_system.h"
#include "../../ipc_messages/include/initialization_interface.h"

#include <coresrv/hal/hal_api.h>
#include <rtl/retcode_hr.h>
#include <uart/uart.h>
#include <bsp/bsp.h>

#include <stdio.h>
#include <unistd.h>
#include <math.h>

#define NAME_MAX_LENGTH 64
#define RETRY_DELAY_SEC 1

char gpsUart[] = "uart3";
char gpsConfigSuffix[] = "default";
UartHandle gpsUartHandler = NULL;

void getSensors() {
    while (true) {
        uint8_t value;
        Retcode rc = UartReadByte(gpsUartHandler, &value);
        if (rc != rcOk) {
            fprintf(stderr, "[%s] Warning: Failed to read from UART %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, gpsUart, RETCODE_HR_PARAMS(rc));
            continue;
        }
        fprintf(stderr, "[%s] byte: %c\n", ENTITY_NAME, value);
    }
}

int initNavigationSystem() {
    while (!waitForInit("ns_pc_connection", "PeripheryController")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
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

int getCoords(int32_t &latitude, int32_t &longitude, int32_t &altitude) {
    bool read = true;
    int mode = 0;
    int idx = 0;
    char head[6];
    char latStr[33];
    char lngStr[33];
    char altStr[33];
    int latSign;
    int lngSign;

    uint8_t value;
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
        case 9: //Hdop
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
    longitude = round(10000000 * atof(latStr + 3) / 60.0f);
    latitude = round(10000000 * atof(lngStr + 2) / 60.0f);
    latStr[2] = '\0';
    lngStr[3] = '\0';
    longitude += 10000000 * atoi(lngStr);
    latitude += 10000000 * atoi(latStr);

    return 1;
}