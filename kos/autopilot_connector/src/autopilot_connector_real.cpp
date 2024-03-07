#include "../include/autopilot_connector.h"
#include "../../ipc_messages/include/initialization_interface.h"

#include <coresrv/hal/hal_api.h>
#include <rtl/retcode_hr.h>
#include <uart/uart.h>
#include <bsp/bsp.h>

#include <stdio.h>
#include <unistd.h>

#define NAME_MAX_LENGTH 64
#define RETRY_DELAY_SEC 1

char autopilotUart[] = "uart3";
char autopilotConfigSuffix[] = "default";
UartHandle autopilotUartHandler = NULL;

int initAutopilotConnector() {
    if (!waitForInit("ap_pc_connection", "PeripheryController")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }

    char boardName[NAME_MAX_LENGTH] = {0};
    if (KnHalGetEnv("board", boardName, sizeof(boardName)) != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to get board name\n", ENTITY_NAME);
        return 0;
    }

    char autopilotConfig[NAME_MAX_LENGTH];
    if (snprintf(autopilotConfig, NAME_MAX_LENGTH, "%s.%s", boardName, autopilotConfigSuffix) < 0) {
        fprintf(stderr, "[%s] Error: Failed to generate UART config name\n", ENTITY_NAME);
        return 0;
    }

    Retcode rc = BspInit(NULL);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to initialize BSP ("RETCODE_HR_FMT")\n", ENTITY_NAME, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    rc = BspEnableModule(autopilotUart);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to enable UART %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, autopilotUart, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    rc = BspSetConfig(autopilotUart, autopilotConfig);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to set BSP config for UART %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, autopilotUart, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    rc = UartInit();
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to initialize UART ("RETCODE_HR_FMT")\n", ENTITY_NAME, RETCODE_HR_PARAMS(rc));
        return 0;
    }

    return 1;
}

int initConnection() {
    Retcode rc = UartOpenPort(autopilotUart, &autopilotUartHandler);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Warning: Failed to open UART %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, autopilotUart, RETCODE_HR_PARAMS(rc));
        return 0;
    }

    return 1;
}

int getAutopilotCommand(uint8_t& command) {
    uint8_t message[sizeof(AutopilotCommandMessage)];
    for (int i = 0; i < AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE; i++) {
        Retcode rc = UartReadByte(autopilotUartHandler, message + i);
        if (rc != rcOk) {
            fprintf(stderr, "[%s] Warning: Failed to read from UART %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, autopilotUart, RETCODE_HR_PARAMS(rc));
            return 0;
        }

        if (message[i] != AutopilotCommandMessageHead[i]) {
            fprintf(stderr, "[%s] Warning: Received message has an unknown header\n", ENTITY_NAME);
            return 0;
        }
    }

    ssize_t expectedSize = sizeof(AutopilotCommandMessage) - AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE;
    rtl_size_t readBytes;
    Retcode rc = UartRead(autopilotUartHandler, (rtl_uint8_t*)(message + AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE),
        expectedSize, NULL, &readBytes);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Warning: Failed to read from UART %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, autopilotUart, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    else if (readBytes != expectedSize) {
        fprintf(stderr, "[%s] Warning: Failed to read message from autopilot: %ld bytes were expected, %ld bytes were received\n", ENTITY_NAME, expectedSize, readBytes);
        return 0;
    }

    command = (uint8_t)(((AutopilotCommandMessage*)message)->command);
    return 1;
}

int sendAutopilotCommand(AutopilotCommand command) {
    AutopilotCommandMessage message = AutopilotCommandMessage(command);

    rtl_size_t writtenBytes;
    ssize_t expectedSize = sizeof(AutopilotCommandMessage);
    Retcode rc = UartWrite(autopilotUartHandler, (uint8_t*)(&message), expectedSize, NULL, &writtenBytes);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Warning: Failed to write to UART %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, autopilotUart, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    else if (writtenBytes != expectedSize) {
        fprintf(stderr, "[%s] Warning: Failed to write message to autopilot: %ld bytes were expected, %ld bytes were sent\n", ENTITY_NAME, expectedSize, writtenBytes);
        return 0;
    }

    return 1;
}