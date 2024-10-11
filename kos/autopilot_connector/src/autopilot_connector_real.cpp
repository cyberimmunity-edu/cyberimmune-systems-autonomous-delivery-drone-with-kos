/**
 * \file
 * \~English
 * \brief Implementation of methods for hardware autopilot communication.
 * \details The file contains implementation of methods,
 * that provide interaction between the security module and a
 * hardware flight controller with a compatible ArduPilot firmware.
 *
 * \~Russian
 * \brief Реализация методов для взаимодействия с аппаратным автопилотом.
 * \details В файле реализованы методы, обеспечивающие
 * взаимодействие между модулем безопасности и аппаратным полетным
 * контроллером с совместимой прошивкой ArduPilot.
 */

#include "../include/autopilot_connector.h"
#include "../../shared/include/ipc_messages_initialization.h"

#include <coresrv/hal/hal_api.h>
#include <rtl/retcode_hr.h>
#include <uart/uart.h>
#include <bsp/bsp.h>

#include <stdio.h>
#include <unistd.h>

/** \cond */
#define NAME_MAX_LENGTH 64

char autopilotUart[] = "uart2";
char autopilotConfigSuffix[] = "default";
UartHandle autopilotUartHandler = NULL;
/** \endcond */

/**
 * \~English Sends the given value to the autopilot via UART interface.
 * \param[in] value Value sent to autopilot.
 * \return Returns 1 on successful value pass, 0 otherwise.
 * \~Russian Передает в автопилот поданное значение через UART-интерфейс.
 * \param[in] value Отправляемое в автопилот значение.
 * \return Возвращает 1 при успешной передаче значения, 0 -- иначе.
 */
int writeIntValue(int32_t value) {
    rtl_size_t writtenBytes;
    ssize_t expectedSize = sizeof(int32_t);
    char logBuffer[256] = {0};
    Retcode rc = UartWrite(autopilotUartHandler, (uint8_t*)(&value), expectedSize, NULL, &writtenBytes);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to write to UART %s ("RETCODE_HR_FMT")", autopilotUart, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    else if (writtenBytes != expectedSize) {
        snprintf(logBuffer, 256, "Failed to write message to autopilot: %ld bytes were expected, %ld bytes were sent", expectedSize, writtenBytes);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    return 1;
}

int initAutopilotConnector() {
    while (!waitForInit("periphery_controller_connection", "PeripheryController")) {
        logEntry("Failed to receive initialization notification from Periphery Controller. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    char boardName[NAME_MAX_LENGTH + 1] = {0};
    if (KnHalGetEnv("board", boardName, sizeof(boardName)) != rcOk) {
        logEntry("Failed to get board name", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    char autopilotConfig[NAME_MAX_LENGTH + 1] = {0};
    if (snprintf(autopilotConfig, NAME_MAX_LENGTH, "%s.%s", boardName, autopilotConfigSuffix) < 0) {
        logEntry("Failed to generate UART config name", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    char logBuffer[256] = {0};
    Retcode rc = BspInit(NULL);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to initialize BSP ("RETCODE_HR_FMT")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    rc = BspEnableModule(autopilotUart);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to enable UART %s ("RETCODE_HR_FMT")", autopilotUart, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    rc = BspSetConfig(autopilotUart, autopilotConfig);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to set BSP config for UART %s ("RETCODE_HR_FMT")", autopilotUart, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    rc = UartInit();
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to initialize UART ("RETCODE_HR_FMT")", RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    return 1;
}

int initConnection() {
    Retcode rc = UartOpenPort(autopilotUart, &autopilotUartHandler);
    if (rc != rcOk) {
        char logBuffer[256] = {0};
        snprintf(logBuffer, 256, "Failed to open UART %s ("RETCODE_HR_FMT")", autopilotUart, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    return 1;
}

int getAutopilotCommand(uint8_t& command) {
    uint8_t message[sizeof(AutopilotCommandMessage)];
    char logBuffer[256] = {0};
    for (int i = 0; i < AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE; i++) {
        Retcode rc = UartReadByte(autopilotUartHandler, message + i);
        if (rc != rcOk) {
            snprintf(logBuffer, 256, "Failed to read from UART %s ("RETCODE_HR_FMT")", autopilotUart, RETCODE_HR_PARAMS(rc));
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }

        if (message[i] != AutopilotCommandMessageHead[i]) {
            logEntry("Received message has an unknown header", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
    }

    ssize_t expectedSize = sizeof(AutopilotCommandMessage) - AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE;
    rtl_size_t readBytes;
    Retcode rc = UartRead(autopilotUartHandler, (rtl_uint8_t*)(message + AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE),
        expectedSize, NULL, &readBytes);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to read from UART %s ("RETCODE_HR_FMT")", autopilotUart, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    else if (readBytes != expectedSize) {
        snprintf(logBuffer, 256, "Failed to read message from autopilot: %ld bytes were expected, %ld bytes were received", expectedSize, readBytes);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    command = (uint8_t)(((AutopilotCommandMessage*)message)->command);
    return 1;
}

int sendAutopilotCommand(AutopilotCommand command) {
    AutopilotCommandMessage message = AutopilotCommandMessage(command);

    rtl_size_t writtenBytes;
    ssize_t expectedSize = sizeof(AutopilotCommandMessage);
    char logBuffer[256] = {0};
    Retcode rc = UartWrite(autopilotUartHandler, (uint8_t*)(&message), expectedSize, NULL, &writtenBytes);
    if (rc != rcOk) {
        snprintf(logBuffer, 256, "Failed to write to UART %s ("RETCODE_HR_FMT")", autopilotUart, RETCODE_HR_PARAMS(rc));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    else if (writtenBytes != expectedSize) {
        snprintf(logBuffer, 256, "Failed to write message to autopilot: %ld bytes were expected, %ld bytes were sent", expectedSize, writtenBytes);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    return 1;
}

int sendAutopilotCommand(AutopilotCommand command, int32_t value) {
    sendAutopilotCommand(command);

    return writeIntValue(value);
}

int sendAutopilotCommand(AutopilotCommand command, int32_t valueFirst, int32_t valueSecond, int32_t valueThird) {
    sendAutopilotCommand(command);

    int fst = writeIntValue(valueFirst);
    int snd = writeIntValue(valueSecond);
    int trd = writeIntValue(valueThird);

    return (fst && snd && trd);
}