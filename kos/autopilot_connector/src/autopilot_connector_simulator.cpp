/**
 * \file
 * \~English
 * \brief Implementation of methods for simulator autopilot communication.
 * \details The file contains implementation of methods,
 * that provide interaction between the security module and a
 * compatible ArduPilot SITL firmware.
 *
 * \~Russian
 * \brief Реализация методов для взаимодействия с симулятором автопилота.
 * \details В файле реализованы методы, обеспечивающие
 * взаимодействие между модулем безопасности и совместимой
 * SITL-прошивкой ArduPilot.
 */

#include "../include/autopilot_connector.h"

#include <kos_net.h>

/** \cond */
int autopilotSocket = NULL;
uint16_t autopilotPort = 5765;
/** \endcond */

int initAutopilotConnector() {
    if (!wait_for_network()) {
        logEntry("Connection to network has failed", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    return 1;
}

int initConnection() {
    close(autopilotSocket);
    autopilotSocket = NULL;

    if ((autopilotSocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        logEntry("Failed to create socket", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    struct sockaddr_in address = { 0 };
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(SIMULATOR_IP);
    address.sin_port = htons(autopilotPort);

    if (connect(autopilotSocket, (struct sockaddr*)&address, sizeof(address)) != 0) {
        char logBuffer[256] = {0};
        snprintf(logBuffer, 256, "Connection to %s:%d has failed", SIMULATOR_IP, autopilotPort);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    return 1;
}

int getAutopilotCommand(uint8_t& command) {
    uint8_t message[sizeof(AutopilotCommandMessage)];
    for (int i = 0; i < AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE; i++) {
        if (read(autopilotSocket, message + i, 1) != 1) {
            logEntry("Failed to read from socket", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }

        if (message[i] != AutopilotCommandMessageHead[i]) {
            logEntry("Received message has an unknown header", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
    }

    ssize_t expectedSize = sizeof(AutopilotCommandMessage) - AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE;
    ssize_t readBytes = read(autopilotSocket, message + AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE, expectedSize);
    if (readBytes != expectedSize) {
        char logBuffer[256] = {0};
        snprintf(logBuffer, 256, "Failed to read message from autopilot: %ld bytes were expected, %ld bytes were received", expectedSize, readBytes);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    command = (uint8_t)(((AutopilotCommandMessage*)message)->command);
    return 1;
}

int sendAutopilotBytes(uint8_t* bytes, ssize_t size) {
    write(autopilotSocket, bytes, size);

    return 1;
}