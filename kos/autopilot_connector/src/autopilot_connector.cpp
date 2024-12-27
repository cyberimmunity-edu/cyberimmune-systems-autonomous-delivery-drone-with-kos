/**
 * \file
 * \~English
 * \brief Implementation of methods for an autopilot communication.
 * \details The file contains implementation of methods, that provide
 * interaction between the security module with a compatible ArduPilot firmware.
 *
 * \~Russian
 * \brief Реализация методов для взаимодействия с автопилотом.
 * \details В файле реализованы методы, обеспечивающие
 * взаимодействие между модулем безопасности и прошивкой ArduPilot.
 */

#include "../include/autopilot_connector.h"

#include <string.h>

int sendAutopilotCommand(AutopilotCommand command) {
    ssize_t size = sizeof(AutopilotCommandMessage);
    uint8_t bytes[size] = {0};

    AutopilotCommandMessage message = AutopilotCommandMessage(command);
    memcpy(bytes, &message, sizeof(AutopilotCommandMessage));

    return sendAutopilotBytes(bytes, size);
}

int sendAutopilotCommand(AutopilotCommand command, int32_t value) {
    ssize_t size = sizeof(AutopilotCommandMessage) + sizeof(int32_t);
    uint8_t bytes[size] = {0};

    AutopilotCommandMessage message = AutopilotCommandMessage(command);
    memcpy(bytes, &message, sizeof(AutopilotCommandMessage));
    memcpy(bytes + sizeof(AutopilotCommandMessage), &value, sizeof(int32_t));

    return sendAutopilotBytes(bytes, size);
}

int sendAutopilotCommand(AutopilotCommand command, int32_t valueFirst, int32_t valueSecond, int32_t valueThird) {
    ssize_t size = sizeof(AutopilotCommandMessage) + 3 * sizeof(int32_t);
    uint8_t bytes[size] = {0};

    AutopilotCommandMessage message = AutopilotCommandMessage(command);
    memcpy(bytes, &message, sizeof(AutopilotCommandMessage));

    int shift = sizeof(AutopilotCommandMessage);
    memcpy(bytes + shift, &valueFirst, sizeof(int32_t));
    shift += sizeof(int32_t);
    memcpy(bytes + shift, &valueSecond, sizeof(int32_t));
    shift += sizeof(int32_t);
    memcpy(bytes + shift, &valueThird, sizeof(int32_t));

    return sendAutopilotBytes(bytes, size);
}