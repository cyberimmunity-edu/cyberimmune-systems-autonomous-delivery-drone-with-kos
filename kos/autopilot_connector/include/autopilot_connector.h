#pragma once

#include <stdint.h>

#define AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE 4

static const uint8_t AutopilotCommandMessageHead[AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE] = { 0x7a, 0xfe, 0xf0, 0x0d };

enum AutopilotCommand : uint8_t {
    ERROR = 0x00,
    ArmRequest = 0x3A,
    ArmPermit = 0xA5,
    ArmForbid = 0xE4,
    PauseFlight = 0xAB,
    ResumeFlight = 0x47,
    ChangeWaypoint = 0x10,
    ChangeSpeed = 0xEE,
    ChangeAltitude = 0xA1
};

struct AutopilotCommandMessage {
    uint8_t head[AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE];
    AutopilotCommand command;

    AutopilotCommandMessage() {
        for (int i = 0; i < AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE; i++)
            head[i] = 0;
        command = AutopilotCommand::ERROR;
    }

    AutopilotCommandMessage(AutopilotCommand _command) {
        for (int i = 0; i < AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE; i++)
            head[i] = AutopilotCommandMessageHead[i];
        command = _command;
    }
};

int initAutopilotConnector();
int initConnection();

int getAutopilotCommand(uint8_t& command);
int sendAutopilotCommand(AutopilotCommand command);
int sendAutopilotCommand(AutopilotCommand command, int32_t value);
int sendAutopilotCommand(AutopilotCommand command, int32_t valueFirst, int32_t valueSecond, int32_t valueThird);