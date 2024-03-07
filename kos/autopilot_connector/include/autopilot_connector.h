#pragma once

#include "../../shared/include/autopilot_command.h"

#define AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE 4

static const uint8_t AutopilotCommandMessageHead[AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE] = { 0x7a, 0xfe, 0xf0, 0x0d };

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