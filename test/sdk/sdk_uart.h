#pragma once
#include <stdlib.h>

#define KOS_COMMAND_MESSAGE_HEAD_SIZE 4

enum KOSCommand {
    ERROR = 0x00,
    ArmRequest = 0x3A,
    ArmPermit = 0xA5,
    ArmForbid = 0xE4,
    AbortFlight = 0xAB
};

struct KOSCommandMessage {
    uint8_t head[KOS_COMMAND_MESSAGE_HEAD_SIZE];
    uint8_t command;

    KOSCommandMessage();
    KOSCommandMessage(KOSCommand com);
};

int waitForCommand();
int sendCommand(KOSCommand com);