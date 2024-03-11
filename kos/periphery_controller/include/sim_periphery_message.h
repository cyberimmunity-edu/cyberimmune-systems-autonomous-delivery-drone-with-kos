#pragma once

#include <stdint.h>

#define SIM_PERIPHERY_MESSAGE_HEAD_SIZE 4

static const uint8_t SimPeripheryMessageHead[SIM_PERIPHERY_MESSAGE_HEAD_SIZE] = { 0x06, 0x66, 0xbe, 0xa7 };

enum SimPeripheryCommand : uint8_t {
    ERROR = 0x00,
    MotorPermit,
    MotorForbid,
    CargoPermit,
    CargoForbid
};

struct SimPeripheryMessage {
    uint8_t head[SIM_PERIPHERY_MESSAGE_HEAD_SIZE];
    SimPeripheryCommand command;
    uint8_t filler[3];

    SimPeripheryMessage(SimPeripheryCommand cmd) {
        for (int i = 0; i < SIM_PERIPHERY_MESSAGE_HEAD_SIZE; i++)
            head[i] = SimPeripheryMessageHead[i];
        command = cmd;
    }
};