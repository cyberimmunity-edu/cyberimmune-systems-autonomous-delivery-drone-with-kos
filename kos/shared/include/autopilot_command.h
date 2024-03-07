#pragma once

#include <stdint.h>

enum AutopilotCommand : uint8_t {
    ERROR = 0x00,
    ArmRequest = 0x3A,
    ArmPermit = 0xA5,
    ArmForbid = 0xE4,
    AbortFlight = 0xAB
};