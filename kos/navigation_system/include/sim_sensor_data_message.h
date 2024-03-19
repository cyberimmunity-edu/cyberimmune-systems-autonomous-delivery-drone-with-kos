#pragma once

#include <stdint.h>

#define SIM_SENSOR_DATA_MESSAGE_HEAD_SIZE 4

static const uint8_t SimSensorDataMessageHead[SIM_SENSOR_DATA_MESSAGE_HEAD_SIZE] = { 0x71, 0x11, 0xda, 0x1a };

struct SimSensorDataMessage {
    uint8_t head[SIM_SENSOR_DATA_MESSAGE_HEAD_SIZE];
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
};