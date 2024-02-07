#pragma once

enum firmwareChannel {
    GPIO,
    UART,
    COMPASS,
    MPU
};

int initializeFirmware();

char* getChannelName(firmwareChannel channel);