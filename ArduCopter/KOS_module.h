#pragma once

#include <AP_HAL/UARTDriver.h>

#define KOS_COMMAND_MESSAGE_HEAD_SIZE 4

class KOSCommModule {
public:
    enum KOSCommandType {
        ERROR = 0x00,
        Command_ArmRequest = 0x3A,
        Command_ArmPermit = 0xA5,
        Command_ArmForbid = 0xE4,
        Command_AbortFlight = 0xAB
    };

    struct KOSCommandMessage {
        uint8_t head[KOS_COMMAND_MESSAGE_HEAD_SIZE];
        uint8_t command;

        KOSCommandMessage();
        KOSCommandMessage(KOSCommandType com);
    };

private:
    AP_HAL::UARTDriver* uart_read;
    AP_HAL::UARTDriver* uart_write;
    
    bool arm_is_allowed;

    uint8_t* command_send;
    uint8_t* command_receive;

public:
    KOSCommModule();
    ~KOSCommModule();

    bool can_arm();

    void send_arm_request();
    void wait_for_KOS_message();
};