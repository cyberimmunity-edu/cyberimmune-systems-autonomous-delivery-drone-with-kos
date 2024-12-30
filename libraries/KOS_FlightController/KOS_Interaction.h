#pragma once

#include <AP_HAL/UARTDriver.h>

#define KOS_MESSAGE_HEAD_SIZE 4

class KOS_InteractionModule {
public:
    enum KOS_Command {
        ERROR = 0x00,
        Command_ArmRequest = 0x3A,
        Command_ArmPermit = 0xA5,
        Command_ArmForbid = 0xE4,
        Command_PauseFlight = 0xAB,
        Command_ResumeFlight = 0x47,
        Command_ChangeWaypoint = 0x10,
        Command_ChangeSpeed = 0xEE,
        Command_ChangeAltitude = 0xA1,
        Command_SetMission = 0x42
    };

    struct KOS_Message {
        uint8_t head[KOS_MESSAGE_HEAD_SIZE];
        uint8_t command;

        KOS_Message();
        KOS_Message(KOS_Command _command);
    };

private:
    AP_HAL::UARTDriver* uart_read;
    AP_HAL::UARTDriver* uart_write;

    uint8_t* received_message;

    bool readInt(int32_t* value);
    bool readLocation(int32_t* lat, int32_t* lng, int32_t* alt);

public:
    KOS_InteractionModule();
    ~KOS_InteractionModule();

    void request_arm();
    void receive_KOS_message();
};