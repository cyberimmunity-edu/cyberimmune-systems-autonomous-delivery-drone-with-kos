#pragma once

//#if AP_SIM_ENABLED
#include <AP_Mission/AP_Mission.h>
#include <AP_AHRS/AP_AHRS.h>

#define KOS_DATA_MESSAGE_HEAD_SIZE 4

class KOS_HardwareSimulation {
public:
    enum KOS_PeripheryCommand {
        ERROR = 0x00,
        MotorPermit,
        MotorForbid,
        CargoPermit,
        CargoForbid
    };

    struct KOS_SensorData {
        uint8_t head[KOS_DATA_MESSAGE_HEAD_SIZE];
        int32_t latitude;
        int32_t longitude;
        int32_t altitude;
        float speed;

        KOS_SensorData();
    };

    struct KOS_PeripheryData {
        uint8_t head[KOS_DATA_MESSAGE_HEAD_SIZE];
        uint8_t command;
        uint8_t filler[3];

        KOS_PeripheryData();
    };

private:
    static KOS_HardwareSimulation* _singleton;
    AP_AHRS* ahrs;
    AP_HAL::UARTDriver* uart_sensor;
    AP_HAL::UARTDriver* uart_periphery;

    uint8_t* received_periphery_data;

    bool arm_is_allowed;
    bool drop_is_allowed;
    bool cargo_is_dropped;

public:
    KOS_HardwareSimulation();
    ~KOS_HardwareSimulation();
    static KOS_HardwareSimulation* get_singleton();

    bool can_arm();
    void simulate_cargo_drop();

    void send_sensor_data();
    void receive_periphery_data();
    void on_new_executed_command(AP_Mission::Mission_Command cmd);
};

//#endif