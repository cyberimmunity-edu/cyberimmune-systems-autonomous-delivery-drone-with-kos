#include "KOS_HardwareSimulation.h"
#if AP_SIM_ENABLED
#include <KOS_FlightController/KOS_CommandExecutor.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Arming/AP_Arming.h>
#include <GCS_MAVLink/GCS.h>

KOS_HardwareSimulation* KOS_HardwareSimulation::_singleton;

static const uint8_t kos_sensor_data_head[KOS_DATA_MESSAGE_HEAD_SIZE] = { 0x71, 0x11, 0xda, 0x1a };
static const uint8_t kos_periphery_data_head[KOS_DATA_MESSAGE_HEAD_SIZE] = { 0x06, 0x66, 0xbe, 0xa7 };

KOS_HardwareSimulation::KOS_SensorData::KOS_SensorData() {
    memcpy(head, kos_sensor_data_head, KOS_DATA_MESSAGE_HEAD_SIZE);
}

KOS_HardwareSimulation::KOS_PeripheryData::KOS_PeripheryData() {
    memcpy(head, kos_periphery_data_head, KOS_DATA_MESSAGE_HEAD_SIZE);
    command = KOS_PeripheryCommand::ERROR;
}

KOS_HardwareSimulation::KOS_HardwareSimulation() {
    _singleton = this;
    ahrs = AP_AHRS::get_singleton();
    received_periphery_data = new uint8_t[sizeof(KOS_PeripheryData)];
    arm_is_allowed = true;
    drop_is_allowed = true;
    cargo_is_dropped = false;
}

KOS_HardwareSimulation::~KOS_HardwareSimulation() {
    delete[] received_periphery_data;
}

KOS_HardwareSimulation* KOS_HardwareSimulation::get_singleton() {
    return _singleton;
}

bool KOS_HardwareSimulation::can_arm() {
    return arm_is_allowed;
}

void KOS_HardwareSimulation::simulate_cargo_drop() {
    if (drop_is_allowed) {
        if (cargo_is_dropped)
            gcs().send_text(MAV_SEVERITY_INFO, "Info: No cargo, it has already been dropped");
        else {
            gcs().send_text(MAV_SEVERITY_INFO, "Info: Cargo is dropped");
            cargo_is_dropped = true;
        }
    }
    else
        gcs().send_text(MAV_SEVERITY_INFO, "Info: Lock prevented cargo from dropping");
}

void KOS_HardwareSimulation::send_sensor_data() {
    if (!uart_sensor) {
        uart_sensor = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_KOS_Sensor_Data, 0);
        if (!uart_sensor)
            return;
        uart_sensor->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }

    Location coord;
    ahrs->get_location(coord);
    KOS_SensorData packet = KOS_SensorData();
    packet.latitude = coord.lat;
    packet.longitude = coord.lng;
    packet.altitude = coord.alt;
    Vector3f speed;
    UNUSED_RESULT(ahrs->get_velocity_NED(speed));
    packet.speed = sqrtF(speed.x * speed.x + speed.y * speed.y);
    uart_sensor->write((uint8_t*)(&packet), sizeof(KOS_SensorData));
}

void KOS_HardwareSimulation::receive_periphery_data() {
    if (!uart_periphery) {
        uart_periphery = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_KOS_Periphery_Data, 0);
        if (!uart_periphery)
            return;
        uart_periphery->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }

    memset(received_periphery_data, 0, KOS_DATA_MESSAGE_HEAD_SIZE);
    for (int i = 0; i < KOS_DATA_MESSAGE_HEAD_SIZE; i++) {
        ssize_t size = uart_periphery->read(received_periphery_data + i, 1);
        if (!size)
            return;
        if (received_periphery_data[i] != kos_periphery_data_head[i]) {
            gcs().send_text(MAV_SEVERITY_INFO, "KOS Periphery Data Error: Unknown message head");
            return;
        }
    }
    ssize_t expected_size = sizeof(KOS_PeripheryData) - KOS_DATA_MESSAGE_HEAD_SIZE;
    ssize_t size = uart_periphery->read(received_periphery_data + KOS_DATA_MESSAGE_HEAD_SIZE, expected_size);
    if (!size || (size != expected_size)) {
        gcs().send_text(MAV_SEVERITY_INFO, "KOS Periphery Data Error: Message is shorter than expected (%d vs %d)", (int)(size + KOS_DATA_MESSAGE_HEAD_SIZE), (int)sizeof(KOS_PeripheryData));
        return;
    }

    KOS_PeripheryData data;
    memcpy(&data, received_periphery_data, sizeof(KOS_PeripheryData));
    switch (data.command) {
        case KOS_PeripheryCommand::MotorPermit:
            arm_is_allowed = true;
            break;
        case KOS_PeripheryCommand::MotorForbid:
            AP_Arming::get_singleton()->disarm(AP_Arming::Method::SCRIPTING, false);
            arm_is_allowed = false;
            break;
        case KOS_PeripheryCommand::CargoPermit:
            drop_is_allowed = true;
            break;
        case KOS_PeripheryCommand::CargoForbid:
            drop_is_allowed = false;
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "KOS Periphery Data Error: Unknown command %d is received", (int)data.command);
            break;
    }
}

void KOS_HardwareSimulation::on_new_executed_command(AP_Mission::Mission_Command cmd) {
    if ((cmd.id == MAV_CMD_DO_SET_SERVO) && (cmd.content.servo.channel == 5) && (cmd.content.servo.pwm == 1200))
        simulate_cargo_drop();
}

#endif