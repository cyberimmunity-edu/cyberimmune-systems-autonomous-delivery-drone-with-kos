#include "KOS_Interaction.h"
#include <KOS_FlightController/KOS_CommandExecutor.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Arming/AP_Arming.h>
#include <GCS_MAVLink/GCS.h>

static const uint8_t kos_message_head[KOS_MESSAGE_HEAD_SIZE] = { 0x7a, 0xfe, 0xf0, 0x0d };

KOS_InteractionModule::KOS_Message::KOS_Message() {
    for (int i = 0; i < KOS_MESSAGE_HEAD_SIZE; i++)
        head[i] = 0;
    command = KOS_Command::ERROR;
}

KOS_InteractionModule::KOS_Message::KOS_Message(KOS_Command _command) {
    memcpy(head, kos_message_head, KOS_MESSAGE_HEAD_SIZE);
    command = (uint8_t)_command;
}

bool KOS_InteractionModule::readInt(int32_t* value) {
    ssize_t expected_size = sizeof(int32_t);
    ssize_t read_size = uart_read->read((uint8_t*)(value), expected_size);
    return (read_size == expected_size);
}

bool KOS_InteractionModule::readLocation(int32_t* lat, int32_t* lng, int32_t* alt) {
    return (readInt(lat) && readInt(lng) && readInt(alt));
}

KOS_InteractionModule::KOS_InteractionModule() {
    received_message = new uint8_t[sizeof(KOS_Message)];
}

KOS_InteractionModule::~KOS_InteractionModule() {
    delete[] received_message;
}

void KOS_InteractionModule::request_arm() {
    if (!uart_write) {
        uart_write = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_KOS_Interaction, 0);
        if (!uart_write) {
            gcs().send_text(MAV_SEVERITY_INFO, "Error: KOS UART is not available");
            return;
        }
        uart_write->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }

    KOS_Message packet = KOS_Message(KOS_Command::Command_ArmRequest);
    uart_write->write((uint8_t*)(&packet), sizeof(KOS_Message));
    gcs().send_text(MAV_SEVERITY_INFO, "Info: Arm request is sent to KOS");
}

void KOS_InteractionModule::receive_KOS_message() {
    if (!uart_read) {
        uart_read = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_KOS_Interaction, 0);
        if (!uart_read)
            return;
        uart_read->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }

    memset(received_message, 0, KOS_MESSAGE_HEAD_SIZE);
    for (int i = 0; i < KOS_MESSAGE_HEAD_SIZE; i++) {
        ssize_t size = uart_read->read(received_message + i, 1);
        if (!size)
            return;
        if (received_message[i] != kos_message_head[i]) {
            gcs().send_text(MAV_SEVERITY_INFO, "KOS Message Error: Unknown message head");
            return;
        }
    }
    ssize_t expected_size = sizeof(KOS_Message) - KOS_MESSAGE_HEAD_SIZE;
    ssize_t size = uart_read->read(received_message + KOS_MESSAGE_HEAD_SIZE, expected_size);
    if (size != expected_size) {
        gcs().send_text(MAV_SEVERITY_INFO, "KOS Message Error: Message is shorter than expected (%d vs %d)", (int)(size + KOS_MESSAGE_HEAD_SIZE), (int)sizeof(KOS_Message));
        return;
    }

    KOS_Message message;
    memcpy(&message, received_message, sizeof(KOS_Message));
    switch (message.command) {
        case KOS_Command::Command_ArmPermit:
            gcs().send_text(MAV_SEVERITY_INFO, "Info: Arm is permitted");
            break;
        case KOS_Command::Command_ArmForbid:
            gcs().send_text(MAV_SEVERITY_INFO, "Info: Arm is forbidden");
            break;
        case KOS_Command::Command_PauseFlight: {
            if (AP_Arming::get_singleton()->is_armed())
                KOS_CommandExecutor::get_singleton()->pause_mission();
            else
                gcs().send_text(MAV_SEVERITY_INFO, "Info: Impossible to pause flight - copter is disarmed");
            break;
        }
        case KOS_Command::Command_ResumeFlight: {
            if (AP_Arming::get_singleton()->is_armed())
                gcs().send_text(MAV_SEVERITY_INFO, "Info: Impossible to resume flight - copter is armed");
            else
                KOS_CommandExecutor::get_singleton()->resume_mission();
            break;
        }
        case KOS_Command::Command_SetMission: {
            uint8_t command_num;
            expected_size = sizeof(uint8_t);
            size = uart_read->read(&command_num, expected_size);
            if (expected_size != size) {
                gcs().send_text(MAV_SEVERITY_INFO, "Info: Autopilot failed to parse command number of sent mission");
                return;
            }
            AP_Mission::Mission_Command commands[command_num];
            for (int i = 0; i < command_num; i++) {
                uint8_t command_type;
                expected_size = sizeof(uint8_t);
                size = uart_read->read(&command_type, expected_size);
                if (size != expected_size) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Info: Autopilot failed to parse command %d type of sent mission", i + 1);
                    return;
                }
                commands[i].index = i;
                switch (command_type) {
                    case 'H': //Home
                        commands[i].id = MAV_CMD_DO_SET_HOME;
                        commands[i].p1 = 1;
                        if (!readLocation(&(commands[i].content.location.lat), &(commands[i].content.location.lng), &(commands[i].content.location.alt))) {
                            gcs().send_text(MAV_SEVERITY_INFO, "Info: Autopilot failed to parse location in 'Home' command %d", i + 1);
                            return;
                        }
                        break;
                    case 'T': //Takeoff
                        commands[i].id = MAV_CMD_NAV_TAKEOFF;
                        if (!readInt(&(commands[i].content.location.alt))) {
                            gcs().send_text(MAV_SEVERITY_INFO, "Info: Autopilot failed to parse altitude in 'Takeoff' command %d", i + 1);
                            return;
                        }
                        break;
                    case 'W': //Waypoint
                        commands[i].id = MAV_CMD_NAV_WAYPOINT;
                        commands[i].p1 = 0;
                        if (!readLocation(&(commands[i].content.location.lat), &(commands[i].content.location.lng), &(commands[i].content.location.alt))) {
                            gcs().send_text(MAV_SEVERITY_INFO, "Info: Autopilot failed to parse location in 'Waypoint' command %d", i + 1);
                            return;
                        }
                        break;
                    case 'L': //Land
                        commands[i].id = MAV_CMD_NAV_LAND;
                        if (!readLocation(&(commands[i].content.location.lat), &(commands[i].content.location.lng), &(commands[i].content.location.alt))) {
                            gcs().send_text(MAV_SEVERITY_INFO, "Info: Autopilot failed to parse location in 'Land' command %d", i + 1);
                            return;
                        }
                        break;
                    case 'S': { //Set Servo
                        commands[i].id = MAV_CMD_DO_SET_SERVO;
                        expected_size = sizeof(uint8_t);
                        size = uart_read->read(&(commands[i].content.servo.channel), expected_size);
                        if (expected_size == size) {
                            expected_size = sizeof(uint16_t);
                            size = uart_read->read((uint8_t*)(&(commands[i].content.servo.pwm)), expected_size);
                        }
                        if (expected_size != size) {
                            gcs().send_text(MAV_SEVERITY_INFO, "Info: Autopilot failed to parse parameters in 'Set servo' command %d", i + 1);
                            return;
                        }
                        break;
                    }
                    case 'D': { //Delay
                        commands[i].id = MAV_CMD_NAV_DELAY;
                        commands[i].content.nav_delay.sec_utc = 0;
                        commands[i].content.nav_delay.min_utc = 0;
                        commands[i].content.nav_delay.hour_utc = 0;
                        expected_size = sizeof(float);
                        size = uart_read->read((uint8_t*)(&(commands[i].content.nav_delay.seconds)), expected_size);
                        if (expected_size != size) {
                            gcs().send_text(MAV_SEVERITY_INFO, "Info: Autopilot failed to parse delay in 'Delay' command %d", i + 1);
                            return;
                        }
                        break;
                    }
                    default:
                        gcs().send_text(MAV_SEVERITY_INFO, "Info: Unknown command type %c in sent mission", command_type);
                        return;
                }
            }
            KOS_CommandExecutor::get_singleton()->set_mission(commands, command_num);
            break;
        }
        case KOS_Command::Command_ChangeWaypoint: {
            int32_t lat, lng, alt;
            bool isRead = readLocation(&lat, &lng, &alt);
            if (KOS_CommandExecutor::get_singleton()->can_execute_command()) {
                gcs().send_text(MAV_SEVERITY_INFO, "Info: Request to change current waypoint");
                if (isRead)
                    KOS_CommandExecutor::get_singleton()->change_waypoint(lat, lng, alt);
                else
                    gcs().send_text(MAV_SEVERITY_INFO, "Info: Autopilot failed to parse sent waypoint");
            }
            else
                gcs().send_text(MAV_SEVERITY_INFO, "Info: Impossible to change waypoint -- no active mission");
            break;
        }
        case KOS_Command::Command_ChangeSpeed: {
            int32_t speed;
            bool isRead = readInt(&speed);
            if (KOS_CommandExecutor::get_singleton()->can_execute_command()) {
                gcs().send_text(MAV_SEVERITY_INFO, "Info: Request to change speed");
                if (isRead)
                    KOS_CommandExecutor::get_singleton()->change_speed(speed);
                else
                    gcs().send_text(MAV_SEVERITY_INFO, "Info: Autopilot failed to parse sent speed");
            }
            else
                gcs().send_text(MAV_SEVERITY_INFO, "Info: Impossible to change speed -- no active mission");
            break;
        }
        case KOS_Command::Command_ChangeAltitude: {
            int32_t alt;
            bool isRead = readInt(&alt);
            if (KOS_CommandExecutor::get_singleton()->can_execute_command()) {
                gcs().send_text(MAV_SEVERITY_INFO, "Info: Request to change altitude");
                if (isRead)
                    KOS_CommandExecutor::get_singleton()->change_altitude(alt);
                else
                    gcs().send_text(MAV_SEVERITY_INFO, "Info: Autopilot failed to parse sent altitude");
            }
            else
                gcs().send_text(MAV_SEVERITY_INFO, "Info: Impossible to change altitude -- no active mission");
            break;
        }
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "KOS Message Error: Unknown command %d is received", (int)message.command);
            break;
    }
}