#include "KOS_module.h"
#include "Copter.h"

static const uint8_t command_message_head[KOS_COMMAND_MESSAGE_HEAD_SIZE] = { 0x7a, 0xfe, 0xf0, 0x0d };

KOSCommModule::KOSCommandMessage::KOSCommandMessage() {
    for (int i = 0; i < KOS_COMMAND_MESSAGE_HEAD_SIZE; i++)
        head[i] = 0;
    command = KOSCommandType::ERROR;
}

KOSCommModule::KOSCommandMessage::KOSCommandMessage(KOSCommandType com) {
    memcpy(head, command_message_head, KOS_COMMAND_MESSAGE_HEAD_SIZE);
    command = (uint8_t)com;
}

KOSCommModule::KOSCommModule() {
    command_send = new uint8_t[sizeof(KOSCommandMessage)];
    command_receive = new uint8_t[sizeof(KOSCommandMessage)];
    arm_is_allowed = false;
}

KOSCommModule::~KOSCommModule() {
    delete[] command_send;
    delete[] command_receive;
}

bool KOSCommModule::can_arm() {
    return arm_is_allowed;
}

void KOSCommModule::send_arm_request() {
    if (!uart_write) {
        uart_write = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_KOS_Comm, 0);
        if (!uart_write) {
            gcs().send_text(MAV_SEVERITY_INFO, "Error: Write UART is not available");
            return;
        }
        uart_write->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }
    KOSCommandMessage packet = KOSCommandMessage(KOSCommandType::Command_ArmRequest);
    memcpy(command_send, &packet, sizeof(KOSCommandMessage));
    uart_write->write(command_send, sizeof(KOSCommandMessage));
    gcs().send_text(MAV_SEVERITY_INFO, "Arm request to KOS is sent");
}

void KOSCommModule::wait_for_KOS_message() {
    if (!uart_read) {
        uart_read = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_KOS_Comm, 0);
        if (!uart_read)
            return;
        uart_read->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }
    memset(command_receive, 0, KOS_COMMAND_MESSAGE_HEAD_SIZE);
    for (int i = 0; i < KOS_COMMAND_MESSAGE_HEAD_SIZE; i++) {
        ssize_t size = uart_read->read(command_receive + i, 1);
        if (!size)
            return;
        if (command_receive[i] != command_message_head[i]) {
            gcs().send_text(MAV_SEVERITY_INFO, "KOS Message Error: Unknown message head");
            return;
        }
    }
    gcs().send_text(MAV_SEVERITY_INFO, "DEBUG: message header is received");
    ssize_t expected_size = sizeof(KOSCommandMessage) - KOS_COMMAND_MESSAGE_HEAD_SIZE;
    ssize_t size = uart_read->read(command_receive + KOS_COMMAND_MESSAGE_HEAD_SIZE, expected_size);
    if (!size || (size != expected_size)) {
        gcs().send_text(MAV_SEVERITY_INFO, "KOS Command Message Error: Message is shorter than expected (%d vs %d)", size + KOS_COMMAND_MESSAGE_HEAD_SIZE, sizeof(KOSCommandMessage));
        return;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "DEBUG: message body is received");
    KOSCommandMessage command_message;
    memcpy(&command_message, command_receive, sizeof(KOSCommandMessage));
    switch (command_message.command) {
        case KOSCommandType::Command_ArmPermit:
            gcs().send_text(MAV_SEVERITY_INFO, "DEBUG: Arm is permitted");
            arm_is_allowed = true;
            break;
        case KOSCommandType::Command_ArmForbid:
            gcs().send_text(MAV_SEVERITY_INFO, "DEBUG: Arm is forbidden");
            break;
        case KOSCommandType::Command_AbortFlight:
            gcs().send_text(MAV_SEVERITY_INFO, "DEBUG: Request to stop flight is received");
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "KOS Command Message Error: Unknown command %d is received", (int)command_message.command);
            break;
    }
}