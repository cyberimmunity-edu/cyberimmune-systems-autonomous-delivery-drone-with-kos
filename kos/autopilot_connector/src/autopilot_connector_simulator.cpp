#include "../include/autopilot_connector.h"

#include <kos_net.h>

int autopilotSocket = NULL;
uint16_t autopilotPort = 5765;

int initAutopilotConnector() {
    if (!wait_for_network()) {
        fprintf(stderr, "[%s] Error: Connection to network has failed\n", ENTITY_NAME);
        return 0;
    }

    return 1;
}

int initConnection() {
    close(autopilotSocket);
    autopilotSocket = NULL;

    if ((autopilotSocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        fprintf(stderr, "[%s] Warning: Failed to create socket\n", ENTITY_NAME);
        return 0;
    }

    struct sockaddr_in address = { 0 };
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(SIMULATOR_IP);
    address.sin_port = htons(autopilotPort);

    if (connect(autopilotSocket, (struct sockaddr*)&address, sizeof(address)) != 0) {
        fprintf(stderr, "[%s] Warning: Connection to %s:%d has failed\n", ENTITY_NAME, SIMULATOR_IP, autopilotPort);
        return 0;
    }

    return 1;
}

int getAutopilotCommand(uint8_t& command) {
    uint8_t message[sizeof(AutopilotCommandMessage)];
    for (int i = 0; i < AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE; i++) {
        if (read(autopilotSocket, message + i, 1) != 1) {
            fprintf(stderr, "[%s] Warning: Failed to read from socket\n", ENTITY_NAME);
            return 0;
        }

        if (message[i] != AutopilotCommandMessageHead[i]) {
            fprintf(stderr, "[%s] Warning: Received message has an unknown header\n", ENTITY_NAME);
            return 0;
        }
    }

    ssize_t expectedSize = sizeof(AutopilotCommandMessage) - AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE;
    ssize_t readBytes = read(autopilotSocket, message + AUTOPILOT_COMMAND_MESSAGE_HEAD_SIZE, expectedSize);
    if (readBytes != expectedSize) {
        fprintf(stderr, "[%s] Warning: Failed to read message from autopilot: %ld bytes were expected, %ld bytes were received\n", ENTITY_NAME, expectedSize, readBytes);
        return 0;
    }

    command = (uint8_t)(((AutopilotCommandMessage*)message)->command);
    return 1;
}

int sendAutopilotCommand(AutopilotCommand command) {
    AutopilotCommandMessage message = AutopilotCommandMessage(command);

    write(autopilotSocket, &message, sizeof(AutopilotCommandMessage));

    return 1;
}