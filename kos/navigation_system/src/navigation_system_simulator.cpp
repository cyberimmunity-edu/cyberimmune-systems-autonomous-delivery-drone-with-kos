#include "../include/navigation_system.h"
#include "../include/sim_sensor_data_message.h"

#include <kos_net.h>

#include <math.h>

int simSensorSocket = NULL;
uint16_t simSensorPort = 5766;

void getSensors() {
    bool restart;
    uint8_t message[sizeof(SimSensorDataMessage)];

    while (true) {
        restart = false;
        for (int i = 0; i < SIM_SENSOR_DATA_MESSAGE_HEAD_SIZE; i++) {
            if (read(simSensorSocket, message + i, 1) != 1) {
                logEntry("Failed to read from socket", ENTITY_NAME, LogLevel::LOG_WARNING);
                restart = true;
                break;
            }

            if (message[i] != SimSensorDataMessageHead[i]) {
                logEntry("Received message has an unknown header", ENTITY_NAME, LogLevel::LOG_WARNING);
                restart = true;
                break;
            }
        }

        if (!restart) {
            ssize_t expectedSize = sizeof(SimSensorDataMessage) - SIM_SENSOR_DATA_MESSAGE_HEAD_SIZE;
            ssize_t readBytes = read(simSensorSocket, message + SIM_SENSOR_DATA_MESSAGE_HEAD_SIZE, expectedSize);
            if (readBytes == expectedSize) {
                SimSensorDataMessage *data = (SimSensorDataMessage*)message;
                setCoords(data->latitude, data->longitude);
                setAltitude(data->altitude);
            }
            else {
                char logBuffer[256];
                snprintf(logBuffer, 256, "Failed to read message from autopilot: %ld bytes were expected, %ld bytes were received", expectedSize, readBytes);
                logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            }
        }
    }
}

int initNavigationSystem() {
    if (!wait_for_network()) {
        logEntry("Connection to network has failed", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    return 1;
}

int initSensors() {
    simSensorSocket = NULL;

    if ((simSensorSocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        logEntry("Failed to create socket", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    struct sockaddr_in address = { 0 };
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(SIMULATOR_IP);
    address.sin_port = htons(simSensorPort);

    if (connect(simSensorSocket, (struct sockaddr*)&address, sizeof(address)) != 0) {
        char logBuffer[256];
        snprintf(logBuffer, 256, "Connection to %s:%d has failed", SIMULATOR_IP, simSensorPort);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    setGpsInfo(1.2f, 10);

    return 1;
}