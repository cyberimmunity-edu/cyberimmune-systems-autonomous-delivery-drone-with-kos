#include "../include/navigation_system.h"
#include "../include/sim_sensor_data_message.h"

#include <kos_net.h>

#include <thread>
#include <mutex>
#include <math.h>

int simSensorSocket = NULL;
uint16_t simSensorPort = 5766;

std::thread sensorThread;
std::mutex sensorMutex;

float simAzimuth = NAN;
float simTemperature = NAN;
float simAcceleration[3] = { NAN, NAN, NAN };
float simGyroscope[3] = { NAN, NAN, NAN };

void getSensors() {
    while (true) {
        bool restart = false;
        uint8_t message[sizeof(SimSensorDataMessage)];
        for (int i = 0; i < SIM_SENSOR_DATA_MESSAGE_HEAD_SIZE; i++) {
            if (read(simSensorSocket, message + i, 1) != 1) {
                fprintf(stderr, "[%s] Warning: Failed to read from socket\n", ENTITY_NAME);
                restart = true;
                break;
            }

            if (message[i] != SimSensorDataMessageHead[i]) {
                fprintf(stderr, "[%s] Warning: Received message has an unknown header\n", ENTITY_NAME);
                restart = true;
                break;
            }
        }

        if (!restart) {
            ssize_t expectedSize = sizeof(SimSensorDataMessage) - SIM_SENSOR_DATA_MESSAGE_HEAD_SIZE;
            ssize_t readBytes = read(simSensorSocket, message + SIM_SENSOR_DATA_MESSAGE_HEAD_SIZE, expectedSize);
            if (readBytes == expectedSize) {
                SimSensorDataMessage *data = (SimSensorDataMessage*)message;
                sensorMutex.lock();
                simAzimuth = data->azimuth;
                simTemperature = data->azimuth;
                for (int i = 0; i < 3; i++) {
                    simAcceleration[i] = data->acceleration[i];
                    simGyroscope[i] = data->gyroscope[i];
                }
                sensorMutex.unlock();
            }
            else
                fprintf(stderr, "[%s] Warning: Failed to read message from autopilot: %ld bytes were expected, %ld bytes were received\n", ENTITY_NAME, expectedSize, readBytes);
        }
    }
}

int initNavigationSystem() {
    if (!wait_for_network()) {
        fprintf(stderr, "[%s] Error: Connection to network has failed\n", ENTITY_NAME);
        return 0;
    }

    return 1;
}

int initSensors() {
    simSensorSocket = NULL;

    if ((simSensorSocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        fprintf(stderr, "[%s] Warning: Failed to create socket\n", ENTITY_NAME);
        return 0;
    }

    struct sockaddr_in address = { 0 };
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(SIMULATOR_IP);
    address.sin_port = htons(simSensorPort);

    if (connect(simSensorSocket, (struct sockaddr*)&address, sizeof(address)) != 0) {
        fprintf(stderr, "[%s] Warning: Connection to %s:%d has failed\n", ENTITY_NAME, SIMULATOR_IP, simSensorPort);
        return 0;
    }

    sensorThread = std::thread(getSensors);

    return 1;
}

int calibrateCompass() {
    return 1;
}

int calibrateMpu() {
    return 1;
}

int getAzimuth(float &azimuth) {
    sensorMutex.lock();
    azimuth = simAzimuth;
    sensorMutex.unlock();

    return 1;
}

int getAcceleration(float &x, float &y, float &z) {
    sensorMutex.lock();
    x = simAcceleration[0];
    y = simAcceleration[1];
    z = simAcceleration[2];
    sensorMutex.unlock();

    return 1;
}

int getGyroscope(float &x, float &y, float &z) {
    sensorMutex.lock();
    x = simGyroscope[0];
    y = simGyroscope[1];
    z = simGyroscope[2];
    sensorMutex.unlock();

    return 1;
}

int getTemperature(float &temperature) {
    sensorMutex.lock();
    temperature = simTemperature;
    sensorMutex.unlock();

    return 1;
}