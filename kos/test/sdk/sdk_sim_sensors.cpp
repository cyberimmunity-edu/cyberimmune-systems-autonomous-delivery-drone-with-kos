#include "sdk_sim_sensors.h"
#ifdef FOR_SITL
#include <sys/socket.h>
#include <arpa/inet.h>
#include <memory.h>
#include <unistd.h>
#include <stdio.h>
#include <thread>
#include <mutex>

#define KOS_SIM_DATA_MESSAGE_HEAD_SIZE 4

int uartSensorsSocket = NULL;
char sitlSensorsIp[] = "172.28.65.87";
uint16_t sitlSensorsPort = 5766;

std::thread simSensorThread;
std::mutex simSensorMutex;

float sitlAzimuth = 0.0f;
float sitlTemperature = 0.0f;
float sitlAccX = 0.0f;
float sitlAccY = 0.0f;
float sitlAccZ = 0.0f;
float sitlGyroX = 0.0f;
float sitlGyroY = 0.0f;
float sitlGyroZ = 0.0f;

static const uint8_t sensorDataMessageHead[KOS_SIM_DATA_MESSAGE_HEAD_SIZE] = { 0x71, 0x11, 0xda, 0x1a };

struct KOSSimSensorDataMessage {
    uint8_t head[KOS_SIM_DATA_MESSAGE_HEAD_SIZE];
    float azimuth;
    float temperature;
    float acceleration[3];
    float gyro[3];
};

void listenSimSensorData() {
    while (1) {
        uint8_t message[sizeof(KOSSimSensorDataMessage)];
        bool restart = false;
        for (int i = 0; i < KOS_SIM_DATA_MESSAGE_HEAD_SIZE; i++) {
            ssize_t readBytes = read(uartSensorsSocket, message + i, 1);
            if (readBytes != 1) {
                fprintf(stderr, "Error: failed to read from socket\n");
                restart = true;
                break;
            }

            if (message[i] != sensorDataMessageHead[i]) {
                fprintf(stderr, "Error: received sensor data has an unknown header\n");
                restart = true;
                break;
            }
        }

        if (!restart) {
            uint32_t expectedSize = sizeof(KOSSimSensorDataMessage) - KOS_SIM_DATA_MESSAGE_HEAD_SIZE;
            ssize_t readBytes = read(uartSensorsSocket, message + KOS_SIM_DATA_MESSAGE_HEAD_SIZE, expectedSize);
            if (readBytes != expectedSize) {
                fprintf(stderr, "Error: failed to read message from autopilot: %d bytes were expected, %d bytes were received\n", expectedSize, readBytes);
                continue;
            }

            KOSSimSensorDataMessage dataMessage;
            memcpy(&dataMessage, message, sizeof(KOSSimSensorDataMessage));

            simSensorMutex.lock();
            sitlAzimuth = dataMessage.azimuth;
            sitlTemperature = dataMessage.temperature;
            sitlAccX = dataMessage.acceleration[0];
            sitlAccY = dataMessage.acceleration[1];
            sitlAccZ = dataMessage.acceleration[2];
            sitlGyroX = dataMessage.gyro[0];
            sitlGyroY = dataMessage.gyro[1];
            sitlGyroZ = dataMessage.gyro[2];
            simSensorMutex.unlock();
            //fprintf(stderr, "Data: %f, %f, %f, %f, %f, %f, %f, %f\n", sitlAzimuth, sitlTemperature, sitlAccX, sitlAccY,
            //    sitlAccZ, sitlGyroX, sitlGyroY, sitlGyroZ);
        }
    }
}

int initializeSensorUart(void) {
    if (uartSensorsSocket)
        return 1;

    struct sockaddr_in sitlAddress = { 0 };

    uartSensorsSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (uartSensorsSocket == -1) {
        fprintf(stderr, "Error: failed to create socket\n");
        return 0;
    }

    sitlAddress.sin_family = AF_INET;
    sitlAddress.sin_addr.s_addr = inet_addr(sitlSensorsIp);
    sitlAddress.sin_port = htons(sitlSensorsPort);

    if (connect(uartSensorsSocket, (struct sockaddr*)&sitlAddress, sizeof(sitlAddress)) != 0) {
        fprintf(stderr, "Error: failed to connect to %s:%d\n", sitlSensorsIp, sitlSensorsPort);
        return 0;
    }

    simSensorThread = std::thread(listenSimSensorData);

    return 1;
}

float getSitlAzimuth() {
    simSensorMutex.lock();
    float a = sitlAzimuth;
    simSensorMutex.unlock();
    return a;
}

float getSitlTemperature() {
    simSensorMutex.lock();
    float t = sitlTemperature;
    simSensorMutex.unlock();
    return t;
}

void getSitlAcceleration(float& x, float& y, float& z) {
    simSensorMutex.lock();
    x = sitlAccX;
    y = sitlAccY;
    z = sitlAccZ;
    simSensorMutex.unlock();
}

void getSitlGyro(float& x, float& y, float& z) {
    simSensorMutex.lock();
    x = sitlGyroX;
    y = sitlGyroY;
    z = sitlGyroZ;
    simSensorMutex.unlock();
}

#endif