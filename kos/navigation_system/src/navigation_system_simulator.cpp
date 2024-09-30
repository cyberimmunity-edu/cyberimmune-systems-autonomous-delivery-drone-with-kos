/**
 * \file
 * \~English
 * \brief Implementation of methods for obtaining simulator drone position.
 * \details The file contains implementation of methods, that obtain current
 * drone position from a compatible ArduPilot SITL firmware via socket.
 *
 * \~Russian
 * \brief Реализация методов для получения данных о местоположении симулятора дрона.
 * \details В файле реализованы методы, обеспечивающие получение информации о
 * текущем местоположении дрона от совместимой SITL-прошивки ArduPilot через сокет.
 */

#include "../include/navigation_system.h"

#include <kos_net.h>

#include <math.h>

/** \cond */
#define SIM_SENSOR_DATA_MESSAGE_HEAD_SIZE 4

static const uint8_t SimSensorDataMessageHead[SIM_SENSOR_DATA_MESSAGE_HEAD_SIZE] = { 0x71, 0x11, 0xda, 0x1a };
/** \endcond */

/**
 * \~English A structure describing a message used by SITL firmware to
 * transmit current drone position.
 * \~Russian Структура, описывающая сообщение, использующееся SITL-прошивкой
 * для передачи данных о текущем местоположении дрона.
 */
struct SimSensorDataMessage {
    /**
     * \~English Fixed header bytes that indicate the start of a new message.
     * \~Russian Фиксированные заголовочные байты, указывающие на начало нового сообщения.
     */
    uint8_t head[SIM_SENSOR_DATA_MESSAGE_HEAD_SIZE];
    /**
     * \~English Latitude of the current drone position in degrees * 10^7.
     * \~Russian Широта текущего местоположения дрона в градусах * 10^7.
     */
    int32_t latitude;
    /**
     * \~English Longitude of the current drone position in degrees * 10^7.
     * \~Russian Долгота текущего местоположения дрона в градусах * 10^7.
     */
    int32_t longitude;
    /**
     * \~English Absolute altitude of the current drone position in cm.
     * \~Russian Абсолютная высота текущего местоположения дрона в см.
     */
    int32_t altitude;
    /**
     * \~English Сurrent speed of the drone in m/s.
     * \~Russian Текущая скорость дрона в м/с.
     */
    float speed;
};

/** \cond */
int simSensorSocket = NULL;
uint16_t simSensorPort = 5766;
/** \endcond */

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
                setSpeed(data->speed);
            }
            else {
                char logBuffer[256] = {0};
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
        char logBuffer[256] = {0};
        snprintf(logBuffer, 256, "Connection to %s:%d has failed", SIMULATOR_IP, simSensorPort);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    setInfo(1.2f, 10);

    return 1;
}