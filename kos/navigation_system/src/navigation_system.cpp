/**
 * \file
 * \~English
 * \brief Implementation of methods for drone position update and transmit.
 * \details The file contains implementation of methods, that update, store and
 * transmit current drone position to the ATM server.
 *
 * \~Russian
 * \brief Реализация методов для обновления и передачи данных о местоположении дрона.
 * \details В файле реализованы методы, обеспечивающие обновление, хранение и
 * передачу данных о текущем местоположении дрона на сервер ОРВД.
 */

#include "../include/navigation_system.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <mutex>

/** \cond */
std::mutex sensorMutex;

bool hasAlt = false;
bool hasCoords = false;

float sensorSpeed = 0.0f;
float sensorDop = 0.0f;
int32_t sensorSats = 0;
int32_t sensorLatitude = 0;
int32_t sensorLongitude = 0;
int32_t sensorAltitude = 0;
/** \endcond */

bool hasPosition() {
    return (hasAlt && hasCoords);
}

void sendCoords() {
    char boardId[32] = {0};
    while (!getBoardId(boardId)) {
        logEntry("Failed to get board ID from ServerConnector. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    char topic[] = "api/telemetry";
    char publication[1024] = {0};

    float dop, speed;
    int32_t prevLat, prevLng, lat, lng, alt, azimuth, sats;
    while (!getPosition(prevLat, prevLng, alt)) {
        logEntry("Failed to get coords from Navigation System. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    while (true) {
        if (!getPosition(lat, lng, alt))
            logEntry("Failed to get GPS coords. Trying again in 500ms", ENTITY_NAME, LogLevel::LOG_WARNING);
        else if (!getInfo(dop, sats))
            logEntry("Failed to get GPS's sats and dop. Trying again in 500ms", ENTITY_NAME, LogLevel::LOG_WARNING);
        else if (!getSpeed(speed))
            logEntry("Failed to get GPS's speed. Trying again in 500ms", ENTITY_NAME, LogLevel::LOG_WARNING);
        else {
            azimuth = round(atan2(lng - prevLng, lat - prevLat) * 1800000000 / M_PI);
            prevLat = lat;
            prevLng = lng;
            snprintf(publication, 1024, "id=%s&lat=%d&lon=%d&alt=%d&azimuth=%d&dop=%f&sats=%d&speed=%f", boardId, lat, lng, alt, azimuth, dop, sats, speed);
            if (!publishMessage(topic, publication))
                logEntry("Failed to publish telemetry message. Trying again in 500ms", ENTITY_NAME, LogLevel::LOG_WARNING);
        }
        usleep(500000);
    }
}

void setInfo(float dop, int32_t sats) {
    sensorMutex.lock();
    sensorDop = dop;
    sensorSats = sats;
    sensorMutex.unlock();
}

int getInfo(float& dop, int32_t& sats) {
    if (hasPosition()) {
        sensorMutex.lock();
        dop = sensorDop;
        sats = sensorSats;
        sensorMutex.unlock();
        return 1;
    }
    else {
        dop = 0.0f;
        sats = 0;
        return 0;
    }
}

void setAltitude(int32_t altitude) {
    sensorMutex.lock();
    sensorAltitude = altitude;
    sensorMutex.unlock();
    if (!hasAlt && (altitude != 0)) {
        hasAlt = true;
        if (hasPosition())
            logEntry("Consistent coordinates are received", ENTITY_NAME, LogLevel::LOG_INFO);
    }
}

void setCoords(int32_t latitude, int32_t longitude) {
    sensorMutex.lock();
    sensorLatitude = latitude;
    sensorLongitude = longitude;
    sensorMutex.unlock();
    if (!hasCoords && (latitude != 0) && (longitude != 0)) {
        hasCoords = true;
        if (hasPosition())
            logEntry("Consistent coordinates are received", ENTITY_NAME, LogLevel::LOG_INFO);
    }
}

int getPosition(int32_t &latitude, int32_t &longitude, int32_t &altitude) {
    if (hasPosition()) {
        sensorMutex.lock();
        latitude = sensorLatitude;
        longitude = sensorLongitude;
        altitude = sensorAltitude;
        sensorMutex.unlock();
        return 1;
    }
    else {
        latitude = 0;
        longitude = 0;
        altitude = 0;
        return 0;
    }
}

void setSpeed(float speed) {
    sensorMutex.lock();
    sensorSpeed = speed;
    sensorMutex.unlock();
}

int getSpeed(float &speed) {
    if (hasPosition()) {
        sensorMutex.lock();
        speed = sensorSpeed;
        sensorMutex.unlock();
        return 1;
    }
    else {
        speed = 0.0f;
        return 0;
    }
}