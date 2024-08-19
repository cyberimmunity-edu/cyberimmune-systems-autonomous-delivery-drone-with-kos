#include "../include/navigation_system.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <mutex>

std::mutex sensorMutex;

bool hasAlt = false;
bool hasCoords = false;

float sensorDop = 0.0f;
int32_t sensorSats = 0;
int32_t sensorLatitude = 0;
int32_t sensorLongitude = 0;
int32_t sensorAltitude = 0;

bool hasPosition() {
    return (hasAlt && hasCoords);
}

void sendCoords() {
    char boardId[32] = {0};
    while (!getBoardId(boardId)) {
        logEntry("Failed to get board ID from ServerConnector. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    char signature[257] = {0};
    char request[1024] = {0};
    char response[1024] = {0};

    float dop;
    int32_t prevLat, prevLng, lat, lng, alt, azimuth, sats;
    while (!getCoords(prevLat, prevLng, alt)) {
        logEntry("Failed to get coords from Navigation System. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    while (true) {
        if (!getCoords(lat, lng, alt))
            logEntry("Failed to get GPS coords. Trying again in 500ms", ENTITY_NAME, LogLevel::LOG_WARNING);
        else {
            if (!getGpsInfo(dop, sats))
                logEntry("Failed to get GPS's sats and dop. Trying again in 500ms", ENTITY_NAME, LogLevel::LOG_WARNING);
            else {
                azimuth = round(atan2(lng - prevLng, lat - prevLat) * 1800000000 / M_PI);
                prevLat = lat;
                prevLng = lng;
                snprintf(request, 1024, "/api/telemetry?%s&lat=%d&lon=%d&alt=%d&azimuth=%d&dop=%f&sats=%d", boardId, lat, lng, alt, azimuth, dop, sats);
                if (!signMessage(request, signature))
                    logEntry("Failed to sign 'coordinate' message at Credential Manager. Trying again in 500ms", ENTITY_NAME, LogLevel::LOG_WARNING);
                else {
                    snprintf(request, 1024, "%s&sig=0x%s", request, signature);
                    if (!sendRequest(request, response))
                        logEntry("Failed to send 'coordinate' request through Server Connector. Trying again in 500ms", ENTITY_NAME, LogLevel::LOG_WARNING);
                }
            }
        }
        usleep(500000);
    }
}

void setGpsInfo(float dop, int32_t sats) {
    sensorMutex.lock();
    sensorDop = dop;
    sensorSats = sats;
    sensorMutex.unlock();
}

int getGpsInfo(float& dop, int32_t& sats) {
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

int getCoords(int32_t &latitude, int32_t &longitude, int32_t &altitude) {
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