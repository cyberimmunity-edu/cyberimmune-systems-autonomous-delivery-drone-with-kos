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
    char signature[257] = {0};
    char request[1024] = {0};
    char response[1024] = {0};

    float dop;
    int32_t prevLat, prevLng, lat, lng, alt, azimuth, sats;
    while (!getCoords(prevLat, prevLng, alt)) {
        fprintf(stderr, "[%s] Warning: Failed to get coords from Navigation System. Trying again in 1s\n", ENTITY_NAME);
        sleep(1);
    }

    while (true) {
        if (!getCoords(lat, lng, alt))
            fprintf(stderr, "[%s] Warning: Failed to get GPS coords. Trying again in 500ms\n", ENTITY_NAME);
        else {
            if (!getGpsInfo(dop, sats))
                fprintf(stderr, "[%s] Warning: Failed to get GPS's sats and dop. Trying again in 500ms\n", ENTITY_NAME);
            else {
                azimuth = round(atan2(lng - prevLng, lat - prevLat) * 1800000000 / M_PI);
                prevLat = lat;
                prevLng = lng;
                snprintf(request, 1024, "/api/telemetry?%s&lat=%d&lon=%d&alt=%d&azimuth=%d&dop=%f&sats=%d", BOARD_ID, lat, lng, alt, azimuth, dop, sats);
                if (!signMessage(request, signature))
                    fprintf(stderr, "[%s] Warning: Failed to sign 'coordinate' message at Credential Manager. Trying again in 500ms\n", ENTITY_NAME);
                else {
                    snprintf(request, 1024, "%s&sig=0x%s", request, signature);
                    if (!sendRequest(request, response))
                        fprintf(stderr, "[%s] Warning: Failed to send 'coordinate' request through Server Connector. Trying again in 500ms\n", ENTITY_NAME);
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
            fprintf(stderr, "[%s] Info: Consistent coordinates are received\n", ENTITY_NAME);
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
            fprintf(stderr, "[%s] Info: Consistent coordinates are received\n", ENTITY_NAME);
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