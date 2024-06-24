#pragma once

#include <stdint.h>

int initNavigationSystem();
int initSensors();

bool hasPosition();

void getSensors();
void sendCoords();

void setGpsInfo(float dop, int32_t sats);
int getGpsInfo(float& dop, int32_t &sats);

void setAltitude(int32_t altitude);
void setCoords(int32_t latitude, int32_t longitude);
int getCoords(int32_t &latitude, int32_t &longitude, int32_t &altitude);