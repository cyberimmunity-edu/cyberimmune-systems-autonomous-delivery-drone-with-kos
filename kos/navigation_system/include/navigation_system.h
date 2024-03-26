#pragma once

#include <stdint.h>

int initNavigationSystem();
int initSensors();

void getSensors();
void sendCoords();

void setCoords(int32_t latitude, int32_t longitude, int32_t altitude);
int getCoords(int32_t &latitude, int32_t &longitude, int32_t &altitude);