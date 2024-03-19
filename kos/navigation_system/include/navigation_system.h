#pragma once

#include <stdint.h>

int initNavigationSystem();
int initSensors();

int getCoords(int32_t &latitude, int32_t &longitude, int32_t &altitude);