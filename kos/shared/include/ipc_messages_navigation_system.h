#pragma once

#include <stdint.h>

int getCoords(int32_t &latitude, int32_t &longitude, int32_t &altitude);
int getGpsInfo(float& dop, int32_t &sats);