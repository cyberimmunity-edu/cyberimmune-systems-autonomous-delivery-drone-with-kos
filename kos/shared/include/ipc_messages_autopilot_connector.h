#pragma once

#include <stdint.h>

int waitForArmRequest();
int permitArm();
int forbidArm();
int pauseFlight();
int resumeFlight();
int changeSpeed(int32_t speed);
int changeAltitude(int32_t altitude);
int changeWaypoint(int32_t latitude, int32_t longitude, int32_t altitude);