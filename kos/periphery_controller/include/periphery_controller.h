#pragma once

#include "../../shared/include/light_mode.h"

int initPeripheryController();
int initGpioPins();

int setLight(bool turnOn);
int setLightMode(LightMode mode);
int setMotorKillSwitch(bool permitted);
int setCargoKillSwitch(bool permitted);