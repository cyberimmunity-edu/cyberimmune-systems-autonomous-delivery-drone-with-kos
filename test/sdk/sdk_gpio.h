#pragma once
#include <gpio/gpio.h>

extern GpioHandle gpioH;

int startGPIO(void);
int setLight(int turnOn);
void blinkLight(void);
void startBlinking(void);
void stopBlinking(void);