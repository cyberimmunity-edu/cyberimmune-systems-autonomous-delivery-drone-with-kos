#pragma once
#include <stdlib.h>

void setLightPin(uint8_t pin);
void setBlinkPeriod(uint32_t ms);

int setLight(int turnOn);
void startBlinking(void);
void stopBlinking(void);