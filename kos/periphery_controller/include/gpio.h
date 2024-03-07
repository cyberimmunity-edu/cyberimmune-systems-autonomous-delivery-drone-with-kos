#pragma once

#include <stdint.h>

int startGpio(char* channel);
int initPin(uint8_t pin);
int setPin(uint8_t pin, bool mode);