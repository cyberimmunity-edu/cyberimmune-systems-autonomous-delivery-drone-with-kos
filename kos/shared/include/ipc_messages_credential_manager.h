#pragma once

#include <stdint.h>

int signMessage(char* message, char* signature);
int checkSignature(char* message, uint8_t &authenticity);