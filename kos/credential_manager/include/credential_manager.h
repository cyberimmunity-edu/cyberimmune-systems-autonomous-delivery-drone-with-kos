#pragma once

#include <stdint.h>

int generateRsaKey();
int shareRsaKey();
int setRsaKey(char* key);

int signMessage(char* message, char* sign);
int checkSignature(char* message, uint8_t &correct);