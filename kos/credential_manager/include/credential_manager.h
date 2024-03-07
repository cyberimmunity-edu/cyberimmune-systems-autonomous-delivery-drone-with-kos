#pragma once

#include <stdint.h>

int generateRsaKey();

int getRsaKey(char* e, char* n);
int setRsaKey(char* key);
int signMessage(char* message, char* sign);
int checkSignature(char* message, uint8_t &correct);