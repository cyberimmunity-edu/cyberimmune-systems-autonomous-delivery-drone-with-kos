#pragma once

#include <stdint.h>

int generateRsaKey();
int loadRsaKey(uint8_t* N, uint8_t* D, char* n, char* e, uint32_t nLen, uint32_t eLen);
int shareRsaKey();

int getRsaKey();
int setRsaKey(char* key);

int signMessage(char* message, char* sign);
int checkSignature(char* message, uint8_t &correct);

char* getKeyN();
char* getKeyE();
char* getKeyD();