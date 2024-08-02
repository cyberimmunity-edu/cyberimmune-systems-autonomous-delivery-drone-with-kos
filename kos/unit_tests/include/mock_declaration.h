#include <stdlib.h>

char* getMockLog();

//credential_manager_online.cpp
uint8_t hexCharToInt(char c);
void stringToBytes(char* source, uint32_t sourceSize, uint8_t* destination);

//credential_manager_shared.cpp
void hashToKey(uint8_t* source, uint32_t sourceSize, uint8_t* destination);
void bytesToString(uint8_t* source, char* destination);