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

int getRsaKey(char* e, char* n);
int setServerRsaKey(char* key);
int signMessage(char* message, char* signature);
int checkSignature(char* message);

int getCoords(int32_t &latitude, int32_t &longitude, int32_t &altitude);

int setBuzzer(uint8_t enable);
int setMotorKillSwitch(uint8_t enable);
int setCargoLock(uint8_t enable);

int sendRequest(char* query, char* response);