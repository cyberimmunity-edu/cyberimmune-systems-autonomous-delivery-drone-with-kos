#pragma once

#include "../../shared/include/autopilot_command.h"
#include "../../shared/include/light_mode.h"

int getAutopilotCommand(AutopilotCommand &command);
int sendAutopilotCommand(AutopilotCommand command);

int getRsaKey(char* e, char* n);
int setServerRsaKey(char* key);
int signMessage(char* message, char* signature);
int checkSignature(char* message);

int getAzimuth(float &azimuth);
int getAcceleration(float &x, float &y, float &z);
int getGyroscope(float &x, float &y, float &z);
int getTemperature(float &temperature);

int setLightMode(LightMode mode);
int setMotorKillSwitch(uint8_t permit);

int sendRequest(char* query, char* response);