#pragma once
#ifdef FOR_SITL

int initializeSensorUart(void);

float getSitlAzimuth();
float getSitlTemperature();
void getSitlAcceleration(float& x, float& y, float& z);
void getSitlGyro(float& x, float& y, float& z);

#endif