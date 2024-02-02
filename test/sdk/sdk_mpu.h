#pragma once
#include <i2c/i2c.h>
#include <stdlib.h>

/*
SCALE
	250 DPS         0x00
	500 DPS         0x08
    1000 DPS        0x10
    2000 DPS        0x18

RANGE
	2G              0x00
	4G              0x08
	8G              0x10
	16G             0x18
*/

struct Vector3f {
    float X;
    float Y;
    float Z;

    Vector3f(float x, float y, float z);
    void multiply(float mult);
    void add(Vector3f add);
    void subtract(Vector3f sub);
};

int startI2CMPU();
int writeRegister(uint8_t reg, uint8_t val);
int readVector(uint8_t reg, Vector3f& val);

void setMPUAddress(uint8_t address);
void setMPUFrequency(uint32_t frequency);

int initializeMPU(uint8_t scale = 0x18, uint8_t range = 0x18);
int calibrateGyro();
Vector3f getAcceleration();
Vector3f getGyro();
float getTemperature();