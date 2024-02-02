#pragma once
#include <i2c/i2c.h>
#include <stdlib.h>

/*
MODE CONTROL (MODE)
	Standby			0x00
	Continuous		0x01

OUTPUT DATA RATE (ODR)
	10Hz        	0x00
	50Hz        	0x04
	100Hz       	0x08
	200Hz       	0x0C

FULL SCALE (RNG)
	2G          	0x00
	8G          	0x10

OVER SAMPLE RATIO (OSR)
	512         	0x00
	256         	0x40
	128         	0x80
	64          	0xC0
*/

struct CompassReading {
    int X;
    int Y;
    int Z;

    CompassReading() {};
    CompassReading(uint8_t v1, uint8_t v2, uint8_t v3, uint8_t v4, uint8_t v5, uint8_t v6);

	void calibrate(float* offsets, float* scales);
};

int startI2CCompass();
int readCompass(CompassReading& res);

void setCompassAddress(uint8_t address);
void setCompassFrequency(uint32_t frequency);
void setMagneticDeclination(int degrees, int minutes);

int initializeCompass(uint8_t mode = 0x01, uint8_t odr = 0x0C, uint8_t rng = 0x10, uint8_t osr = 0x00);
int calibrateCompass();
float getAzimuth();