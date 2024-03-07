#pragma once

#include <stdlib.h>

struct CompassReading {
    int X;
    int Y;
    int Z;

    CompassReading() {}

    CompassReading(uint8_t v1, uint8_t v2, uint8_t v3, uint8_t v4, uint8_t v5, uint8_t v6) {
        X = (int)(int16_t)(v1 | (v2 << 8));
	    Y = (int)(int16_t)(v3 | (v4 << 8));
	    Z = (int)(int16_t)(v5 | (v6 << 8));
    };

	void calibrate(float* offsets, float* scales) {
        X = (X - offsets[0]) * scales[0];
        Y = (Y - offsets[1]) * scales[1];
        Z = (Z - offsets[2]) * scales[2];
    }
};

int startCompass(char* channel);
int compassCalibration();

int readAzimuth(float &azimuth);