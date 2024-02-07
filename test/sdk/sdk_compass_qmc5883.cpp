#include "sdk_compass_qmc5883.h"
#include "sdk_firmware.h"

#ifndef FOR_SITL
#include <i2c/i2c.h>
#endif

#include <stdio.h>
#include <unistd.h>
#include <math.h>

#ifndef FOR_SITL
I2cHandle i2cCompassH = NULL;
#endif

int compassInitialized = false;
uint8_t compassAddress = 0x0D;
uint32_t compassFrequency = 100000;
float magneticDeclination = 0.0f;
uint8_t compassMode = 0x01;
uint8_t compassOdr = 0x0C;
uint8_t compassRng = 0x10;
uint8_t compassOsr = 0x00;
float compassCalibrationOffsets[3] = { 0.0f, 0.0f, 0.0f };
float compassCalibrationScales[3] = { 1.0f, 1.0f, 1.0f };

CompassReading::CompassReading(uint8_t v1, uint8_t v2, uint8_t v3, uint8_t v4, uint8_t v5, uint8_t v6) {
    X = (int)(int16_t)(v1 | (v2 << 8));
	Y = (int)(int16_t)(v3 | (v4 << 8));
	Z = (int)(int16_t)(v5 | (v6 << 8));
}

void CompassReading::calibrate(float* offsets, float* scales) {
    X = (X - offsets[0]) * scales[0];
    Y = (Y - offsets[1]) * scales[1];
    Z = (Z - offsets[2]) * scales[2];
}

#ifndef FOR_SITL

int openCompassChannel() {
    char* channel = getChannelName(firmwareChannel::COMPASS);
    Retcode rc = I2cOpenChannel(channel, &i2cCompassH);
    if (rc != rcOk)
    {
        fprintf(stderr, "Failed to open '%s' channel\n", channel);
        return 0;
    }
    return 1;
}

int startCompass() {
    uint8_t flags = compassMode | compassOdr | compassRng | compassOsr;

    I2cMsg messages[2];
    uint8_t bufInit[2] = { 0x0B, 0x01 };
    uint8_t bufMode[2] = { 0x09, flags };

    messages[0].addr = compassAddress;
    messages[0].flags = 0;
    messages[0].buf = bufInit;
    messages[0].len = 2;

    messages[1].addr = compassAddress;
    messages[1].flags = 0;
    messages[1].buf = bufMode;
    messages[1].len = 2;

    Retcode rc = I2cXfer(i2cCompassH, compassFrequency, messages, 2);
    if (rc != rcOk) {
        fprintf(stderr, "Failed to start compass\n");
        return 0;
    }

    compassInitialized = true;

    return 1;
}

int readCompass(CompassReading& res) {
    I2cMsg messages[2];
    uint8_t writeBuffer[] = { 0x00 };
    uint8_t readBuffer[6];

    messages[0].addr = compassAddress;
    messages[0].flags = 0;
    messages[0].buf = writeBuffer;
    messages[0].len = 1;

    messages[1].addr = compassAddress;
    messages[1].flags = I2C_FLAG_RD;
    messages[1].buf = readBuffer;
    messages[1].len = 6;

    I2cError rc = I2cXfer(i2cCompassH, compassFrequency, messages, 2);
    if (rc != rcOk) {
        fprintf(stderr, "Failed to read from compass\n");
        return 0;
    }

    res = CompassReading(readBuffer[0], readBuffer[1], readBuffer[2], readBuffer[3], readBuffer[4], readBuffer[5]);

    res.calibrate(compassCalibrationOffsets, compassCalibrationScales);

    return 1;
}

int initializeCompass() {
    if ((i2cCompassH == NULL) && !openCompassChannel())
        return 0;

    if (!compassInitialized && !startCompass())
        return 0;

    return 1;
}

#endif

void setCompassAddress(uint8_t address) {
    compassAddress = address;
}

void setCompassFrequency(uint32_t frequency) {
    compassFrequency = frequency;
}

void setMagneticDeclination(int degrees, int minutes) {
    magneticDeclination = degrees + minutes / 60.0f;
}

void setCompassMode(uint8_t mode) {
    compassMode = mode;
}

void setCompassOutputDataRate(uint8_t odr) {
    compassOdr = odr;
}

void setCompassFullScale(uint8_t rng) {
    compassRng = rng;
}

void setCompassOverSampleRatio(uint8_t osr) {
    compassOsr = osr;
}

int calibrateCompass() {
#ifdef FOR_SITL

    return 1;

#else

    if (!initializeCompass())
        return 0;

    for (int i = 0; i < 3; i++) {
        compassCalibrationOffsets[i] = 0.0f;
        compassCalibrationScales[i] = 1.0f;
    }

    CompassReading r;
    if (!readCompass(r)) {
        fprintf(stderr, "Failed to calibrate compass\n");
        return 0;
    }

    int num = 2000;
    int32_t calibrationData[3][2] = { { r.X, r.X }, { r.Y, r.Y }, { r.Z, r.Z } };
    for (int i = 0; i < num; i++) {
        if (!readCompass(r)) {
            fprintf(stderr, "Failed to calibrate compass\n");
            return 0 ;
        }

		if(r.X < calibrationData[0][0])
			calibrationData[0][0] = r.X;
		if(r.X > calibrationData[0][1])
			calibrationData[0][1] = r.X;
		if(r.Y < calibrationData[1][0])
			calibrationData[1][0] = r.Y;
		if(r.Y > calibrationData[1][1])
			calibrationData[1][1] = r.Y;
		if(r.Z < calibrationData[2][0])
			calibrationData[2][0] = r.Z;
		if(r.Z > calibrationData[2][1])
			calibrationData[2][1] = r.Z;

        usleep(5000);
    }

    float delta = 0;
    for (int i = 0; i < 3; i++) {
        compassCalibrationOffsets[i] = (calibrationData[i][0] + calibrationData[i][1]) / 2.0f;
        compassCalibrationScales[i] = (calibrationData[i][1] - calibrationData[i][0]) / 2.0f;
        delta += compassCalibrationScales[i];
    }
    delta /= 3;
    for (int i = 0; i < 3; i++)
        compassCalibrationScales[i] = delta / compassCalibrationScales[i];

    fprintf(stderr, "Compass is calibrated\n");
    fprintf(stderr, "Calibration offsets: %f, %f, %f\n", compassCalibrationOffsets[0], compassCalibrationOffsets[1], compassCalibrationOffsets[2]);
    fprintf(stderr, "Calibration scales: %f, %f, %f\n", compassCalibrationScales[0], compassCalibrationScales[1], compassCalibrationScales[2]);

    return 1;

#endif
}

float getAzimuth() {
#ifdef FOR_SITL

    return 0.0f;

#else

    if (!initializeCompass())
        return NAN;

    CompassReading r;
    if (!readCompass(r)) {
        fprintf(stderr, "Failed to get an azimuth\n");
        return NAN;
    }
    float azimuth = (float)(atan2(r.Y, r.X) * 180.0 / M_PI);
    azimuth += magneticDeclination;
    while (azimuth < 0)
        azimuth += 360;
    while (azimuth >= 360)
        azimuth -= 360;
	return azimuth;

#endif
}