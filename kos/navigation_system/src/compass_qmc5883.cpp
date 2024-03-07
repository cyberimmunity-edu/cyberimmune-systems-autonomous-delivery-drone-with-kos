#include "../include/compass_qmc5883.h"

#include <rtl/retcode_hr.h>
#include <i2c/i2c.h>

#include <math.h>
#include <stdio.h>
#include <unistd.h>

I2cHandle compassHandler = NULL;

uint8_t compassAddress = 0x0D;
uint32_t compassFrequency = 100000;
uint8_t compassMode = 0x01;
uint8_t compassOdr = 0x0C;
uint8_t compassRng = 0x10;
uint8_t compassOsr = 0x00;

float magneticDeclination = 0.0f;
float compassCalibrationOffsets[3] = { 0.0f, 0.0f, 0.0f };
float compassCalibrationScales[3] = { 1.0f, 1.0f, 1.0f };

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

    I2cError rc = I2cXfer(compassHandler, compassFrequency, messages, 2);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to read from compass\n", ENTITY_NAME);
        return 0;
    }

    res = CompassReading(readBuffer[0], readBuffer[1], readBuffer[2], readBuffer[3], readBuffer[4], readBuffer[5]);

    res.calibrate(compassCalibrationOffsets, compassCalibrationScales);

    return 1;
}

int startCompass(char* channel) {
    Retcode rc = I2cOpenChannel(channel, &compassHandler);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Warning: Failed to open I2C %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, channel, RETCODE_HR_PARAMS(rc));
        return 0;
    }

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

    rc = I2cXfer(compassHandler, compassFrequency, messages, 2);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Warning: Failed to start compass\n", ENTITY_NAME);
        return 0;
    }

    return 1;
}

int compassCalibration() {
    for (int i = 0; i < 3; i++) {
        compassCalibrationOffsets[i] = 0.0f;
        compassCalibrationScales[i] = 1.0f;
    }

    CompassReading r;
    if (!readCompass(r)) {
        fprintf(stderr, "[%s] Warning: Failed to calibrate compass\n", ENTITY_NAME);
        return 0;
    }

    int num = 2000;
    int32_t calibrationData[3][2] = { { r.X, r.X }, { r.Y, r.Y }, { r.Z, r.Z } };
    for (int i = 0; i < num; i++) {
        if (!readCompass(r)) {
            fprintf(stderr, "[%s] Warning: Failed to calibrate compass\n", ENTITY_NAME);
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

    fprintf(stderr, "[%s] Info: Compass is calibrated\n", ENTITY_NAME);
    fprintf(stderr, "[%s] Info: Calibration offsets: %f, %f, %f\n", ENTITY_NAME, compassCalibrationOffsets[0], compassCalibrationOffsets[1], compassCalibrationOffsets[2]);
    fprintf(stderr, "[%s] Info: Calibration scales: %f, %f, %f\n", ENTITY_NAME, compassCalibrationScales[0], compassCalibrationScales[1], compassCalibrationScales[2]);

    return 1;
}

int readAzimuth(float &azimuth) {
    CompassReading r;
    if (!readCompass(r)) {
        fprintf(stderr, "[%s] Warning: Failed to get an azimuth from compass\n", ENTITY_NAME);
        return 0;
    }
    azimuth = (float)atan2(r.Y, r.X) + magneticDeclination;

	return 1;
}