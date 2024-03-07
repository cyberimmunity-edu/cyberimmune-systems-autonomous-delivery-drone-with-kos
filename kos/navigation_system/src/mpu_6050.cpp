#include "../include/mpu_6050.h"

#include <rtl/retcode_hr.h>
#include <i2c/i2c.h>

#include <stdio.h>
#include <unistd.h>
#include <math.h>

I2cHandle mpuHandler = NULL;

uint8_t mpuAddress = 0x68;
uint32_t mpuFrequency = 400000;
uint8_t mpuScale = 0x18;
uint8_t mpuRange = 0x18;
uint8_t mpuDLPF = 0x00;
float dpsPerDigit = 1.0f;
float rangePerDigit = 1.0f;

Vector3f gyroscopeDelta = Vector3f(0.0f, 0.0f, 0.0f);

int writeRegister(uint8_t reg, uint8_t val) {
    I2cMsg messages[1];
    uint8_t buf[2] = { reg, val };

    messages[0].addr = mpuAddress;
    messages[0].flags = 0;
    messages[0].buf = buf;
    messages[0].len = 2;

    Retcode rc = I2cXfer(mpuHandler, mpuFrequency, messages, 1);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Warning: Failed to write to MPU ("RETCODE_HR_FMT")\n", ENTITY_NAME, RETCODE_HR_PARAMS(rc));
        return 0;
    }

    return 1;
}

int readVector(uint8_t reg, Vector3f& val) {
    I2cMsg messages[2];
    uint8_t writeBuffer[1] = { reg };
    uint8_t readBuffer[6];

    messages[0].addr = mpuAddress;
    messages[0].flags = 0;
    messages[0].buf = writeBuffer;
    messages[0].len = 1;

    messages[1].addr = mpuAddress;
    messages[1].flags = I2C_FLAG_RD;
    messages[1].buf = readBuffer;
    messages[1].len = 6;

    Retcode rc = I2cXfer(mpuHandler, mpuFrequency, messages, 2);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Warning: Failed to read from MPU ("RETCODE_HR_FMT")\n", ENTITY_NAME, RETCODE_HR_PARAMS(rc));
        return 0;
    }

    int16_t x = (readBuffer[0] << 8) | readBuffer[1];
    int16_t y = (readBuffer[2] << 8) | readBuffer[3];
    int16_t z = (readBuffer[4] << 8) | readBuffer[5];
    val = Vector3f(x, y, z);

    return 1;
}

int startMpu(char* channel) {
    Retcode rc = I2cOpenChannel(channel, &mpuHandler);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Warning: Failed to open I2C %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, channel, RETCODE_HR_PARAMS(rc));
        return 0;
    }

    if (!writeRegister(0x6B, 0x01)) {
        fprintf(stderr, "[%s] Warning: Failed to set MPU config\n", ENTITY_NAME);
        return 0;
    }

    if (mpuDLPF >= 0x07) {
        fprintf(stderr, "[%s] Warning: Could not initialize MPU with DLPF %d\n", ENTITY_NAME, mpuDLPF);
        return 0;
    }
    if (!writeRegister(0x1A, mpuDLPF)) {
        fprintf(stderr, "[%s] Warning: Failed to write config to MPU\n", ENTITY_NAME);
        return 0;
    }

    switch (mpuScale) {
    case 0x00:
	    dpsPerDigit /= 131.0f;
	    break;
	case 0x08:
	    dpsPerDigit /= 65.5f;
	    break;
	case 0x10:
	    dpsPerDigit /= 32.8f;
	    break;
	case 0x18:
	    dpsPerDigit /= 16.4f;
	    break;
    default:
        fprintf(stderr, "[%s] Warning: Could not initialize MPU with scale %d\n", ENTITY_NAME, mpuScale);
        return 0;
    }
    if (!writeRegister(0x1B, mpuScale)) {
        fprintf(stderr, "[%s] Warning: Failed to write gyro config to MPU\n", ENTITY_NAME);
        return 0;
    }

    switch (mpuRange) {
	case 0x00:
	    rangePerDigit /= 16384.0f;
	    break;
	case 0x08:
	    rangePerDigit /= 8192.0f;
	    break;
	case 0x10:
	    rangePerDigit /= 4096.0f;
	    break;
	case 0x18:
	    rangePerDigit /= 2048.0f;
	    break;
	default:
        fprintf(stderr, "[%s] Warning: Could not initialize MPU with range %d\n", ENTITY_NAME, mpuRange);
	    return 0;
    }
    if (!writeRegister(0x1C, mpuRange)) {
        fprintf(stderr, "[%s] Warning: Failed to write accelerometer config to MPU\n", ENTITY_NAME);
        return 0;
    }

    return 1;
}

int gyroscopeCalibration() {
    int num = 2000;
    Vector3f sum(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < num; i++) {
	    Vector3f gyro(NAN, NAN, NAN);
        if(!readVector(0x43, gyro)) {
            fprintf(stderr, "[%s] Warning: Failed to read data from gyroscope\n", ENTITY_NAME);
            return 0;
        }
        sum.add(gyro);
	    usleep(5000);
    }

    float numR = 1.0f / num;
    sum.multiply(numR);
    gyroscopeDelta = sum;

    fprintf(stderr, "[%s] Info: Gyroscope is calibrated\n", ENTITY_NAME);
    fprintf(stderr, "[%s] Info: Gyroscope delta: %f, %f, %f\n", ENTITY_NAME, gyroscopeDelta.X, gyroscopeDelta.Y, gyroscopeDelta.Z);

    return 1;
}

int readAcceleration(float &x, float &y, float &z) {
    Vector3f acc;
    if (!readVector(0x3B, acc)) {
        fprintf(stderr, "[%s] Warning: Failed to read data from accelerometer\n", ENTITY_NAME);
        return 0;
    }

    acc.multiply(rangePerDigit * 9.80665f);

    x = acc.X;
    y = acc.Y;
    z = acc.Z;

    return 1;
}

int readGyroscope(float &x, float &y, float &z) {
    Vector3f gyro;
    if(!readVector(0x43, gyro)) {
        fprintf(stderr, "[%s] Warning: Failed to read data from gyroscope\n", ENTITY_NAME);
        return 0;
    }

    gyro.subtract(gyroscopeDelta);
    gyro.multiply(dpsPerDigit);

    return 1;
}

int readTemperature(float &temperature) {
    I2cMsg messages[2];
    uint8_t writeBuffer[1] = { 0x41 };
    uint8_t readBuffer[2];

    messages[0].addr = mpuAddress;
    messages[0].flags = 0;
    messages[0].buf = writeBuffer;
    messages[0].len = 1;

    messages[1].addr = mpuAddress;
    messages[1].flags = I2C_FLAG_RD;
    messages[1].buf = readBuffer;
    messages[1].len = 2;

    Retcode rc = I2cXfer(mpuHandler, mpuFrequency, messages, 2);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Warning: Failed to temperature from MPU\n", ENTITY_NAME);
        return NAN;
    }

    int16_t temp = (readBuffer[0] << 8) | readBuffer[1];
    temperature = temp / 340.0f + 36.5f;

    return 1;
}