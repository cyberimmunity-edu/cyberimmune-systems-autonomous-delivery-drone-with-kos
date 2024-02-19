#include "sdk_mpu_6050.h"
#include "sdk_firmware.h"

#ifdef FOR_SITL
#include "sdk_sim_sensors.h"
#else
#include <i2c/i2c.h>
#endif

#include <stdio.h>
#include <unistd.h>
#include <math.h>

#ifndef FOR_SITL
I2cHandle i2cMPUH = NULL;
#endif

int mpuInitialized = false;
uint8_t mpuAddress = 0x68;
uint32_t mpuFrequency = 400000;
uint8_t mpuScale = 0x18;
uint8_t mpuRange = 0x18;
float dpsPerDigit = 1.0f;
float rangePerDigit = 1.0f;
Vector3f gyroDelta = Vector3f(0.0f, 0.0f, 0.0f);

Vector3f::Vector3f(float x, float y, float z) {
    X = x;
    Y = y;
    Z = z;
}

void Vector3f::multiply(float mult) {
    X *= mult;
    Y *= mult;
    Z *= mult;
}

void Vector3f::add(Vector3f add) {
    X += add.X;
    Y += add.Y;
    Z += add.Z;
}

void Vector3f::subtract(Vector3f sub) {
    X -= sub.X;
    Y -= sub.Y;
    Z -= sub.Z;
}

#ifndef FOR_SITL
int openMpuChannel() {
    char* channel = getChannelName(firmwareChannel::MPU);
    Retcode rc = I2cOpenChannel(channel, &i2cMPUH);
    if (rc != rcOk)
    {
        fprintf(stderr, "Error: failed to open MPU channel '%s'\n", channel);
        return 0;
    }
    return 1;
}

int writeRegister(uint8_t reg, uint8_t val) {
    I2cMsg messages[1];
    uint8_t buf[2] = { reg, val };

    messages[0].addr = mpuAddress;
    messages[0].flags = 0;
    messages[0].buf = buf;
    messages[0].len = 2;

    Retcode rc = I2cXfer(i2cMPUH, mpuFrequency, messages, 1);
    if (rc != rcOk) {
        fprintf(stderr, "Erorr: failed to write to MPU\n");
        return 0;
    }

    return 1;
}

int startMpu() {
    if (!writeRegister(0x6B, 0x01)) {
        fprintf(stderr, "Error: failed to set MPU config\n");
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
        fprintf(stderr, "Error: could not initialize MPU with scale %d", mpuScale);
        return 0;
    }
    if (!writeRegister(0x1B, mpuScale)) {
        fprintf(stderr, "Error: failed to write gyro config to MPU\n");
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
        fprintf(stderr, "Error: could not initialize MPU with range %d", mpuRange);
	    return 0;
    }
    if (!writeRegister(0x1C, mpuRange)) {
        fprintf(stderr, "Error: failed to write accelerometer config to MPU\n");
        return 0;
    }

    mpuInitialized = true;

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

    I2cError rc = I2cXfer(i2cMPUH, mpuFrequency, messages, 2);
    if (rc != rcOk) {
        fprintf(stderr, "Error: failed to read from MPU\n");
        return 0;
    }

    int16_t x = (readBuffer[0] << 8) | readBuffer[1];
    int16_t y = (readBuffer[2] << 8) | readBuffer[3];
    int16_t z = (readBuffer[4] << 8) | readBuffer[5];
    val = Vector3f(x, y, z);

    return 1;
}

int initializeMpu() {
    if ((i2cMPUH == NULL) && !openMpuChannel())
        return 0;

    if (!mpuInitialized && !startMpu())
        return 0;

    return 1;
}
#endif

void setMpuAddress(uint8_t address) {
    mpuAddress = address;
}

void setMpuFrequency(uint32_t frequency) {
    mpuFrequency = frequency;
}

void setMpuScale(uint8_t scale) {
    mpuScale = scale;
}

void setMpuRange(uint8_t range) {
    mpuRange = range;
}

int calibrateGyro() {
#ifndef FOR_SITL
    if (!initializeMpu())
        return 0;

    int num = 2000;
    Vector3f sum(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < num; i++) {
	    Vector3f gyro(NAN, NAN, NAN);
        if(!readVector(0x43, gyro)) {
            fprintf(stderr, "Error: failed to read data from gyro\n");
            return 0;
        }
        sum.add(gyro);
	    usleep(5000);
    }

    float numR = 1.0f / num;
    sum.multiply(numR);
    gyroDelta = sum;

    fprintf(stderr, "Info: gyro is calibrated\n");
    fprintf(stderr, "Info: gyro delta: %f, %f, %f\n", gyroDelta.X, gyroDelta.Y, gyroDelta.Z);
#endif

    return 1;
}

Vector3f getAcceleration() {
#ifdef FOR_SITL
    Vector3f acc = Vector3f(0.0f, 0.0f, 0.0f);
    getSitlAcceleration(acc.X, acc.Y, acc.Z);
    return acc;
#else
    if (!initializeMpu())
        return Vector3f(NAN, NAN, NAN);

    Vector3f acc(NAN, NAN, NAN);
    if (!readVector(0x3B, acc)) {
        fprintf(stderr, "Error: failed to read data from accelerometer\n");
        return acc;
    }

    acc.multiply(rangePerDigit * 9.80665f);

    return acc;
#endif
}

Vector3f getGyro() {
#ifdef FOR_SITL
    Vector3f gyro = Vector3f(0.0f, 0.0f, 0.0f);
    getSitlGyro(gyro.X, gyro.Y, gyro.Z);
    return gyro;
#else
    if (!initializeMpu())
        return Vector3f(NAN, NAN, NAN);

    Vector3f gyro(NAN, NAN, NAN);
    if(!readVector(0x43, gyro)) {
        fprintf(stderr, "Error: failed to read data from gyro\n");
        return gyro;
    }

    gyro.subtract(gyroDelta);
    gyro.multiply(dpsPerDigit);

    return gyro;
#endif
}

float getTemperature() {
#ifdef FOR_SITL
    return getSitlTemperature();
#else
    if (!initializeMpu())
        return NAN;

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

    I2cError rc = I2cXfer(i2cMPUH, mpuFrequency, messages, 2);
    if (rc != rcOk) {
        fprintf(stderr, "Erorr: failed to read from MPU\n");
        return NAN;
    }

    int16_t temp = (readBuffer[0] << 8) | readBuffer[1];
    return (temp / 340.0f + 36.5f);
#endif
}