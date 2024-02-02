#include "sdk_mpu.h"
#include <bsp/bsp.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#define I2C_MPU_CHANNEL "i2c1"
#define I2C_MPU_CONFIG "rpi4_bcm2711.p2-3"

I2cHandle i2cMPUH = NULL;
uint8_t mpuAddress = 0x68;
uint32_t mpuFrequency = 400000;
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

int startI2CMPU() {
    Retcode rc = BspEnableModule(I2C_MPU_CHANNEL);
    if (rc != rcOk) {
        fprintf(stderr, "Failed to enable '%s' module\n", I2C_MPU_CHANNEL);
        return 0;
    }
    rc = BspSetConfig(I2C_MPU_CHANNEL, I2C_MPU_CONFIG);
    if (rc != rcOk) {
        fprintf(stderr, "Failed to set pmux configuration for '%s' channel\n", I2C_MPU_CHANNEL);
        return 0;
    }
    rc = I2cInit();
    if (rc != rcOk) {
        fprintf(stderr, "Failed to initialize I2C\n");
        return 0;
    }
    rc = I2cOpenChannel(I2C_MPU_CHANNEL, &i2cMPUH);
    if (rc != rcOk)
    {
        fprintf(stderr, "Failed to open '%s' channel\n", I2C_MPU_CHANNEL);
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
        fprintf(stderr, "Failed to write to MPU\n");
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

    I2cError rc = I2cXfer(i2cMPUH, mpuFrequency, messages, 2);
    if (rc != I2C_EOK) {
        fprintf(stderr, "Failed to read from MPU\n");
        return 0;
    }

    int16_t x = (readBuffer[0] << 8) | readBuffer[1];
    int16_t y = (readBuffer[2] << 8) | readBuffer[3];
    int16_t z = (readBuffer[4] << 8) | readBuffer[5];
    val = Vector3f(x, y, z);

    return 1;
}

void setMPUAddress(uint8_t address) {
    mpuAddress = address;
}

void setMPUFrequency(uint32_t frequency) {
    mpuFrequency = frequency;
}

int initializeMPU(uint8_t scale, uint8_t range) {
    if (!writeRegister(0x6B, 0x01)) {
        fprintf(stderr, "Failed to set MPU config\n");
        return 0;
    }

    switch (scale) {
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
        fprintf(stderr, "Could not initialize MPU with scale %d", scale);
        return 0;
    }
    if (!writeRegister(0x1B, scale)) {
        fprintf(stderr, "Failed to write gyro config\n");
        return 0;
    }

    switch (range) {
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
        fprintf(stderr, "Could not initialize MPU with range %d", range);
	    return 0;
    }
    if (!writeRegister(0x1C, range)) {
        fprintf(stderr, "Failed to write accelerometer config\n");
        return 0;
    }

    return 1;
}

int calibrateGyro() {
    int num = 2000;
    Vector3f sum(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < num; i++) {
	    Vector3f gyro(NAN, NAN, NAN);
        if(!readVector(0x43, gyro)) {
            fprintf(stderr, "Failed to read gyro during calibration\n");
            return 0;
        }
        sum.add(gyro);
	    usleep(5000);
    }

    float numR = 1.0f / num;
    sum.multiply(numR);
    gyroDelta = sum;

    fprintf(stderr, "Gyro is calibrated\n");
    fprintf(stderr, "Gyro delta: %f, %f, %f\n", gyroDelta.X, gyroDelta.Y, gyroDelta.Z);

    return 1;
}

Vector3f getAcceleration() {
    Vector3f acc(NAN, NAN, NAN);
    if (!readVector(0x3B, acc)) {
        fprintf(stderr, "Failed to read accelerometer\n");
        return acc;
    }

    acc.multiply(rangePerDigit * 9.80665f);

    return acc;
}

Vector3f getGyro() {
    Vector3f gyro(NAN, NAN, NAN);
    if(!readVector(0x43, gyro)) {
        fprintf(stderr, "Failed to read gyro\n");
        return gyro;
    }

    gyro.subtract(gyroDelta);
    gyro.multiply(dpsPerDigit);

    return gyro;
}

float getTemperature() {
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
    if (rc != I2C_EOK) {
        fprintf(stderr, "Failed to read from MPU\n");
        return NAN;
    }

    int16_t temp = (readBuffer[0] << 8) | readBuffer[1];
    return (temp / 340.0f + 36.5f);
}