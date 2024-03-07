#pragma once

#include <stdint.h>

struct Vector3f {
    float X;
    float Y;
    float Z;

    Vector3f() {};

    Vector3f(float x, float y, float z) {
        X = x;
        Y = y;
        Z = z;
    };

    void multiply(float mult) {
        X *= mult;
        Y *= mult;
        Z *= mult;
    };
    void add(Vector3f add) {
        X += add.X;
        Y += add.Y;
        Z += add.Z;
    };
    void subtract(Vector3f sub) {
        X -= sub.X;
        Y -= sub.Y;
        Z -= sub.Z;
    };
};

int startMpu(char* channel);
int gyroscopeCalibration();

int readAcceleration(float &x, float &y, float &z);
int readGyroscope(float &x, float &y, float &z);
int readTemperature(float &temperature);