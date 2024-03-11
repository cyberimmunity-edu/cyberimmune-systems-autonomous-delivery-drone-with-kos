#pragma once

int initNavigationSystem();
int initSensors();
int calibrateCompass();
int calibrateMpu();

int getAzimuth(float &azimuth);
int getAcceleration(float &x, float &y, float &z);
int getGyroscope(float &x, float &y, float &z);
int getTemperature(float &temperature);