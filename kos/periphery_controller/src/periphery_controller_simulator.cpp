#include "../include/periphery_controller.h"

#include <stdio.h>

bool light = false;
bool motorKillSwitch = false;
bool cargoKillSwitch = false;

int initPeripheryController() {
    return 1;
}

int initGpioPins() {
    return 1;
}

int setLight(bool turnOn) {
    light = turnOn;
    fprintf(stderr, "[%s] Info: Light is turned %s\n", ENTITY_NAME, light ? "on" : "off");
    return 1;
}

int setMotorKillSwitch(bool permitted) {
    motorKillSwitch = permitted;
    fprintf(stderr, "[%s] Info: Kill-switch now %s to use motors\n", ENTITY_NAME, motorKillSwitch ? "permits" : "forbids");
    return 1;
}

int setCargoKillSwitch(bool permitted) {
    cargoKillSwitch = permitted;
    fprintf(stderr, "[%s] Info: Kill-switch now %s to use cargo servomotor\n", ENTITY_NAME, cargoKillSwitch ? "permits" : "forbids");
    return 1;
}