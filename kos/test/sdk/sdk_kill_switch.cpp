#include "sdk_kill_switch.h"
#include "sdk_gpio.h"

uint8_t ksPinFst = 20;
uint8_t ksPinSnd = 21;

void setKillSwitchPins(uint8_t pin1, uint8_t pin2) {
    ksPinFst = pin1;
    ksPinSnd = pin2;
}

int setKillSwitch(int turnOn) {
    if (turnOn) {
        if (!setGpioPin(ksPinFst, 0))
            return 0;
        return setGpioPin(ksPinSnd, 1);
    }
    else {
        if (!setGpioPin(ksPinFst, 0))
            return 0;
        return setGpioPin(ksPinSnd, 0);
    }
}