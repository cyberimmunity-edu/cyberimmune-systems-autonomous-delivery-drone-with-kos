#include "sdk_light.h"
#include "sdk_gpio.h"
#include <unistd.h>
#include <thread>
#include <mutex>

uint8_t lightPin = 8;

int blinkStop = false;
uint32_t blinkPeriod = 1;
std::thread blinkThread;
std::mutex blinkMutex;

void blinkLight(void) {
    bool mode = true;
    while (true) {
        blinkMutex.lock();
        if (blinkStop) {
            blinkMutex.unlock();
            break;
        }
        blinkMutex.unlock();
        if (!setLight(mode)) {
            fprintf(stderr, "Error: light blink process failed\n");
            return;
        }
        mode = !mode;
        sleep(blinkPeriod);
    }
}

void setLightPin(uint8_t pin) {
    lightPin = pin;
}

void setBlinkPeriod(uint32_t ms) {
    blinkPeriod = ms;
}

int setLight(int turnOn) {
    return setGpioPin(lightPin, turnOn);
}

void startBlinking(void) {
    if (!blinkStop)
        return;
#ifndef FOR_SITL
    setLight(0);
    blinkStop = false;
    blinkThread = std::thread(blinkLight);
#endif
}

void stopBlinking(void) {
    if (blinkStop)
        return;
#ifndef FOR_SITL
    blinkMutex.lock();
    blinkStop = true;
    blinkMutex.unlock();
    blinkThread.join();
#endif
}