#include "../include/periphery_controller.h"

#include <stdio.h>
#include <unistd.h>
#include <thread>
#include <mutex>

#define BLINK_PERIOD_SEC 2

LightMode currentMode = (LightMode)(-1);
bool blinkStop = false;
std::thread blinkThread;
std::mutex blinkMutex;

void blinkLight() {
    bool mode = true;
    while (true) {
        blinkMutex.lock();
        if (blinkStop) {
            blinkMutex.unlock();
            break;
        }
        blinkMutex.unlock();
        if (!setLight(mode)) {
            fprintf(stderr, "[%s] Warning: Light blink process failed\n", ENTITY_NAME);
            return;
        }
        mode = !mode;
        sleep(BLINK_PERIOD_SEC);
    }
}

int setLightMode(LightMode mode) {
    if (currentMode == mode)
        return 1;

    if (currentMode == LightMode::Blinking) {
        blinkMutex.lock();
        blinkStop = true;
        blinkMutex.unlock();
        blinkThread.join();
    }

    currentMode = mode;
    switch (currentMode) {
    case LightMode::Off:
        return setLight(false);
    case LightMode::Blinking:
        if (!setLight(false))
            return 0;
        blinkStop = false;
        blinkThread = std::thread(blinkLight);
        return 1;
    case LightMode::On:
        return setLight(true);
    default:
        fprintf(stderr, "[%s] Warning: Cannot set an unknown light mode %d\n", ENTITY_NAME, currentMode);
        return 0;
    }
}