#include "sdk_gpio.h"
#include "sdk_firmware.h"

#ifndef FOR_SITL
#include <gpio/gpio.h>
#endif

#include <stdio.h>
#include <unistd.h>
#include <thread>
#include <mutex>

#define GPIO_LOW_LEVEL 0
#define GPIO_HIGH_LEVEL 1

#ifndef FOR_SITL
GpioHandle gpioH = NULL;
#endif

uint8_t lightPin = 8;
uint32_t blinkPeriod = 1;
std::thread blinkThread;
std::mutex blinkMutex;
bool blinkStop;

#ifndef FOR_SITL

int openGpioChannel() {
    char* channel = getChannelName(firmwareChannel::GPIO);
    Retcode rc = GpioOpenPort(channel, &gpioH);
    if (rcOk != rc) {
        fprintf(stderr, "GpioOpenPort port %s failed, error code: %d\n", channel, RC_GET_CODE(rc));
        return 0;
    }
    else if (gpioH == GPIO_INVALID_HANDLE) {
        fprintf(stderr, "GPIO module %s handle is invalid\n", channel);
        return 0;
    }
    rc = GpioSetMode(gpioH, lightPin, GPIO_DIR_OUT);
    if (rcOk != rc)
    {
        fprintf(stderr, "GpioSetMode for module %s pin %u failed, error code: %d\n", channel, lightPin, RC_GET_CODE(rc));
        return 0;
    }
    return 1;
}

int initializeGpio() {
    if ((gpioH == NULL) && !openGpioChannel())
        return 0;

    return 1;
}

#endif

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
            fprintf(stderr, "Error: Light blink failed\n");
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
#ifdef FOR_SITL

    return 1;

#else

    if (!initializeGpio())
        return 0;

    Retcode rc = GpioOut(gpioH, lightPin, turnOn ? GPIO_HIGH_LEVEL : GPIO_LOW_LEVEL);
    if (rcOk != rc)
    {
        fprintf(stderr, "GpioOut %d for pin %u failed, error code: %d\n", turnOn, lightPin, RC_GET_CODE(rc));
        return 0;
    }
    return 1;

#endif
}

void startBlinking(void) {
#ifdef FOR_SITL

#else

    if (!initializeGpio())
        return;

    blinkStop = false;
    blinkThread = std::thread(blinkLight);

#endif
}

void stopBlinking(void) {
#ifdef FOR_SITL

#else

    blinkMutex.lock();
    blinkStop = true;
    blinkMutex.unlock();
    blinkThread.join();

#endif
}