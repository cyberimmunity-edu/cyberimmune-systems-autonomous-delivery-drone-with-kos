#include "sdk_gpio.h"
#include <stdio.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <bsp/bsp.h>

#define GPIO_MODULE "gpio0"
#define GPIO_CONFIG "rpi4_bcm2711.default"
#define LIGHT_PIN 8
#define GPIO_LOW_LEVEL 0
#define GPIO_HIGH_LEVEL 1
#define BLINK_PERIOD_S 1

GpioHandle gpioH = GPIO_INVALID_HANDLE;
std::thread blinkThread;
std::mutex blinkMutex;
bool blinkStop;

int startGPIO(void) {
    Retcode rc = BspSetConfig(GPIO_MODULE, GPIO_CONFIG);
    if (rcOk != rc) {
        fprintf(stderr, "Failed to set mux configuration for %s module, error code: %d\n", GPIO_MODULE, RC_GET_CODE(rc));
        return 0;
    }
    rc = GpioInit();
    if (rcOk != rc) {
        fprintf(stderr, "GpioInit failed, error code: %d\n", RC_GET_CODE(rc));
        return 0;
    }
    rc = GpioOpenPort(GPIO_MODULE, &gpioH);
    if (rcOk != rc) {
        fprintf(stderr, "GpioOpenPort port %s failed, error code: %d\n", GPIO_MODULE, RC_GET_CODE(rc));
        return 0;
    }
    else if (gpioH == GPIO_INVALID_HANDLE) {
        fprintf(stderr, "GPIO module %s handle is invalid\n", GPIO_MODULE);
        return 0;
    }
    rc = GpioSetMode(gpioH, LIGHT_PIN, GPIO_DIR_OUT);
    if (rcOk != rc)
    {
        fprintf(stderr, "GpioSetMode for module %s pin %u failed, error code: %d\n", GPIO_MODULE, LIGHT_PIN, RC_GET_CODE(rc));
        return 0;
    }
    return 1;
}

int setLight(int turnOn) {
    Retcode rc = GpioOut(gpioH, LIGHT_PIN, turnOn ? GPIO_HIGH_LEVEL : GPIO_LOW_LEVEL);
    if (rcOk != rc)
    {
        fprintf(stderr, "GpioOut %d for module %s pin %u failed, error code: %d\n", turnOn, GPIO_MODULE, LIGHT_PIN, RC_GET_CODE(rc));
        return 0;
    }
    return 1;
}

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
        sleep(BLINK_PERIOD_S);
    }
}

void startBlinking(void) {
    blinkStop = false;
    blinkThread = std::thread(blinkLight);
}

void stopBlinking(void) {
    blinkMutex.lock();
    blinkStop = true;
    blinkMutex.unlock();
    blinkThread.join();
}