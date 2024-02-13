#include "sdk_gpio.h"
#include "sdk_firmware.h"

#ifndef FOR_SITL
#include <gpio/gpio.h>
#endif

#include <stdio.h>

#define MAX_PIN_NUM 40

#ifndef FOR_SITL
GpioHandle gpioH = NULL;
#endif

int8_t pinInit[MAX_PIN_NUM] = {0};

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
    return 1;
}

int openPin(uint8_t pin) {
    if (pin >= MAX_PIN_NUM) {
        fprintf(stderr, "Cannot open pin %u. Its number should be lesser than %d\n", pin, MAX_PIN_NUM);
        return 0;
    }

    if (pinInit[pin])
        return 1;

    Retcode rc = GpioSetMode(gpioH, pin, GPIO_DIR_OUT);
    if (rcOk != rc) {
        fprintf(stderr, "GpioSetMode for pin %u failed, error code: %d\n", pin, RC_GET_CODE(rc));
        return 0;
    }
    pinInit[pin] = 1;

    return 1;
}

int initializeGpio(uint8_t pin) {
    if ((gpioH == NULL) && !openGpioChannel())
        return 0;

    if (!openPin(pin))
        return 0;

    return 1;
}
#endif

int setGpioPin(uint8_t pin, int turnOn) {
#ifndef FOR_SITL
    if (!initializeGpio(pin))
        return 0;

    Retcode rc = GpioOut(gpioH, pin, turnOn ? 1 : 0);
    if (rcOk != rc)
    {
        fprintf(stderr, "GpioOut %d for pin %u failed, error code: %d\n", turnOn, pin, RC_GET_CODE(rc));
        return 0;
    }
#endif

    return 1;
}