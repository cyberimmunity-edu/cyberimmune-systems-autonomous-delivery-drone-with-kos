#include "sdk_gpio.h"
#include "sdk_firmware.h"

#ifndef FOR_SITL
#include <rtl/retcode_hr.h>
#include <gpio/gpio.h>
#endif

#include <stdio.h>

#define MAX_PIN_NUM 28

#ifndef FOR_SITL
GpioHandle gpioH = NULL;
#endif

int8_t pinInit[MAX_PIN_NUM] = {0};

#ifndef FOR_SITL
int openGpioChannel() {
    char* channel = getChannelName(firmwareChannel::GPIO);
    Retcode rc = GpioOpenPort(channel, &gpioH);
    if (rcOk != rc) {
        fprintf(stderr, "Error: failed top open GPIO port ("RETCODE_HR_FMT")\n", RETCODE_HR_PARAMS(rc));
        return 0;
    }
    return 1;
}

int openPin(uint8_t pin) {
    if (pin >= MAX_PIN_NUM) {
        fprintf(stderr, "Error: cannot open pin %u -- it is larger than max available\n", pin);
        return 0;
    }

    if (pinInit[pin])
        return 1;

    Retcode rc = GpioSetMode(gpioH, pin, GPIO_DIR_OUT);
    if (rcOk != rc) {
        fprintf(stderr, "Error: failed to set GPIO pin %u mode ("RETCODE_HR_FMT")\n", pin, RETCODE_HR_PARAMS(rc));
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
        fprintf(stderr, "Error: failed to set GPIO pin %d to %d ("RETCODE_HR_FMT")\n", pin, turnOn, RETCODE_HR_PARAMS(rc));
        return 0;
    }
#endif

    return 1;
}