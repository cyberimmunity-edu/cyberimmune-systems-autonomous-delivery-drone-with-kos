#include "../include/gpio.h"

#include <rtl/retcode_hr.h>
#include <gpio/gpio.h>

#include <stdio.h>

GpioHandle gpioHandler = NULL;

int startGpio(char* channel) {
    Retcode rc = GpioOpenPort(channel, &gpioHandler);
    if (rcOk != rc) {
        fprintf(stderr, "[%s] Warning: Failed top open GPIO %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, channel, RETCODE_HR_PARAMS(rc));
        return 0;
    }

    return 1;
}

int initPin(uint8_t pin) {
    Retcode rc = GpioSetMode(gpioHandler, pin, GPIO_DIR_OUT);
    if (rcOk != rc) {
        fprintf(stderr, "[%s] Warning: Failed to set GPIO pin %u mode ("RETCODE_HR_FMT")\n", ENTITY_NAME, pin, RETCODE_HR_PARAMS(rc));
        return 0;
    }

    return 1;
}

int setPin(uint8_t pin, bool mode) {
    Retcode rc = GpioOut(gpioHandler, pin, mode);
    if (rcOk != rc) {
        fprintf(stderr, "[%s] Warning: Failed to set GPIO pin %d to %d ("RETCODE_HR_FMT")\n", ENTITY_NAME, pin, mode, RETCODE_HR_PARAMS(rc));
        return 0;
    }

    return 1;
}