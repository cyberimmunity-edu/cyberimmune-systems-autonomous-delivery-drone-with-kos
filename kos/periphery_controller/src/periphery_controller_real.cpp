#include "../include/periphery_controller.h"
#include "../include/gpio.h"
#include "../../ipc_messages/include/transport_interface.h"
#include "../../ipc_messages/include/initialization_interface.h"

#include <coresrv/hal/hal_api.h>
#include <rtl/retcode_hr.h>
#include <gpio/gpio.h>
#include <bsp/bsp.h>

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/PeripheryController.edl.h>

#define NAME_MAX_LENGTH 64
#define RETRY_DELAY_SEC 1

char gpio[] = "gpio0";
char gpioConfigSuffix[] = "default";

uint8_t pinBuzzer = 21;
uint8_t pinKillSwitchFirst = 22;
uint8_t pinKillSwitchSecond = 27;

int initPeripheryController() {
    char boardName[NAME_MAX_LENGTH] = {0};
    if (KnHalGetEnv("board", boardName, sizeof(boardName)) != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to get board name\n", ENTITY_NAME);
        return 0;
    }

    char gpioConfig[NAME_MAX_LENGTH];
    if (snprintf(gpioConfig, NAME_MAX_LENGTH, "%s.%s", boardName, gpioConfigSuffix) < 0) {
        fprintf(stderr, "[%s] Error: Failed to generate GPIO config name\n", ENTITY_NAME);
        return 0;
    }

    Retcode rc = BspInit(NULL);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to initialize BSP ("RETCODE_HR_FMT")\n", ENTITY_NAME, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    rc = BspSetConfig(gpio, gpioConfig);
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to set BSP config for GPIO %s ("RETCODE_HR_FMT")\n", ENTITY_NAME, gpio, RETCODE_HR_PARAMS(rc));
        return 0;
    }
    rc = GpioInit();
    if (rc != rcOk) {
        fprintf(stderr, "[%s] Error: Failed to initialize GPIO ("RETCODE_HR_FMT")\n", ENTITY_NAME, RC_GET_CODE(rc));
        return 0;
    }

    NkKosTransport transportAutopilotConnector, transportNavigationSystem;
    initReceiverInterface("ap_pc_connection", transportAutopilotConnector);
    initReceiverInterface("ns_pc_connection", transportNavigationSystem);

    PeripheryController_entity entity;
    PeripheryController_entity_init(&entity, CreateInitializationImpl(), NULL);
    PeripheryController_entity_req req;
    PeripheryController_entity_res res;

    nk_req_reset(&req);
    if (nk_transport_recv(&transportAutopilotConnector.base, &req.base_, NULL) == NK_EOK) {
        PeripheryController_entity_dispatch(&entity, &req.base_, NULL, &res.base_, NULL);
        if (nk_transport_reply(&transportAutopilotConnector.base, &res.base_, NULL) != NK_EOK)
            fprintf(stderr, "[%s] Warning: Failed to send a reply to IPC-message\n", ENTITY_NAME);
    }
    else
        fprintf(stderr, "[%s] Warning: Failed to receive IPC-message\n", ENTITY_NAME);

    nk_req_reset(&req);
    if (nk_transport_recv(&transportNavigationSystem.base, &req.base_, NULL) == NK_EOK) {
        PeripheryController_entity_dispatch(&entity, &req.base_, NULL, &res.base_, NULL);
        if (nk_transport_reply(&transportNavigationSystem.base, &res.base_, NULL) != NK_EOK)
            fprintf(stderr, "[%s] Warning: Failed to send a reply to IPC-message\n", ENTITY_NAME);
    }
    else
        fprintf(stderr, "[%s] Warning: Failed to receive IPC-message\n", ENTITY_NAME);

    return 1;
}

int initGpioPins() {
    if (!startGpio(gpio))
        return 0;

    uint8_t pins[] = { pinBuzzer, pinKillSwitchFirst, pinKillSwitchSecond };
    for (uint8_t pin : pins)
        if (!initPin(pin))
            return 0;

    return 1;
}

int setBuzzer(bool enable) {
    return setPin(pinBuzzer, enable);
}

int setKillSwitch(bool enable) {
    if (enable) {
        if (!setPin(pinKillSwitchFirst, false))
            return 0;
        return setPin(pinKillSwitchSecond, true);
    }
    else {
        if (!setPin(pinKillSwitchFirst, false))
            return 0;
        return setPin(pinKillSwitchSecond, false);
    }
}

int setCargoLock(bool enable) {
    //Not implemented yet

    return 1;
}