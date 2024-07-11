#include "../include/periphery_controller.h"
#include "../include/periphery_controller_interface.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <thread>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/PeripheryController.edl.h>

std::thread killSwitchCheckThread;

int main(void) {
    while (!waitForInit("logger_connection", "Logger")) {
        logEntry("Failed to receive initialization notification from Logger. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    if (!initPeripheryController())
        return EXIT_FAILURE;

    while (!initGpioPins()) {
        logEntry("Trying again to initialize GPIO pins in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    while (!setKillSwitch(false)) {
        logEntry("Trying again to turn off kill-switch in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    while (!setBuzzer(false)) {
        logEntry("Trying again to turn off buzzer in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    while (!setCargoLock(true)) {
        logEntry("Trying again to lock cargo in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    killSwitchCheckThread = std::thread(checkKillSwitchPermission);

    logEntry("Initialization is finished", ENTITY_NAME, LogLevel::LOG_INFO);

    NkKosTransport transport;
    initReceiverInterface("periphery_controller_connection", transport);

    PeripheryController_entity entity;
    PeripheryController_entity_init(&entity, CreateInitializationImpl(), CreatePeripheryControllerInterfaceImpl());

    PeripheryController_entity_req req;
    PeripheryController_entity_res res;
    char reqBuffer[PeripheryController_entity_req_arena_size];
    char resBuffer[PeripheryController_entity_res_arena_size];
    struct nk_arena reqArena = NK_ARENA_INITIALIZER(reqBuffer, reqBuffer + sizeof(reqBuffer));
    struct nk_arena resArena = NK_ARENA_INITIALIZER(resBuffer, resBuffer + sizeof(resBuffer));

    while (true) {
        nk_req_reset(&req);
        nk_arena_reset(&reqArena);
        nk_arena_reset(&resArena);
        if (nk_transport_recv(&transport.base, &req.base_, &reqArena) == NK_EOK) {
            PeripheryController_entity_dispatch(&entity, &req.base_, &reqArena, &res.base_, &resArena);
            if (nk_transport_reply(&transport.base, &res.base_, &resArena) != NK_EOK)
                logEntry("Failed to send a reply to IPC-message", ENTITY_NAME, LogLevel::LOG_WARNING);
        }
        else
            logEntry("Failed to receive IPC-message", ENTITY_NAME, LogLevel::LOG_WARNING);
    };

    return EXIT_SUCCESS;
}