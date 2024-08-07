#include "../include/navigation_system.h"
#include "../include/navigation_system_interface.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <thread>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/NavigationSystem.edl.h>

std::thread sensorThread;
std::thread senderThread;

int main(void) {
    while (!waitForInit("logger_connection", "Logger")) {
        logEntry("Failed to receive initialization notification from Logger. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    if (!initNavigationSystem())
        return EXIT_FAILURE;

    while (!initSensors()) {
        logEntry("Trying again to init sensors in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    sensorThread = std::thread(getSensors);

    while (!hasPosition()) {
        logEntry("Inconsistent coordinates are received. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    senderThread = std::thread(sendCoords);

    logEntry("Initialization is finished", ENTITY_NAME, LogLevel::LOG_INFO);

    NkKosTransport transport;
    initReceiverInterface("navigation_system_connection", transport);

    NavigationSystem_entity entity;
    NavigationSystem_entity_init(&entity, CreateInitializationImpl(), CreateNavigationSystemInterfaceImpl());

    NavigationSystem_entity_req req;
    NavigationSystem_entity_res res;
    char reqBuffer[NavigationSystem_entity_req_arena_size];
    char resBuffer[NavigationSystem_entity_res_arena_size];
    struct nk_arena reqArena = NK_ARENA_INITIALIZER(reqBuffer, reqBuffer + sizeof(reqBuffer));
    struct nk_arena resArena = NK_ARENA_INITIALIZER(resBuffer, resBuffer + sizeof(resBuffer));

    while (true) {
        nk_req_reset(&req);
        nk_arena_reset(&reqArena);
        nk_arena_reset(&resArena);
        if (nk_transport_recv(&transport.base, &req.base_, &reqArena) == NK_EOK) {
            NavigationSystem_entity_dispatch(&entity, &req.base_, &reqArena, &res.base_, &resArena);
            if (nk_transport_reply(&transport.base, &res.base_, &resArena) != NK_EOK)
                logEntry("Failed to send a reply to IPC-message", ENTITY_NAME, LogLevel::LOG_WARNING);
        }
        else
            logEntry("Failed to receive IPC-message", ENTITY_NAME, LogLevel::LOG_WARNING);
    };

    return EXIT_SUCCESS;
}