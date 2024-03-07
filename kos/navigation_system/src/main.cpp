#include "../include/navigation_system.h"
#include "../include/navigation_system_interface.h"
#include "../../ipc_messages/include/transport_interface.h"
#include "../../ipc_messages/include/initialization_interface.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/NavigationSystem.edl.h>

#define RETRY_DELAY_SEC 1

int main(void) {
    if (!initNavigationSystem())
        return EXIT_FAILURE;

    while (!initSensors()) {
        fprintf(stderr, "[%s] Info: Trying again to connect in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }

    while (!calibrateCompass()) {
        fprintf(stderr, "[%s] Info: Trying again to calibrate compass in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    fprintf(stderr, "[%s] Info: Compass is calibrated\n", ENTITY_NAME);

    while (!calibrateMpu()) {
        fprintf(stderr, "[%s] Info: Trying again to calibrate MPU gyroscope in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    fprintf(stderr, "[%s] Info: MPU gyroscope is calibrated\n", ENTITY_NAME);

    fprintf(stderr, "[%s] Info: Initialization is finished\n", ENTITY_NAME);

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
                fprintf(stderr, "[%s] Warning: Failed to send a reply to IPC-message\n", ENTITY_NAME);
        }
        else
            fprintf(stderr, "[%s] Warning: Failed to receive IPC-message\n", ENTITY_NAME);
        /*if (listenSensorSocket(simAzimuth, simTemperature, simAcceleration, simGyroscope))
            fprintf(stderr, "Data: %f, %f, %f, %f, %f, %f, %f, %f\n", simAzimuth, simTemperature,
                simAcceleration[0], simAcceleration[1], simAcceleration[2],
                simGyroscope[0], simGyroscope[1], simGyroscope[2]);
        else
            while (!connectToSensorSocket()) {
                fprintf(stderr, "[%s] Info: Trying again to connect in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
                sleep(RETRY_DELAY_SEC);
            }*/
    };

    return EXIT_SUCCESS;
}