#include "../include/autopilot_connector.h"
#include "../include/autopilot_connector_interface.h"
#include "../../ipc_messages/include/transport_interface.h"
#include "../../ipc_messages/include/initialization_interface.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/AutopilotConnector.edl.h>

#define RETRY_DELAY_SEC 1

int main(void) {
    if (!initAutopilotConnector())
        return EXIT_FAILURE;

    while (!initConnection()) {
        fprintf(stderr, "[%s] Info: Trying again to connect in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }

    fprintf(stderr, "[%s] Info: Initialization is finished\n", ENTITY_NAME);

    NkKosTransport transport;
    initReceiverInterface("autopilot_connector_connection", transport);

    AutopilotConnector_entity entity;
    AutopilotConnector_entity_init(&entity, CreateInitializationImpl(), CreateAutopilotConnectorInterfaceImpl());

    AutopilotConnector_entity_req req;
    AutopilotConnector_entity_res res;
    char reqBuffer[AutopilotConnector_entity_req_arena_size];
    char resBuffer[AutopilotConnector_entity_res_arena_size];
    struct nk_arena reqArena = NK_ARENA_INITIALIZER(reqBuffer, reqBuffer + sizeof(reqBuffer));
    struct nk_arena resArena = NK_ARENA_INITIALIZER(resBuffer, resBuffer + sizeof(resBuffer));

    while (true) {
        nk_req_reset(&req);
        nk_arena_reset(&reqArena);
        nk_arena_reset(&resArena);
        if (nk_transport_recv(&transport.base, &req.base_, &reqArena) == NK_EOK) {
            AutopilotConnector_entity_dispatch(&entity, &req.base_, &reqArena, &res.base_, &resArena);
            if (nk_transport_reply(&transport.base, &res.base_, &resArena) != NK_EOK)
                fprintf(stderr, "[%s] Warning: Failed to send a reply to IPC-message\n", ENTITY_NAME);
        }
        else
            fprintf(stderr, "[%s] Warning: Failed to receive IPC-message\n", ENTITY_NAME);
    };

    return EXIT_SUCCESS;
}