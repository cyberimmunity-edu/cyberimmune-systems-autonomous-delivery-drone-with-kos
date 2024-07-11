#include "../include/logger.h"
#include "../include/logger_interface.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_logger.h"

#include <stdio.h>
#include <stdlib.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/Logger.edl.h>

int main(void) {
    if (!createLog())
        return EXIT_FAILURE;

    addLogEntry("[Logger] Initialization is finished", LogLevel::LOG_INFO);

    NkKosTransport transport;
    initReceiverInterface("logger_connection", transport);

    Logger_entity entity;
    Logger_entity_init(&entity, CreateInitializationImpl(), CreateLoggerInterfaceImpl());

    Logger_entity_req req;
    Logger_entity_res res;
    char reqBuffer[Logger_entity_req_arena_size];
    char resBuffer[Logger_entity_res_arena_size];
    struct nk_arena reqArena = NK_ARENA_INITIALIZER(reqBuffer, reqBuffer + sizeof(reqBuffer));
    struct nk_arena resArena = NK_ARENA_INITIALIZER(resBuffer, resBuffer + sizeof(resBuffer));

    while (true) {
        nk_req_reset(&req);
        nk_arena_reset(&reqArena);
        nk_arena_reset(&resArena);
        if (nk_transport_recv(&transport.base, &req.base_, &reqArena) == NK_EOK) {
            Logger_entity_dispatch(&entity, &req.base_, &reqArena, &res.base_, &resArena);
            if (nk_transport_reply(&transport.base, &res.base_, &resArena) != NK_EOK)
                addLogEntry("[Logger] Failed to send a reply to IPC-message", LogLevel::LOG_WARNING);
        }
        else
            addLogEntry("[Logger] Failed to receive IPC-message", LogLevel::LOG_WARNING);
    };

    return EXIT_SUCCESS;
}