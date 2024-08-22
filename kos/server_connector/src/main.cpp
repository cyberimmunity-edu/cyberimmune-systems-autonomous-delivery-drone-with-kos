/**
 * \file
 * \~English \brief Implementation of the security module ServerConnector component main loop.
 * \~Russian \brief Реализация основного цикла компонента ServerConnector модуля безопасности.
 */

#include "../include/server_connector.h"
#include "../include/server_connector_interface.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/ServerConnector.edl.h>

/**
 * \~English \brief ServerConnector component main program entry point.
 * \details First, waits for the Logger component to initialize. After that, the connection to the ATM server
 * is established and the unique drone ID is determined. Then the program enters a loop,
 * where it receives IPC messages from other security module components,
 * \~Russian \brief Точка входа в основную программу компонента ServerConnector.
 * \details Сначала производится ожидание инициализации компонента Logger. После этого инициализируется
 * подключение к серверу ОРВД и определяется уникальный ID дрона. Далее программа входит в цикл, в котором
 * получает IPC-сообщения от других компонентов модуля безопасности, исполняет запрашиваемые действияи отправляет IPC-ответы.
 */
int main(void) {
    while (!waitForInit("logger_connection", "Logger")) {
        logEntry("Failed to receive initialization notification from Logger. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    if (!initServerConnector())
        return EXIT_FAILURE;

    logEntry("Initialization is finished", ENTITY_NAME, LogLevel::LOG_INFO);

    NkKosTransport transport;
    initReceiverInterface("server_connector_connection", transport);

    ServerConnector_entity entity;
    ServerConnector_entity_init(&entity, CreateInitializationImpl(), CreateServerConnectorInterfaceImpl());

    ServerConnector_entity_req req;
    ServerConnector_entity_res res;
    char reqBuffer[ServerConnector_entity_req_arena_size];
    char resBuffer[ServerConnector_entity_res_arena_size];
    struct nk_arena reqArena = NK_ARENA_INITIALIZER(reqBuffer, reqBuffer + sizeof(reqBuffer));
    struct nk_arena resArena = NK_ARENA_INITIALIZER(resBuffer, resBuffer + sizeof(resBuffer));

    while (true) {
        nk_req_reset(&req);
        nk_arena_reset(&reqArena);
        nk_arena_reset(&resArena);
        if (nk_transport_recv(&transport.base, &req.base_, &reqArena) == NK_EOK) {
            ServerConnector_entity_dispatch(&entity, &req.base_, &reqArena, &res.base_, &resArena);
            if (nk_transport_reply(&transport.base, &res.base_, &resArena) != NK_EOK)
                logEntry("Failed to send a reply to IPC-message", ENTITY_NAME, LogLevel::LOG_WARNING);
        }
        else
            logEntry("Failed to receive IPC-message", ENTITY_NAME, LogLevel::LOG_WARNING);
    };

    return EXIT_SUCCESS;
}