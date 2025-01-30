/**
 * \file
 * \~English \brief Implementation of the security module AutopilotConnector component main loop.
 * \~Russian \brief Реализация основного цикла компонента AutopilotConnector модуля безопасности.
 */

#include "../include/autopilot_connector.h"
#include "../include/autopilot_connector_interface.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"

#include <stdio.h>
#include <stdlib.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/AutopilotConnector.edl.h>

/**
 * \~English \brief AutopilotConnector component main program entry point.
 * \details First, waits for the Logger component to initialize. After that,
 * communication with the autopilot is prepared and established. Then the program
 * enters a loop, where it receives IPC messages from other security module components,
 * performs the requested actions and sends IPC responses.
 * \~Russian \brief Точка входа в основную программу компонента AutopilotConnector.
 * \details Сначала производится ожидание инициализации компонента Logger. После этого подготавливается
 * и устанавливается связь с автопилотом. Далее программа входит в цикл, в котором получает IPC-сообщения
 * от других компонентов модуля безопасности, исполняет запрашиваемые действия и отправляет IPC-ответы.
 */
int main(void) {
    while (!waitForInit("logger_connection", "Logger")) {
        logEntry("Failed to receive initialization notification from Logger. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    if (!initAutopilotConnector())
        return EXIT_FAILURE;

    while (!initConnection()) {
        logEntry("Trying again to connect in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    logEntry("Initialization is finished", ENTITY_NAME, LogLevel::LOG_INFO);

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
                logEntry("Failed to send a reply to IPC-message", ENTITY_NAME, LogLevel::LOG_WARNING);
        }
        else
            logEntry("Failed to receive IPC-message", ENTITY_NAME, LogLevel::LOG_WARNING);
    };

    return EXIT_SUCCESS;
}