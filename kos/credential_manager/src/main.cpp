/**
 * \file
 * \~English \brief Implementation of the security module CredentialManager component main loop.
 * \~Russian \brief Реализация основного цикла компонента CredentialManager модуля безопасности.
 */

#include "../include/credential_manager.h"
#include "../include/credential_manager_interface.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/CredentialManager.edl.h>

/**
 * \~English \brief CredentialManager component main program entry point.
 * \details First, waits for the Logger component to initialize. After that, the saved RSA key
 * of the security module is loaded (if there is no saved key, a new one is generated).
 * Then, through the ServerConnector component, the public parts of the key are exchanged with the ATM server.
 * The public part of the ATM server key is saved. Then the program enters a loop, where it receives
 * IPC messages from other security module components, performs the requested actions and sends IPC responses.
 * \~Russian \brief Точка входа в основную программу компонента CredentialManager.
 * \details Сначала производится ожидание инициализации компонента Logger. После этого загружается сохраненный
 * RSA-ключ модуля безопасности (при отсутствии сохраненного ключа генерируется новый). Затем через компонент
 * ServerConnector происходит обмен открытыми частями ключа с сервером ОРВД. Открытая часть ключа сервера ОРВД
 * сохраняется. Далее программа входит в цикл, в котором получает IPC-сообщения от других компонентов модуля
 * безопасности, исполняет запрашиваемые действия и отправляет IPC-ответы.
 */
int main(void) {
    while (!waitForInit("logger_connection", "Logger")) {
        logEntry("Failed to receive initialization notification from Logger. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    if (!getRsaKey())
        return EXIT_FAILURE;

    if (!shareRsaKey())
        return EXIT_FAILURE;

    logEntry("Initialization is finished", ENTITY_NAME, LogLevel::LOG_INFO);

    NkKosTransport transport;
    initReceiverInterface("credential_manager_connection", transport);

    CredentialManager_entity entity;
    CredentialManager_entity_init(&entity, CreateInitializationImpl(), CreateCredentialManagerInterfaceImpl());

    CredentialManager_entity_req req;
    CredentialManager_entity_res res;
    char reqBuffer[CredentialManager_entity_req_arena_size];
    char resBuffer[CredentialManager_entity_res_arena_size];
    struct nk_arena reqArena = NK_ARENA_INITIALIZER(reqBuffer, reqBuffer + sizeof(reqBuffer));
    struct nk_arena resArena = NK_ARENA_INITIALIZER(resBuffer, resBuffer + sizeof(resBuffer));

    while (true) {
        nk_req_reset(&req);
        nk_arena_reset(&reqArena);
        nk_arena_reset(&resArena);
        if (nk_transport_recv(&transport.base, &req.base_, &reqArena) == NK_EOK) {
            CredentialManager_entity_dispatch(&entity, &req.base_, &reqArena, &res.base_, &resArena);
            if (nk_transport_reply(&transport.base, &res.base_, &resArena) != NK_EOK)
                logEntry("Failed to send a reply to IPC-message", ENTITY_NAME, LogLevel::LOG_WARNING);
        }
        else
            logEntry("Failed to receive IPC-message", ENTITY_NAME, LogLevel::LOG_WARNING);
    };

    return EXIT_SUCCESS;
}