/**
 * \file
 * \~English \brief Implementation of the security module PeripheryController component main loop.
 * \~Russian \brief Реализация основного цикла компонента PeripheryController модуля безопасности.
 */

#include "../include/periphery_controller.h"
#include "../include/periphery_controller_interface.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/PeripheryController.edl.h>

/**
 * \~English \brief PeripheryController component main program entry point.
 * \details First, waits for the Logger component to initialize. After that,
 * interfaces for peripherals management are prepared. The peripherals are then set to default mode.
 * Then the program enters a loop, where it receives IPC messages from other security module components,
 * performs the requested actions and sends IPC responses. In parallel with the main loop,
 * the other process is executed. If the motors are supplied with the power, than this process constantly
 * asks (2 time per second) the ATM server whether the power supply is allowed. Requests are signed and sent
 * through the CredentialManager and ServerConnector components respectively.
 * \~Russian \brief Точка входа в основную программу компонента PeripheryController.
 * \details Сначала производится ожидание инициализации компонента Logger. После этого подготавливаются интерфейсы
 * для управления периферийными устройствами. Затем периферийные устройства переводятся в режим по умолчанию.
 * Далее программа входит в цикл, в котором получает IPC-сообщения от других компонентов модуля безопасности,
 * исполняет запрашиваемые действия и отправляет IPC-ответы. Параллельно с основным циклом запускается процесс.
 * Этот процесс, в случае если подача питания на двигатели разрешена, регулярно (2 раза в секунду) опрашивает сервер
 * ОРВД, можно ли продолжать подавать питание на двигатели. Подпись и отправка запросов осуществляются через
 * компоненты CredentialManager и ServerConnector соответственно.
 */
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