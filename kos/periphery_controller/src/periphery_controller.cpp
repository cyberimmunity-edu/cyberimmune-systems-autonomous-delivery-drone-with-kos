/**
 * \file
 * \~English
 * \brief Implementation of methods for peripherals control.
 * \details The file contains implementation of methods of general peripherals control logic,
 * regardless of peripheral implementation.
 *
 * \~Russian
 * \brief Реализация методов для управления периферией.
 * \details В файле реализованы методы, реализующие общую логику управления периферийными
 * устройствами, вне знависимости от реализации периферии.
 */

#include "../include/periphery_controller.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <thread>

/** \cond */
std::thread buzzerThread;
bool buzzerEnabled = false;
int buzzTime = 2;
/** \endcond */

/**
 * \~English The procedure turns on the buzzer, waits for 2 seconds, and then turns it off.
 * It is assumed that this procedure is performed in a parallel thread.
 * \~Russian Процедура включает зуммер, ожидает 2 секунды, после чего отключает его.
 * Предполагается, что данная процедура выполняется в параллельной нити.
 */
void buzz() {
    buzzerEnabled = true;
    setBuzzer(true);
    clock_t startTime = clock();
    while (true) {
        clock_t time = clock() - startTime;
        if ((time / CLOCKS_PER_SEC) >= buzzTime)
            break;
    }
    setBuzzer(false);
    buzzerEnabled = false;
}

int startBuzzer() {
    if (buzzerEnabled)
        return 0;
    if (buzzerThread.joinable())
        buzzerThread.join();
    buzzerThread = std::thread(buzz);
    return 1;
}

void checkKillSwitchPermission() {
    char boardId[32] = {0};
    while (!getBoardId(boardId)) {
        logEntry("Failed to get board ID from ServerConnector. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    char signature[257] = {0};
    char request[512] = {0};
    char response[1024] = {0};

    snprintf(request, 512, "/api/kill_switch?id=%s", boardId);
    while (!signMessage(request, signature, 257)) {
        logEntry("Failed to sign 'kill switch permission' message at Credential Manager. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }
    snprintf(request, 512, "%s&sig=0x%s", request, signature);

    uint8_t authenticity;
    while (true) {
        if (isKillSwitchEnabled()) {
            authenticity = 0;
            if (!sendRequest(request, response, 1024))
                logEntry("Failed to send 'kill switch permission' request through Server Connector. Trying again in 500ms", ENTITY_NAME, LogLevel::LOG_WARNING);
            else if (!checkSignature(response, authenticity) || !authenticity)
                logEntry("Failed to check signature of 'kill switch permission' response received through Server Connector. Trying again in 500ms", ENTITY_NAME, LogLevel::LOG_WARNING);
            else if (strstr(response, "$KillSwitch: 0#") != NULL) {
                if (!startBuzzer())
                    logEntry("Failed to enable buzzer", ENTITY_NAME, LogLevel::LOG_WARNING);
                while (!setKillSwitch(false)) {
                    logEntry("Failed to forbid motor usage. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
                    sleep(1);
                }
            }
        }
        usleep(500000);
    }
}