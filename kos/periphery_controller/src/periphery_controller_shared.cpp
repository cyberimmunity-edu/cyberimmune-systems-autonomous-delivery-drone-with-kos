#include "../include/periphery_controller.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <thread>

std::thread buzzerThread;
bool buzzerEnabled = false;
int buzzTime = 2;

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

int enableBuzzer() {
    if (buzzerEnabled)
        return 0;
    if (buzzerThread.joinable())
        buzzerThread.join();
    buzzerThread = std::thread(buzz);
    return 1;
}

void checkKillSwitchPermission() {
    char signature[257] = {0};
    char request[1024] = {0};
    char response[1024] = {0};

    snprintf(request, 1024, "/api/kill_switch?%s", BOARD_ID);
    while (!signMessage(request, signature)) {
        fprintf(stderr, "[%s] Warning: Failed to sign 'kill switch permission' message at Credential Manager. Trying again in 1s\n", ENTITY_NAME);
        sleep(1);
    }
    snprintf(request, 1024, "%s&sig=0x%s", request, signature);

    while (true) {
        if (isKillSwitchEnabled()) {
            if (!sendRequest(request, response))
                fprintf(stderr, "[%s] Warning: Failed to send 'kill switch permission' request through Server Connector. Trying again in 500ms\n", ENTITY_NAME);
            else if (!checkSignature(response))
                fprintf(stderr, "[%s] Warning: Failed to check signature of 'kill switch permission' response received through Server Connector. Trying again in 500ms\n", ENTITY_NAME);
            else if (strstr(response, "$KillSwitch: 0#") != NULL) {
                while (!setKillSwitch(false)) {
                    fprintf(stderr, "[%s] Warning: Failed to forbid motor usage. Trying again in 1s\n", ENTITY_NAME);
                    sleep(1);
                }
            }
        }
        usleep(500000);
    }
}