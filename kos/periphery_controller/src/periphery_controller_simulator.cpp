#include "../include/periphery_controller.h"
#include "../include/sim_periphery_message.h"

#include <kos_net.h>

#include <stdio.h>

int peripherySocket = NULL;
uint16_t peripheryPort = 5767;

bool killSwitchEnabled;

int initPeripheryController() {
    if (!wait_for_network()) {
        logEntry("Connection to network has failed", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    return 1;
}

int initGpioPins() {
    peripherySocket = NULL;

    if ((peripherySocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        logEntry("Failed to create socket", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    struct sockaddr_in address = { 0 };
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(SIMULATOR_IP);
    address.sin_port = htons(peripheryPort);

    if (connect(peripherySocket, (struct sockaddr*)&address, sizeof(address)) != 0) {
        char logBuffer[256];
        snprintf(logBuffer, 256, "Connection to %s:%d has failed", SIMULATOR_IP, peripheryPort);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    return 1;
}

bool isKillSwitchEnabled() {
    return killSwitchEnabled;
}

int setBuzzer(bool enable) {
    char logBuffer[256];
    snprintf(logBuffer, 256, "Buzzer is %s", enable ? "enabled" : "disabled");
    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);

    return 1;
}

int setKillSwitch(bool enable) {
    SimPeripheryMessage message = SimPeripheryMessage(enable ? SimPeripheryCommand::MotorPermit : SimPeripheryCommand::MotorForbid);
    write(peripherySocket, &message, sizeof(SimPeripheryMessage));
    killSwitchEnabled = enable;

    return 1;
}

int setCargoLock(bool enable) {
    SimPeripheryMessage message = SimPeripheryMessage(enable ? SimPeripheryCommand::CargoPermit : SimPeripheryCommand::CargoForbid);
    write(peripherySocket, &message, sizeof(SimPeripheryMessage));

    return 1;
}