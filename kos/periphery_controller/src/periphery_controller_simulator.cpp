#include "../include/periphery_controller.h"
#include "../include/sim_periphery_message.h"

#include <kos_net.h>

#include <stdio.h>

int peripherySocket = NULL;
uint16_t peripheryPort = 5767;

bool light = false;

int initPeripheryController() {
    if (!wait_for_network()) {
        fprintf(stderr, "[%s] Error: Connection to network has failed\n", ENTITY_NAME);
        return 0;
    }

    return 1;
}

int initGpioPins() {
    peripherySocket = NULL;

    if ((peripherySocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        fprintf(stderr, "[%s] Warning: Failed to create socket\n", ENTITY_NAME);
        return 0;
    }

    struct sockaddr_in address = { 0 };
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(SIMULATOR_IP);
    address.sin_port = htons(peripheryPort);

    if (connect(peripherySocket, (struct sockaddr*)&address, sizeof(address)) != 0) {
        fprintf(stderr, "[%s] Warning: Connection to %s:%d has failed\n", ENTITY_NAME, SIMULATOR_IP, peripheryPort);
        return 0;
    }

    return 1;
}

int setLight(bool turnOn) {
    light = turnOn;
    fprintf(stderr, "[%s] Info: Light is turned %s\n", ENTITY_NAME, light ? "on" : "off");
    return 1;
}

int setMotorKillSwitch(bool permitted) {
    SimPeripheryMessage message = SimPeripheryMessage(permitted ? SimPeripheryCommand::MotorPermit : SimPeripheryCommand::MotorForbid);
    write(peripherySocket, &message, sizeof(SimPeripheryMessage));
    return 1;
}

int setCargoKillSwitch(bool permitted) {
    SimPeripheryMessage message = SimPeripheryMessage(permitted ? SimPeripheryCommand::CargoPermit : SimPeripheryCommand::CargoForbid);
    write(peripherySocket, &message, sizeof(SimPeripheryMessage));
    return 1;
}