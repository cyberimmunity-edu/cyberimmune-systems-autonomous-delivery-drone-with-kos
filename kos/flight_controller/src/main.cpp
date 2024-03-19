#include "../include/mission.h"
#include "../include/flight_controller_interface.h"
#include "../../ipc_messages/include/initialization_interface.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000

int sendSignedMessage(char* method, char* response, char* errorMessage, uint8_t delay, bool addId) {
    char message[512] = {0};
    char signature[257] = {0};
    char request[1024] = {0};

    if (addId)
        snprintf(message, 512, "%s?%s", method, BOARD_ID);
    else
        strcpy(message, method);

    while (!signMessage(message, signature)) {
        fprintf(stderr, "[%s] Warning: Failed to sign %s message at Credential Manager. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }
    snprintf(request, 1024, "%s&sig=0x%s", message, signature);

    while (!sendRequest(request, response)) {
        fprintf(stderr, "[%s] Warning: Failed to send %s request through Server Connector. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }

    while (!checkSignature(response)) {
        fprintf(stderr, "[%s] Warning: Failed to check signature of %s response received through Server Connector. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }

    return 1;
}

int main(void) {
    while (!waitForInit("autopilot_connector_connection", "AutopilotConnector")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("credential_manager_connection", "CredentialManager")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Credential Manager. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("navigation_system_connection", "NavigationSystem")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Navigation System. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("periphery_controller_connection", "PeripheryController")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("server_connector_connection", "ServerConnector")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Server Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    fprintf(stderr, "[%s] Info: Initialization is finished\n", ENTITY_NAME);

    char rsaKeyE[257] = {0};
    char rsaKeyN[257] = {0};
    char rsaServerRequest[1024] = {0};
    char rsaServerResponse[1024] = {0};
    while (!getRsaKey(rsaKeyE, rsaKeyN)) {
        fprintf(stderr, "[%s] Warning: Failed to get RSA key from Credential Manager. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    snprintf(rsaServerRequest, 1024, "key?%s&e=0x%s&n=0x%s", BOARD_ID, rsaKeyE, rsaKeyN);
    while (!sendRequest(rsaServerRequest, rsaServerResponse)) {
        fprintf(stderr, "[%s] Warning: Failed to send request through Server Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_REQUEST_DELAY_SEC);
        sleep(RETRY_REQUEST_DELAY_SEC);
    }
    while (!setServerRsaKey(rsaServerResponse)) {
        fprintf(stderr, "[%s] Warning: Failed to request server RSA key setting from Credential Manager. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    fprintf(stderr, "[%s] Info: Successfully exchanged RSA keys with the server\n", ENTITY_NAME);

    char authResponse[1024] = {0};
    sendSignedMessage("auth", authResponse, "authentication", RETRY_DELAY_SEC, true);
    fprintf(stderr, "[%s] Info: Successfully authenticated on the server\n", ENTITY_NAME);

    while (true) {
        char missionRespone[1024] = {0};
        if (sendSignedMessage("fmission_kos", missionRespone, "mission", RETRY_DELAY_SEC, true) && parseMission(missionRespone)) {
            fprintf(stderr, "[%s] Info: Successfully received mission from the server\n", ENTITY_NAME);
            printMission();
            break;
        }
        sleep(RETRY_REQUEST_DELAY_SEC);
    }

    fprintf(stderr, "[%s] Info: Ready to arm\n", ENTITY_NAME);
    while (true) {
        while (!waitForArmRequest()) {
            fprintf(stderr, "[%s] Warning: Failed to receive an arm request from Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
            sleep(RETRY_DELAY_SEC);
        }
        fprintf(stderr, "[%s] Info: Received arm request. Notifying the server\n", ENTITY_NAME);

        char armRespone[1024] = {0};
        sendSignedMessage("arm", armRespone, "arm", RETRY_DELAY_SEC, true);

        if (strstr(armRespone, "$Arm: 0#") != NULL) {
            fprintf(stderr, "[%s] Info: Arm is permitted\n", ENTITY_NAME);
            if (!setBuzzer(false))
                fprintf(stderr, "[%s] Warning: Failed to turn off buzzer at Periphery Controller\n", ENTITY_NAME);
            while (!setMotorKillSwitch(true)) {
                fprintf(stderr, "[%s] Warning: Failed to permit motor usage at Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
                sleep(RETRY_DELAY_SEC);
            }
            if (!permitArm())
                fprintf(stderr, "[%s] Warning: Failed to permit arm through Autopilot Connector\n", ENTITY_NAME);
            break;
        }
        else if (strstr(armRespone, "$Arm: 1#") != NULL) {
            fprintf(stderr, "[%s] Info: Arm is forbidden\n", ENTITY_NAME);
            if (!forbidArm())
                fprintf(stderr, "[%s] Warning: Failed to forbid arm through Autopilot Connector\n", ENTITY_NAME);
        }
        else
            fprintf(stderr, "[%s] Warning: Failed to parse server response\n", ENTITY_NAME);
        fprintf(stderr, "[%s] Warning: Arm was not allowed. Waiting for another arm request from autopilot\n", ENTITY_NAME);
    };

    int32_t prevLat, prevLng, lat, lng, alt, azimuth;
    while (!getCoords(prevLat, prevLng, alt)) {
        fprintf(stderr, "[%s] Warning: Failed to get coords from Navigation System. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }

    bool flightPaused = false;
    while (true) {
        if (!getCoords(lat, lng, alt)) {
            fprintf(stderr, "[%s] Warning: Failed to get coords from Navigation System. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
            usleep(FLY_ACCEPT_PERIOD_US);
            continue;
        }
        azimuth = round(atan2(lng - prevLng, lat - prevLat) * 1800000000 / M_PI);
        prevLat = lat;
        prevLng = lng;

        char flyRequest[1024] = {0};
        char flyRespone[1024] = {0};
        snprintf(flyRequest, 1024, "fly_accept?%s&lat=%d&lon=%d&alt=%d&azimuth=%d", BOARD_ID, lat, lng, alt, azimuth);
        sendSignedMessage(flyRequest, flyRespone, "fly allowance", RETRY_DELAY_SEC, false);

        if (strstr(flyRespone, "$Arm: 0#") != NULL) {
            if (flightPaused) {
                flightPaused = false;
                while (!resumeFlight()) {
                    fprintf(stderr, "[%s] Warning: Failed to resume flight through Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
                    sleep(RETRY_DELAY_SEC);
                }
            }
        }
        else if (strstr(flyRespone, "$Arm: 1#") != NULL) {
            if (!flightPaused) {
                flightPaused = true;
                while (!pauseFlight()) {
                    fprintf(stderr, "[%s] Warning: Failed to pause flight through Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
                    sleep(RETRY_DELAY_SEC);
                }
            }
        }
        else
            fprintf(stderr, "[%s] Warning: Failed to parse server response\n", ENTITY_NAME);

        usleep(FLY_ACCEPT_PERIOD_US);
    }

    while (true) {
        fprintf(stderr, "[%s] Info: Finishing the flight\n", ENTITY_NAME);
        sleep(RETRY_REQUEST_DELAY_SEC);
    }

    return EXIT_SUCCESS;
}