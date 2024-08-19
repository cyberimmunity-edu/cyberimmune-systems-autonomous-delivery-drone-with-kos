#include "../include/mission.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"
#include "../../shared/include/ipc_messages_logger.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000

char boardId[32] = {0};

int sendSignedMessage(char* method, char* response, char* errorMessage, uint8_t delay) {
    char message[512] = {0};
    char signature[257] = {0};
    char request[1024] = {0};
    snprintf(message, 512, "%s?%s", method, boardId);

    while (!signMessage(message, signature)) {
        char logBuffer[256];
        snprintf(logBuffer, 256, "Failed to sign %s message at Credential Manager. Trying again in %ds", errorMessage, delay);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(delay);
    }
    snprintf(request, 1024, "%s&sig=0x%s", message, signature);

    while (!sendRequest(request, response)) {
        char logBuffer[256];
        snprintf(logBuffer, 256, "Failed to send %s request through Server Connector. Trying again in %ds", errorMessage, delay);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(delay);
    }

    uint8_t authenticity = 0;
    while (!checkSignature(response, authenticity) || !authenticity) {
        char logBuffer[256];
        snprintf(logBuffer, 256, "Failed to check signature of %s response received through Server Connector. Trying again in %ds", errorMessage, delay);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(delay);
    }

    return 1;
}

int main(void) {
    //Before do anything, we need to ensure, that other modules are ready to work
    while (!waitForInit("logger_connection", "Logger")) {
        char logBuffer[256];
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Logger. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("periphery_controller_connection", "PeripheryController")) {
        char logBuffer[256];
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Periphery Controller. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("autopilot_connector_connection", "AutopilotConnector")) {
        char logBuffer[256];
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Autopilot Connector. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("navigation_system_connection", "NavigationSystem")) {
        char logBuffer[256];
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Navigation System. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("server_connector_connection", "ServerConnector")) {
        char logBuffer[256];
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Server Connector. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("credential_manager_connection", "CredentialManager")) {
        char logBuffer[256];
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Credential Manager. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }

    //Get ID from ServerConnector
    while (!getBoardId(boardId)) {
        logEntry("Failed to get board ID from ServerConnector. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }
    char initNotification[64] = {0};
    snprintf(initNotification, 64, "Board '%s' is initialized", boardId);
    logEntry(initNotification, ENTITY_NAME, LogLevel::LOG_INFO);

    //Enable buzzer to indicate, that all modules has been initialized
    if (!enableBuzzer())
        logEntry("Failed to enable buzzer at Periphery Controller", ENTITY_NAME, LogLevel::LOG_WARNING);

    //Copter need to be registered at ORVD
    char authResponse[1024] = {0};
    sendSignedMessage("/api/auth", authResponse, "authentication", RETRY_DELAY_SEC);
    logEntry("Successfully authenticated on the server", ENTITY_NAME, LogLevel::LOG_INFO);

    //Constantly ask server, if mission for the drone is available. Parse it and ensure, that mission is correct
    while (true) {
        char missionResponse[1024] = {0};
        if (sendSignedMessage("/api/fmission_kos", missionResponse, "mission", RETRY_DELAY_SEC) && parseMission(missionResponse)) {
            logEntry("Successfully received mission from the server", ENTITY_NAME, LogLevel::LOG_INFO);
            printMission();
            break;
        }
        sleep(RETRY_REQUEST_DELAY_SEC);
    }

    //The drone is ready to arm
    logEntry("Ready to arm", ENTITY_NAME, LogLevel::LOG_INFO);
    while (true) {
        //Wait, until autopilot wants to arm (and fails so, as motors are disabled by default)
        while (!waitForArmRequest()) {
            char logBuffer[256];
            snprintf(logBuffer, 256, "Failed to receive an arm request from Autopilot Connector. Trying again in %ds", RETRY_DELAY_SEC);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            sleep(RETRY_DELAY_SEC);
        }
        logEntry("Received arm request. Notifying the server", ENTITY_NAME, LogLevel::LOG_INFO);

        //When autopilot asked for arm, we need to receive permission from ORVD
        char armRespone[1024] = {0};
        sendSignedMessage("/api/arm", armRespone, "arm", RETRY_DELAY_SEC);

        if (strstr(armRespone, "$Arm: 0#") != NULL) {
            //If arm was permitted, we enable motors
            logEntry("Arm is permitted", ENTITY_NAME, LogLevel::LOG_INFO);
            while (!setKillSwitch(true)) {
                char logBuffer[256];
                snprintf(logBuffer, 256, "Failed to permit motor usage at Periphery Controller. Trying again in %ds", RETRY_DELAY_SEC);
                logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
                sleep(RETRY_DELAY_SEC);
            }
            if (!permitArm())
                logEntry("Failed to permit arm through Autopilot Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
            break;
        }
        else if (strstr(armRespone, "$Arm: 1#") != NULL) {
            logEntry("Arm is forbidden", ENTITY_NAME, LogLevel::LOG_INFO);
            if (!forbidArm())
                logEntry("Failed to forbid arm through Autopilot Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
        }
        else
            logEntry("Failed to parse server response", ENTITY_NAME, LogLevel::LOG_WARNING);
        logEntry("Arm was not allowed. Waiting for another arm request from autopilot", ENTITY_NAME, LogLevel::LOG_WARNING);
    };

    //If we get here, the drone is able to arm and start the mission
    //The flight is need to be controlled from now on
    //Also we need to check on ORVD, whether the flight is still allowed or it is need to be paused

    while (true)
        sleep(1000);

    return EXIT_SUCCESS;
}