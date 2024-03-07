#include "../include/mission.h"
#include "../include/flight_controller.h"
#include "../include/flight_controller_interface.h"
#include "../../ipc_messages/include/initialization_interface.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5

int main(void) {
    if (!waitForInit("autopilot_connector_connection", "AutopilotConnector")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    if (!waitForInit("credential_manager_connection", "CredentialManager")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Credential Manager. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    if (!waitForInit("navigation_system_connection", "NavigationSystem")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Navigation System. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    if (!waitForInit("periphery_controller_connection", "PeripheryController")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    if (!waitForInit("server_connector_connection", "ServerConnector")) {
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
    sendSignedMessage("auth", authResponse, "authentication", RETRY_DELAY_SEC);
    fprintf(stderr, "[%s] Info: Successfully authenticated on the server\n", ENTITY_NAME);

    while (true) {
        char missionRespone[1024] = {0};
        if (sendSignedMessage("fmission_kos", missionRespone, "mission", RETRY_DELAY_SEC) && parseMission(missionRespone)) {
            fprintf(stderr, "[%s] Info: Successfully received mission from the server\n", ENTITY_NAME);
            printMission();
            break;
        }
        sleep(RETRY_REQUEST_DELAY_SEC);
    }

    fprintf(stderr, "[%s] Info: Ready to arm\n", ENTITY_NAME);
    while (true) {
        AutopilotCommand command = AutopilotCommand::ERROR;
        command = AutopilotCommand::ArmRequest;
        while (!getAutopilotCommand(command)) {
            fprintf(stderr, "[%s] Warning: Failed to get command from Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
            sleep(RETRY_DELAY_SEC);
        }
        if (command == AutopilotCommand::ArmRequest) {
            fprintf(stderr, "[%s] Info: Received arm request. Notifying the server\n", ENTITY_NAME);
            if (!setLightMode(LightMode::Blinking))
                fprintf(stderr, "[%s] Warning: Failed to set light mode to blinking at Periphery Controller\n", ENTITY_NAME);

            char armRespone[1024] = {0};
            sendSignedMessage("arm", armRespone, "arm", RETRY_DELAY_SEC);

            if (strstr(armRespone, "$Arm: 0#") != NULL) {
                fprintf(stderr, "[%s] Info: Arm is permitted\n", ENTITY_NAME);
                if (!setLightMode(LightMode::On))
                    fprintf(stderr, "[%s] Warning: Failed to turn on the light at Periphery Controller\n", ENTITY_NAME);
                while (!setMotorKillSwitch(true)) {
                    fprintf(stderr, "[%s] Warning: Failed to permit motor usage at Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
                    sleep(RETRY_DELAY_SEC);
                }
                if (!sendAutopilotCommand(AutopilotCommand::ArmPermit))
                    fprintf(stderr, "[%s] Warning: Failed to permit arm through Autopilot Connector\n", ENTITY_NAME);
                break;
            }
            else if (strstr(armRespone, "$Arm: 1#") != NULL) {
                fprintf(stderr, "[%s] Info: Arm is forbidden\n", ENTITY_NAME);
                if (!setLightMode(LightMode::Off))
                    fprintf(stderr, "[%s] Warning: Failed to turn off the light at Periphery Controller\n", ENTITY_NAME);
                if (!sendAutopilotCommand(AutopilotCommand::ArmForbid))
                    fprintf(stderr, "[%s] Warning: Failed to forbid arm through Autopilot Connector\n", ENTITY_NAME);
            }
            else
                fprintf(stderr, "[%s] Warning: Failed to parse server response\n", ENTITY_NAME);
        }
        else
            fprintf(stderr, "[%s] Warning: Received command %d has no handler\n", ENTITY_NAME, command);
        fprintf(stderr, "[%s] Warning: Arm was not allowed. Waiting for another arm request from autopilot\n", ENTITY_NAME);
    };

    while (true) {
        char flyRespone[1024] = {0};
        sendSignedMessage("fly_accept", flyRespone, "fly allowance", RETRY_DELAY_SEC);

        if (strstr(flyRespone, "$Arm: 0#") != NULL) {
            float azimuth;
            if (!getAzimuth(azimuth))
                fprintf(stderr, "[%s] Warning: Failed to get azimuth from Navigation System\n", ENTITY_NAME);
            else {
                azimuth *= (180.0 / M_PI);
                while (azimuth < 0)
                    azimuth += 360;
                while (azimuth >= 360)
                    azimuth -= 360;
                fprintf(stderr, "[%s] Info: Azimuth: %f\n", ENTITY_NAME, azimuth);
            }

            float accelerationX, accelerationY, accelerationZ;
            if (!getAcceleration(accelerationX, accelerationY, accelerationZ))
                fprintf(stderr, "[%s] Warning: Failed to get acceleration from Navigation System\n", ENTITY_NAME);
            else
                fprintf(stderr, "[%s] Info: Acceleration: %f, %f, %f\n", ENTITY_NAME, accelerationX, accelerationY, accelerationZ);

            float gyroscopeX, gyroscopeY, gyroscopeZ;
            if (!getGyroscope(gyroscopeX, gyroscopeY, gyroscopeZ))
                fprintf(stderr, "[%s] Warning: Failed to get gyroscope from Navigation System\n", ENTITY_NAME);
            else
                fprintf(stderr, "[%s] Info: Gyroscope: %f, %f, %f\n", ENTITY_NAME, gyroscopeX, gyroscopeY, gyroscopeZ);

            float temperature;
            if (!getTemperature(temperature))
                fprintf(stderr, "[%s] Warning: Failed to get temperature from Navigation System\n", ENTITY_NAME);
            else
                fprintf(stderr, "[%s] Info: Temperature: %f\n", ENTITY_NAME, temperature);
        }
        else if (strstr(flyRespone, "$Arm: 1#") != NULL) {
            fprintf(stderr, "[%s] Info: Received request to stop the flight\n", ENTITY_NAME);
            if (!setLightMode(LightMode::Off))
                fprintf(stderr, "[%s] Warning: Failed to turn off the light at Periphery Controller\n", ENTITY_NAME);
            while (!sendAutopilotCommand(AutopilotCommand::AbortFlight)) {
                fprintf(stderr, "[%s] Warning: Failed to abort flight through Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
                sleep(RETRY_DELAY_SEC);
            }
            break;
        }
        else
            fprintf(stderr, "[%s] Warning: Failed to parse server response\n", ENTITY_NAME);
    }

    while (true) {
        fprintf(stderr, "[%s] Info: Finishing the flight\n", ENTITY_NAME);
        sleep(RETRY_REQUEST_DELAY_SEC);
    }

    return EXIT_SUCCESS;
}