#include "../sdk/sdk_firmware.h"
#include "../sdk/sdk_net.h"
#include "../sdk/sdk_autopilot_communication.h"
#include "../sdk/sdk_light.h"
#include "../sdk/sdk_kill_switch.h"
#include "../sdk/sdk_compass_qmc5883.h"
#include "../sdk/sdk_mpu_6050.h"
#include "../sdk/sdk_mission.h"
#include "../sdk/sdk_authenticity.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

uint8_t requestRepeatDelay = 5;
const uint32_t responseMaxLength = 1024;
char serverResponse[responseMaxLength] = {0};

int main(int argc, char* argv[]) {
    if (!initializeFirmware()) {
        fprintf(stderr, "Critical Error: firmware was not initialized. System is not ready for flight control\n");
        return EXIT_FAILURE;
    }

    fprintf(stderr, "Info: calibrating compass\n");
    if (!calibrateCompass())
        fprintf(stderr, "Error: failed to calibrate compass\n");

    fprintf(stderr, "Info: calibrating MPU gyro\n");
    if (!calibrateGyro())
        fprintf(stderr, "Error: failed to calibrate gyro\n");

    fprintf(stderr, "Generating RSA keys and sharing it with the server\n");
    while (!shareRsaKey(serverResponse)) {
        fprintf(stderr, "Warning: failed to share RSA key with the server. Repeating in %d s...\n", requestRepeatDelay);
        sleep(requestRepeatDelay);
    };

    fprintf(stderr, "Authenticating on the server\n");
    while (!sendRequest("auth", serverResponse)) {
        fprintf(stderr, "Warning: failed to authenticate on the server. Repeating in %d s...\n", requestRepeatDelay);
        sleep(requestRepeatDelay);
    };
    fprintf(stderr, "Authentication message is completed\n");

    fprintf(stderr, "Requesting mission from the server\n");
    while (!requestMission()) {
        fprintf(stderr, "Warning: failed to get mission from the server. Repeating in %d s...\n", requestRepeatDelay);
        sleep(requestRepeatDelay);
    };
    fprintf(stderr, "Correct mission is received from the server\n");
    printMission();

    fprintf(stderr, "Ready to arm\n");
    while (true) {
        while (!waitForCommand()) {
            fprintf(stderr, "Warning: some problem happened during waiting for arm request from autopilot\n");
            sleep(1);
        }
        fprintf(stderr, "Arm request from autopilot is received. Sending request to the server\n");
        startBlinking();

        while (!sendRequest("arm", serverResponse)) {
            fprintf(stderr, "Warning: failed to send arm request to the server. Repeating in %d s...\n", requestRepeatDelay);
            sleep(requestRepeatDelay);
        }

        if (strstr(serverResponse, "$Arm: 0#") != NULL) {
            fprintf(stderr, "Response on arm request from the server is received: arm is permitted\n");
            if (!setLight(true))
                fprintf(stderr, "Warning: failed to turn on the light\n");
            while (!setKillSwitch(true)) {
                fprintf(stderr, "Error: failed to disable kill switch. Retrying...");
                sleep(1);
            }
            if (!sendCommand(KOSCommand::ArmPermit))
                fprintf(stderr, "Warning: failed to send message about allowed arm to autopilot\n");
            break;
        }
        else if (strstr(serverResponse, "$Arm: 1#") != NULL) {
            fprintf(stderr, "Response on arm request from the server is received: arm is prohibited\n");
            if (!setLight(false))
                fprintf(stderr, "Warning: failed to turn off the light\n");
            if (!sendCommand(KOSCommand::ArmForbid))
                fprintf(stderr, "Warning: failed to send message about forbidden arm to autopilot\n");
        }
        else
            fprintf(stderr, "Warning: failed to parse server response\n");
        fprintf(stderr, "Warning: arm was not allowed. Waiting for another arm request from autopilot...\n");
    };
    stopBlinking();

    while (true) {
        if (sendRequest("fly_accept", serverResponse)) {
            if (strstr(serverResponse, "$Arm: 0#") != NULL) {
                fprintf(stderr, "Azimuth: %f\n", getAzimuth());
                Vector3f acc = getAcceleration();
                fprintf(stderr, "Acceleration: %f, %f, %f\n", acc.X, acc.Y, acc.Z);
                Vector3f gyro = getGyro();
                fprintf(stderr, "Gyro: %f, %f, %f\n", gyro.X, gyro.Y, gyro.Z);
                fprintf(stderr, "Temperature: %f\n", getTemperature());
                fprintf(stderr, "\n");
            }
            else if (strstr(serverResponse, "$Arm: 1#") != NULL) {
                fprintf(stderr, "A request to stop the flight was received from the server\n");
                if (!setLight(false))
                    fprintf(stderr, "Warning: failed to turn off the light\n");
                if (!sendCommand(KOSCommand::AbortFlight))
                    fprintf(stderr, "Warning: failed to send message about flight abortion to autopilot\n");
                break;
            }
            else
                fprintf(stderr, "Warning: failed to parse server response\n");
        }
        else
            fprintf(stderr, "Warning: failed to confirm flight clearance from the server");

        sleep(1);
    };

    while (true) {
        fprintf(stderr, "Finishing flight...\n");
        sleep(5);
    }

    return EXIT_SUCCESS;
}
