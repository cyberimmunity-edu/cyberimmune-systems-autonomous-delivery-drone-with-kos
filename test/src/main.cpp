#include "../sdk/sdk_firmware.h"
#include "../sdk/sdk_net.h"
#include "../sdk/sdk_gpio.h"
#include "../sdk/sdk_autopilot_communication.h"
#include "../sdk/sdk_compass_qmc5883.h"
#include "../sdk/sdk_mpu_6050.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>

char serverResponse[1024];

int main(int argc, char* argv[]) {
    if (!initializeFirmware())
        return EXIT_FAILURE;

    fprintf(stderr, "Calibrating compass...\n");
    if (!calibrateCompass())
        return EXIT_FAILURE;
    fprintf(stderr, "Compass calibration is finished\n");

    fprintf(stderr, "Calibrating MPU gyro...\n");
    if (!calibrateGyro())
        return EXIT_FAILURE;
    fprintf(stderr, "MPU gyro calibration is finished\n");

    if (!sendRequest("auth?id=1", serverResponse))
        return EXIT_FAILURE;
    fprintf(stderr, "Authentication message is sent to the server\n");

    fprintf(stderr, "Waiting for an arm...\n");

    if (!waitForCommand()) {
        fprintf(stderr, "Error in waiting for ardupilot arm-request\n");
        return EXIT_FAILURE;
    }

    startBlinking();
    fprintf(stderr, "Request for server is sent. Waiting for response...\n");

    //sleep(10);
    if (!sendRequest("arm?id=1", serverResponse))
        return EXIT_FAILURE;
    fprintf(stderr, "Server response is received\n");

    if (strstr(serverResponse, "Arm: 0 ") != NULL) {
        stopBlinking();
        if (!setLight(true))
            return EXIT_FAILURE;
        fprintf(stderr, "Response from the server is received: arm is permitted\n");
        if (!sendCommand(KOSCommand::ArmPermit))
            return EXIT_FAILURE;
        fprintf(stderr, "Arm permit is passed to ardupilot\n");
    }
    else if (strstr(serverResponse, "Arm: 1 ") != NULL) {
        stopBlinking();
        if (!setLight(false))
            return EXIT_FAILURE;
        fprintf(stderr, "Response from the server is received: arm is prohibited\n");
        if (!sendCommand(KOSCommand::ArmForbid))
            return EXIT_FAILURE;
        fprintf(stderr, "Arm prohibition is passed to ardupilot\n");
        return EXIT_SUCCESS;
    }
    else {
        fprintf(stderr, "Failed to parse server response\n");
        return EXIT_FAILURE;
    }

    //int tmpRep = 0;
    while(1) {
        fprintf(stderr, "Azimuth: %f\n", getAzimuth());
        Vector3f acc = getAcceleration();
        fprintf(stderr, "Acceleration: %f, %f, %f\n", acc.X, acc.Y, acc.Z);
        Vector3f gyro = getGyro();
        fprintf(stderr, "Gyro: %f, %f, %f\n", gyro.X, gyro.Y, gyro.Z);
        fprintf(stderr, "Temperature: %f\n", getTemperature());
        fprintf(stderr, "\n");

        if (!sendRequest("fly_accept?id=1", serverResponse))
            return EXIT_FAILURE;
        if (strstr(serverResponse, "Arm: 0 ") != NULL) {
            //Continue fligt
        }
        else if (strstr(serverResponse, "Arm: 1 ") != NULL) {
    //    if (tmpRep >= 20) {
            fprintf(stderr, "A request to stop the flight was received from the server\n");
            if (!setLight(false))
                return EXIT_FAILURE;
            if (!sendCommand(KOSCommand::AbortFlight))
                return EXIT_FAILURE;
            fprintf(stderr, "A request to stop flight is passed to ardupilot\n");
            return EXIT_SUCCESS;
        }
        else {
            fprintf(stderr, "Failed to parse server response\n");
            return EXIT_FAILURE;
        }
    //    tmpRep++;
        sleep(1);
    };

    return EXIT_SUCCESS;
}
