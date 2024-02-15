#include "sdk_mission.h"
#include "sdk_net.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define MISSION_RESPONSE_MAX_LENGTH 2048
#define COMMAND_MAX_STRING_LEN 32

uint32_t commandNum = 0;
MissionCommand *commands = NULL;
int hasMission = false;

CommandHome::CommandHome(float* values) {
    latitude = values[0];
    longitude = values[1];
    altitude = values[2];
}

CommandTakeoff::CommandTakeoff(float* values) {
    altitude = values[0];
}

CommandWaypoint::CommandWaypoint(float* values) {
    holdTime = values[0];
    latitude = values[1];
    longitude = values[2];
    altitude = values[3];
}

CommandLand::CommandLand(float* values) {
    latitude = values[0];
    longitude = values[1];
    altitude = values[2];
}

CommandSetServo::CommandSetServo(float* values) {
    number = values[0];
    pwm = values[1];
}

int parseValue(uint32_t num, char* string, float* values) {
    uint32_t ptrG = 0;
    for (uint32_t i = 0; i < num; i++) {
        uint32_t ptrL = 0;
        char value[COMMAND_MAX_STRING_LEN];
        while (string[ptrG] != '_' && string[ptrG] != '&' && string[ptrG] != '#') {
            if (ptrL >= COMMAND_MAX_STRING_LEN) {
                fprintf(stderr, "Error: failed to parse mission command\n");
                return 0;
            }
            value[ptrL] = string[ptrG];
            ptrL++;
            ptrG++;
        }
        value[ptrL] = '\0';
        ptrG++;
        values[i] = atof(value);
    }
    return 1;
}

int parseMission(char* str) {
    commandNum = 0;
    for (uint32_t i = 0; ; i++) {
        if (str[i] == '#')
            break;
        if (str[i] == 'H' || str[i] == 'T' || str[i] == 'W' || str[i] == 'L' || str[i] == 'S')
            commandNum++;
    }
    if (commandNum == 0) {
        fprintf(stderr, "Error: mission contains no commands\n");
        return 0;
    }

    commands = (MissionCommand*)malloc(commandNum * sizeof(MissionCommand));
    uint32_t ptr = 0;
    for (uint32_t i = 0; i < commandNum; i++) {
        uint32_t end = ptr;
        for (uint32_t j = ptr + 1; ; j++)
            if (str[j] == '&' || str[j] == '#') {
                end = j;
                break;
            }
        float *values;
        switch (str[ptr]) {
        case 'H':
            values = (float*)malloc(3 * sizeof(float));
            if (!parseValue(3, str + ptr + 1, values)) {
                fprintf(stderr, "Error: failed to parse values for 'home' command\n");
                free(values);
                free(commands);
                return 0;
            }
            commands[i].type = CommandType::HOME;
            commands[i].content.home = CommandHome(values);
            break;
        case 'T':
            values = (float*)malloc(sizeof(float));
            if (!parseValue(1, str + ptr + 1, values)) {
                fprintf(stderr, "Error: failed to parse values for 'takeoff' command\n");
                free(values);
                free(commands);
                return 0;
            }
            commands[i].type = CommandType::TAKEOFF;
            commands[i].content.takeoff = CommandTakeoff(values);
            break;
        case 'W':
            values = (float*)malloc(4 * sizeof(float));
            if (!parseValue(4, str + ptr + 1, values)) {
                fprintf(stderr, "Error: failed to parse values for 'waypoint' command\n");
                free(values);
                free(commands);
                return 0;
            }
            commands[i].type = CommandType::WAYPOINT;
            commands[i].content.waypoint = CommandWaypoint(values);
            break;
        case 'L':
            values = (float*)malloc(3 * sizeof(float));
            if (!parseValue(3, str + ptr + 1, values)) {
                fprintf(stderr, "Error: failed to parse values for 'land' command\n");
                free(values);
                free(commands);
                return 0;
            }
            commands[i].type = CommandType::LAND;
            commands[i].content.land = CommandLand(values);
            break;
        case 'S':
            values = (float*)malloc(2 * sizeof(float));
            if (!parseValue(2, str + ptr + 1, values)) {
                fprintf(stderr, "Error: failed to parse values for 'set servo' command\n");
                free(values);
                free(commands);
                return 0;
            }
            commands[i].type = CommandType::SET_SERVO;
            commands[i].content.setServo = CommandSetServo(values);
            break;
        default:
            fprintf(stderr, "Error: cannot parse unknown command %c\n", str[ptr]);
            free(commands);
            return 0;
        }
        free(values);
        ptr = end + 1;
    }

    hasMission = 1;
    return 1;
}

int missionAvailable() {
    return hasMission;
}

int requestMission() {
    char response[MISSION_RESPONSE_MAX_LENGTH] = {0};
    if (!sendRequest("fmission_kos", response))
        return 0;

    if (strstr(response, " No mission ") != NULL) {
        fprintf(stderr, "Warning: no mission is available on the server\n");
        return 0;
    }

    char missionHeader[] = "$FlightMission ";
    char* missionStart = strstr(response, missionHeader);
    if (missionStart == NULL) {
        fprintf(stderr, "Error: response from the server does not contain mission\n");
        return 0;
    }
    missionStart += strlen(missionHeader);

    if (!parseMission(missionStart)) {
        fprintf(stderr, "Error: failed to parse received mission\n");
        return 0;
    }
    return 1;
}

void deleteMission() {
    hasMission = 0;
    commandNum = 0;
    free(commands);
}

void printMission() {
    if (!hasMission) {
        fprintf(stderr, "No mission\n");
        return;
    }
    fprintf(stderr, "Mission: \n");
    for (int i = 0; i < commandNum; i++) {
        switch (commands[i].type) {
        case CommandType::HOME:
            fprintf(stderr, "Home: %f, %f, %f\n", commands[i].content.home.latitude, commands[i].content.home.longitude,
                commands[i].content.home.altitude);
            break;
        case CommandType::TAKEOFF:
            fprintf(stderr, "Takeoff: %f\n", commands[i].content.takeoff.altitude);
            break;
        case CommandType::WAYPOINT:
            fprintf(stderr, "Waypoint: %f, %f, %f, %f\n", commands[i].content.waypoint.holdTime, commands[i].content.waypoint.latitude,
                commands[i].content.waypoint.longitude, commands[i].content.waypoint.altitude);
            break;
        case CommandType::LAND:
            fprintf(stderr, "Land: %f, %f, %f\n", commands[i].content.land.latitude, commands[i].content.land.longitude,
                commands[i].content.land.altitude);
            break;
        case CommandType::SET_SERVO:
            fprintf(stderr, "Set servo: %f, %f\n", commands[i].content.setServo.number, commands[i].content.setServo.pwm);
            break;
        default:
            fprintf(stderr, "Error: unknown command\n");
            break;
        }
    }
}