#include "../include/mission.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define COMMAND_MAX_STRING_LEN 32

uint32_t commandNum = 0;
MissionCommand *commands = NULL;
int hasMission = false;

int parseValue(uint32_t num, char* string, float* values) {
    uint32_t ptrG = 0;
    for (uint32_t i = 0; i < num; i++) {
        uint32_t ptrL = 0;
        char value[COMMAND_MAX_STRING_LEN];
        while (string[ptrG] != '_' && string[ptrG] != '&' && string[ptrG] != '#') {
            if (ptrL >= COMMAND_MAX_STRING_LEN) {
                fprintf(stderr, "[%s] Warning: Failed to parse mission command\n", ENTITY_NAME);
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

int parseCommands(char* str) {
    commandNum = 0;
    for (uint32_t i = 0; ; i++) {
        if (str[i] == '#')
            break;
        if (str[i] == 'H' || str[i] == 'T' || str[i] == 'W' || str[i] == 'L' || str[i] == 'S')
            commandNum++;
    }
    if (commandNum == 0) {
        fprintf(stderr, "[%s] Warning: Mission contains no commands\n", ENTITY_NAME);
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
                fprintf(stderr, "[%s] Warning: Failed to parse values for 'home' command\n", ENTITY_NAME);
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
                fprintf(stderr, "[%s] Warning: Failed to parse values for 'takeoff' command\n", ENTITY_NAME);
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
                fprintf(stderr, "[%s] Warning: Failed to parse values for 'waypoint' command\n", ENTITY_NAME);
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
                fprintf(stderr, "[%s] Warning: Failed to parse values for 'land' command\n", ENTITY_NAME);
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
                fprintf(stderr, "[%s] Warning: Failed to parse values for 'set servo' command\n", ENTITY_NAME);
                free(values);
                free(commands);
                return 0;
            }
            commands[i].type = CommandType::SET_SERVO;
            commands[i].content.setServo = CommandSetServo(values);
            break;
        default:
            fprintf(stderr, "[%s] Warning: Cannot parse an unknown command %c\n", ENTITY_NAME, str[ptr]);
            free(commands);
            return 0;
        }
        free(values);
        ptr = end + 1;
    }

    hasMission = 1;
    return 1;
}

int parseMission(char* response) {
    if (strstr(response, "$-1#") != NULL) {
        fprintf(stderr, "[%s] Warning: No mission is available on the server\n", ENTITY_NAME);
        return 0;
    }

    char header[] = "$FlightMission ";
    char* start = strstr(response, header);
    if (start == NULL) {
        fprintf(stderr, "[%s] Warning: Response from the server does not contain mission\n", ENTITY_NAME);
        return 0;
    }
    start += strlen(header);

    return parseCommands(start);
}

void printMission() {
    if (!hasMission) {
        fprintf(stderr, "[%s] Info: No available mission\n", ENTITY_NAME);
        return;
    }
    fprintf(stderr, "[%s] Info: Mission: \n", ENTITY_NAME);
    for (int i = 0; i < commandNum; i++) {
        switch (commands[i].type) {
        case CommandType::HOME:
            fprintf(stderr, "[%s] Info: Home: %f, %f, %f\n", ENTITY_NAME, commands[i].content.home.latitude,
                commands[i].content.home.longitude, commands[i].content.home.altitude);
            break;
        case CommandType::TAKEOFF:
            fprintf(stderr, "[%s] Info: Takeoff: %f\n", ENTITY_NAME, commands[i].content.takeoff.altitude);
            break;
        case CommandType::WAYPOINT:
            fprintf(stderr, "[%s] Info: Waypoint: %f, %f, %f, %f\n", ENTITY_NAME, commands[i].content.waypoint.holdTime,
                commands[i].content.waypoint.latitude, commands[i].content.waypoint.longitude, commands[i].content.waypoint.altitude);
            break;
        case CommandType::LAND:
            fprintf(stderr, "[%s] Info: Land: %f, %f, %f\n", ENTITY_NAME, commands[i].content.land.latitude,
                commands[i].content.land.longitude, commands[i].content.land.altitude);
            break;
        case CommandType::SET_SERVO:
            fprintf(stderr, "[%s] Info: Set servo: %f, %f\n", ENTITY_NAME, commands[i].content.setServo.number,
                commands[i].content.setServo.pwm);
            break;
        default:
            fprintf(stderr, "[%s] Warning: An unknown command\n", ENTITY_NAME);
            break;
        }
    }
}