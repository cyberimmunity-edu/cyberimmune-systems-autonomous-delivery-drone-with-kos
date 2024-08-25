/**
 * \file
 * \~English \brief Implementation of methods for flight mission management.
 * \~Russian \brief Реализация методов для работы с полетной миссией.
 */

#include "../include/mission.h"
#include "../../shared/include/ipc_messages_logger.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** \cond */
#define COMMAND_MAX_STRING_LEN 32

uint32_t commandNum = 0;
MissionCommand *commands = NULL;
int hasMission = false;
/** \endcond */


/**
 * \~English Identifies mission string number end character.
 * \param [in] character Mission string character.
 * \return Returns 1 if the character finishes number, 0 otherwise.
 * \note Considered end characters: "_" -- separates arguments," & "-- separates commands and
 * "#" -- finishes the entire mission.
 * \~Russian Определяет символ окончания записи числа в миссии.
 * \param[in] character Символ строки миссии.
 * \return Возвращает 1 если символ оканчивает запись числа, иначе -- 0.
 * \note Концом записи считаются: "_", отделяющий аргументы, "&" -- отделяющий команды и
 * "#" -- завершающий всю миссию.
 */
int isStopSymbol(char character) {
    return ((character == '_') || (character == '&') || (character == '#'));
}

/**
 * \~English Recognizes and converts fractional number from mission string to an integer.
 * \param [in, out] str Pointer to a string. Will be shifted to the beginning of the next number/command.
 * \param [out] value Recognized number.
 * \param [in] numAfterPoint Number of decimal places. Excess places will be discarded. Missing zeroes will be added.
 * \return Returns 1 on successful parse, 0 otherwise.
 * \~Russian Распознает и преобразует дробную запись одного числа в строке миссии в целое.
 * \param[in, out] str Указатель на строку. Будет сдвинут к началу следующего числа/команды.
 * \param[out] value Распознанное число.
 * \param[in] numAfterPoint Число точек, после запятой. Если в записи поданного числа цифр после запятой больше указанного,
 * лишние будут отброшены. Если в записи поданного числа цифр после запятой меньше указанного, недостающие нули
 * будут добавлены.
 * \return Возвращает 1, если число было распознано, иначе -- 0.
 */
int parseInt(char*& string, int32_t& value, uint32_t numAfterPoint) {
    char stringValue[COMMAND_MAX_STRING_LEN];
    uint32_t strPtr = 0, valPtr = 0;
    while (!isStopSymbol(string[strPtr])) {
        if (valPtr >= COMMAND_MAX_STRING_LEN) {
            logEntry("Failed to parse mission command", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        else if (string[strPtr] == '.') {
            strPtr++;
            break;
        }
        else {
            stringValue[valPtr] = string[strPtr];
            strPtr++;
            valPtr++;
        }
    }
    int i = 0;
    while (!isStopSymbol(string[strPtr])) {
        if (valPtr >= COMMAND_MAX_STRING_LEN) {
            logEntry("Failed to parse mission command", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        else if (i < numAfterPoint) {
            stringValue[valPtr] = string[strPtr];
            strPtr++;
            valPtr++;
            i++;
        }
        else
            strPtr++;
    }
    string += strPtr + 1;
    for (int j = i; j < numAfterPoint; j++) {
        if (valPtr >= COMMAND_MAX_STRING_LEN) {
            logEntry("Failed to parse mission command", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        else {
            stringValue[valPtr] = '0';
            valPtr++;
        }
    }
    stringValue[valPtr] = '\0';
    value = atoi(stringValue);
    return 1;
}

/**
 * \~English Parses mission commands.
 * \param[in] str Pointer to mission string. Parses commands until "#" character.
 * \return Returns 1 on successful parse, 0 otherwise.
 * \~Russian Распознает команды миссии.
 * \param[in] str Указатель на строку с миссией. Распознает команды до символа "#".
 * \return Возвращает 1, если команды были распознаны, иначе -- 0.
 */
int parseCommands(char* str) {
    commandNum = 0;
    for (uint32_t i = 0; ; i++) {
        if (str[i] == '#')
            break;
        if (str[i] == 'H' || str[i] == 'T' || str[i] == 'W' || str[i] == 'L' || str[i] == 'S')
            commandNum++;
    }
    if (commandNum == 0) {
        logEntry("Mission contains no commands", ENTITY_NAME, LogLevel::LOG_WARNING);
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
        char* stringPtr = str + ptr + 1;
        int32_t lat, lng, alt;
        switch (str[ptr]) {
        case 'H':
            if (!parseInt(stringPtr, lat, 7) || !parseInt(stringPtr, lng, 7) || !parseInt(stringPtr, alt, 2)) {
                logEntry("Failed to parse values for 'home' command", ENTITY_NAME, LogLevel::LOG_WARNING);
                free(commands);
                return 0;
            }
            commands[i].type = CommandType::HOME;
            commands[i].content.waypoint = CommandWaypoint(lat, lng, alt);
            break;
        case 'T':
            if (!parseInt(stringPtr, alt, 2)) {
                logEntry("Failed to parse values for 'takeoff' command", ENTITY_NAME, LogLevel::LOG_WARNING);
                free(commands);
                return 0;
            }
            commands[i].type = CommandType::TAKEOFF;
            commands[i].content.takeoff = CommandTakeoff(alt);
            break;
        case 'W': {
            int32_t hold;
            if (!parseInt(stringPtr, hold, 0) || !parseInt(stringPtr, lat, 7) || !parseInt(stringPtr, lng, 7) || !parseInt(stringPtr, alt, 2)) {
                logEntry("Failed to parse values for 'waypoint' command", ENTITY_NAME, LogLevel::LOG_WARNING);
                free(commands);
                return 0;
            }
            commands[i].type = CommandType::WAYPOINT;
            commands[i].content.waypoint = CommandWaypoint(lat, lng, alt);
            break;
        }
        case 'L':
            if (!parseInt(stringPtr, lat, 7) || !parseInt(stringPtr, lng, 7) || !parseInt(stringPtr, alt, 2)) {
                logEntry("Failed to parse values for 'land' command", ENTITY_NAME, LogLevel::LOG_WARNING);
                free(commands);
                return 0;
            }
            commands[i].type = CommandType::LAND;
            commands[i].content.waypoint = CommandWaypoint(lat, lng, alt);
            break;
        case 'S': {
            int32_t num, pwm;
            if (!parseInt(stringPtr, num, 0) || !parseInt(stringPtr, pwm, 0)) {
                logEntry("Failed to parse values for 'set servo' command", ENTITY_NAME, LogLevel::LOG_WARNING);
                free(commands);
                return 0;
            }
            commands[i].type = CommandType::SET_SERVO;
            commands[i].content.servo = CommandServo(num, pwm);
            break;
        }
        default: {
            char logBuffer[256];
            snprintf(logBuffer, 256, "Cannot parse an unknown command %c", str[ptr]);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            free(commands);
            return 0;
        }
        }
        ptr = end + 1;
    }

    hasMission = 1;
    return 1;
}

int parseMission(char* response) {
    if (strstr(response, "$-1#") != NULL) {
        logEntry("No mission is available on the server", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    char header[] = "$FlightMission ";
    char* start = strstr(response, header);
    if (start == NULL) {
        logEntry("Response from the server does not contain mission", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    start += strlen(header);

    return parseCommands(start);
}

void printMission() {
    if (!hasMission) {
        logEntry("No available mission", ENTITY_NAME, LogLevel::LOG_INFO);
        return;
    }
    char logBuffer[256];
    logEntry("Mission: ", ENTITY_NAME, LogLevel::LOG_INFO);
    for (int i = 0; i < commandNum; i++) {
        switch (commands[i].type) {
        case CommandType::HOME:
            snprintf(logBuffer, 256, "Home: %d, %d, %d", commands[i].content.waypoint.latitude,
                commands[i].content.waypoint.longitude, commands[i].content.waypoint.altitude);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
            break;
        case CommandType::TAKEOFF:
            snprintf(logBuffer, 256, "Takeoff: %d", commands[i].content.takeoff.altitude);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
            break;
        case CommandType::WAYPOINT:
            snprintf(logBuffer, 256, "Waypoint: %d, %d, %d", commands[i].content.waypoint.latitude,
                commands[i].content.waypoint.longitude, commands[i].content.waypoint.altitude);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
            break;
        case CommandType::LAND:
            snprintf(logBuffer, 256, "Land: %d, %d, %d", commands[i].content.waypoint.latitude,
                commands[i].content.waypoint.longitude, commands[i].content.waypoint.altitude);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
            break;
        case CommandType::SET_SERVO:
            snprintf(logBuffer, 256, "Set servo: %d, %d", commands[i].content.servo.number, commands[i].content.servo.pwm);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
            break;
        default:
            logEntry("An unknown command", ENTITY_NAME, LogLevel::LOG_WARNING);
            break;
        }
    }
}