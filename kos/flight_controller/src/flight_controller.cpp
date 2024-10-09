/**
 * \file
 * \~English \brief Implementation of methods for flight mission management.
 * \~Russian \brief Реализация методов для работы с полетной миссией.
 */

#include "../include/flight_controller.h"
#include "../../shared/include/ipc_messages_logger.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <mbedtls_v3/sha256.h>

/** \cond */
#define COMMAND_MAX_STRING_LEN 32

uint32_t commandNum = 0;
MissionCommand *commands = NULL;

uint32_t areaNum = 0;
NoFlightArea *areas = NULL;
char areasHash[65] = {0};
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
    char stringValue[COMMAND_MAX_STRING_LEN + 1] = {0};
    uint32_t strPtr = 0, valPtr = 0;
    while (string[strPtr] != '.')
        if (valPtr >= COMMAND_MAX_STRING_LEN) {
            logEntry("Failed to parse int value: too long", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        else if (isStopSymbol(string[strPtr])) {
            strPtr--;
            break;
        }
        else if (isdigit(string[strPtr])) {
            stringValue[valPtr] = string[strPtr];
            strPtr++;
            valPtr++;
        }
        else {
            logEntry("Failed to parse int value: contains non-digit symbols", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
    strPtr++;
    int i = 0;
    while (!isStopSymbol(string[strPtr]))
        if (valPtr >= COMMAND_MAX_STRING_LEN) {
            logEntry("Failed to parse int value: too long", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        else if (!isdigit(string[strPtr])) {
            logEntry("Failed to parse int value: contains non-digit symbols", ENTITY_NAME, LogLevel::LOG_WARNING);
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
    string += strPtr + 1;
    for (int j = i; j < numAfterPoint; j++)
        if (valPtr >= COMMAND_MAX_STRING_LEN) {
            logEntry("Failed to parse int value: too many digits after point is required", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        else {
            stringValue[valPtr] = '0';
            valPtr++;
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
    for (uint32_t i = 0; ; i++)
        if (str[i] == '#')
            break;
        else if (str[i] == 'H' || str[i] == 'T' || str[i] == 'W' || str[i] == 'L' || str[i] == 'S')
            commandNum++;
        else if (str[i] == '\0') {
            logEntry("Cannot parse commands: no correct ending", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
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
            else if (str[j] == '\0') {
                logEntry("Failed to parse commands: unexpected string end", ENTITY_NAME, LogLevel::LOG_WARNING);
                return 0;
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
            if (!parseInt(stringPtr, lat, 7) || !parseInt(stringPtr, lng, 7) || !parseInt(stringPtr, alt, 2)) {
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
        case 'D': {
            if (!parseInt(stringPtr, alt, 0)) {
                logEntry("Failed to parse values for 'delay' command", ENTITY_NAME, LogLevel::LOG_WARNING);
                free(commands);
                return 0;
            }
            commands[i].type = CommandType::DELAY;
            commands[i].content.delay = CommandDelay(alt);
            break;
        }
        default: {
            char logBuffer[256] = {0};
            snprintf(logBuffer, 256, "Cannot parse an unknown command %c", str[ptr]);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            free(commands);
            return 0;
        }
        }
        ptr = end + 1;
    }

    return 1;
}

/**
 * \~English Parses no-flight areas.
 * \param[in] str Pointer to no-flight areas string. Parses areas until "#" character.
 * \return Returns 1 on successful parse, 0 otherwise.
 * \~Russian Распознает бесполетные зоны.
 * \param[in] str Указатель на строку с бесполетными зонами. Распознает зоны до символа "#".
 * \return Возвращает 1, если зоны были распознаны, иначе -- 0.
 */
int parseAreas(char* str) {
    int32_t num;
    if (!parseInt(str, num, 0)) {
        logEntry("Failed to parse a number of no-flight areas", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    areaNum = num;
    areas = (NoFlightArea*)malloc(areaNum * sizeof(NoFlightArea));

    for (int i = 0; i < areaNum; i++) {
        char* nameEnd = strstr(str, "&");
        if (nameEnd == NULL) {
            char logBuffer[256];
            snprintf(logBuffer, 256, "Failed to parse a name of no-flight area %d", i + 1);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        int nameLen = nameEnd - str;
        if (nameLen > AREA_NAME_MAX_LEN)
            nameLen = AREA_NAME_MAX_LEN;
        strncpy(areas[i].name, str, nameLen);
        str = nameEnd + 1;
        int32_t pointNum;
        if (!parseInt(str, pointNum, 0)) {
            char logBuffer[256];
            snprintf(logBuffer, 256, "Failed to parse a number of no-flight area %d points", i + 1);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        areas[i].pointNum = pointNum;
        areas[i].points = (Point2D*)malloc(pointNum * sizeof(Point2D));

        for (int j = 0; j < pointNum; j++)
            if (!parseInt(str, areas[i].points[j].latitude, 7) || !parseInt(str, areas[i].points[j].longitude, 7)) {
                char logBuffer[256];
                snprintf(logBuffer, 256, "Failed to parse point %d of no-flight area %d", j + 1, i + 1);
                logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
                return 0;
            }
    }

    return 1;
}

/**
 * \~English Parses changes in no-flight areas.
 * \param[in] str Pointer to no-flight areas changes string. Parses changes until "#" character.
 * \return Returns 1 on successful parse, 0 otherwise.
 * \~Russian Распознает изменения бесполетных зон.
 * \param[in] str Указатель на строку с изменениями бесполетных зон. Распознает изменения до символа "#".
 * \return Возвращает 1, если изменения были распознаны, иначе -- 0.
 */
int parseAreasDelta(char* str) {
    int32_t num;
    if (!parseInt(str, num, 0)) {
        logEntry("Failed to parse a number of no-flight areas changes", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    char* end = NULL;
    char areaName[AREA_NAME_MAX_LEN + 1] = {0};
    char modeType[AREA_NAME_MAX_LEN + 1] = {0};
    for (int i = 0; i < num; i++) {
        end = strstr(str, "&");
        if (end == NULL) {
            char logBuffer[256];
            snprintf(logBuffer, 256, "Failed to parse a name of no-flight area %d", i + 1);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        int len = end - str;
        if (len > AREA_NAME_MAX_LEN)
            len = AREA_NAME_MAX_LEN;
        strncpy(areaName, str, len);
        areaName[len] = '\0';
        str = end + 1;
        end = strstr(str, "&");
        if (end == NULL) {
            char logBuffer[256];
            snprintf(logBuffer, 256, "Failed to parse delta type of no-flight area %d", i + 1);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        len = end - str;
        strncpy(modeType, str, len);
        modeType[len] = '\0';
        str = end + 1;
        if (!strcmp(modeType, "modified")) {
            int32_t pointNum;
            if (!parseInt(str, pointNum, 0)) {
                logEntry("Failed to parse point number of no-flight area to modify", ENTITY_NAME, LogLevel::LOG_WARNING);
                return 0;
            }
            Point2D* points = (Point2D*)malloc(pointNum * sizeof(Point2D));
            for (int j = 0; j < pointNum; j++)
                if (!parseInt(str, points[j].latitude, 7) || !parseInt(str, points[j].longitude, 7)) {
                    char logBuffer[256];
                    snprintf(logBuffer, 256, "Failed to parse point %d of no-flight area to modify", j + 1);
                    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
                    return 0;
                }
            bool found = false;
            for (int j = 0; j < areaNum; j++)
                if (!strcmp(areas[j].name, areaName)) {
                    free(areas[j].points);
                    areas[j].pointNum = pointNum;
                    areas[j].points = points;
                    found = true;
                    break;
                }
            if (!found)
                free(points);
        }
        else if (!strcmp(modeType, "added")) {
            areas = (NoFlightArea*)realloc(areas, (areaNum + 1) * sizeof(NoFlightArea));
            strncpy(areas[areaNum].name, areaName, strlen(areaName));
            if (!parseInt(str, areas[areaNum].pointNum, 0)) {
                char logBuffer[256];
                snprintf(logBuffer, 256, "Failed to parse a number of no-flight area %d points", areaNum + 1);
                logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
                return 0;
            }
            areas[areaNum].points = (Point2D*)malloc(areas[areaNum].pointNum * sizeof(Point2D));
            for (int k = 0; k < areas[areaNum].pointNum; k++)
                if (!parseInt(str, areas[areaNum].points[k].latitude, 7) || !parseInt(str, areas[areaNum].points[k].longitude, 7)) {
                    char logBuffer[256];
                    snprintf(logBuffer, 256, "Failed to parse point %d of no-flight area %d", k + 1, areaNum + 1);
                    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
                    return 0;
                }
            areaNum++;
        }
        else if (!strcmp(modeType, "deleted")) {
            int32_t pointNum, coord;
            if (!parseInt(str, pointNum, 0)) {
                logEntry("Failed to parse point number of no-flight area to delete", ENTITY_NAME, LogLevel::LOG_WARNING);
                return 0;
            }
            for (int k = 0; k < pointNum; k++)
                if (!parseInt(str, coord, 7) || !parseInt(str, coord, 7)) {
                    char logBuffer[256];
                    snprintf(logBuffer, 256, "Failed to parse point %d of no-flight area to delete", k + 1);
                    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
                    return 0;
            }
            for (int j = 0; j < areaNum; j++)
                if (!strcmp(areas[j].name, areaName)) {
                    free(areas[j].points);
                    for (int k = j + 1; k < areaNum; k++) {
                        strncpy(areas[k - 1].name, areas[k].name, strlen(areas[k].name));
                        areas[k - 1].pointNum = areas[k].pointNum;
                        areas[k - 1].points = areas[k].points;
                    }
                    areaNum--;
                    areas = (NoFlightArea*)realloc(areas, areaNum * sizeof(NoFlightArea));
                    break;
                }
        }
        else {
            char logBuffer[256];
            snprintf(logBuffer, 256, "Unknown delta type '%s'", modeType);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
    }

    return 1;
}

/**
 * \~English Converts coordinate into a string.
 * \param[out] string Pointer to string to write converted coordinate.
 * \param[in] len Length of string to write coordinate.
 * \param[in] coord Coordinate in degrees * 10^7.
 * \~Russian Преобразует координату в строку.
 * \param[out] string Указатель на строку, куда будет записана координата.
 * \param[in] len Длина строка для записи координаты.
 * \param[in] coord Координата в градусах * 10^7.
 */
void coordToString(char* string, uint8_t len, int32_t coord) {
    snprintf(string, len, "%d.%d", coord / 10000000, abs(coord % 10000000));
    for (int k = strlen(string) - 1; k >= 0; k--)
        if (string[k] == '0')
            string[k] = '\0';
        else if (string[k] == '.') {
            string[k] = '\0';
            break;
        }
        else
            break;
}

int loadMission(char* mission) {
    if (strstr(mission, "$-1#") != NULL) {
        logEntry("No mission is available on the server", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    char header[] = "$FlightMission ";
    char* start = strstr(mission, header);
    if (start == NULL) {
        logEntry("Response from the server does not contain mission", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    start += strlen(header);

    return parseCommands(start);
}

void printMission() {
    if (!commandNum) {
        logEntry("No available mission", ENTITY_NAME, LogLevel::LOG_INFO);
        return;
    }
    char logBuffer[256] = {0};
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
        case CommandType::DELAY:
            snprintf(logBuffer, 256, "Delay: %d", commands[i].content.delay.delay);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
            break;
        default:
            logEntry("An unknown command", ENTITY_NAME, LogLevel::LOG_WARNING);
            break;
        }
    }
}

MissionCommand* getMissionCommands(int &num) {
    num = commandNum;
    return commands;
}

int loadNoFlightAreas(char* areas) {
    char header[] = "$ForbiddenZones ";
    char* start = strstr(areas, header);
    if (start == NULL) {
        logEntry("Response from the server does not contain no-flight areas", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    start += strlen(header);
    for (int i = 0; i < 65; i++)
        areasHash[i] = '\0';

    return parseAreas(start);
}

int updateNoFlightAreas(char* areas) {
    char header[] = "$ForbiddenZonesDelta ";
    char* start = strstr(areas, header);
    if (start == NULL) {
        logEntry("Response from the server does not contain no-flight areas delta", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    start += strlen(header);
    for (int i = 0; i < 65; i++)
        areasHash[i] = '\0';

    return parseAreasDelta(start);
}

void deleteNoFlightAreas() {
    for (int i = 0; i < areaNum; i++)
        free(areas[i].points);
    free(areas);
}

void printNoFlightAreas() {
    if (!areaNum) {
        logEntry("No-flight areas list is empty", ENTITY_NAME, LogLevel::LOG_INFO);
        return;
    }
    char logBuffer[256];
    snprintf(logBuffer, 256, "Number of no-flight areas: %d", areaNum);
    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
    for (int i = 0; i < areaNum; i++) {
        snprintf(logBuffer, 256, "Area '%s' contains %d points", areas[i].name, areas[i].pointNum);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
        for (int j = 0; j < areas[i].pointNum; j++) {
            snprintf(logBuffer, 256, "Point %d: %d, %d", j + 1, areas[i].points[j].latitude, areas[i].points[j].longitude);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
        }
    }
}

NoFlightArea* getNoFlightAreas(int &num) {
    num = areaNum;
    return areas;
}

char* getNoFlightAreasHash() {
    if (!strlen(areasHash)) {
        char areasString[512] = {0};
        snprintf(areasString, 512, "$ForbiddenZones %d", areaNum);
        for (int i = 0; i < areaNum; i++) {
            snprintf(areasString, 512, "%s&%s&%d", areasString, areas[i].name, areas[i].pointNum);
            for (int j = 0; j < areas[i].pointNum; j++) {
                char lat[13] = {0};
                char lng[13] = {0};
                coordToString(lat, 13, areas[i].points[j].latitude);
                coordToString(lng, 13, areas[i].points[j].longitude);
                snprintf(areasString, 512, "%s&%s_%s", areasString, lat, lng);
            }
        }

        uint8_t hash[32] = {0};
        mbedtls_sha256_context sha256;
        mbedtls_sha256_init(&sha256);
        if (mbedtls_sha256_starts(&sha256, 0) != 0) {
            logEntry("Failed to calculate no-flight areas hash", ENTITY_NAME, LogLevel::LOG_WARNING);
            mbedtls_sha256_free(&sha256);
            return areasHash;
        }
        if (mbedtls_sha256_update(&sha256, (unsigned char*)areasString, strlen(areasString)) != 0) {
            logEntry("Failed to calculate no-flight areas hash", ENTITY_NAME, LogLevel::LOG_WARNING);
            mbedtls_sha256_free(&sha256);
            return areasHash;
        }
        if (mbedtls_sha256_finish(&sha256, hash) != 0) {
            logEntry("Failed to calculate no-flight areas hash", ENTITY_NAME, LogLevel::LOG_WARNING);
            mbedtls_sha256_free(&sha256);
            return areasHash;
        }
        mbedtls_sha256_free(&sha256);

        for (int i = 0; i < 32; i++)
            snprintf(areasHash, 65, "%s%02x", areasHash, hash[i]);
    }

    return areasHash;
}

char* extractNoFlightAreasHash(char* hash) {
    char header[] = "$ForbiddenZonesHash ";
    char* begin = strstr(hash, header);
    if (begin)
        begin += strlen(header);
    else {
        logEntry("Response from the server does not contain no-flight areas hash", ENTITY_NAME, LogLevel::LOG_WARNING);
        return hash;
    }
    if (char* end = strstr(begin, "#")) {
        *end = '\0';
        if (end - begin == 63) {
            begin--;
            *begin = '0';
        }
    }
    return begin;
}