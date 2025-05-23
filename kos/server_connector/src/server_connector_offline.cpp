/**
 * \file
 * \~English
 * \brief Implementation of methods for ATM server communication simulation.
 * \details The file contains implementation of methods, that simulate
 * requests to an ATM server send and received responses process.
 *
 * \~Russian
 * \brief Реализация методов для имитации общения с сервером ОРВД.
 * \details В файле реализованы методы, имитирующие отправку запросов на сервер ОРВД
 * и обработку полученных ответов.
 */

#include "../include/server_connector.h"

#include <stdio.h>
#include <string.h>

int initServerConnector() {
    if (strlen(BOARD_ID))
        setBoardName(BOARD_ID);
    else
        setBoardName("00:00:00:00:00:00");

    return 1;
}

int requestServer(char* query, char* response, uint32_t responseSize) {
    if (strstr(query, "/api/flight_info?") != NULL) {
        if (responseSize < 103) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$Flight 0$ForbiddenZonesHash 0faa8891ae3eb2d172ed240008cc93b80b568dfea98c9335f5217fb315f9ff69$Delay 1#", 103);
    }
    else if (strstr(query, "/api/auth?") != NULL) {
        if (responseSize < 10) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$Success#", 10);
    }
    else if (strstr(query, "/api/fmission_kos?") != NULL) {
        if (responseSize < 511) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$FlightMission H60.0144850_27.8200870_20.00&T1.0&W60.0144222_27.8200360_1.0&W60.0144105_27.8201212_1.0&W60.0144478_27.8201520_1.0&W60.0144280_27.8202989_1.0&W60.0143923_27.8202660_1.0&W60.0143809_27.8203599_1.0&W60.0144471_27.8204216_1.0&D3.0&S5.0_1200.0&D1.0&S5.0_1800.0&W60.0144756_27.8201668_1.0&L60.0144756_27.8201668_0.0#", 327);
    }
    else if (strstr(query, "/api/nmission?") != NULL) {
        if (responseSize < 13) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$Approve 0#", 13);
    }
    else if ((strstr(query, "/api/arm?") != NULL) || (strstr(query, "/api/fly_accept?") != NULL)) {
        if (responseSize < 16) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$Arm 0$Delay 1#", 16);
    }
    else if (strstr(query, "/api/get_all_forbidden_zones?") != NULL) {
        if (responseSize < 191) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$ForbiddenZones 0#", 19);
    }
    else {
        if (responseSize < 3) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$#", 3);
    }

    return 1;
}

int publish(char* topic, char* publication) {
    return 1;
}