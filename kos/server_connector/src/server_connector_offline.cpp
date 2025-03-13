/**
 * \file
 * \~English
 * \brief Implementation of methods for AFCS server communication simulation.
 * \details The file contains implementation of methods, that simulate
 * requests to an AFCS server send and received responses process.
 *
 * \~Russian
 * \brief Реализация методов для имитации общения с сервером СУПА.
 * \details В файле реализованы методы, имитирующие отправку запросов на сервер СУПА
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
        strncpy(response, "$FlightMission H59.8804975_29.8291898_28.56&W59.8806160_29.8292515_0.0&W59.8814666_29.8300011_0.0&W59.8815019_29.8301346_0.0&W59.8815756_29.8300856_0.0&D3.0&S5.0_1200.0&W59.8815460_29.8299435_0.0&W59.8814666_29.8300011_0.0&W59.8806160_29.8292515_0.0&W59.8804975_29.8291898_0.0#", 278);
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