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
        strncpy(response, "$FlightMission H59.8804975_29.8291898_28.56&T5.0&W59.8805769_29.8292528_5.0&W59.8810763_29.8270480_5.0&W59.8796967_29.8258477_5.0&W59.8794410_29.8269233_5.0&W59.8803360_29.8277199_5.0&W59.8802055_29.8282644_5.0&D3.0&S5.0_1200.0&W59.8793030_29.8274678_5.0&W59.8784510_29.8312001_5.0&W59.8795796_29.8321885_5.0&W59.8799827_29.8319229_5.0&W59.8805769_29.8292528_5.0&W59.8804975_29.8291898_5.0&L59.8804975_29.8291898_0.0#", 418);
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