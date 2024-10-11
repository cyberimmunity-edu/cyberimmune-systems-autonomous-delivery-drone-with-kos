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
    if (strstr(query, "/api/kill_switch?") != NULL) {
        if (responseSize < 16) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$KillSwitch: 1#", 16);
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
        strncpy(response, "$FlightMission H53.1019446_107.3774394_846.22&T5.0&W53.1020863_107.3774180_5.0&W53.1021926_107.3775065_5.0&W53.1023102_107.3776701_5.0&W53.1023682_107.3779464_5.0&W53.1023923_107.3782736_5.0&W53.1023279_107.3786089_5.0&W53.1021991_107.3787698_5.0&D2.0&S5.0_1200.0&W53.1020284_107.3788181_5.0&W53.1018818_107.3786679_5.0&W53.1018206_107.3782790_5.0&W53.1017900_107.3778149_5.0&W53.1018480_107.3775575_5.0&W53.1019446_107.3774394_5.0&L53.1019446_107.3774394_846.22#", 464);
    }
    else if ((strstr(query, "/api/arm?") != NULL) || (strstr(query, "/api/fly_accept?") != NULL)) {
        if (responseSize < 9) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$Arm: 0#", 9);
    }
    else if (strstr(query, "/api/get_all_forbidden_zones?") != NULL) {
        if (responseSize < 191) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$ForbiddenZones 1&test_area&7&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299&53.1020767_107.3784882&53.1019962_107.3782709&53.1019189_107.3779812&53.1019656_107.3777157#", 191);
    }
    else if (strstr(query, "/api/forbidden_zones_hash?") != NULL) {
        if (responseSize < 86) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$ForbiddenZonesHash bdbac12490e31f4d0b3b5ee45a37dea125a35510b100e48d79e143eb3f419205#", 86);
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