/**
 * \file
 * \~English \brief Implementation of the security module FlightController component main loop.
 * \~Russian \brief Реализация основного цикла компонента FlightController модуля безопасности.
 */

#include "../include/flight_controller.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"
#include "../../shared/include/ipc_messages_logger.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <thread>

/** \cond */
#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000

char boardId[32] = {0};
uint32_t sessionDelay;
std::thread sessionThread;
/** \endcond */

/**
 * \~English Auxiliary procedure. Adds drone ID to request and signs it, sends message to the ATM server
 * and checks the authenticity of the received response.
 * \param[in] method Request to the ATM server. "/api/method" form is expected.
 * Drone ID and signature will be added.
 * \param[out] response Significant part of the response from the server. Authenticity is checked.
 * \param[in] errorMessage String identifier of request. This will be displayed in error text on occured error in the procedure.
 * \param[in] delay Delay in seconds before request resend if an error occurs.
 * \return Returns 1 on successful send, 0 otherwise.
 * \~Russian Вспомогательная процедура. Снабжает запрос идентификатором дрона,
 * подписывает его, отправляет на сервер ОРВД и проверяет аутентичность полученного ответа.
 * \param[in] method Запрос к серверу ОРВД. Ожидается вид "/api/method".
 * Идентификатор дрона и подпись будут добавлены.
 * \param[out] response Значимая часть ответа от сервера. Аутентичность проверена.
 * \param[in] errorMessage Строковый идентификатор отправляемого запроса, который будет отображен в тексте ошибки при
 * возникновении ошибок во время процедуры.
 * \param[in] delay Задержка в сек. перед повторной отправкой запроса при возникновении ошибок при отправке.
 * \return Возвращает 1 при успешной отправке, иначе -- 0.
 */
int sendSignedMessage(char* method, char* response, char* errorMessage, uint8_t delay) {
    char logBuffer[256] = {0};
    char signature[257] = {0};
    char request[512] = {0};
    if (strstr(method, "?"))
        snprintf(request, 512, "%s&id=%s", method, boardId);
    else
        snprintf(request, 512, "%s?id=%s", method, boardId);

    while (!signMessage(request, signature, 257)) {
        snprintf(logBuffer, 256, "Failed to sign %s message at Credential Manager. Trying again in %ds", errorMessage, delay);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(delay);
    }
    snprintf(request, 512, "%s&sig=0x%s", request, signature);

    while (!sendRequest(request, response, 4096)) {
        snprintf(logBuffer, 256, "Failed to send %s request through Server Connector. Trying again in %ds", errorMessage, delay);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(delay);
    }
    if (!strcmp(response, "TIMEOUT"))
        return 0;

    uint8_t authenticity = 0;
    while (!checkSignature(response, authenticity) || !authenticity) {
        snprintf(logBuffer, 256, "Failed to check signature of %s response received through Server Connector. Trying again in %ds", errorMessage, delay);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(delay);
    }

    return 1;
}

/**
 * \~English Procedure that checks the flight status at the ATM server. Check includes permission to continue flight, changes in
 * no-flight areas and time until next communication session.
 * \~Russian Процедура, запрашивающая статус полета от сервера ОРВД. Статус включает в себя разрешение на продолжение полета,
 * изменения в бесполетных зонах и время до следующей коммуникации с сервером.
 */
void serverSession() {
    sleep(sessionDelay);
    char response[4096] = {0};
    while (true) {
        sendSignedMessage("/api/flight_info", response, "session", RETRY_DELAY_SEC);
        //If connection is failed, flight must be paused
        //Processing status of flight
        if (strstr(response, "$Flight -1$")) {
            logEntry("Emergency stop request is received. Disabling motors", ENTITY_NAME, LogLevel::LOG_INFO);
            if (!enableBuzzer())
                logEntry("Failed to enable buzzer", ENTITY_NAME, LogLevel::LOG_WARNING);
            while (!setKillSwitch(false)) {
                logEntry("Failed to forbid motor usage. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
                sleep(1);
            }
        }
        //The response has two other possible options:
        //  "$Flight 1$" that requires to pause flight and remain landed
        //  "$Flight 0$" that requires to resume flight and keep flying
        //Implementation is required to be done

        //Processing no-flight areas updates
        char receivedHash[65] = {0};
        char* calculatedHash = getNoFlightAreasHash();
        parseNoFlightAreasHash(response, receivedHash, 65);
        if (strcmp(receivedHash, calculatedHash)) {
            logEntry("No-flight areas on the server were updated", ENTITY_NAME, LogLevel::LOG_INFO);
            char hash[65] = {0};
            char responseDelta[4096] = {0};
            strcpy(hash, receivedHash);
            sendSignedMessage("/api/get_forbidden_zones_delta", responseDelta, "no-flight areas", RETRY_DELAY_SEC);
            int successful = updateNoFlightAreas(responseDelta);
            if (successful) {
                calculatedHash = getNoFlightAreasHash();
                successful = !strcmp(hash, calculatedHash);
            }
            if (!successful) {
                logEntry("Completely redownloading no-flight areas", ENTITY_NAME, LogLevel::LOG_INFO);
                deleteNoFlightAreas();
                sendSignedMessage("/api/get_all_forbidden_zones", responseDelta, "no-flight areas", RETRY_DELAY_SEC);
                loadNoFlightAreas(responseDelta);
            }
            printNoFlightAreas();
        }

        //Processing delay until next session
        sessionDelay = parseDelay(strstr(response, "$Delay "));

        sleep(sessionDelay);
    }
}

/**
 * \~English Auxiliary procedure. Asks the ATM server to approve new mission and parses its response.
 * \param[in] mission New mission in string format.
 * \param[out] result ATM server response: 1 if mission approved, 0 otherwise.
 * \return Returns 1 on successful send, 0 otherwise.
 * \~Russian Вспомогательная процедура. Просит у сервера ОРВД одобрения новой миссии и обрабатывает ответ.
 * \param[in] mission Новая миссия в виде строки.
 * \param[out] result Ответ сервера ОРВД: 1 при одобрении миссии, иначе -- 0.
 * \return Возвращает 1 при успешной отправке, иначе -- 0.
 */
int askForMissionApproval(char* mission, int& result) {
    int requestSize = 512 + strlen(mission);

    char signature[257] = {0};
    char *request = (char*)malloc(requestSize);
    snprintf(request, requestSize, "/api/nmission?id=%s&mission=%s", boardId, mission);

    if (!signMessage(request, signature, 257)) {
        logEntry("Failed to sign New Mission request at Credential Manager", ENTITY_NAME, LogLevel::LOG_WARNING);
        free(request);
        return 0;
    }
    snprintf(request, 512, "%s&sig=0x%s", request, signature);

    char response[4096] = {0};
    if (!sendRequest(request, response, 4096)) {
        logEntry("Failed to send New Mission request through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
        free(request);
        return 0;
    }
    free(request);

    uint8_t authenticity = 0;
    while (!checkSignature(response, authenticity) || !authenticity) {
        logEntry("Failed to check signature of New Mission request response received through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    if (strstr(response, "$Approve 0#") != NULL)
        result = 1;
    else if (strstr(response, "$Approve 1#") != NULL)
        result = 0;
    else {
        logEntry("Failed to parse server response on New Mission request", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    return 1;
}

/**
 * \~English Security module main loop. Waits for all other components to initialize. Authenticates
 * on the ATM server and receives the mission from it. After a mission and an arm request from the autopilot
 * are received, requests permission to take off from the ATM server. On receive supplies power to motors.
 * Then flight control must be performed.
 * \return Returns 1 on completion with no errors.
 * \~Russian Основной цикл модуля безопасности. Ожидает инициализации всех остальных компонентов. Аутентифицируется
 * на сервере ОРВД и получает от него миссию. После получения миссии и запроса на арминг от автопилота, запрашивает разрешение
 * на взлет у сервера ОРВД. При его получении подает питание на двигатели. Далее должен выполняться контроль полета.
 * \return Возвращает 1 при завершении без ошибок.
 */
int main(void) {
    char logBuffer[256] = {0};
    char responseBuffer[4096] = {0};

    while (!waitForInit("logger_connection", "Logger")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Logger. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("periphery_controller_connection", "PeripheryController")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Periphery Controller. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("autopilot_connector_connection", "AutopilotConnector")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Autopilot Connector. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("navigation_system_connection", "NavigationSystem")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Navigation System. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("server_connector_connection", "ServerConnector")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Server Connector. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("credential_manager_connection", "CredentialManager")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Credential Manager. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    logEntry("Board is initialized", ENTITY_NAME, LogLevel::LOG_INFO);

    if (!setKillSwitch(true))
        logEntry("Failed to open kill-switch", ENTITY_NAME, LogLevel::LOG_WARNING);
    else
        logEntry("Kill-switch is opened: arm can be done", ENTITY_NAME, LogLevel::LOG_INFO);

    if (!setCargoLock(true))
        logEntry("Failed to open cargo lock", ENTITY_NAME, LogLevel::LOG_WARNING);
    else
        logEntry("Cargo lock is opened: cargo can be dropped", ENTITY_NAME, LogLevel::LOG_INFO);

    if (!enableBuzzer())
        logEntry("Failed to enable buzzer at Periphery Controller", ENTITY_NAME, LogLevel::LOG_WARNING);
    else
        logEntry("Buzzer is enabled", ENTITY_NAME, LogLevel::LOG_INFO);

    int32_t lat, lng, alt;
    float speed, pressure;
    while (true) {
        getCoords(lat, lng, alt);
        snprintf(logBuffer, 256, "Coordinates: %d.%d°, %d.%d°",
            lat / 10000000, abs(lat % 10000000), lng / 10000000, abs(lng % 10000000));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
        getEstimatedPressure(pressure);
        snprintf(logBuffer, 256, "Pressure: %f Pa (%d.%d m)", pressure, alt / 100, abs(alt % 100));
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
        getEstimatedSpeed(speed);
        snprintf(logBuffer, 256, "Speed: %f m/s", speed);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);
        sleep(1);
    }

    return EXIT_SUCCESS;
}