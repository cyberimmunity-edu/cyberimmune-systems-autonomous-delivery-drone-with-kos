/**
 * \file
 * \~English \brief Implementation of the security module FlightController component main loop.
 * \~Russian \brief Реализация основного цикла компонента FlightController модуля безопасности.
 */

#include "../include/mission.h"
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

/** \cond */
#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000

char boardId[32] = {0};
/** \endcond */

/**
 * \~English Auxiliary procedure. Adds drone ID to request and signs it, sends message to the ATM server
 * and checks the authenticity of the received response.
 * \param[in] method Request to the ATM server. "/api/query&param=value" form is expected/
 * Drone ID and signature will be added.
 * \param[out] response Significant part of the response from the server. Authenticity is checked.
 * \param[in] errorMessage String identifier of request. This will be displayed in error text on occured error in the procedure.
 * \param[in] delay Delay in seconds before request resend if an error occurs.
 * \return Returns 1 on successful send, 0 otherwise.
 * \~Russian Вспомогательная процедура. Снабжает запрос идентификатором дрона,
 * подписывает его, отправляет на сервер ОРВД и проверяет аутентичность полученного ответа.
 * \param[in] method Запрос к серверу ОРВД. Ожидается вид "/api/query&param=value".
 * Идентификатор дрона и подпись будут добавлены.
 * \param[out] response Значимая часть ответа от сервера. Аутентичность проверена.
 * \param[in] errorMessage Строковый идентификатор отправляемого запроса, который будет отображен в тексте ошибки при
 * возникновении ошибок во время процедуры.
 * \param[in] delay Задержка в сек. перед повторной отправкой запроса при возникновении ошибок при отправке.
 * \return Возвращает 1 при успешной отправке, иначе -- 0.
 */
int sendSignedMessage(char* method, char* response, char* errorMessage, uint8_t delay) {
    char message[513] = {0};
    char signature[257] = {0};
    char request[1025] = {0};
    char logBuffer[257] = {0};
    snprintf(message, 512, "%s?id=%s", method, boardId);

    while (!signMessage(message, signature, 257)) {
        snprintf(logBuffer, 256, "Failed to sign %s message at Credential Manager. Trying again in %ds", errorMessage, delay);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(delay);
    }
    snprintf(request, 1024, "%s&sig=0x%s", message, signature);

    while (!sendRequest(request, response, 1025)) {
        snprintf(logBuffer, 256, "Failed to send %s request through Server Connector. Trying again in %ds", errorMessage, delay);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(delay);
    }

    uint8_t authenticity = 0;
    while (!checkSignature(response, authenticity) || !authenticity) {
        snprintf(logBuffer, 256, "Failed to check signature of %s response received through Server Connector. Trying again in %ds", errorMessage, delay);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(delay);
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
    char logBuffer[257] = {0};

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