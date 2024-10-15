/**
 * \file
 * \~English \brief Declaration of wrapper methods that send IPC messages to ServerConnector component.
 * \~Russian \brief Объявление методов-оберток для отправки IPC-сообщений компоненту ServerConnector.
 */

#pragma once
#include <stdint.h>

/**
 * \~English Returns drone ID.
 * \note In online build ID is the MAC address of the "en0" interface.
 * In offline build ID is "00:00:00:00:00:00".
 * \param[out] id Drone ID.
 * \return Returns 1 on successful ID retrieve, 0 otherwise.
 * \~Russian Возвращает ID дрона.
 * \note При online-сборке идентификатором является значение MAC-адреса интерфейса "en0".
 * При offline-сборке идентфикатором является "00:00:00:00:00:00".
 * \param[out] id Идентификатор дрона.
 * \return Возвращает 1 при успешном получении ID дрона, иначе -- 0.
 */
int getBoardId(char* id);
/**
 * \~English Performs a request-response procedure with the ATM server.
 * \param[in] query Request to the ATM server. Requires only a meaningful signed part of the request.
 * \param[out] response Response from the ATM server. Only a meaningfuд part of the received response is returned. If connection
 * is timed out, response will contain "TIMEOUT" string.
 * \param[in] responseSize Size of output response buffer.
 * \return Returns 1 on successful message exchange with the ATM server or on connection timeout, 0 otherwise.
 * \~Russian Выполняет процедуру отправки запроса -- получения ответа с сервером ОРВД.
 * \param[in] query Запрос к серверу ОРВД. Ожидается только значимая подписанная часть запроса.
 * \param[out] response Ответ от сервера ОРВД. Возвращается лишь значимая часть ответа, полученного от сервера ОРВД. Если
 * истекло время ожидания подключения к серверу, ответ будет содержать строку "TIMEOUT".
 * \param[in] responseSize Размер буфера, куда записывается ответ от сервера.
 * \return Возвращает 1, если обмен сообщениями с сервером был успешен или истекло время ожидания подключения к серверу, иначе -- 0.
 */
int sendRequest(char* query, char* response, uint32_t responseSize);
/**
 * \~English Performs a message publication via MQTT-protocol.
 * \param[in] topic Name of topic to publish message to.
 * \param[in] publication Message to publish.
 * \return Returns 1 on successful message publish, 0 otherwise.
 * \~Russian Производит публикацию сообщения по MQTT-протоколу.
 * \param[in] topic Тема, в которую будет опубликовано сообщение.
 * \param[in] publication Сообщение, которое будет опубликовано.
 * \return Возвращает 1, если сообщениями было успешно опубликовано, иначе -- 0.
 */
int publishMessage(char* topic, char* publication);