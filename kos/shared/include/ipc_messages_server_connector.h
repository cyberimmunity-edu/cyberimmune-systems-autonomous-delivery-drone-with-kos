/**
 * \file
 * \~English \brief Declaration of wrapper methods that send IPC messages to ServerConnector component.
 * \~Russian \brief Объявление методов-оберток для отправки IPC-сообщений компоненту ServerConnector.
 */

#pragma once

/**
 * \~English Returns drone ID.
 * \note In online build ID is the MAC address of the "en0" interface.
 * In offline build ID is "offline-drone" string.
 * \param[out] id Drone ID.
 * \return Returns 1 on successful ID retrieve, 0 otherwise.
 * \~Russian Возвращает ID дрона.
 * \note При online-сборке идентификатором является значение MAC-адреса интерфейса "en0".
 * При offline-сборке идентфикатором является строка "offline-drone".
 * \param[out] id Идентификатор дрона.
 * \return Возвращает 1 при успешном получении ID дрона, иначе -- 0.
 */
int getBoardId(char* id);
/**
 * \~English Performs a request-response procedure with the ATM server.
 * \param[in] query Request to the ATM server. Requires only a meaningful signed part of the request.
 * \param[out] response Response from the ATM server. Only a meaningfuk part of the received response is returned.
 * \return Returns 1 on successful message exchange with the ATM server, 0 otherwise.
 * \return Возвращает 1, если обмен сообщениями с сервером был успешен, иначе -- 0.
 * \~Russian Выполняет процедуру отправки запроса -- получения ответа с сервером ОРВД.
 * \param[in] query Запрос к серверу ОРВД. Ожидается только значимая подписанная часть запроса.
 * \param[out] response Ответ от сервера ОРВД. Возвращается лишь значимая часть ответа, полученного от сервера ОРВД.
 * \return Возвращает 1, если обмен сообщениями с сервером был успешен, иначе -- 0.
 */
int sendRequest(char* query, char* response);