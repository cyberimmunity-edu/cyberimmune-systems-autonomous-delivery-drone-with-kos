/**
 * \file
 * \~English
 * \brief Declaration of methods for the AFCS server communication.
 * \details The file contains declaration of methods, that provide
 * interaction between the security module and the AFCS server.
 *
 * \~Russian
 * \brief Объявление методов для взаимодействия с сервером СУПА.
 * \details В файле объявлены публичныe методы, необходимые
 * для взаимодействия между модулем безопасности и сервером СУПА.
 */

#pragma once

#include "../../shared/include/ipc_messages_logger.h"

/**
 * \~English Initializes softwate and hardware components required
 * for the AFCS server communication establishment.
 * \return Returns 1 on successful initialization, 0 otherwise.
 * \~Russian Производит инициализацию программных и аппаратных компонентов, необходимых
 * для установки связи с сервером СУПА.
 * \return Возвращает 1 при успешной инициализации, 0 -- иначе.
 */
int initServerConnector();

/**
 * \~English Saves given drone ID.
 * \param[in] id New drone ID.
 * \~Russian Запоминает поданный идентификатор дрона.
 * \param[in] id Новый идентификатор дрона.
 */
void setBoardName(char* id);
/**
 * \~English Returns saved drone ID.
 * \return Drone ID.
 * \~Russian Возвращает сохраненный идентификатор дрона.
 * \return Идентификатор дрона.
 */
char* getBoardName();

/**
 * \~English Requests the AFCS server and receives a response from it.
 * \param[in] query Request to the AFCS server.
 * \param[out] response Response from the AFCS server. Only a meaningful part of the response is returned. If connection is timed
 * out, response will contain "TIMEOUT" string.
 * \param[in] responseSize Size of output response buffer.
 * \return Returns 1 on successful message exchange with the AFCS server or on connection timeout, 0 otherwise.
 * \~Russian Отправляет запрос на сервер СУПА и получает от него ответ.
 * \param[in] query Запрос к серверу СУПА.
 * \param[out] response Ответ от сервера СУПА. Возвращается лишь значимая часть ответа. Если истекло время ожидания подключения
 * к серверу, ответ будет содержать строку "TIMEOUT".
 * \param[in] responseSize Размер буфера, куда записывается ответ от сервера.
 * \return Возвращает 1, если обмен сообщениями с сервером был успешен или истекло время ожидания подключения к серверу, иначе -- 0.
 */
int requestServer(char* query, char* response, uint32_t responseSize);
/**
 * \~English Publish the message via MQTT-protocol.
 * \param[in] topic Name of topic to publish message to.
 * \param[in] publication Message to publish.
 * \return Returns 1 on successful message publish, 0 otherwise.
 * \~Russian Публикует сообщение по MQTT-протоколу.
 * \param[in] topic Тема, в которую будет опубликовано сообщение.
 * \param[in] publication Сообщение, которое будет опубликовано.
 * \return Возвращает 1, если сообщениями было успешно опубликовано, иначе -- 0.
 */
int publish(char* topic, char* publication);