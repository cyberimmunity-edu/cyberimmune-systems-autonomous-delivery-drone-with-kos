/**
 * \file
 * \~English \brief Declaration of wrapper methods that send IPC messages to Logger component.
 * \~Russian \brief Объявление методов-оберток для отправки IPC-сообщений компоненту Logger.
 */

#pragma once

#include <stdint.h>

/** \cond */
#define MAX_LOG_BUFFER 256
/** \endcond */

/**
 * \~English Available log levels. Match the levels of spdlog library.
 * \~Russian Использующиеся уровни логирования. Совпадают с уровнями библиотеки spdlog.
 */
enum LogLevel { LOG_TRACE, LOG_DEBUG, LOG_INFO, LOG_WARNING, LOG_ERROR, LOG_CRITICAL };

/**
 * \~English Writes a message to the log.
 * \note Each logged message is printed to the console. The log is stored on the memory card in "logs/flight_controller.log".
 * \param[in] entry Message to log. The maximum size is 256 bytes.
 * \param[in] entity Component name, that adds the message.
 * \param[in] level Log level of the message.
 * \return Returns 1 on successful logging, 0 otherwise
 * \~Russian Выполняет запись сообщения в лог.
 * \note Каждое записанное в лог сообщение выводится в консоль. Лог сообщений хранится на карте памяти в "logs/flight_controller.log".
 * \param[in] entry Логируемое сообщение. Максимальный размер -- 256 байт.
 * \param[in] entity Имя компонента, производящего логирование.
 * \param[in] level Уровень важности логируемого сообщения.
 * \return Возвращает 1 при успешном добавлении записи в лог, иначе -- 0.
 */
int logEntry(char* entry, char* entity, LogLevel level);