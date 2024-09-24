/**
 * \file
 * \~English
 * \brief Declaration of methods for work with a log.
 * \details The file contains declaration of methods,
 * that are required to log messages.
 * These methods rely on spdlog library.
 *
 * \~Russian
 * \brief Объявление методов для работы с логом.
 * \details В файле объявлены публичныe методы, необходимые
 * для логирования сообщений.
 * Эти методы полагаются на библиотеку spdlog.
 */

#pragma once

/**
 * \~English Создает лог, сохраняющий сообщения в файл "logs/flight_controller.log" и дублирующий их в консоль.
 * \return Returns 1 on successful log creation, 0 otherwise.
 * \~Russian Создает лог, сохраняющий сообщения в файл "logs/flight_controller.log" и дублирующий их в консоль.
 * \return Возвращает 1 при успешном создании лога, иначе -- 0.
 */
int createLog();
/**
 * \~English Writes a message to the log. Each logged message is printed to the console.
 * \param[in] entry Message to log.
 * \param[in] level Log level of the message.
 * \return Returns 1 on successful logging, 0 otherwise
 * \~Russian Выполняет запись сообщения в лог. Каждое записанное в лог сообщение выводится в консоль.
 * \param[in] entry Логируемое сообщение.
 * \param[in] level Уровень важности логируемого сообщения.
 * \return Возвращает 1 при успешном добавлении записи в лог, иначе -- 0.
 */
int addLogEntry(char* entry, int level);