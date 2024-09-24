/**
 * \file
 * \~English \brief Declaration of wrapper methods that send IPC messages
 * to any security module component (except for FlightController).
 * \~Russian \brief Объявление методов-оберток, общих для всех компонентов модуля безопасности (кроме FlightController).
 */

#pragma once

/**
 * \~English Waits for the specified component to initialize.
 * \param[in] connection Connection name used by the component. Has the form "component_name_connection".
 * \param[in] receiverEntity Component name to wait for initialization. Has the form "ComponentName".
 * \return Returns 1 on successful component initialization, 0 otherwise.
 * \~Russian Ожидает инициализации указанного компонента.
 * \param[in] connection Имя соединения, использующегося компонентом. Имеет вид "название_компонента_connection".
 * \param[in] receiverEntity Имя компонента, чья инициализация ожидается. Имеет вид "НазваниеКомпонента".
 * \return Возвращает 1, если компонент был успешно инициализирован, иначе -- 0.
 */
int waitForInit(const char* connection, const char* receiverEntity);