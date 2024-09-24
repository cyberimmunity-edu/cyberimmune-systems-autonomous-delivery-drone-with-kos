/**
 * \file
 * \~English
 * \brief Declaration of LoggerInterface IDL-interface methods.
 * \details Interaction between Logger component and other components
 * of the security module is done via IPC message sent through LoggerInterface IDL-interface.
 * The file contains declaration of the C++ methods of this interface. All methods have a standard set of parameters.
 * \param[in] self Pointer to the LoggerInterface structure.
 * \param[in] req Pointer to a structure with fixed-length data received in an IPC message.
 * \param[in] reqArena Pointer to a structure with variable length data received in an IPC message.
 * \param[out] res Pointer to a structure with fixed-length data sent in an IPC response.
 * \param[out] resArena Pointer to a structure with variable length data sent in an IPC response.
 * \return Returns NK_EOK if message and response were successfully exchanged.
 *
 * \~Russian
 * \brief Объявление методов IDL-интерфейса LoggerInterface.
 * \details Взаимодействие с компонентом Logger другими компонентами
 * модуля безопасности осуществляется через отправку IPC-сообщение через
 * IDL-интерфейс LoggerInterface. В этом файле объявлены методы этого интерфейса
 * на языке C++. Все методы имеют стандартный набор параметров.
 * \param[in] self Указатель на структуру, соответствующую интерфейсу LoggerInterface.
 * \param[in] req Указатель на структуру с полученными в IPC-сообщении данными фиксированной длины.
 * \param[in] reqArena Указатель на структуру с полученными в IPC-сообщении данными произвольной длины.
 * \param[out] res Указатель на структуру с отправляемыми в IPC-ответе данными фиксированной длины.
 * \param[out] resArena Указатель на структуру с отправляемыми в IPC-ответе данными произвольной длины.
 * \return Возвращает NK_EOK в случае успешного обмена сообщением и ответом.
 */

#pragma once

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/LoggerInterface.idl.h>

/**
 * \~English IPC message handler. See \ref logEntry.
 * \~Russian Обработчик IPC-сообщения. См. \ref logEntry.
 */
nk_err_t LogImpl(struct LoggerInterface *self,
                    const LoggerInterface_Log_req *req, const struct nk_arena *reqArena,
                    LoggerInterface_Log_res *res, struct nk_arena *resArena);

/**
 * \~English Creates an LoggerInterface C++ interface and maps its methods to IPC message handlers.
 * \~Russian Создает C++ интерфейс LoggerInterface и сопоставляет его методы с обработчиками IPC-сообщений.
 */
static struct LoggerInterface *CreateLoggerInterfaceImpl(void) {
    static const struct LoggerInterface_ops Ops = {
        .Log = LogImpl
    };

    static LoggerInterface obj = {
        .ops = &Ops
    };

    return &obj;
}