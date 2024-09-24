/**
 * \file
 * \~English \brief Declaration of Initialization IDL-interface methods and methods
 * for wrappers over IPC message handlers.
 * \~Russian \brief Объявление методов IDL-интерфейса Initialization и методов, необходимых
 * оберткам над обработчиками IPC-сообщений.
 */

#pragma once

#include <coresrv/nk/transport-kos.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/Initialization.idl.h>

/**
 * \~English Initializes interface for IPC message send.
 * Used by all wrappers over IPC message send methods.
 * \param[in] connection Name of the connection used by IPC message receiver component.
 * \param[in] endpoint Full name of IDL method in "project.entity.endpoint" format.
 * \param[out] transport KasperskyOS transport structure.
 * \param[out] riid Interface realisation ID.
 * \~Russian Инициализирует интерфейс для отправки IPC-сообщений.
 * Используется всеми обертками над методами отправки IPC-сообщений.
 * \param[in] connection Имя соединения, используемого компонентом-получателем IPC-сообщения.
 * \param[in] endpoint Полное имя IDL-метода в формате "проект.сущность.конечная точка".
 * \param[out] transport Транспортная структура KasperskyOS.
 * \param[out] riid ID реализации интерфейса.
 */
void initSenderInterface(const char* connection, const char* endpoint, NkKosTransport &transport, nk_iid_t &riid);
/**
 * \~English Initializes interface for IPC message receive.
 * Used by all components with IPC-message handling main loops.
 * \param[in] connection Name of the connection used by IPC message receiver component.
 * \param[out] transport KasperskyOS transport structure.
 * \~Russian Инициализирует интерфейс для получения IPC-сообщений.
 * Используется всеми компонентами с основным циклом в виде обработки IPC-сообщений.
 * \param[in] connection Имя соединения, используемого компонентом-получателем IPC-сообщения.
 * \param[out] transport Транспортная структура KasperskyOS.
 */
void initReceiverInterface(const char* connection, NkKosTransport &transport);

/**
 * \~English IPC message handler. See \ref waitForInit.
 * \param[in] self Pointer to the Initialization structure.
 * \param[in] req Pointer to a structure with fixed-length data received in an IPC message.
 * \param[in] reqArena Pointer to a structure with variable length data received in an IPC message.
 * \param[out] res Pointer to a structure with fixed-length data sent in an IPC response.
 * \param[out] resArena Pointer to a structure with variable length data sent in an IPC response.
 * \return Returns NK_EOK if message and response were successfully exchanged.
 *
 * \~Russian Обработчик IPC-сообщения. См. \ref waitForInit.
 * \param[in] self Указатель на структуру, соответствующую интерфейсу Initialization.
 * \param[in] req Указатель на структуру с полученными в IPC-сообщении данными фиксированной длины.
 * \param[in] reqArena Указатель на структуру с полученными в IPC-сообщении данными произвольной длины.
 * \param[out] res Указатель на структуру с отправляемыми в IPC-ответе данными фиксированной длины.
 * \param[out] resArena Указатель на структуру с отправляемыми в IPC-ответе данными произвольной длины.
 * \return Возвращает NK_EOK в случае успешного обмена сообщением и ответом.
 */
nk_err_t WaitForInitImpl(struct Initialization *self,
    const Initialization_WaitForInit_req *req, const struct nk_arena *reqArena,
    Initialization_WaitForInit_res *res, struct nk_arena *resArena);

/**
 * \~English Creates an Initialization C++ interface and maps its methods to IPC message handlers.
 * \~Russian Создает C++ интерфейс Initialization и сопоставляет его методы с обработчиками IPC-сообщений.
 */
static struct Initialization *CreateInitializationImpl(void) {
    static const struct Initialization_ops Ops = {
        .WaitForInit = WaitForInitImpl
    };

    static Initialization obj = {
        .ops = &Ops
    };

    return &obj;
}