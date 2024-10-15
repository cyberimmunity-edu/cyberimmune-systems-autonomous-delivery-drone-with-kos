/**
 * \file
 * \~English
 * \brief Declaration of ServerConnectorInterface IDL-interface methods.
 * \details Interaction between ServerConnector component and other components
 * of the security module is done via IPC message sent through ServerConnectorInterface IDL-interface.
 * The file contains declaration of the C++ methods of this interface. All methods have a standard set of parameters.
 * \param[in] self Pointer to the ServerConnectorInterface structure.
 * \param[in] req Pointer to a structure with fixed-length data received in an IPC message.
 * \param[in] reqArena Pointer to a structure with variable length data received in an IPC message.
 * \param[out] res Pointer to a structure with fixed-length data sent in an IPC response.
 * \param[out] resArena Pointer to a structure with variable length data sent in an IPC response.
 * \return Returns NK_EOK if message and response were successfully exchanged.
 *
 * \~Russian
 * \brief Объявление методов IDL-интерфейса ServerConnectorInterface.
 * \details Взаимодействие с компонентом ServerConnector другими компонентами
 * модуля безопасности осуществляется через отправку IPC-сообщение через
 * IDL-интерфейс ServerConnectorInterface. В этом файле объявлены методы этого интерфейса
 * на языке C++. Все методы имеют стандартный набор параметров.
 * \param[in] self Указатель на структуру, соответствующую интерфейсу ServerConnectorInterface.
 * \param[in] req Указатель на структуру с полученными в IPC-сообщении данными фиксированной длины.
 * \param[in] reqArena Указатель на структуру с полученными в IPC-сообщении данными произвольной длины.
 * \param[out] res Указатель на структуру с отправляемыми в IPC-ответе данными фиксированной длины.
 * \param[out] resArena Указатель на структуру с отправляемыми в IPC-ответе данными произвольной длины.
 * \return Возвращает NK_EOK в случае успешного обмена сообщением и ответом.
 */

#pragma once

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/ServerConnectorInterface.idl.h>

/**
 * \~English IPC message handler. See \ref getBoardId.
 * \~Russian Обработчик IPC-сообщения. См. \ref getBoardId.
 */
nk_err_t GetBoardIdImpl(struct ServerConnectorInterface *self,
                    const ServerConnectorInterface_GetBoardId_req *req, const struct nk_arena *reqArena,
                    ServerConnectorInterface_GetBoardId_res *res, struct nk_arena *resArena);
/**
 * \~English IPC message handler. See \ref sendRequest.
 * \~Russian Обработчик IPC-сообщения. См. \ref sendRequest.
 */
nk_err_t SendRequestImpl(struct ServerConnectorInterface *self,
                    const ServerConnectorInterface_SendRequest_req *req, const struct nk_arena *reqArena,
                    ServerConnectorInterface_SendRequest_res *res, struct nk_arena *resArena);
/**
 * \~English IPC message handler. See \ref sendRequest.
 * \~Russian Обработчик IPC-сообщения. См. \ref sendRequest.
 */
nk_err_t PublishMessageImpl(struct ServerConnectorInterface *self,
                    const ServerConnectorInterface_PublishMessage_req *req, const struct nk_arena *reqArena,
                    ServerConnectorInterface_PublishMessage_res *res, struct nk_arena *resArena);

/**
 * \~English Creates an ServerConnectorInterface C++ interface and maps its methods to IPC message handlers.
 * \~Russian Создает C++ интерфейс ServerConnectorInterface и сопоставляет его методы с обработчиками IPC-сообщений.
 */
static struct ServerConnectorInterface *CreateServerConnectorInterfaceImpl(void) {
    static const struct ServerConnectorInterface_ops Ops = {
        .GetBoardId = GetBoardIdImpl, .SendRequest = SendRequestImpl, .PublishMessage = PublishMessageImpl
    };

    static ServerConnectorInterface obj = {
        .ops = &Ops
    };

    return &obj;
}