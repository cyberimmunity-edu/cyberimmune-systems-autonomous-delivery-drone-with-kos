/**
 * \file
 * \~English
 * \brief Declaration of CredentialManagerInterface IDL-interface methods.
 * \details Interaction between CredentialManager component and other components
 * of the security module is done via IPC message sent through CredentialManagerInterface IDL-interface.
 * The file contains declaration of the C++ methods of this interface. All methods have a standard set of parameters.
 * \param[in] self Pointer to the CredentialManagerInterface structure.
 * \param[in] req Pointer to a structure with fixed-length data received in an IPC message.
 * \param[in] reqArena Pointer to a structure with variable length data received in an IPC message.
 * \param[out] res Pointer to a structure with fixed-length data sent in an IPC response.
 * \param[out] resArena Pointer to a structure with variable length data sent in an IPC response.
 * \return Returns NK_EOK if message and response were successfully exchanged.
 *
 * \~Russian
 * \brief Объявление методов IDL-интерфейса CredentialManagerInterface.
 * \details Взаимодействие с компонентом CredentialManager другими компонентами
 * модуля безопасности осуществляется через отправку IPC-сообщение через
 * IDL-интерфейс CredentialManagerInterface. В этом файле объявлены методы этого интерфейса
 * на языке C++. Все методы имеют стандартный набор параметров.
 * \param[in] self Указатель на структуру, соответствующую интерфейсу CredentialManagerInterface.
 * \param[in] req Указатель на структуру с полученными в IPC-сообщении данными фиксированной длины.
 * \param[in] reqArena Указатель на структуру с полученными в IPC-сообщении данными произвольной длины.
 * \param[out] res Указатель на структуру с отправляемыми в IPC-ответе данными фиксированной длины.
 * \param[out] resArena Указатель на структуру с отправляемыми в IPC-ответе данными произвольной длины.
 * \return Возвращает NK_EOK в случае успешного обмена сообщением и ответом.
 */

#pragma once

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/CredentialManagerInterface.idl.h>

/**
 * \~English IPC message handler. See \ref signMessage.
 * \~Russian Обработчик IPC-сообщения. См. \ref signMessage.
 */
nk_err_t SignMessageImpl(struct CredentialManagerInterface *self,
                    const CredentialManagerInterface_SignMessage_req *req, const struct nk_arena *reqArena,
                    CredentialManagerInterface_SignMessage_res *res, struct nk_arena *resArena);
/**
 * \~English IPC message handler. See \ref checkSignature.
 * \~Russian Обработчик IPC-сообщения. См. \ref checkSignature.
 */
nk_err_t CheckSignatureImpl(struct CredentialManagerInterface *self,
                    const CredentialManagerInterface_CheckSignature_req *req, const struct nk_arena *reqArena,
                    CredentialManagerInterface_CheckSignature_res *res, struct nk_arena *resArena);

/**
 * \~English Creates an CredentialManagerInterface C++ interface and maps its methods to IPC message handlers.
 * \~Russian Создает C++ интерфейс CredentialManagerInterface и сопоставляет его методы с обработчиками IPC-сообщений.
 */
static struct CredentialManagerInterface *CreateCredentialManagerInterfaceImpl(void) {
    static const struct CredentialManagerInterface_ops Ops = {
        .SignMessage = SignMessageImpl, .CheckSignature = CheckSignatureImpl
    };

    static CredentialManagerInterface obj = {
        .ops = &Ops
    };

    return &obj;
}