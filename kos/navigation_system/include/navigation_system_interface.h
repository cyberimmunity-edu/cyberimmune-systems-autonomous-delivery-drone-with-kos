/**
 * \file
 * \~English
 * \brief Declaration of NavigationSystemInterface IDL-interface methods.
 * \details Interaction between NavigationSystem component and other components
 * of the security module is done via IPC message sent through NavigationSystemInterface IDL-interface.
 * The file contains declaration of the C++ methods of this interface. All methods have a standard set of parameters.
 * \param[in] self Pointer to the NavigationSystemInterface structure.
 * \param[in] req Pointer to a structure with fixed-length data received in an IPC message.
 * \param[in] reqArena Pointer to a structure with variable length data received in an IPC message.
 * \param[out] res Pointer to a structure with fixed-length data sent in an IPC response.
 * \param[out] resArena Pointer to a structure with variable length data sent in an IPC response.
 * \return Returns NK_EOK if message and response were successfully exchanged.
 *
 * \~Russian
 * \brief Объявление методов IDL-интерфейса NavigationSystemInterface.
 * \details Взаимодействие с компонентом NavigationSystem другими компонентами
 * модуля безопасности осуществляется через отправку IPC-сообщение через
 * IDL-интерфейс NavigationSystemInterface. В этом файле объявлены методы этого интерфейса
 * на языке C++. Все методы имеют стандартный набор параметров.
 * \param[in] self Указатель на структуру, соответствующую интерфейсу NavigationSystemInterface.
 * \param[in] req Указатель на структуру с полученными в IPC-сообщении данными фиксированной длины.
 * \param[in] reqArena Указатель на структуру с полученными в IPC-сообщении данными произвольной длины.
 * \param[out] res Указатель на структуру с отправляемыми в IPC-ответе данными фиксированной длины.
 * \param[out] resArena Указатель на структуру с отправляемыми в IPC-ответе данными произвольной длины.
 * \return Возвращает NK_EOK в случае успешного обмена сообщением и ответом.
 */

#pragma once

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/NavigationSystemInterface.idl.h>

/**
 * \~English IPC message handler. See \ref getCoords.
 * \~Russian Обработчик IPC-сообщения. См. \ref getCoords.
 */
nk_err_t GetCoordsImpl(struct NavigationSystemInterface *self,
                    const NavigationSystemInterface_GetCoords_req *req, const struct nk_arena *reqArena,
                    NavigationSystemInterface_GetCoords_res *res, struct nk_arena *resArena);
/**
 * \~English IPC message handler. See \ref getGpsInfo.
 * \~Russian Обработчик IPC-сообщения. См. \ref getGpsInfo.
 */
nk_err_t GetGpsInfoImpl(struct NavigationSystemInterface *self,
                    const NavigationSystemInterface_GetGpsInfo_req *req, const struct nk_arena *reqArena,
                    NavigationSystemInterface_GetGpsInfo_res *res, struct nk_arena *resArena);
/**
 * \~English IPC message handler. See \ref getGpsInfo.
 * \~Russian Обработчик IPC-сообщения. См. \ref getGpsInfo.
 */
nk_err_t GetSpeedImpl(struct NavigationSystemInterface *self,
                    const NavigationSystemInterface_GetSpeed_req *req, const struct nk_arena *reqArena,
                    NavigationSystemInterface_GetSpeed_res *res, struct nk_arena *resArena);

/**
 * \~English Creates an NavigationSystemInterface C++ interface and maps its methods to IPC message handlers.
 * \~Russian Создает C++ интерфейс NavigationSystemInterface и сопоставляет его методы с обработчиками IPC-сообщений.
 */
static struct NavigationSystemInterface *CreateNavigationSystemInterfaceImpl(void) {
    static const struct NavigationSystemInterface_ops Ops = {
        .GetCoords = GetCoordsImpl, .GetGpsInfo = GetGpsInfoImpl, .GetSpeed = GetSpeedImpl
    };

    static NavigationSystemInterface obj = {
        .ops = &Ops
    };

    return &obj;
}