/**
 * \file
 * \~English
 * \brief Declaration of PeripheryControllerInterface IDL-interface methods.
 * \details Interaction between PeripheryController component and other components
 * of the security module is done via IPC message sent through PeripheryControllerInterface IDL-interface.
 * The file contains declaration of the C++ methods of this interface. All methods have a standard set of parameters.
 * \param[in] self Pointer to the PeripheryControllerInterface structure.
 * \param[in] req Pointer to a structure with fixed-length data received in an IPC message.
 * \param[in] reqArena Pointer to a structure with variable length data received in an IPC message.
 * \param[out] res Pointer to a structure with fixed-length data sent in an IPC response.
 * \param[out] resArena Pointer to a structure with variable length data sent in an IPC response.
 * \return Returns NK_EOK if message and response were successfully exchanged.
 *
 * \~Russian
 * \brief Объявление методов IDL-интерфейса PeripheryControllerInterface.
 * \details Взаимодействие с компонентом PeripheryController другими компонентами
 * модуля безопасности осуществляется через отправку IPC-сообщение через
 * IDL-интерфейс PeripheryControllerInterface. В этом файле объявлены методы этого интерфейса
 * на языке C++. Все методы имеют стандартный набор параметров.
 * \param[in] self Указатель на структуру, соответствующую интерфейсу PeripheryControllerInterface.
 * \param[in] req Указатель на структуру с полученными в IPC-сообщении данными фиксированной длины.
 * \param[in] reqArena Указатель на структуру с полученными в IPC-сообщении данными произвольной длины.
 * \param[out] res Указатель на структуру с отправляемыми в IPC-ответе данными фиксированной длины.
 * \param[out] resArena Указатель на структуру с отправляемыми в IPC-ответе данными произвольной длины.
 * \return Возвращает NK_EOK в случае успешного обмена сообщением и ответом.
 */

#pragma once

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/PeripheryControllerInterface.idl.h>

/**
 * \~English IPC message handler. See \ref enableBuzzer.
 * \~Russian Обработчик IPC-сообщения. См. \ref enableBuzzer.
 */
nk_err_t EnableBuzzerImpl(struct PeripheryControllerInterface *self,
                    const PeripheryControllerInterface_EnableBuzzer_req *req, const struct nk_arena *reqArena,
                    PeripheryControllerInterface_EnableBuzzer_res *res, struct nk_arena *resArena);
/**
 * \~English IPC message handler. See \ref setKillSwitch.
 * \~Russian Обработчик IPC-сообщения. См. \ref setKillSwitch.
 */
nk_err_t SetKillSwitchImpl(struct PeripheryControllerInterface *self,
                    const PeripheryControllerInterface_SetKillSwitch_req *req, const struct nk_arena *reqArena,
                    PeripheryControllerInterface_SetKillSwitch_res *res, struct nk_arena *resArena);
/**
 * \~English IPC message handler. See \ref setCargoLock.
 * \~Russian Обработчик IPC-сообщения. См. \ref setCargoLock.
 */
nk_err_t SetCargoLockImpl(struct PeripheryControllerInterface *self,
                    const PeripheryControllerInterface_SetCargoLock_req *req, const struct nk_arena *reqArena,
                    PeripheryControllerInterface_SetCargoLock_res *res, struct nk_arena *resArena);

/**
 * \~English Creates an PeripheryControllerInterface C++ interface and maps its methods to IPC message handlers.
 * \~Russian Создает C++ интерфейс PeripheryControllerInterface и сопоставляет его методы с обработчиками IPC-сообщений.
 */
static struct PeripheryControllerInterface *CreatePeripheryControllerInterfaceImpl(void) {
    static const struct PeripheryControllerInterface_ops Ops = {
        .EnableBuzzer = EnableBuzzerImpl, .SetKillSwitch = SetKillSwitchImpl, .SetCargoLock = SetCargoLockImpl
    };

    static PeripheryControllerInterface obj = {
        .ops = &Ops
    };

    return &obj;
}