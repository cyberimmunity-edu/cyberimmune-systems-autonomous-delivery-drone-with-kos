/**
 * \file
 * \~English
 * \brief Declaration of AutopilotConnectorInterface IDL-interface methods.
 * \details Interaction between AutopilotConnector component and other components
 * of the security module is done via IPC message sent through AutopilotConnectorInterface IDL-interface.
 * The file contains declaration of the C++ methods of this interface. All methods have a standard set of parameters.
 * \param[in] self Pointer to the AutopilotConnectorInterface structure.
 * \param[in] req Pointer to a structure with fixed-length data received in an IPC message.
 * \param[in] reqArena Pointer to a structure with variable length data received in an IPC message.
 * \param[out] res Pointer to a structure with fixed-length data sent in an IPC response.
 * \param[out] resArena Pointer to a structure with variable length data sent in an IPC response.
 * \return Returns NK_EOK if message and response were successfully exchanged.
 *
 * \~Russian
 * \brief Объявление методов IDL-интерфейса AutopilotConnectorInterface.
 * \details Взаимодействие с компонентом AutopilotConnector другими компонентами
 * модуля безопасности осуществляется через отправку IPC-сообщение через
 * IDL-интерфейс AutopilotConnectorInterface. В этом файле объявлены методы этого интерфейса
 * на языке C++. Все методы имеют стандартный набор параметров.
 * \param[in] self Указатель на структуру, соответствующую интерфейсу AutopilotConnectorInterface.
 * \param[in] req Указатель на структуру с полученными в IPC-сообщении данными фиксированной длины.
 * \param[in] reqArena Указатель на структуру с полученными в IPC-сообщении данными произвольной длины.
 * \param[out] res Указатель на структуру с отправляемыми в IPC-ответе данными фиксированной длины.
 * \param[out] resArena Указатель на структуру с отправляемыми в IPC-ответе данными произвольной длины.
 * \return Возвращает NK_EOK в случае успешного обмена сообщением и ответом.
 */

#pragma once

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/AutopilotConnectorInterface.idl.h>

/**
 * \~English IPC message handler. See \ref waitForArmRequest.
 * \~Russian Обработчик IPC-сообщения. См. \ref waitForArmRequest.
 */
nk_err_t WaitForArmRequestImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_WaitForArmRequest_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_WaitForArmRequest_res *res, struct nk_arena *resArena);
/**
 * \~English IPC message handler. See \ref permitArm.
 * \~Russian Обработчик IPC-сообщения. См. \ref permitArm.
 */
nk_err_t PermitArmImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_PermitArm_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_PermitArm_res *res, struct nk_arena *resArena);
/**
 * \~English IPC message handler. See \ref forbidArm.
 * \~Russian Обработчик IPC-сообщения. См. \ref forbidArm.
 */
nk_err_t ForbidArmImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_ForbidArm_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_ForbidArm_res *res, struct nk_arena *resArena);
/**
 * \~English IPC message handler. See \ref pauseFlight.
 * \~Russian Обработчик IPC-сообщения. См. \ref pauseFlight.
 */
nk_err_t PauseFlightImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_PauseFlight_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_PauseFlight_res *res, struct nk_arena *resArena);
/**
 * \~English IPC message handler. See \ref resumeFlight.
 * \~Russian Обработчик IPC-сообщения. См. \ref resumeFlight.
 */
nk_err_t ResumeFlightImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_ResumeFlight_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_ResumeFlight_res *res, struct nk_arena *resArena);
/**
 * \~English IPC message handler. See \ref abortMission.
 * \~Russian Обработчик IPC-сообщения. См. \ref abortMission.
 */
nk_err_t AbortMissionImpl(struct AutopilotConnectorInterface *self,
    const AutopilotConnectorInterface_AbortMission_req *req, const struct nk_arena *reqArena,
    AutopilotConnectorInterface_AbortMission_res *res, struct nk_arena *resArena);
/**
 * \~English IPC message handler. See \ref changeSpeed.
 * \~Russian Обработчик IPC-сообщения. См. \ref changeSpeed.
 */
nk_err_t ChangeSpeedImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_ChangeSpeed_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_ChangeSpeed_res *res, struct nk_arena *resArena);
/**
 * \~English IPC message handler. See \ref changeAltitude.
 * \~Russian Обработчик IPC-сообщения. См. \ref changeAltitude.
 */
nk_err_t ChangeAltitudeImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_ChangeAltitude_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_ChangeAltitude_res *res, struct nk_arena *resArena);
/**
 * \~English IPC message handler. See \ref changeWaypoint.
 * \~Russian Обработчик IPC-сообщения. См. \ref changeWaypoint.
 */
nk_err_t ChangeWaypointImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_ChangeWaypoint_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_ChangeWaypoint_res *res, struct nk_arena *resArena);
/**
 * \~English IPC message handler. See \ref setMission.
 * \~Russian Обработчик IPC-сообщения. См. \ref setMission.
 */
nk_err_t SetMissionImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_SetMission_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_SetMission_res *res, struct nk_arena *resArena);

/**
 * \~English Creates an AutopilotConnectorInterface C++ interface and maps its methods to IPC message handlers.
 * \~Russian Создает C++ интерфейс AutopilotConnectorInterface и сопоставляет его методы с обработчиками IPC-сообщений.
 */
static struct AutopilotConnectorInterface *CreateAutopilotConnectorInterfaceImpl(void) {
    static const struct AutopilotConnectorInterface_ops Ops = {
        .WaitForArmRequest = WaitForArmRequestImpl, .PermitArm = PermitArmImpl, .ForbidArm = ForbidArmImpl,
        .PauseFlight = PauseFlightImpl, .ResumeFlight = ResumeFlightImpl, .AbortMission = AbortMissionImpl,
        .ChangeSpeed = ChangeSpeedImpl, .ChangeAltitude = ChangeAltitudeImpl, .ChangeWaypoint = ChangeWaypointImpl,
        .SetMission = SetMissionImpl
    };

    static AutopilotConnectorInterface obj = {
        .ops = &Ops
    };

    return &obj;
}