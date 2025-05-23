/**
 * \file
 * \~English \brief Implementation of AutopilotConnectorInterface IDL-interface methods.
 * \~Russian \brief Реализация методов IDL-интерфейса AutopilotConnectorInterface.
 */

#include "../include/autopilot_connector.h"
#include "../include/autopilot_connector_interface.h"

#include <string.h>

nk_err_t WaitForArmRequestImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_WaitForArmRequest_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_WaitForArmRequest_res *res, struct nk_arena *resArena) {
    uint8_t command;
    res->success = (getAutopilotCommand(command) && (command == AutopilotCommand::ArmRequest));

    return NK_EOK;
}

nk_err_t PermitArmImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_PermitArm_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_PermitArm_res *res, struct nk_arena *resArena) {
    res->success = sendAutopilotCommand(AutopilotCommand::ArmPermit);

    return NK_EOK;
}

nk_err_t ForbidArmImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_ForbidArm_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_ForbidArm_res *res, struct nk_arena *resArena) {
    res->success = sendAutopilotCommand(AutopilotCommand::ArmForbid);

    return NK_EOK;
}

nk_err_t PauseFlightImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_PauseFlight_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_PauseFlight_res *res, struct nk_arena *resArena) {
    res->success = sendAutopilotCommand(AutopilotCommand::PauseFlight);

    return NK_EOK;
}

nk_err_t ResumeFlightImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_ResumeFlight_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_ResumeFlight_res *res, struct nk_arena *resArena) {
    res->success = sendAutopilotCommand(AutopilotCommand::ResumeFlight);

    return NK_EOK;
}

nk_err_t AbortMissionImpl(struct AutopilotConnectorInterface *self,
    const AutopilotConnectorInterface_AbortMission_req *req, const struct nk_arena *reqArena,
    AutopilotConnectorInterface_AbortMission_res *res, struct nk_arena *resArena) {
    res->success = sendAutopilotCommand(AutopilotCommand::AbortMission);

    return NK_EOK;
}

nk_err_t ChangeSpeedImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_ChangeSpeed_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_ChangeSpeed_res *res, struct nk_arena *resArena) {
    res->success = sendAutopilotCommand(AutopilotCommand::ChangeSpeed, req->speed);

    return NK_EOK;
}

nk_err_t ChangeAltitudeImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_ChangeAltitude_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_ChangeAltitude_res *res, struct nk_arena *resArena) {
    res->success = sendAutopilotCommand(AutopilotCommand::ChangeAltitude, req->altitude);

    return NK_EOK;
}

nk_err_t ChangeWaypointImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_ChangeWaypoint_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_ChangeWaypoint_res *res, struct nk_arena *resArena) {
    res->success = sendAutopilotCommand(AutopilotCommand::ChangeWaypoint, req->latitude, req->longitude, req->altitude);

    return NK_EOK;
}

nk_err_t SetMissionImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_SetMission_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_SetMission_res *res, struct nk_arena *resArena) {
    uint8_t bytes[AutopilotConnectorInterface_MaxMissionLength] = {0};

    nk_uint32_t len = 0;
    nk_uint8_t* msg = nk_arena_get(nk_uint8_t, reqArena, &(req->mission), &len);
    if (msg == NULL)
        return NK_EBADMSG;
    else if (len > AutopilotConnectorInterface_MaxMissionLength)
        return NK_ENOMEM;
    memcpy(bytes, msg, len);

    res->success = sendAutopilotCommand(AutopilotCommand::SetMission, bytes, req->size);

    return NK_EOK;
}