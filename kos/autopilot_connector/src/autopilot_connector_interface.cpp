#include "../include/autopilot_connector.h"
#include "../include/autopilot_connector_interface.h"

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