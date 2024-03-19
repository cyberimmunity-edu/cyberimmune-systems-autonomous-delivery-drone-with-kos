#pragma once

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/AutopilotConnectorInterface.idl.h>

nk_err_t WaitForArmRequestImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_WaitForArmRequest_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_WaitForArmRequest_res *res, struct nk_arena *resArena);
nk_err_t PermitArmImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_PermitArm_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_PermitArm_res *res, struct nk_arena *resArena);
nk_err_t ForbidArmImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_ForbidArm_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_ForbidArm_res *res, struct nk_arena *resArena);
nk_err_t PauseFlightImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_PauseFlight_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_PauseFlight_res *res, struct nk_arena *resArena);
nk_err_t ResumeFlightImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_ResumeFlight_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_ResumeFlight_res *res, struct nk_arena *resArena);

static struct AutopilotConnectorInterface *CreateAutopilotConnectorInterfaceImpl(void) {
    static const struct AutopilotConnectorInterface_ops Ops = {
        .WaitForArmRequest = WaitForArmRequestImpl, .PermitArm = PermitArmImpl, .ForbidArm = ForbidArmImpl,
        .PauseFlight = PauseFlightImpl, .ResumeFlight = ResumeFlightImpl
    };

    static AutopilotConnectorInterface obj = {
        .ops = &Ops
    };

    return &obj;
}