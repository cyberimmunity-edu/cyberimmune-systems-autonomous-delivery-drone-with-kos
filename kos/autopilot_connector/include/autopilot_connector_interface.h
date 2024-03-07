#pragma once

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/AutopilotConnectorInterface.idl.h>

nk_err_t GetAutopilotCommandImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_GetAutopilotCommand_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_GetAutopilotCommand_res *res, struct nk_arena *resArena);
nk_err_t SendAutopilotCommandImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_SendAutopilotCommand_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_SendAutopilotCommand_res *res, struct nk_arena *resArena);

static struct AutopilotConnectorInterface *CreateAutopilotConnectorInterfaceImpl(void) {
    static const struct AutopilotConnectorInterface_ops Ops = {
        .GetAutopilotCommand = GetAutopilotCommandImpl, .SendAutopilotCommand = SendAutopilotCommandImpl
    };

    static AutopilotConnectorInterface obj = {
        .ops = &Ops
    };

    return &obj;
}