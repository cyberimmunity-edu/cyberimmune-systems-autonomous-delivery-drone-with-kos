#include "../include/autopilot_connector.h"
#include "../include/autopilot_connector_interface.h"

nk_err_t GetAutopilotCommandImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_GetAutopilotCommand_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_GetAutopilotCommand_res *res, struct nk_arena *resArena) {
    res->success = getAutopilotCommand(res->command);

    return NK_EOK;
}

nk_err_t SendAutopilotCommandImpl(struct AutopilotConnectorInterface *self,
                    const AutopilotConnectorInterface_SendAutopilotCommand_req *req, const struct nk_arena *reqArena,
                    AutopilotConnectorInterface_SendAutopilotCommand_res *res, struct nk_arena *resArena) {
    res->success = sendAutopilotCommand((AutopilotCommand)(req->command));

    return NK_EOK;
}