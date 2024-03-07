#pragma once

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/ServerConnectorInterface.idl.h>

nk_err_t SendRequestImpl(struct ServerConnectorInterface *self,
                    const ServerConnectorInterface_SendRequest_req *req, const struct nk_arena *reqArena,
                    ServerConnectorInterface_SendRequest_res *res, struct nk_arena *resArena);

static struct ServerConnectorInterface *CreateServerConnectorInterfaceImpl(void) {
    static const struct ServerConnectorInterface_ops Ops = {
        .SendRequest = SendRequestImpl
    };

    static ServerConnectorInterface obj = {
        .ops = &Ops
    };

    return &obj;
}