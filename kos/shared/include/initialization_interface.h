#pragma once

#include <coresrv/nk/transport-kos.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/Initialization.idl.h>

void initSenderInterface(const char* connection, const char* endpoint, NkKosTransport &transport, nk_iid_t &riid);
void initReceiverInterface(const char* connection, NkKosTransport &transport);

nk_err_t WaitForInitImpl(struct Initialization *self,
    const Initialization_WaitForInit_req *req, const struct nk_arena *reqArena,
    Initialization_WaitForInit_res *res, struct nk_arena *resArena);

static struct Initialization *CreateInitializationImpl(void) {
    static const struct Initialization_ops Ops = {
        .WaitForInit = WaitForInitImpl
    };

    static Initialization obj = {
        .ops = &Ops
    };

    return &obj;
}