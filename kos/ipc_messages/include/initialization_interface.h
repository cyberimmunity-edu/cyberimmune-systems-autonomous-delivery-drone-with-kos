#pragma once

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/Initialization.idl.h>

nk_err_t WaitForInitImpl(struct Initialization *self,
    const Initialization_WaitForInit_req *req, const struct nk_arena *reqArena,
    Initialization_WaitForInit_res *res, struct nk_arena *resArena);

int waitForInit(const char* connection, const char* receiverEntity);

static struct Initialization *CreateInitializationImpl(void) {
    static const struct Initialization_ops Ops = {
        .WaitForInit = WaitForInitImpl
    };

    static Initialization obj = {
        .ops = &Ops
    };

    return &obj;
}