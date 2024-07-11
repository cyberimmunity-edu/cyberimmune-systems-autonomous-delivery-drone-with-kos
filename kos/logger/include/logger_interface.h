#pragma once

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/LoggerInterface.idl.h>

nk_err_t LogImpl(struct LoggerInterface *self,
                    const LoggerInterface_Log_req *req, const struct nk_arena *reqArena,
                    LoggerInterface_Log_res *res, struct nk_arena *resArena);

static struct LoggerInterface *CreateLoggerInterfaceImpl(void) {
    static const struct LoggerInterface_ops Ops = {
        .Log = LogImpl
    };

    static LoggerInterface obj = {
        .ops = &Ops
    };

    return &obj;
}