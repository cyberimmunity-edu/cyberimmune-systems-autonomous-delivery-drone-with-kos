#pragma once

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/PeripheryControllerInterface.idl.h>

nk_err_t EnableBuzzerImpl(struct PeripheryControllerInterface *self,
                    const PeripheryControllerInterface_EnableBuzzer_req *req, const struct nk_arena *reqArena,
                    PeripheryControllerInterface_EnableBuzzer_res *res, struct nk_arena *resArena);
nk_err_t SetKillSwitchImpl(struct PeripheryControllerInterface *self,
                    const PeripheryControllerInterface_SetKillSwitch_req *req, const struct nk_arena *reqArena,
                    PeripheryControllerInterface_SetKillSwitch_res *res, struct nk_arena *resArena);
nk_err_t SetCargoLockImpl(struct PeripheryControllerInterface *self,
                    const PeripheryControllerInterface_SetCargoLock_req *req, const struct nk_arena *reqArena,
                    PeripheryControllerInterface_SetCargoLock_res *res, struct nk_arena *resArena);

static struct PeripheryControllerInterface *CreatePeripheryControllerInterfaceImpl(void) {
    static const struct PeripheryControllerInterface_ops Ops = {
        .EnableBuzzer = EnableBuzzerImpl, .SetKillSwitch = SetKillSwitchImpl, .SetCargoLock = SetCargoLockImpl
    };

    static PeripheryControllerInterface obj = {
        .ops = &Ops
    };

    return &obj;
}