#pragma once

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/PeripheryControllerInterface.idl.h>

nk_err_t SetLightModeImpl(struct PeripheryControllerInterface *self,
                    const PeripheryControllerInterface_SetLightMode_req *req, const struct nk_arena *reqArena,
                    PeripheryControllerInterface_SetLightMode_res *res, struct nk_arena *resArena);
nk_err_t SetMotorKillSwitchImpl(struct PeripheryControllerInterface *self,
                    const PeripheryControllerInterface_SetMotorKillSwitch_req *req, const struct nk_arena *reqArena,
                    PeripheryControllerInterface_SetMotorKillSwitch_res *res, struct nk_arena *resArena);

static struct PeripheryControllerInterface *CreatePeripheryControllerInterfaceImpl(void) {
    static const struct PeripheryControllerInterface_ops Ops = {
        .SetLightMode = SetLightModeImpl, .SetMotorKillSwitch = SetMotorKillSwitchImpl
    };

    static PeripheryControllerInterface obj = {
        .ops = &Ops
    };

    return &obj;
}