#include "../include/periphery_controller.h"
#include "../include/periphery_controller_interface.h"

nk_err_t SetLightModeImpl(struct PeripheryControllerInterface *self,
                    const PeripheryControllerInterface_SetLightMode_req *req, const struct nk_arena *reqArena,
                    PeripheryControllerInterface_SetLightMode_res *res, struct nk_arena *resArena) {
    res->success = setLightMode((LightMode)(req->mode));

    return NK_EOK;
}

nk_err_t SetMotorKillSwitchImpl(struct PeripheryControllerInterface *self,
                    const PeripheryControllerInterface_SetMotorKillSwitch_req *req, const struct nk_arena *reqArena,
                    PeripheryControllerInterface_SetMotorKillSwitch_res *res, struct nk_arena *resArena) {
    res->success = setMotorKillSwitch(req->permit);

    return NK_EOK;
}