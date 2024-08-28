/**
 * \file
 * \~English \brief Implementation of PeripheryControllerInterface IDL-interface methods.
 * \~Russian \brief Реализация методов IDL-интерфейса PeripheryControllerInterface.
 */

#include "../include/periphery_controller.h"
#include "../include/periphery_controller_interface.h"

nk_err_t EnableBuzzerImpl(struct PeripheryControllerInterface *self,
                    const PeripheryControllerInterface_EnableBuzzer_req *req, const struct nk_arena *reqArena,
                    PeripheryControllerInterface_EnableBuzzer_res *res, struct nk_arena *resArena) {
    res->success = startBuzzer();

    return NK_EOK;
}

nk_err_t SetKillSwitchImpl(struct PeripheryControllerInterface *self,
                    const PeripheryControllerInterface_SetKillSwitch_req *req, const struct nk_arena *reqArena,
                    PeripheryControllerInterface_SetKillSwitch_res *res, struct nk_arena *resArena) {
    res->success = setKillSwitch(req->enable);

    return NK_EOK;
}

nk_err_t SetCargoLockImpl(struct PeripheryControllerInterface *self,
                    const PeripheryControllerInterface_SetCargoLock_req *req, const struct nk_arena *reqArena,
                    PeripheryControllerInterface_SetCargoLock_res *res, struct nk_arena *resArena) {
    res->success = setCargoLock(req->enable);

    return NK_EOK;
}