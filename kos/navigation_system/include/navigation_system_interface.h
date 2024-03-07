#pragma once

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/NavigationSystemInterface.idl.h>

nk_err_t GetAzimuthImpl(struct NavigationSystemInterface *self,
                    const NavigationSystemInterface_GetAzimuth_req *req, const struct nk_arena *reqArena,
                    NavigationSystemInterface_GetAzimuth_res *res, struct nk_arena *resArena);
nk_err_t GetAccelerationImpl(struct NavigationSystemInterface *self,
                    const NavigationSystemInterface_GetAcceleration_req *req, const struct nk_arena *reqArena,
                    NavigationSystemInterface_GetAcceleration_res *res, struct nk_arena *resArena);
nk_err_t GetGyroscopeImpl(struct NavigationSystemInterface *self,
                    const NavigationSystemInterface_GetGyroscope_req *req, const struct nk_arena *reqArena,
                    NavigationSystemInterface_GetGyroscope_res *res, struct nk_arena *resArena);
nk_err_t GetTemperatureImpl(struct NavigationSystemInterface *self,
                    const NavigationSystemInterface_GetTemperature_req *req, const struct nk_arena *reqArena,
                    NavigationSystemInterface_GetTemperature_res *res, struct nk_arena *resArena);

static struct NavigationSystemInterface *CreateNavigationSystemInterfaceImpl(void) {
    static const struct NavigationSystemInterface_ops Ops = {
        .GetAzimuth = GetAzimuthImpl, .GetAcceleration = GetAccelerationImpl,
        .GetGyroscope = GetGyroscopeImpl, .GetTemperature = GetTemperatureImpl
    };

    static NavigationSystemInterface obj = {
        .ops = &Ops
    };

    return &obj;
}