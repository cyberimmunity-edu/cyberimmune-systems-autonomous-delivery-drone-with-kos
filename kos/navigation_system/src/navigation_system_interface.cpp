#include "../include/navigation_system.h"
#include "../include/navigation_system_interface.h"

nk_err_t GetAzimuthImpl(struct NavigationSystemInterface *self,
                    const NavigationSystemInterface_GetAzimuth_req *req, const struct nk_arena *reqArena,
                    NavigationSystemInterface_GetAzimuth_res *res, struct nk_arena *resArena) {
    float azimuth;
    res->success = getAzimuth(azimuth);

    memcpy(&(res->azimuth), &azimuth, sizeof(float));

    return NK_EOK;
}

nk_err_t GetAccelerationImpl(struct NavigationSystemInterface *self,
                    const NavigationSystemInterface_GetAcceleration_req *req, const struct nk_arena *reqArena,
                    NavigationSystemInterface_GetAcceleration_res *res, struct nk_arena *resArena) {
    float x, y, z;
    res->success = getAcceleration(x, y, z);

    memcpy(&(res->accelerationX), &x, sizeof(float));
    memcpy(&(res->accelerationY), &y, sizeof(float));
    memcpy(&(res->accelerationZ), &z, sizeof(float));

    return NK_EOK;
}

nk_err_t GetGyroscopeImpl(struct NavigationSystemInterface *self,
                    const NavigationSystemInterface_GetGyroscope_req *req, const struct nk_arena *reqArena,
                    NavigationSystemInterface_GetGyroscope_res *res, struct nk_arena *resArena) {
    float x, y, z;
    res->success = getGyroscope(x, y, z);

    memcpy(&(res->gyroscopeX), &x, sizeof(float));
    memcpy(&(res->gyroscopeY), &y, sizeof(float));
    memcpy(&(res->gyroscopeZ), &z, sizeof(float));

    return NK_EOK;
}

nk_err_t GetTemperatureImpl(struct NavigationSystemInterface *self,
                    const NavigationSystemInterface_GetTemperature_req *req, const struct nk_arena *reqArena,
                    NavigationSystemInterface_GetTemperature_res *res, struct nk_arena *resArena) {
    float temperature;
    res->success = getTemperature(temperature);

    memcpy(&(res->temperature), &temperature, sizeof(float));

    return NK_EOK;
}