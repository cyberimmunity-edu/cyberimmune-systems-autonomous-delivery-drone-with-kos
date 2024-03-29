#include "../include/navigation_system.h"
#include "../include/navigation_system_interface.h"

nk_err_t GetCoordsImpl(struct NavigationSystemInterface *self,
                    const NavigationSystemInterface_GetCoords_req *req, const struct nk_arena *reqArena,
                    NavigationSystemInterface_GetCoords_res *res, struct nk_arena *resArena) {
    int32_t latitude, longitude, altitude;

    res->success = getCoords(latitude, longitude, altitude);
    res->lat = latitude;
    res->lng = longitude;
    res->alt = altitude;

    return NK_EOK;
}

nk_err_t GetDopImpl(struct NavigationSystemInterface *self,
                    const NavigationSystemInterface_GetDop_req *req, const struct nk_arena *reqArena,
                    NavigationSystemInterface_GetDop_res *res, struct nk_arena *resArena) {
    float dop;

    res->success = getDop(dop);

    memcpy(&(res->dop), &dop, sizeof(float));

    return NK_EOK;
}