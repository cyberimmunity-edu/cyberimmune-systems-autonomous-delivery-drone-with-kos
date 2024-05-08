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

nk_err_t GetGpsInfoImpl(struct NavigationSystemInterface *self,
                    const NavigationSystemInterface_GetGpsInfo_req *req, const struct nk_arena *reqArena,
                    NavigationSystemInterface_GetGpsInfo_res *res, struct nk_arena *resArena) {
    float dop;
    int32_t sats;

    res->success = getGpsInfo(dop, sats);

    memcpy(&(res->dop), &dop, sizeof(float));
    memcpy(&(res->sats), &sats, sizeof(int32_t));

    return NK_EOK;
}