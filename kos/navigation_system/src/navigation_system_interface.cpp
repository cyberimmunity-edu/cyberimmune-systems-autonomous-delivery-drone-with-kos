#include "../include/navigation_system.h"
#include "../include/navigation_system_interface.h"

nk_err_t GetCoordsImpl(struct NavigationSystemInterface *self,
                    const NavigationSystemInterface_GetCoords_req *req, const struct nk_arena *reqArena,
                    NavigationSystemInterface_GetCoords_res *res, struct nk_arena *resArena) {
    int32_t latitude, longitude, altitude;

    res->success = getCoords(latitude, longitude, altitude);

    memcpy(&(res->lat), &latitude, sizeof(int32_t));
    memcpy(&(res->lng), &longitude, sizeof(int32_t));
    memcpy(&(res->alt), &altitude, sizeof(int32_t));

    return NK_EOK;
}