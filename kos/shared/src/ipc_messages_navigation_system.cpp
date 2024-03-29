#include "../include/ipc_messages_navigation_system.h"
#include "../include/initialization_interface.h"

#include <stddef.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/NavigationSystemInterface.idl.h>

int getCoords(int32_t &latitude, int32_t &longitude, int32_t &altitude) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("navigation_system_connection", "drone_controller.NavigationSystem.interface", transport, riid);

    struct NavigationSystemInterface_proxy proxy;
    NavigationSystemInterface_proxy_init(&proxy, &transport.base, riid);

    NavigationSystemInterface_GetCoords_req req;
    NavigationSystemInterface_GetCoords_res res;

    if ((NavigationSystemInterface_GetCoords(&proxy.base, &req, NULL, &res, NULL) != rcOk) || !res.success)
        return 0;

    latitude = res.lat;
    longitude = res.lng;
    altitude = res.alt;

    return 1;
}

int getDop(float& dop) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("navigation_system_connection", "drone_controller.NavigationSystem.interface", transport, riid);

    struct NavigationSystemInterface_proxy proxy;
    NavigationSystemInterface_proxy_init(&proxy, &transport.base, riid);

    NavigationSystemInterface_GetDop_req req;
    NavigationSystemInterface_GetDop_res res;

    if ((NavigationSystemInterface_GetDop(&proxy.base, &req, NULL, &res, NULL) != rcOk) || !res.success)
        return 0;

    memcpy(&dop, &(res.dop), sizeof(float));

    return 1;
}