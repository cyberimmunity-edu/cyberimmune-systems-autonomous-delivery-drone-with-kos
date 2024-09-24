/**
 * \file
 * \~English \brief Implementation of wrapper methods that send IPC messages to NavigationSystem component.
 * \~Russian \brief Реализация методов-оберток для отправки IPC-сообщений компоненту NavigationSystem.
 */

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

int getGpsInfo(float& dop, int32_t& sats) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("navigation_system_connection", "drone_controller.NavigationSystem.interface", transport, riid);

    struct NavigationSystemInterface_proxy proxy;
    NavigationSystemInterface_proxy_init(&proxy, &transport.base, riid);

    NavigationSystemInterface_GetGpsInfo_req req;
    NavigationSystemInterface_GetGpsInfo_res res;

    if ((NavigationSystemInterface_GetGpsInfo(&proxy.base, &req, NULL, &res, NULL) != rcOk) || !res.success)
        return 0;

    memcpy(&dop, &(res.dop), sizeof(float));
    memcpy(&sats, &(res.sats), sizeof(int32_t));

    return 1;
}

int getEstimatedSpeed(float& speed) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("navigation_system_connection", "drone_controller.NavigationSystem.interface", transport, riid);

    struct NavigationSystemInterface_proxy proxy;
    NavigationSystemInterface_proxy_init(&proxy, &transport.base, riid);

    NavigationSystemInterface_GetSpeed_req req;
    NavigationSystemInterface_GetSpeed_res res;

    if ((NavigationSystemInterface_GetSpeed(&proxy.base, &req, NULL, &res, NULL) != rcOk) || !res.success)
        return 0;

    memcpy(&speed, &(res.speed), sizeof(float));

    return 1;
}