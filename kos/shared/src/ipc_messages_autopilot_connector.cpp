/**
 * \file
 * \~English \brief Implementation of wrapper methods that send IPC messages to AutopilotConnector component.
 * \~Russian \brief Реализация методов-оберток для отправки IPC-сообщений компоненту AutopilotConnector.
 */

#include "../include/ipc_messages_autopilot_connector.h"
#include "../include/initialization_interface.h"

#include <stddef.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/AutopilotConnectorInterface.idl.h>

int waitForArmRequest() {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_WaitForArmRequest_req req;
    AutopilotConnectorInterface_WaitForArmRequest_res res;

    return ((AutopilotConnectorInterface_WaitForArmRequest(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int permitArm() {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_PermitArm_req req;
    AutopilotConnectorInterface_PermitArm_res res;

    return ((AutopilotConnectorInterface_PermitArm(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int forbidArm() {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_ForbidArm_req req;
    AutopilotConnectorInterface_ForbidArm_res res;

    return ((AutopilotConnectorInterface_ForbidArm(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int pauseFlight() {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_PauseFlight_req req;
    AutopilotConnectorInterface_PauseFlight_res res;

    return ((AutopilotConnectorInterface_PauseFlight(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int resumeFlight() {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_ResumeFlight_req req;
    AutopilotConnectorInterface_ResumeFlight_res res;

    return ((AutopilotConnectorInterface_ResumeFlight(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int abortMission() {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_AbortMission_req req;
    AutopilotConnectorInterface_AbortMission_res res;

    return ((AutopilotConnectorInterface_AbortMission(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int changeSpeed(int32_t speed) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_ChangeSpeed_req req;
    AutopilotConnectorInterface_ChangeSpeed_res res;

    req.speed = speed;

    return ((AutopilotConnectorInterface_ChangeSpeed(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int changeAltitude(int32_t altitude) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_ChangeAltitude_req req;
    AutopilotConnectorInterface_ChangeAltitude_res res;

    req.altitude = altitude;

    return ((AutopilotConnectorInterface_ChangeAltitude(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int changeWaypoint(int32_t latitude, int32_t longitude, int32_t altitude) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_ChangeWaypoint_req req;
    AutopilotConnectorInterface_ChangeWaypoint_res res;

    req.latitude = latitude;
    req.longitude = longitude;
    req.altitude = altitude;

    return ((AutopilotConnectorInterface_ChangeWaypoint(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int setMission(uint8_t* mission, uint32_t missionSize) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_SetMission_req req;
    AutopilotConnectorInterface_SetMission_res res;

    req.size = missionSize;
    char reqBuffer[AutopilotConnectorInterface_SetMission_req_arena_size];
    struct nk_arena reqArena = NK_ARENA_INITIALIZER(reqBuffer, reqBuffer + sizeof(reqBuffer));
    nk_arena_reset(&reqArena);

    nk_uint8_t* msg = nk_arena_alloc(nk_uint8_t, &reqArena, &(req.mission), missionSize);
    if ((msg == NULL) || (missionSize > AutopilotConnectorInterface_SetMission_req_arena_size))
        return 0;
    memcpy(msg, mission, missionSize);

    return ((AutopilotConnectorInterface_SetMission(&proxy.base, &req, &reqArena, &res, NULL) == rcOk) && res.success);
}