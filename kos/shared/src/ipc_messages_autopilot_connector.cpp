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