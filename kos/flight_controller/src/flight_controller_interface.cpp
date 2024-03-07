#include "../include/flight_controller_interface.h"
#include "../../ipc_messages/include/transport_interface.h"

#include <string.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/AutopilotConnectorInterface.idl.h>
#include <drone_controller/CredentialManagerInterface.idl.h>
#include <drone_controller/NavigationSystemInterface.idl.h>
#include <drone_controller/PeripheryControllerInterface.idl.h>
#include <drone_controller/ServerConnectorInterface.idl.h>

int getAutopilotCommand(AutopilotCommand &command) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_GetAutopilotCommand_req req;
    AutopilotConnectorInterface_GetAutopilotCommand_res res;

    if ((AutopilotConnectorInterface_GetAutopilotCommand(&proxy.base, &req, NULL, &res, NULL) != rcOk) || !res.success)
        return 0;

    command = (AutopilotCommand)res.command;
    return 1;
}

int sendAutopilotCommand(AutopilotCommand command) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_SendAutopilotCommand_req req;
    AutopilotConnectorInterface_SendAutopilotCommand_res res;

    req.command = (uint8_t)command;

    return ((AutopilotConnectorInterface_SendAutopilotCommand(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int getRsaKey(char* e, char* n) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("credential_manager_connection", "drone_controller.CredentialManager.interface", transport, riid);

    struct CredentialManagerInterface_proxy proxy;
    CredentialManagerInterface_proxy_init(&proxy, &transport.base, riid);

    CredentialManagerInterface_GetRsaKey_req req;
    CredentialManagerInterface_GetRsaKey_res res;
    char resBuffer[CredentialManagerInterface_GetRsaKey_res_arena_size];
    struct nk_arena resArena = NK_ARENA_INITIALIZER(resBuffer, resBuffer + sizeof(resBuffer));
    nk_arena_reset(&resArena);

    if ((CredentialManagerInterface_GetRsaKey(&proxy.base, &req, NULL, &res, &resArena) != rcOk) || !res.success)
        return 0;

    nk_uint32_t len = 0;
    nk_char_t *msg = nk_arena_get(nk_char_t, &resArena, &(res.e), &len);
    if (msg == NULL)
        return 0;
    strcpy(e, msg);

    msg = nk_arena_get(nk_char_t, &resArena, &(res.n), &len);
    if (msg == NULL)
        return 0;
    strcpy(n, msg);

    return 1;
}

int setServerRsaKey(char* key) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("credential_manager_connection", "drone_controller.CredentialManager.interface", transport, riid);

    struct CredentialManagerInterface_proxy proxy;
    CredentialManagerInterface_proxy_init(&proxy, &transport.base, riid);

    CredentialManagerInterface_SetRsaKey_req req;
    CredentialManagerInterface_SetRsaKey_res res;
    char reqBuffer[CredentialManagerInterface_SetRsaKey_req_arena_size];
    struct nk_arena reqArena = NK_ARENA_INITIALIZER(reqBuffer, reqBuffer + sizeof(reqBuffer));
    nk_arena_reset(&reqArena);

    nk_char_t *msg = nk_arena_alloc(nk_char_t, &reqArena, &(req.key), strlen(key) + 1);
    if (msg == NULL)
        return 0;
    strcpy(msg, key);

    return ((CredentialManagerInterface_SetRsaKey(&proxy.base, &req, &reqArena, &res, NULL) == rcOk) && res.success);
}

int signMessage(char* message, char* signature) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("credential_manager_connection", "drone_controller.CredentialManager.interface", transport, riid);

    struct CredentialManagerInterface_proxy proxy;
    CredentialManagerInterface_proxy_init(&proxy, &transport.base, riid);

    CredentialManagerInterface_SignMessage_req req;
    CredentialManagerInterface_SignMessage_res res;
    char reqBuffer[CredentialManagerInterface_SignMessage_req_arena_size];
    char resBuffer[CredentialManagerInterface_SignMessage_res_arena_size];
    struct nk_arena reqArena = NK_ARENA_INITIALIZER(reqBuffer, reqBuffer + sizeof(reqBuffer));
    struct nk_arena resArena = NK_ARENA_INITIALIZER(resBuffer, resBuffer + sizeof(resBuffer));
    nk_arena_reset(&reqArena);
    nk_arena_reset(&resArena);

    nk_char_t *msg = nk_arena_alloc(nk_char_t, &reqArena, &(req.message), strlen(message) + 1);
    if (msg == NULL)
        return 0;
    strcpy(msg, message);

    if ((CredentialManagerInterface_SignMessage(&proxy.base, &req, &reqArena, &res, &resArena) != rcOk) || !res.success)
        return 0;

    nk_uint32_t len = 0;
    msg = nk_arena_get(nk_char_t, &resArena, &(res.signature), &len);
    if (msg == NULL)
        return 0;
    strcpy(signature, msg);

    return 1;
}

int checkSignature(char* message) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("credential_manager_connection", "drone_controller.CredentialManager.interface", transport, riid);

    struct CredentialManagerInterface_proxy proxy;
    CredentialManagerInterface_proxy_init(&proxy, &transport.base, riid);

    CredentialManagerInterface_CheckSignature_req req;
    CredentialManagerInterface_CheckSignature_res res;
    char reqBuffer[CredentialManagerInterface_CheckSignature_req_arena_size];
    struct nk_arena reqArena = NK_ARENA_INITIALIZER(reqBuffer, reqBuffer + sizeof(reqBuffer));
    nk_arena_reset(&reqArena);

    nk_char_t *msg = nk_arena_alloc(nk_char_t, &reqArena, &(req.message), strlen(message) + 1);
    if (msg == NULL)
        return 0;
    strcpy(msg, message);

    return ((CredentialManagerInterface_CheckSignature(&proxy.base, &req, &reqArena, &res, NULL) == rcOk) && res.success && res.correct);
}

int getAzimuth(float &azimuth) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("navigation_system_connection", "drone_controller.NavigationSystem.interface", transport, riid);

    struct NavigationSystemInterface_proxy proxy;
    NavigationSystemInterface_proxy_init(&proxy, &transport.base, riid);

    NavigationSystemInterface_GetAzimuth_req req;
    NavigationSystemInterface_GetAzimuth_res res;

    if ((NavigationSystemInterface_GetAzimuth(&proxy.base, &req, NULL, &res, NULL) != rcOk) || !res.success)
        return 0;

    memcpy(&azimuth, &res.azimuth, sizeof(float));

    return 1;
}

int getAcceleration(float &x, float &y, float &z) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("navigation_system_connection", "drone_controller.NavigationSystem.interface", transport, riid);

    struct NavigationSystemInterface_proxy proxy;
    NavigationSystemInterface_proxy_init(&proxy, &transport.base, riid);

    NavigationSystemInterface_GetAcceleration_req req;
    NavigationSystemInterface_GetAcceleration_res res;

    if ((NavigationSystemInterface_GetAcceleration(&proxy.base, &req, NULL, &res, NULL) != rcOk) || !res.success)
        return 0;

    memcpy(&x, &res.accelerationX, sizeof(float));
    memcpy(&y, &res.accelerationY, sizeof(float));
    memcpy(&z, &res.accelerationZ, sizeof(float));

    return 1;
}

int getGyroscope(float &x, float &y, float &z) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("navigation_system_connection", "drone_controller.NavigationSystem.interface", transport, riid);

    struct NavigationSystemInterface_proxy proxy;
    NavigationSystemInterface_proxy_init(&proxy, &transport.base, riid);

    NavigationSystemInterface_GetGyroscope_req req;
    NavigationSystemInterface_GetGyroscope_res res;

    if ((NavigationSystemInterface_GetGyroscope(&proxy.base, &req, NULL, &res, NULL) != rcOk) || !res.success)
        return 0;

    memcpy(&x, &res.gyroscopeX, sizeof(float));
    memcpy(&y, &res.gyroscopeY, sizeof(float));
    memcpy(&z, &res.gyroscopeZ, sizeof(float));

    return 1;
}

int getTemperature(float &temperature) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("navigation_system_connection", "drone_controller.NavigationSystem.interface", transport, riid);

    struct NavigationSystemInterface_proxy proxy;
    NavigationSystemInterface_proxy_init(&proxy, &transport.base, riid);

    NavigationSystemInterface_GetTemperature_req req;
    NavigationSystemInterface_GetTemperature_res res;

    if ((NavigationSystemInterface_GetTemperature(&proxy.base, &req, NULL, &res, NULL) != rcOk) || !res.success)
        return 0;

    memcpy(&temperature, &res.temperature, sizeof(float));

    return 1;
}

int setLightMode(LightMode mode) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("periphery_controller_connection", "drone_controller.PeripheryController.interface", transport, riid);

    struct PeripheryControllerInterface_proxy proxy;
    PeripheryControllerInterface_proxy_init(&proxy, &transport.base, riid);

    PeripheryControllerInterface_SetLightMode_req req;
    PeripheryControllerInterface_SetLightMode_res res;

    req.mode = (uint8_t)mode;

    return ((PeripheryControllerInterface_SetLightMode(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int setMotorKillSwitch(uint8_t permit) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("periphery_controller_connection", "drone_controller.PeripheryController.interface", transport, riid);

    struct PeripheryControllerInterface_proxy proxy;
    PeripheryControllerInterface_proxy_init(&proxy, &transport.base, riid);

    PeripheryControllerInterface_SetMotorKillSwitch_req req;
    PeripheryControllerInterface_SetMotorKillSwitch_res res;

    req.permit = permit;

    return ((PeripheryControllerInterface_SetMotorKillSwitch(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int sendRequest(char* query, char* response) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("server_connector_connection", "drone_controller.ServerConnector.interface", transport, riid);

    struct ServerConnectorInterface_proxy proxy;
    ServerConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    ServerConnectorInterface_SendRequest_req req;
    ServerConnectorInterface_SendRequest_res res;
    char reqBuffer[ServerConnectorInterface_SendRequest_req_arena_size];
    char resBuffer[ServerConnectorInterface_SendRequest_res_arena_size];
    struct nk_arena reqArena = NK_ARENA_INITIALIZER(reqBuffer, reqBuffer + sizeof(reqBuffer));
    struct nk_arena resArena = NK_ARENA_INITIALIZER(resBuffer, resBuffer + sizeof(resBuffer));
    nk_arena_reset(&reqArena);
    nk_arena_reset(&resArena);

    nk_char_t *msg = nk_arena_alloc(nk_char_t, &reqArena, &(req.query), strlen(query) + 1);
    if (msg == NULL)
        return 0;
    strcpy(msg, query);

    if ((ServerConnectorInterface_SendRequest(&proxy.base, &req, &reqArena, &res, &resArena) != rcOk) || !res.success)
        return 0;

    nk_uint32_t len = 0;
    msg = nk_arena_get(nk_char_t, &resArena, &(res.response), &len);
    if (msg == NULL)
        return 0;
    strcpy(response, msg);

    return 1;
}