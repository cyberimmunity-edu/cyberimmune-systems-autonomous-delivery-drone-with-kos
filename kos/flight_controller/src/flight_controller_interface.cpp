#include "../include/flight_controller_interface.h"
#include "../../ipc_messages/include/transport_interface.h"

#include <string.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/AutopilotConnectorInterface.idl.h>
#include <drone_controller/CredentialManagerInterface.idl.h>
#include <drone_controller/NavigationSystemInterface.idl.h>
#include <drone_controller/PeripheryControllerInterface.idl.h>
#include <drone_controller/ServerConnectorInterface.idl.h>

int waitForArmRequest() {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_WaitForArmRequest_req req;
    AutopilotConnectorInterface_WaitForArmRequest_res res;

    if ((AutopilotConnectorInterface_WaitForArmRequest(&proxy.base, &req, NULL, &res, NULL) != rcOk) || !res.success)
        return 0;

    return 1;
}

int permitArm() {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_PermitArm_req req;
    AutopilotConnectorInterface_PermitArm_res res;

    if ((AutopilotConnectorInterface_PermitArm(&proxy.base, &req, NULL, &res, NULL) != rcOk) || !res.success)
        return 0;

    return 1;
}

int forbidArm() {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_ForbidArm_req req;
    AutopilotConnectorInterface_ForbidArm_res res;

    if ((AutopilotConnectorInterface_ForbidArm(&proxy.base, &req, NULL, &res, NULL) != rcOk) || !res.success)
        return 0;

    return 1;
}

int pauseFlight() {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_PauseFlight_req req;
    AutopilotConnectorInterface_PauseFlight_res res;

    if ((AutopilotConnectorInterface_PauseFlight(&proxy.base, &req, NULL, &res, NULL) != rcOk) || !res.success)
        return 0;

    return 1;
}

int resumeFlight() {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("autopilot_connector_connection", "drone_controller.AutopilotConnector.interface", transport, riid);

    struct AutopilotConnectorInterface_proxy proxy;
    AutopilotConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    AutopilotConnectorInterface_ResumeFlight_req req;
    AutopilotConnectorInterface_ResumeFlight_res res;

    if ((AutopilotConnectorInterface_ResumeFlight(&proxy.base, &req, NULL, &res, NULL) != rcOk) || !res.success)
        return 0;

    return 1;
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

    memcpy(&latitude, &res.lat, sizeof(int32_t));
    memcpy(&longitude, &res.lng, sizeof(int32_t));
    memcpy(&altitude, &res.alt, sizeof(int32_t));

    return 1;
}

int setBuzzer(uint8_t enable) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("periphery_controller_connection", "drone_controller.PeripheryController.interface", transport, riid);

    struct PeripheryControllerInterface_proxy proxy;
    PeripheryControllerInterface_proxy_init(&proxy, &transport.base, riid);

    PeripheryControllerInterface_SetBuzzer_req req;
    PeripheryControllerInterface_SetBuzzer_res res;

    req.enable = enable;

    return ((PeripheryControllerInterface_SetBuzzer(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int setMotorKillSwitch(uint8_t enable) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("periphery_controller_connection", "drone_controller.PeripheryController.interface", transport, riid);

    struct PeripheryControllerInterface_proxy proxy;
    PeripheryControllerInterface_proxy_init(&proxy, &transport.base, riid);

    PeripheryControllerInterface_SetKillSwitch_req req;
    PeripheryControllerInterface_SetKillSwitch_res res;

    req.enable = enable;

    return ((PeripheryControllerInterface_SetKillSwitch(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int setCargoLock(uint8_t enable) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("periphery_controller_connection", "drone_controller.PeripheryController.interface", transport, riid);

    struct PeripheryControllerInterface_proxy proxy;
    PeripheryControllerInterface_proxy_init(&proxy, &transport.base, riid);

    PeripheryControllerInterface_SetCargoLock_req req;
    PeripheryControllerInterface_SetCargoLock_res res;

    req.enable = enable;

    return ((PeripheryControllerInterface_SetCargoLock(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
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