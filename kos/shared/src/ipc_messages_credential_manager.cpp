#include "../include/ipc_messages_credential_manager.h"
#include "../include/initialization_interface.h"

#include <string.h>
#include <stddef.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/CredentialManagerInterface.idl.h>

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