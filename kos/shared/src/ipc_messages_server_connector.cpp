#include "../include/ipc_messages_server_connector.h"
#include "../include/initialization_interface.h"

#include <string.h>
#include <stddef.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/ServerConnectorInterface.idl.h>

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
