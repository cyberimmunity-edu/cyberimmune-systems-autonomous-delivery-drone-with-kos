/**
 * \file
 * \~English \brief Implementation of wrapper methods that send IPC messages to ServerConnector component.
 * \~Russian \brief Реализация методов-оберток для отправки IPC-сообщений компоненту ServerConnector.
 */

#include "../include/ipc_messages_server_connector.h"
#include "../include/initialization_interface.h"

#include <string.h>
#include <stddef.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/ServerConnectorInterface.idl.h>

int getBoardId(char* id) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("server_connector_connection", "drone_controller.ServerConnector.interface", transport, riid);

    struct ServerConnectorInterface_proxy proxy;
    ServerConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    ServerConnectorInterface_GetBoardId_req req;
    ServerConnectorInterface_GetBoardId_res res;
    char resBuffer[ServerConnectorInterface_GetBoardId_res_arena_size];
    struct nk_arena resArena = NK_ARENA_INITIALIZER(resBuffer, resBuffer + sizeof(resBuffer));
    nk_arena_reset(&resArena);

    if ((ServerConnectorInterface_GetBoardId(&proxy.base, &req, NULL, &res, &resArena) != rcOk) || !res.success)
        return 0;

    nk_uint32_t len = 0;
    nk_char_t *msg = nk_arena_get(nk_char_t, &resArena, &(res.id), &len);
    if ((msg == NULL) || (len > ServerConnectorInterface_GetBoardId_res_arena_size))
        return 0;
    strncpy(id, msg, len);

    return 1;
}

int sendRequest(char* query, char* response, uint32_t responseSize) {
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

    nk_uint32_t len = strlen(query);
    nk_char_t *msg = nk_arena_alloc(nk_char_t, &reqArena, &(req.query), len + 1);
    if ((msg == NULL) || (len > ServerConnectorInterface_SendRequest_req_arena_size))
        return 0;
    strncpy(msg, query, len);

    if ((ServerConnectorInterface_SendRequest(&proxy.base, &req, &reqArena, &res, &resArena) != rcOk) || !res.success)
        return 0;

    len = 0;
    msg = nk_arena_get(nk_char_t, &resArena, &(res.response), &len);
    if ((msg == NULL) || (len > responseSize))
        return 0;
    strncpy(response, msg, len);

    return 1;
}

int publishMessage(char* topic, char* publication) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("server_connector_connection", "drone_controller.ServerConnector.interface", transport, riid);

    struct ServerConnectorInterface_proxy proxy;
    ServerConnectorInterface_proxy_init(&proxy, &transport.base, riid);

    ServerConnectorInterface_PublishMessage_req req;
    ServerConnectorInterface_PublishMessage_res res;
    char reqBuffer[ServerConnectorInterface_PublishMessage_req_arena_size];
    struct nk_arena reqArena = NK_ARENA_INITIALIZER(reqBuffer, reqBuffer + sizeof(reqBuffer));
    nk_arena_reset(&reqArena);

    nk_uint32_t len = strlen(topic);
    nk_char_t *msg = nk_arena_alloc(nk_char_t, &reqArena, &(req.topic), len + 1);
    if ((msg == NULL) || (len > ServerConnectorInterface_PublishMessage_req_arena_size))
        return 0;
    strncpy(msg, topic, len);

    len = strlen(publication);
    msg = nk_arena_alloc(nk_char_t, &reqArena, &(req.publication), len + 1);
    if ((msg == NULL) || (len > ServerConnectorInterface_PublishMessage_req_arena_size))
        return 0;
    strncpy(msg, publication, len);

    return ((ServerConnectorInterface_PublishMessage(&proxy.base, &req, &reqArena, &res, NULL) == rcOk) && res.success);
}