/**
 * \file
 * \~English \brief Implementation of ServerConnectorInterface IDL-interface methods.
 * \~Russian \brief Реализация методов IDL-интерфейса ServerConnectorInterface.
 */

#include "../include/server_connector_interface.h"
#include "../include/server_connector.h"

#include <string.h>

nk_err_t GetBoardIdImpl(struct ServerConnectorInterface *self,
                    const ServerConnectorInterface_GetBoardId_req *req, const struct nk_arena *reqArena,
                    ServerConnectorInterface_GetBoardId_res *res, struct nk_arena *resArena) {
    char* id = getBoardName();
    uint8_t len = strlen(id);
    res->success = (len > 0);

    nk_char_t *msg = nk_arena_alloc(nk_char_t, resArena, &(res->id), len + 1);
    if (msg == NULL)
        return NK_EBADMSG;
    else if (len > ServerConnectorInterface_MaxIdLength)
        return NK_ENOMEM;
    strncpy(msg, id, len);

    return NK_EOK;
}

nk_err_t SendRequestImpl(struct ServerConnectorInterface *self,
                    const ServerConnectorInterface_SendRequest_req *req, const struct nk_arena *reqArena,
                    ServerConnectorInterface_SendRequest_res *res, struct nk_arena *resArena) {
    char query[ServerConnectorInterface_MaxQueryLength + 1] = {0};
    char response[ServerConnectorInterface_MaxResponseLength + 1] = {0};

    nk_uint32_t len = 0;
    nk_char_t *msg = nk_arena_get(nk_char_t, reqArena, &(req->query), &len);
    if (msg == NULL)
        return NK_EBADMSG;
    else if (len > ServerConnectorInterface_MaxQueryLength)
        return NK_ENOMEM;
    strncpy(query, msg, len);

    res->success = requestServer(query, response, ServerConnectorInterface_MaxResponseLength + 1);

    len = strlen(response);
    msg = nk_arena_alloc(nk_char_t, resArena, &(res->response), len + 1);
    if (msg == NULL)
        return NK_EBADMSG;
    else if (len > ServerConnectorInterface_MaxResponseLength)
        return NK_ENOMEM;
    strncpy(msg, response, len);

    return NK_EOK;
}

nk_err_t PublishMessageImpl(struct ServerConnectorInterface *self,
                    const ServerConnectorInterface_PublishMessage_req *req, const struct nk_arena *reqArena,
                    ServerConnectorInterface_PublishMessage_res *res, struct nk_arena *resArena) {
    char topic[ServerConnectorInterface_MaxTopicLength + 1] = {0};
    char publication[ServerConnectorInterface_MaxPublicationLength + 1] = {0};

    nk_uint32_t len = 0;
    nk_char_t *msg = nk_arena_get(nk_char_t, reqArena, &(req->topic), &len);
    if (msg == NULL)
        return NK_EBADMSG;
    else if (len > ServerConnectorInterface_MaxTopicLength)
        return NK_ENOMEM;
    strncpy(topic, msg, len);

    len = 0;
    msg = nk_arena_get(nk_char_t, reqArena, &(req->publication), &len);
    if (msg == NULL)
        return NK_EBADMSG;
    else if (len > ServerConnectorInterface_MaxPublicationLength)
        return NK_ENOMEM;
    strncpy(publication, msg, len);

    res->success = publish(topic, publication);
    return NK_EOK;
}