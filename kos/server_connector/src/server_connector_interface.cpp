#include "../include/server_connector_interface.h"
#include "../include/server_connector.h"

#include <string.h>

nk_err_t GetBoardIdImpl(struct ServerConnectorInterface *self,
                    const ServerConnectorInterface_GetBoardId_req *req, const struct nk_arena *reqArena,
                    ServerConnectorInterface_GetBoardId_res *res, struct nk_arena *resArena) {
    char id[ServerConnectorInterface_MaxIdLength] = {0};

    strcpy(id, getBoardName());
    uint8_t idLen = strlen(id);
    res->success = (idLen > 0);

    nk_char_t *msg = nk_arena_alloc(nk_char_t, resArena, &(res->id), idLen + 1);
    if (msg == NULL)
        return NK_EBADMSG;
    strcpy(msg, id);

    return NK_EOK;
}

nk_err_t SendRequestImpl(struct ServerConnectorInterface *self,
                    const ServerConnectorInterface_SendRequest_req *req, const struct nk_arena *reqArena,
                    ServerConnectorInterface_SendRequest_res *res, struct nk_arena *resArena) {
    char query[ServerConnectorInterface_MaxQueryLength] = {0};
    char response[ServerConnectorInterface_MaxResponseLength] = {0};

    nk_uint32_t len = 0;
    nk_char_t *msg = nk_arena_get(nk_char_t, reqArena, &(req->query), &len);
    if (msg == NULL)
        return NK_EBADMSG;
    strcpy(query, msg);

    res->success = sendRequest(query, response);

    msg = nk_arena_alloc(nk_char_t, resArena, &(res->response), strlen(response) + 1);
    if (msg == NULL)
        return NK_EBADMSG;
    strcpy(msg, response);

    return NK_EOK;
}