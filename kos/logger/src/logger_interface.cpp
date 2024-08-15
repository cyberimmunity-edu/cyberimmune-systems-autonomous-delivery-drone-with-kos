/**
 * \file
 * \~English \brief Implementation of LoggerInterface IDL-interface methods.
 * \~Russian \brief Реализация методов IDL-интерфейса LoggerInterface.
 */

#include "../include/logger_interface.h"
#include "../include/logger.h"

#include <string.h>

nk_err_t LogImpl(struct LoggerInterface *self,
                    const LoggerInterface_Log_req *req, const struct nk_arena *reqArena,
                    LoggerInterface_Log_res *res, struct nk_arena *resArena) {
    char message[LoggerInterface_MaxLogEntry] = {0};

    nk_uint32_t len = 0;
    nk_char_t *msg = nk_arena_get(nk_char_t, reqArena, &(req->logEntry), &len);
    if (msg == NULL)
        return NK_EBADMSG;
    strcpy(message, msg);

    res->success = addLogEntry(message, req->logLevel);

    return NK_EOK;
}