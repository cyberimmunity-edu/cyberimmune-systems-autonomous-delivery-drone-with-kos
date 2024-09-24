/**
 * \file
 * \~English \brief Implementation of wrapper methods that send IPC messages to Logger component.
 * \~Russian \brief Реализация методов-оберток для отправки IPC-сообщений компоненту Logger.
 */

#include "../include/ipc_messages_logger.h"
#include "../include/initialization_interface.h"

#include <string.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/LoggerInterface.idl.h>

int logEntry(char* entry, char* entity, LogLevel level) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("logger_connection", "drone_controller.Logger.interface", transport, riid);

    struct LoggerInterface_proxy proxy;
    LoggerInterface_proxy_init(&proxy, &transport.base, riid);

    LoggerInterface_Log_req req;
    LoggerInterface_Log_res res;
    char reqBuffer[LoggerInterface_Log_req_arena_size];
    struct nk_arena reqArena = NK_ARENA_INITIALIZER(reqBuffer, reqBuffer + sizeof(reqBuffer));
    nk_arena_reset(&reqArena);

    char message[MAX_LOG_BUFFER + 1] = {0};
    snprintf(message, MAX_LOG_BUFFER, "[%s] %s", entity, entry);

    nk_uint32_t len = strlen(message);
    nk_char_t *msg = nk_arena_alloc(nk_char_t, &reqArena, &(req.logEntry), len + 1);
    if ((msg == NULL) || (len > LoggerInterface_Log_req_arena_size))
        return 0;
    strncpy(msg, message, len);

    req.logLevel = (uint8_t)level;

    return ((LoggerInterface_Log(&proxy.base, &req, &reqArena, &res, NULL) == rcOk) && res.success);
}