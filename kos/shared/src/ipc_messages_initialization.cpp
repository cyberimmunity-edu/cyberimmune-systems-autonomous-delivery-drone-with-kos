/**
 * \file
 * \~English \brief Implementation of wrapper methods that send IPC messages
 * to any security module component (except for FlightController).
 * \~Russian \brief Реализация методов-оберток, общих для всех компонентов модуля безопасности (кроме FlightController).
 */

#include "../include/ipc_messages_initialization.h"
#include "../include/initialization_interface.h"

#include <stddef.h>

/** \cond */
#define MAX_METHOD_IMPL_LEN 128
/** \endcond */

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/Initialization.idl.h>

int waitForInit(const char* connection, const char* receiverEntity) {
    char destination[MAX_METHOD_IMPL_LEN] = {0};
    snprintf(destination, MAX_METHOD_IMPL_LEN, "drone_controller.%s.waitForInit", receiverEntity);

    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface(connection, destination, transport, riid);

    struct Initialization_proxy proxy;
    Initialization_proxy_init(&proxy, &transport.base, riid);

    Initialization_WaitForInit_req req;
    Initialization_WaitForInit_res res;

    return ((Initialization_WaitForInit(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}