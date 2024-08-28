/**
 * \file
 * \~English \brief Implementation of Initialization IDL-interface methods and methods
 * for wrappers over IPC message handlers.
 * \~Russian \brief Реализация методов IDL-интерфейса Initialization и методов, необходимых
 * оберткам над обработчиками IPC-сообщений.
 */

#include "../include/initialization_interface.h"

#include <coresrv/sl/sl_api.h>

#include <stddef.h>
#include <assert.h>

void initSenderInterface(const char* connection, const char* endpoint, NkKosTransport &transport, nk_iid_t &riid) {
    Handle handle = ServiceLocatorConnect(connection);
    assert(handle != INVALID_HANDLE);
    NkKosTransport_Init(&transport, handle, NK_NULL, 0);
    riid = ServiceLocatorGetRiid(handle, endpoint);
    assert(riid != INVALID_RIID);
}

void initReceiverInterface(const char* connection, NkKosTransport &transport) {
    ServiceId id;
    Handle handle = ServiceLocatorRegister(connection, NULL, 0, &id);
    assert(handle != INVALID_HANDLE);
    NkKosTransport_Init(&transport, handle, NK_NULL, 0);
}

nk_err_t WaitForInitImpl(struct Initialization *self,
                        const Initialization_WaitForInit_req *req, const struct nk_arena *reqArena,
                        Initialization_WaitForInit_res *res, struct nk_arena *resArena) {
    res->success = true;
    return NK_EOK;
}
