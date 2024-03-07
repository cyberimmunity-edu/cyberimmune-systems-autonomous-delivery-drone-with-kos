#include "../include/initialization_interface.h"
#include "../include/transport_interface.h"

#include <string.h>

#define MAX_METHOD_IMPL_LEN 128

nk_err_t WaitForInitImpl(struct Initialization *self,
                        const Initialization_WaitForInit_req *req, const struct nk_arena *reqArena,
                        Initialization_WaitForInit_res *res, struct nk_arena *resArena) {
    res->success = true;
    return NK_EOK;
}

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