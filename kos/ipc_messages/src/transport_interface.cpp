#include "../include/transport_interface.h"

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