#pragma once

#include <coresrv/sl/sl_api.h>
#include <coresrv/nk/transport-kos.h>

void initSenderInterface(const char* connection, const char* endpoint, NkKosTransport &transport, nk_iid_t &riid);
void initReceiverInterface(const char* connection, NkKosTransport &transport);