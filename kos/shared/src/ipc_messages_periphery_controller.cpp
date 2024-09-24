/**
 * \file
 * \~English \brief Implementation of wrapper methods that send IPC messages to PeripheryController component.
 * \~Russian \brief Реализация методов-оберток для отправки IPC-сообщений компоненту PeripheryController.
 */

#include "../include/ipc_messages_periphery_controller.h"
#include "../include/initialization_interface.h"

#include <stddef.h>

#define NK_USE_UNQUALIFIED_NAMES
#include <drone_controller/PeripheryControllerInterface.idl.h>

int enableBuzzer() {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("periphery_controller_connection", "drone_controller.PeripheryController.interface", transport, riid);

    struct PeripheryControllerInterface_proxy proxy;
    PeripheryControllerInterface_proxy_init(&proxy, &transport.base, riid);

    PeripheryControllerInterface_EnableBuzzer_req req;
    PeripheryControllerInterface_EnableBuzzer_res res;

    return ((PeripheryControllerInterface_EnableBuzzer(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int setKillSwitch(uint8_t enable) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("periphery_controller_connection", "drone_controller.PeripheryController.interface", transport, riid);

    struct PeripheryControllerInterface_proxy proxy;
    PeripheryControllerInterface_proxy_init(&proxy, &transport.base, riid);

    PeripheryControllerInterface_SetKillSwitch_req req;
    PeripheryControllerInterface_SetKillSwitch_res res;

    req.enable = enable;

    return ((PeripheryControllerInterface_SetKillSwitch(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}

int setCargoLock(uint8_t enable) {
    NkKosTransport transport;
    nk_iid_t riid;
    initSenderInterface("periphery_controller_connection", "drone_controller.PeripheryController.interface", transport, riid);

    struct PeripheryControllerInterface_proxy proxy;
    PeripheryControllerInterface_proxy_init(&proxy, &transport.base, riid);

    PeripheryControllerInterface_SetCargoLock_req req;
    PeripheryControllerInterface_SetCargoLock_res res;

    req.enable = enable;

    return ((PeripheryControllerInterface_SetCargoLock(&proxy.base, &req, NULL, &res, NULL) == rcOk) && res.success);
}