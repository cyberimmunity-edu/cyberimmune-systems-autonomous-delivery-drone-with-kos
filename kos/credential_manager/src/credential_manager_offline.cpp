/**
 * \file
 * \~English
 * \brief Implementation of methods for RSA message signature check simulation.
 * \details The file contains implementation of methods, that simulate
 * message authenticity check. These methods are intended for offline build,
 * where there is no connection with the ATM server, and therefore any message is considered authentic.
 *
 * \~Russian
 * \brief Реализация методов для имитации проверки RSA-подписей сообщений.
 * \details В файле реализованы методы, имитирующие проверку аутентичности сообщений.
 * Эти методы предназначены для оффлайн-сборки, где отсутствует связь с сервером ОРВД,
 * а потому любые сообщения считаются аутентичными.
 */

#include "../include/credential_manager.h"

int getRsaKey() {
    return generateRsaKey();
}

int setRsaKey(char* key) {
    return 1;
}

int checkMessageSignature(char* message, uint8_t &correct) {
    correct = 1;
    return 1;
}