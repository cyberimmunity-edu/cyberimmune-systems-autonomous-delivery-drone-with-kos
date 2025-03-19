/**
 * \file
 * \~English
 * \brief Implementation of methods for managing security module RSA keys.
 * \details The file contains implementation of methods, that read/write RSA key from/to a file,
 * generate the key and sign messages for the ATM server.
 * message authenticity check. These methods are intended for offline build,
 * where there is no connection with the ATM server, and therefore any message is considered authentic.
 *
 * \~Russian
 * \brief Реализация методов для работы с RSA-ключами модуля безопасности.
 * \details В файле реализованы методы, предназначенные для чтения/записи RSA-ключа из/в файл,
 * их генерации и подписи сообщений, предназначенных для отправки серверу ОРВД.
 */

#include "../include/credential_manager.h"
#include "../../shared/include/ipc_messages_initialization.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>
#include <mbedtls/rsa.h>
#include <mbedtls/sha256.h>

#include <string.h>
#include <unistd.h>

/** \cond */
mbedtls_rsa_context rsaSelf;
char keyN[257] = {0};
char keyE[257] = {0};
char keyD[257] = {0};
/** \endcond */

/**
 * \~English Converts a decimal array into a hexadecimal string.
 * \details The method is designed to convert a message sign, that will be sent to the ATM server,
 * and therefore it is designed to work with arrays no longer than 128 bytes.
 * If the length is less than 128 bytes, the missing zeros are assumed to precede the array.
 * \param[in] source Pointer to a decimal array.
 * \param[in] sourceSize Size of the decimal array.
 * \param[out] destination Pointer to a string to write hexadecimal result.
 * \param[in] destinationSize Size of a string to write the result.
 * \~Russian Переводит 10-ричный массив в 16-ричную строку.
 * \details Метод предназначен для перевода подписи сообщения, предназначенного для сервера ОРВД,
 * поэтому рассчитан на работу с массивами в 128 байт. Если длина меньше 128 байт,
 * считается, что перед массивом находятся недостающие нули.
 * \param[in] source Указатель на 10-ричный массив.
 * \param[in] sourceSize Размер 10-ричного массива.
 * \param[out] destination Указатель на строку, куда будет записан 16-ричный результат.
 * \param[in] destinationSize Размер строки для записи результата.
 */
void bytesToString(uint8_t* source, uint32_t sourceSize, char* destination, uint32_t destinationSize) {
    int start = 0;
    int end = 128;
    if (sourceSize < end)
        end = sourceSize;
    if (sourceSize * 2 + 1 > destinationSize) {
        logEntry("Cannot convert decimal array into a hexadecimal string: destination string is too short", ENTITY_NAME, LogLevel::LOG_WARNING);
        return;
    }
    for (int i = 0; i < 128; i++) {
        if (!start && source[i])
            start = 1;
        if (start) {
            char hex[3] = {0};
            sprintf(hex, "%02x", source[i]);
            strcat(destination, hex);
        }
    }
}

int generateRsaKey() {
    while (!waitForInit("server_connector_connection", "ServerConnector")) {
        logEntry("Failed to receive initialization notification from ServerConnector. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context drbg;
    mbedtls_mpi N, E, D;

    mbedtls_entropy_init(&entropy);
    mbedtls_ctr_drbg_init(&drbg);
    mbedtls_rsa_init(&rsaSelf);
    mbedtls_mpi_init(&N);
    mbedtls_mpi_init(&E);
    mbedtls_mpi_init(&D);

    char boardId[32] = {0};
    while (!getBoardId(boardId)) {
        logEntry("Failed to get board ID from ServerConnector. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    if (mbedtls_ctr_drbg_seed(&drbg, mbedtls_entropy_func, &entropy, (unsigned char*)boardId, strlen(boardId)) != 0) {
        logEntry("Failed to get drbg seed", ENTITY_NAME, LogLevel::LOG_ERROR);
        mbedtls_entropy_free(&entropy);
        mbedtls_ctr_drbg_free(&drbg);
        mbedtls_mpi_free(&N);
        mbedtls_mpi_free(&E);
        mbedtls_mpi_free(&D);
        return 0;
    }
    if(mbedtls_rsa_gen_key(&rsaSelf, mbedtls_ctr_drbg_random, &drbg, 1024, 65537) != 0) {
        logEntry("Failed to generate RSA key", ENTITY_NAME, LogLevel::LOG_ERROR);
        mbedtls_entropy_free(&entropy);
        mbedtls_ctr_drbg_free(&drbg);
        mbedtls_mpi_free(&N);
        mbedtls_mpi_free(&E);
        mbedtls_mpi_free(&D);
        return 0;
    }

    mbedtls_rsa_export(&rsaSelf, &N, NULL, NULL, &D, &E);

    size_t resSize;
    mbedtls_mpi_write_string(&E, 16, keyE, 1024, &resSize);
    mbedtls_mpi_write_string(&N, 16, keyN, 1024, &resSize);
    mbedtls_mpi_write_string(&D, 16, keyD, 1024, &resSize);

    mbedtls_rsa_import(&rsaSelf, NULL, NULL, NULL, NULL, &D);

    mbedtls_entropy_free(&entropy);
    mbedtls_ctr_drbg_free(&drbg);
    mbedtls_mpi_free(&N);
    mbedtls_mpi_free(&E);
    mbedtls_mpi_free(&D);

    return 1;
}

int loadRsaKey(uint8_t* N, uint8_t* D, char* n, char* e, uint32_t nLen, uint32_t eLen) {
    mbedtls_rsa_init(&rsaSelf);
    mbedtls_rsa_import_raw(&rsaSelf, N, 128, NULL, 0, NULL, 0, NULL, 0, D, 128);

    for (int i = 0; i < nLen; i++)
        keyN[i] = n[i];
    for (int i = 0; i < eLen; i++)
        keyE[i] = e[i];

    return 1;
}

int shareRsaKey() {
    char boardId[32] = {0};
    while (!getBoardId(boardId)) {
        logEntry("Failed to get board ID from ServerConnector. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }

    char request[1024] = {0};
    char response[1024] = {0};
    snprintf(request, 1024, "/api/key?id=%s&e=0x%s&n=0x%s", boardId, keyE, keyN);
    while (!sendRequest(request, response, 1024) || !strcmp(response, "TIMEOUT")) {
        logEntry("Failed to share RSA key. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }
    return setRsaKey(response);
}

int getMessageSignature(char* message, char* sign) {
    uint8_t hash[32] = {0};
    mbedtls_sha256_context sha256;
    mbedtls_sha256_init(&sha256);
    if (mbedtls_sha256_starts(&sha256, 0) != 0) {
        logEntry("Failed to calculate message hash", ENTITY_NAME, LogLevel::LOG_WARNING);
        mbedtls_sha256_free(&sha256);
        return 0;
    }
    if (mbedtls_sha256_update(&sha256, (unsigned char*)message, strlen(message)) != 0) {
        logEntry("Failed to calculate message hash", ENTITY_NAME, LogLevel::LOG_WARNING);
        mbedtls_sha256_free(&sha256);
        return 0;
    }
    if (mbedtls_sha256_finish(&sha256, hash) != 0) {
        logEntry("Failed to calculate message hash", ENTITY_NAME, LogLevel::LOG_WARNING);
        mbedtls_sha256_free(&sha256);
        return 0;
    }
    mbedtls_sha256_free(&sha256);

    uint8_t key[128] = {0};
    memcpy(key + 96, hash, 32);

    uint8_t result[128] = {0};
    if (mbedtls_rsa_public(&rsaSelf, key, result) != 0) {
        logEntry("Failed to sign message", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    bytesToString(result, 128, sign, 257);

    return 1;
}

char* getKeyN() {
    return keyN;
}

char* getKeyE() {
    return keyE;
}

char* getKeyD() {
    return keyD;
}