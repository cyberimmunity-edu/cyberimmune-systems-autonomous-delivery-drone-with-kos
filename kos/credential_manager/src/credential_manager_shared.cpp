#include "../include/credential_manager.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <mbedtls_v3/ctr_drbg.h>
#include <mbedtls_v3/entropy.h>
#include <mbedtls_v3/rsa.h>
#include <mbedtls_v3/sha256.h>

#include <string.h>
#include <unistd.h>

mbedtls_rsa_context rsaSelf;
char keyE[257] = {0};
char keyN[257] = {0};
char keyD[257] = {0};

void hashToKey(uint8_t* source, uint32_t sourceSize, uint8_t* destination) {
    int j = 127;
    for (int i = sourceSize - 1; (i >= 0) && (j >= 0); i--) {
        destination[j] = source[i];
        j--;
    }
}

void bytesToString(uint8_t* source, char* destination) {
    int start = 0;
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
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context drbg;
    mbedtls_mpi N, E, D;

    mbedtls_entropy_init(&entropy);
    mbedtls_ctr_drbg_init(&drbg);
    mbedtls_rsa_init(&rsaSelf);
    mbedtls_mpi_init(&N);
    mbedtls_mpi_init(&E);
    mbedtls_mpi_init(&D);

    if (mbedtls_ctr_drbg_seed(&drbg, mbedtls_entropy_func, &entropy, (unsigned char*)BOARD_ID, strlen(BOARD_ID)) != 0) {
        fprintf(stderr, "[%s] Error: Failed to get drbg seed\n", ENTITY_NAME);
        mbedtls_entropy_free(&entropy);
        mbedtls_ctr_drbg_free(&drbg);
        mbedtls_mpi_free(&N);
        mbedtls_mpi_free(&E);
        mbedtls_mpi_free(&D);
        return 0;
    }
    if(mbedtls_rsa_gen_key(&rsaSelf, mbedtls_ctr_drbg_random, &drbg, 1024, 65537) != 0) {
        fprintf(stderr, "[%s] Error: Failed to generate RSA key\n", ENTITY_NAME);
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
    char rsaServerRequest[1024] = {0};
    char rsaServerResponse[1024] = {0};
    snprintf(rsaServerRequest, 1024, "/api/key?%s&e=0x%s&n=0x%s", BOARD_ID, keyE, keyN);
    while (!sendRequest(rsaServerRequest, rsaServerResponse)) {
        fprintf(stderr, "[%s] Warning: Failed to share RSA key. Trying again in 1s\n", ENTITY_NAME);
        sleep(1);
    }
    return setRsaKey(rsaServerResponse);
}

int signMessage(char* message, char* sign) {
    uint8_t hash[32] = {0};
    mbedtls_sha256_context sha256;
    mbedtls_sha256_init(&sha256);
    if (mbedtls_sha256_starts(&sha256, 0) != 0) {
        fprintf(stderr, "[%s] Warning: Failed to calculate message hash\n", ENTITY_NAME);
        mbedtls_sha256_free(&sha256);
        return 0;
    }
    if (mbedtls_sha256_update(&sha256, (unsigned char*)message, strlen(message)) != 0) {
        fprintf(stderr, "[%s] Warning: Failed to calculate message hash\n", ENTITY_NAME);
        mbedtls_sha256_free(&sha256);
        return 0;
    }
    if (mbedtls_sha256_finish(&sha256, hash) != 0) {
        fprintf(stderr, "[%s] Warning: Failed to calculate message hash\n", ENTITY_NAME);
        mbedtls_sha256_free(&sha256);
        return 0;
    }
    mbedtls_sha256_free(&sha256);

    uint8_t key[128] = {0};
    hashToKey(hash, 32, key);

    uint8_t result[128] = {0};
    if (mbedtls_rsa_public(&rsaSelf, key, result) != 0) {
        fprintf(stderr, "[%s] Warning: Failed to sign message\n", ENTITY_NAME);
        return 0;
    }

    bytesToString(result, sign);

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