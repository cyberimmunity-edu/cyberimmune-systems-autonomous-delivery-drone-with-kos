#include "sdk_authenticity.h"
#include "sdk_net.h"
#include <mbedtls_v3/entropy.h>
#include <mbedtls_v3/ctr_drbg.h>
#include <mbedtls_v3/rsa.h>
#include <mbedtls_v3/sha256.h>
#include <string.h>

mbedtls_rsa_context rsaSelf;
mbedtls_rsa_context rsaServer;

uint8_t hexCharToInt(char c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    else if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    else if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    else {
        fprintf(stderr, "Error: %c is not a viable hex value\n", c);
        return 0;
    }
}

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

void stringToBytes(char* source, uint32_t sourceSize, uint8_t* destination) {
    int j = 127;
    for (int32_t i = sourceSize - 1; i >= 0; i -= 2) {
        if (i > 0)
            destination[j] = 16 * hexCharToInt(source[i - 1]) + hexCharToInt(source[i]);
        else
            destination[j] = hexCharToInt(source[i]);
        j--;
    }
}

int generateRsaKey(char* n, char* e) {
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context drbg;
    mbedtls_mpi N, E, D;

    mbedtls_entropy_init(&entropy);
    mbedtls_ctr_drbg_init(&drbg);
    mbedtls_rsa_init(&rsaSelf);
    mbedtls_mpi_init(&N);
    mbedtls_mpi_init(&E);
    mbedtls_mpi_init(&D);

    char pers[] = "id=1";
    if (mbedtls_ctr_drbg_seed(&drbg, mbedtls_entropy_func, &entropy, (unsigned char*)pers, strlen(pers)) != 0) {
        fprintf(stderr, "Error: failed to generate RSA keys\n");
        return 0;
    }
    if(mbedtls_rsa_gen_key(&rsaSelf, mbedtls_ctr_drbg_random, &drbg, 1024, 65537) != 0) {
        fprintf(stderr, "Error: failed to generate RSA keys\n");
        return 0;
    }

    mbedtls_rsa_export(&rsaSelf, &N, NULL, NULL, &D, &E);

    size_t resSize;
    mbedtls_mpi_write_string(&N, 16, n, 1024, &resSize);
    mbedtls_mpi_write_string(&E, 16, e, 1024, &resSize);

    mbedtls_rsa_import(&rsaSelf, NULL, NULL, NULL, NULL, &D);

    mbedtls_entropy_free(&entropy);
    mbedtls_ctr_drbg_free(&drbg);
    mbedtls_mpi_free(&N);
    mbedtls_mpi_free(&E);
    mbedtls_mpi_free(&D);
    return 1;
}

int shareRsaKey(char* response) {
    char n[257] = {0};
    char e[257] = {0};

    generateRsaKey(n, e);
    char query[1024] = {0};
    snprintf(query, 1024, "n=0x%s&e=0x%s", n, e);
    sendRequest("key", query, response, 0);

    uint8_t N[128] = {0};
    uint8_t E[128] = {0};

    char header[] = "$Key: ";
    char* nStart = strstr(response, header);
    nStart += strlen(header);

    char* eStart = strstr(nStart, " ");
    stringToBytes(nStart, eStart - nStart, N);

    eStart++;
    stringToBytes(eStart, strlen(eStart), E);

    mbedtls_rsa_init(&rsaServer);
    mbedtls_rsa_import_raw(&rsaServer, N, 128, NULL, 0, NULL, 0, NULL, 0, E, 128);

    return 1;
}

int signMessage(char* message, char* signature) {
    uint8_t hash[32] = {0};
    mbedtls_sha256_context sha256;
    mbedtls_sha256_init(&sha256);
    if (mbedtls_sha256_starts(&sha256, 0) != 0) {
        fprintf(stderr, "Error: failed to calculate sha256 hash\n");
        return 0;
    }
    if (mbedtls_sha256_update(&sha256, (unsigned char*)message, strlen(message)) != 0) {
        fprintf(stderr, "Error: failed to calculate sha256 hash\n");
        return 0;
    }
    if (mbedtls_sha256_finish(&sha256, hash) != 0) {
        fprintf(stderr, "Error: failed to calculate sha256 hash\n");
        return 0;
    }
    mbedtls_sha256_free(&sha256);

    uint8_t content[128] = {0};
    uint8_t result[128] = {0};
    hashToKey(hash, 32, content);

    if (mbedtls_rsa_public(&rsaSelf, content, result) != 0) {
        fprintf(stderr, "Error: Failed to sign message\n");
        return 0;
    }

    bytesToString(result, signature);

    return 1;
}

int checkSignature(char* message) {
    char* signatureStart = strstr(message, "#");
    if (signatureStart == NULL) {
        fprintf(stderr, "Error: received mission does not have signature\n");
        return 0;
    }
    uint32_t messageLength = signatureStart - message;

    uint8_t hash[32] = {0};
    mbedtls_sha256_context sha256;
    mbedtls_sha256_init(&sha256);
    if (mbedtls_sha256_starts(&sha256, 0) != 0) {
        fprintf(stderr, "Error: failed to calculate sha256 hash\n");
        return 0;
    }
    if (mbedtls_sha256_update(&sha256, (unsigned char*)message, messageLength) != 0) {
        fprintf(stderr, "Error: failed to calculate sha256 hash\n");
        return 0;
    }
    if (mbedtls_sha256_finish(&sha256, hash) != 0) {
        fprintf(stderr, "Error: failed to calculate sha256 hash\n");
        return 0;
    }
    mbedtls_sha256_free(&sha256);

    uint8_t signature[128] = {0};
    uint8_t result[128] = {0};
    signatureStart++;
    stringToBytes(signatureStart, strlen(signatureStart), signature);
    if (mbedtls_rsa_public(&rsaServer, signature, result) != 0) {
        fprintf(stderr, "Error: Failed to decode server signature\n");
        return 0;
    }

    int j = 31;
    for (int i = 127; i >= 0; i--) {
        int check = (j >= 0) ? (result[i] == hash[j]) : !result[i];
        if (!check) {
            fprintf(stderr, "Error: authenticity is not confirmed\n");
            return 0;
        }
        j--;
    }

    return 1;
}