#include "../include/credential_manager.h"

#include <mbedtls_v3/rsa.h>
#include <mbedtls_v3/sha256.h>

#include <fcntl.h>
#include <unistd.h>
#include <string.h>

mbedtls_rsa_context rsaServer;

uint8_t hexCharToInt(char c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    else if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    else if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    else {
        char logBuffer[256];
        snprintf(logBuffer, 256, "%c is not a viable hex value", c);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
}

void stringToBytes(char* source, uint32_t sourceSize, uint8_t* destination) {
    int j = 127;
    for (int32_t i = sourceSize - 1; i >= 0; i -= 2) {
        if (j < 0) {
            char logBuffer[512];
            snprintf(logBuffer, 512, "String '%s' contains more bytes than expected", source);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            return;
        }
        if (i > 0)
            destination[j] = 16 * hexCharToInt(source[i - 1]) + hexCharToInt(source[i]);
        else
            destination[j] = hexCharToInt(source[i]);
        j--;
    }
}

int getRsaKey() {
    int file = open("/rsa", O_RDONLY);
    if (file == -1) {
        if (!generateRsaKey())
            return 0;
        file = open("/rsa", O_RDWR | O_CREAT);
        if (file == -1) {
            logEntry("Failed to create file to store generated RSA key", ENTITY_NAME, LogLevel::LOG_ERROR);
            return 0;
        }
        char key[1024] = {0};
        sprintf(key, "%s\n%s\n%s\n", getKeyN(), getKeyE(), getKeyD());
        uint32_t len = strlen(key);
        if (write(file, key, len) != len) {
            logEntry("Failed to store RSA key in file", ENTITY_NAME, LogLevel::LOG_ERROR);
            close(file);
            return 0;
        }
        close(file);
        return 1;
    }
    else {
        char keys[3][257] = {0};
        uint32_t lens[3] = {0};
        for (int i = 0; i < 3; i++) {
            int j = 0;
            while (true) {
                char letter;
                if (read(file, &letter, 1) != 1) {
                    logEntry("Failed to read RSA key from file", ENTITY_NAME, LogLevel::LOG_ERROR);
                    close(file);
                    return 0;
                }
                if ((letter == '\n') || (letter == '\0'))
                    break;
                keys[i][j] = letter;
                j++;
            }
            keys[i][j] = '\0';
            lens[i] = j;
        }
        close(file);

        uint8_t N[128] = {0};
        uint8_t D[128] = {0};

        stringToBytes(keys[0], lens[0], N);
        stringToBytes(keys[2], lens[2], D);

        return loadRsaKey(N, D, keys[0], keys[1], lens[0], lens[1]);
    }
}

int setRsaKey(char* key) {
    char header[] = "$Key: ";
    char* nStart = strstr(key, header);

    if (nStart == NULL) {
        logEntry("Failed to parse public RSA key received from the server", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    nStart += strlen(header);
    char* eStart = strstr(nStart, " ");

    if (eStart == NULL) {
        logEntry("Failed to parse public RSA key received from the server", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    uint8_t N[128] = {0};
    uint8_t E[128] = {0};

    stringToBytes(nStart, eStart - nStart, N);
    eStart++;
    stringToBytes(eStart, strlen(eStart), E);

    mbedtls_rsa_init(&rsaServer);
    mbedtls_rsa_import_raw(&rsaServer, N, 128, NULL, 0, NULL, 0, NULL, 0, E, 128);

    return 1;
}

int checkSignature(char* message, uint8_t &correct) {
    correct = 0;

    char* signatureStart = strstr(message, "#");
    if (signatureStart == NULL) {
        logEntry("Received mission has no signature", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    uint32_t messageLength = signatureStart - message;

    uint8_t hash[32] = {0};
    mbedtls_sha256_context sha256;
    mbedtls_sha256_init(&sha256);
    if (mbedtls_sha256_starts(&sha256, 0) != 0) {
        logEntry("Failed to calculate hash of received message", ENTITY_NAME, LogLevel::LOG_WARNING);
        mbedtls_sha256_free(&sha256);
        return 0;
    }
    if (mbedtls_sha256_update(&sha256, (unsigned char*)message, messageLength) != 0) {
        logEntry("Failed to calculate hash of received message", ENTITY_NAME, LogLevel::LOG_WARNING);
        mbedtls_sha256_free(&sha256);
        return 0;
    }
    if (mbedtls_sha256_finish(&sha256, hash) != 0) {
        logEntry("Failed to calculate hash of received message", ENTITY_NAME, LogLevel::LOG_WARNING);
        mbedtls_sha256_free(&sha256);
        return 0;
    }
    mbedtls_sha256_free(&sha256);

    uint8_t signature[128] = {0};
    uint8_t result[128] = {0};
    signatureStart++;
    stringToBytes(signatureStart, strlen(signatureStart), signature);
    if (mbedtls_rsa_public(&rsaServer, signature, result) != 0) {
        logEntry("Failed to decode server signature", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    int j = 31;
    for (int i = 127; i >= 0; i--) {
        int check = (j >= 0) ? (result[i] == hash[j]) : !result[i];
        if (!check) {
            logEntry("Authenticity is not confirmed", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        j--;
    }

    correct = 1;
    return 1;
}