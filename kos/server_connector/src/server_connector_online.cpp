#include "../include/server_connector.h"

#include <kos_net.h>

#define BUFFER_SIZE 1024

uint16_t serverPort = 8080;

int initServerConnector() {
    if (!wait_for_network()) {
        logEntry("Connection to network has failed", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    return 1;
}

int sendRequest(char* query, char* response) {
    char request[BUFFER_SIZE] = {0};
    snprintf(request, BUFFER_SIZE, "GET %s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", query, SERVER_IP);

    int socketDesc = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socketDesc < 0) {
        logEntry("Failed to create a socket", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    sockaddr_in serverAddress = {0};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(serverPort);
    serverAddress.sin_addr.s_addr = inet_addr(SERVER_IP);
    if (connect(socketDesc, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0) {
        char logBuffer[256];
        snprintf(logBuffer, 256, "Connection to %s:%d has failed", SERVER_IP, serverPort);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        close(socketDesc);
        return 0;
    }

    if (send(socketDesc, request, sizeof(request), 0) < 0) {
        logEntry("Failed to send a request", ENTITY_NAME, LogLevel::LOG_WARNING);
        close(socketDesc);
        return 0;
    }

    ssize_t contentLength;
    char buffer[BUFFER_SIZE] = {0};
    char content[BUFFER_SIZE] = {0};
    while ((contentLength = recv(socketDesc, buffer, sizeof(buffer), 0)) > 0)
        strncat(content, buffer, contentLength);
    close(socketDesc);

    char* msg = strstr(content, "$");
    if (msg == NULL) {
        logEntry("Failed to parse response content", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    strcpy(response, msg);

    return 1;
}