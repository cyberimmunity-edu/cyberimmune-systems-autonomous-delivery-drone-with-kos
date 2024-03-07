#include "../include/server_connector.h"

#include <kos_net.h>

#define BUFFER_SIZE 1024

char serverIp[] = "192.168.1.78";
uint8_t serverPort = 80;

int initServerConnector() {
    if (!wait_for_network()) {
        fprintf(stderr, "[%s] Error: Connection to network has failed\n", ENTITY_NAME);
        return 0;
    }

    return 1;
}

int sendRequest(char* query, char* response) {
    char request[BUFFER_SIZE] = {0};
    snprintf(request, BUFFER_SIZE, "GET /%s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", query, serverIp);

    int socketDesc = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socketDesc < 0) {
        fprintf(stderr, "[%s] Warning: Failed to create a socket\n", ENTITY_NAME);
        return 0;
    }

    sockaddr_in serverAddress = {0};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(serverPort);
    serverAddress.sin_addr.s_addr = inet_addr(serverIp);
    if (connect(socketDesc, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0) {
	    fprintf(stderr, "[%s] Warning: Connection to %s:%d has failed\n", ENTITY_NAME, serverIp, serverPort);
		close(socketDesc);
        return 0;
	}

    if (send(socketDesc, request, sizeof(request), 0) < 0) {
		fprintf(stderr, "[%s] Warning: Failed to send a request\n", ENTITY_NAME);
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
        fprintf(stderr, "[%s] Warning: Failed to parse response content\n", ENTITY_NAME);
		return 0;
    }

    strcpy(response, msg);

    return 1;
}