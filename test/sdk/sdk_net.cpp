#include "sdk_net.h"
#include "sdk_authenticity.h"
#include <kos_net.h>
#include <string.h>

char boardId[] = "id=1";
char serverIp[16] = "192.168.0.105";
uint8_t serverPort = 80;
uint16_t bufferSize = 2048;

void setServerIP(char* address) {
    strcpy(serverIp, address);
}

void setServerPort(uint8_t port) {
    serverPort = port;
}

int sendRequest(char* method, char* response, int auth) {
    return sendRequest(method, "", response, auth);
}

int sendRequest(char* method, char* query, char* response, int auth) {
    char message[bufferSize] = {0};
    if (strlen(query))
        snprintf(message, bufferSize, "%s?%s&%s", method, boardId, query);
    else
        snprintf(message, bufferSize, "%s?%s", method, boardId);

    char request[bufferSize] = {0};
    if (auth) {
        char signature[bufferSize] = {0};
        if (!signMessage(message, signature))
            return 0;
        snprintf(request, bufferSize, "GET /%s&sig=0x%s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", message, signature, serverIp);
    }
    else
        snprintf(request, bufferSize, "GET /%s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", message, serverIp);
    //fprintf(stderr, "DEBUG: request content: %s\n", request);

    int socketDesc = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socketDesc < 0) {
        fprintf(stderr, "Failed to create a socket: %s\n", strerror(errno));
        return 0;
    }

    sockaddr_in serverAddress = {0};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(serverPort);
    serverAddress.sin_addr.s_addr = inet_addr(serverIp);
    if (connect(socketDesc, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0) {
	    fprintf(stderr, "Failed to connect: %s\n", strerror(errno));
		close(socketDesc);
        return 0;
	}

    if (send(socketDesc, request, sizeof(request), 0) < 0) {
		fprintf(stderr, "Failed to send a request: %s\n", strerror(errno));
	    close(socketDesc);
		return 0;
	}

    ssize_t responseLength;
    char buffer[bufferSize] = {0};
    strcpy(response, "");
    while ((responseLength = recv(socketDesc, buffer, sizeof(buffer), 0)) > 0)
        strncat(response, buffer, responseLength);
    close(socketDesc);
    //fprintf(stderr, "DEBUG: response content: %s\n", response);

    if (auth) {
        char* content = strstr(response, "$");
        if ((content == NULL) || !checkSignature(content)) {
            fprintf(stderr, "Error: authenticity of received response was not confirmed\n");
            return 0;
        }
    }

    return 1;
}