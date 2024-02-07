#include "sdk_net.h"
#include <kos_net.h>
#include <string.h>

char serverIP[] = "192.168.1.78";
uint8_t serverPort = 80;
uint16_t responseBufferSize = 1024;

void setServerIP(char* address) {
    strcpy(serverIP, address);
}

void setServerPort(uint8_t port) {
    serverPort = port;
}

int sendRequest(char* query, char* response) {
    int socketDesc = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socketDesc < 0) {
        fprintf(stderr, "Failed to create a socket: %s\n", strerror(errno));
        return 0;
    }

    sockaddr_in serverAddress = {0};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(serverPort);
    serverAddress.sin_addr.s_addr = inet_addr(serverIP);
    if (connect(socketDesc, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0) {
	    fprintf(stderr, "Failed to connect: %s\n", strerror(errno));
		close(socketDesc);
        return 0;
	}

    char request[responseBufferSize] = {0};
    strcat(request, "GET /");
    strcat(request, query);
    strcat(request, " HTTP/1.1\r\nHost: ");
    strcat(request, serverIP);
    strcat(request, "\r\nConnection: close\r\n\r\n");
    if (send(socketDesc, request, sizeof(request), 0) < 0) {
		fprintf(stderr, "Failed to send a request: %s\n", strerror(errno));
	    close(socketDesc);
		return 0;
	}

    ssize_t responseLength;
    char buffer[responseBufferSize];
    strcpy(response, "");
    while ((responseLength = recv(socketDesc, buffer, sizeof(buffer), 0)) > 0)
        strncat(response, buffer, responseLength);
    close(socketDesc);
    return 1;
}