#pragma once
#include <stdint.h>

void setServerIP(char* address);
void setServerPort(uint8_t port);

int sendRequest(char* method, char* response, int auth = 1);
int sendRequest(char* method, char* query, char* response, int auth = 1);