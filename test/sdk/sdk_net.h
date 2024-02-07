#pragma once
#include <stdlib.h>

void setServerIP(char* address);
void setServerPort(uint8_t port);

int sendRequest(char* query, char* response);