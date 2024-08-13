#pragma once

#include "../../shared/include/ipc_messages_logger.h"

int initServerConnector();

void setBoardName(char* id);
char* getBoardName();

int sendRequest(char* query, char* response);