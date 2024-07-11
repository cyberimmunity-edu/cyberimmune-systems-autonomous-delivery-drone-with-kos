#pragma once

#include "../../shared/include/ipc_messages_logger.h"

int initServerConnector();

int sendRequest(char* query, char* response);