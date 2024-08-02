#include "../../shared/include/ipc_messages_logger.h"
#include <string.h>

char mockLog[1024] = {0};

char* getMockLog() {
    return mockLog;
}

int logEntry(char* entry, char* entity, LogLevel level) {
    strcpy(mockLog, entry);
    return 1;
}

int sendRequest(char* query, char* response) {
    return 1;
}