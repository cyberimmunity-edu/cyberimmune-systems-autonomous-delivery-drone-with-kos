#include "../../shared/include/ipc_messages_logger.h"
#include <string.h>

char mockLog[1024] = {0};
bool mockBuzzerEnabled = false;

char* getMockLog() {
    return mockLog;
}

bool getMockBuzzer() {
    return mockBuzzerEnabled;
}

void setMockBuzzer(bool enable) {
    mockBuzzerEnabled = enable;
}

int logEntry(char* entry, char* entity, LogLevel level) {
    strcpy(mockLog, entry);
    return 1;
}

int sendRequest(char* query, char* response) {
    return 1;
}

bool isKillSwitchEnabled() {
    return false;
}

int setBuzzer(bool enable) {
    mockBuzzerEnabled = enable;
    return 1;
}

int setKillSwitch(bool enable) {
    return 1;
}

int waitForInit(const char* connection, const char* receiverEntity) {
    return 1;
}

int getBoardId(char* id) {
    strcpy(id, "00:00:00:00:00:00");
    return 1;
}