#pragma once

#include <stdint.h>

#define MAX_LOG_BUFFER 256

enum LogLevel { LOG_TRACE, LOG_DEBUG, LOG_INFO, LOG_WARNING, LOG_ERROR, LOG_CRITICAL };

int logEntry(char* entry, char* entity, LogLevel level);