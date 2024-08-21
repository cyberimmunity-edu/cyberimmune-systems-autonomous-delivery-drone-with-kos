#include "../include/server_connector.h"

#include <string.h>

#include <stdio.h>

#define BOARD_NAME_MAX_LEN 32

char boardName[BOARD_NAME_MAX_LEN + 1] = {0};

void setBoardName(char* id) {
    strncpy(boardName, id, BOARD_NAME_MAX_LEN);
}

char* getBoardName() {
    return boardName;
}