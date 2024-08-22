/**
 * \file
 * \~English
 * \brief Implementation of methods for drone ID management.
 * \details The file contains implementation of methods to write and read drone ID.
 *
 * \~Russian
 * \brief Реализация методов для работы с ID дрона.
 * \details В файле реализованы методы для записи и чтения ID дрона.
 */

#include "../include/server_connector.h"

#include <string.h>

#include <stdio.h>

/** \cond */
#define BOARD_NAME_MAX_LEN 32

char boardName[BOARD_NAME_MAX_LEN + 1] = {0};
/** \endcond */

void setBoardName(char* id) {
    strncpy(boardName, id, BOARD_NAME_MAX_LEN);
}

char* getBoardName() {
    return boardName;
}