#include "../include/credential_manager.h"

int getRsaKey() {
    return generateRsaKey();
}

int setRsaKey(char* key) {
    return 1;
}

int checkSignature(char* message, uint8_t &correct) {
    correct = 1;
    return 1;
}