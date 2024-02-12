#pragma once

int shareRsaKey(char* response);
int signMessage(char* message, char* signature);
int checkSignature(char* message);