#include "gtest/gtest.h"
#include "../include/mock_declaration.h"
#include "../../shared/include/ipc_messages_logger.h"

//Credential Manager
TEST(CredentialManager, HexCharToInt) {
    logEntry("Empty log", "", LOG_INFO);
    for (int i = 0; i < 256; i++) {
        char ch = (char)i;
        uint8_t val = hexCharToInt(ch);
        switch (ch) {
        case '0':
            EXPECT_EQ(val, 0);
            break;
        case '1':
            EXPECT_EQ(val, 1);
            break;
        case '2':
            EXPECT_EQ(val, 2);
            break;
        case '3':
            EXPECT_EQ(val, 3);
            break;
        case '4':
            EXPECT_EQ(val, 4);
            break;
        case '5':
            EXPECT_EQ(val, 5);
            break;
        case '6':
            EXPECT_EQ(val, 6);
            break;
        case '7':
            EXPECT_EQ(val, 7);
            break;
        case '8':
            EXPECT_EQ(val, 8);
            break;
        case '9':
            EXPECT_EQ(val, 9);
            break;
        case 'a':
        case 'A':
            EXPECT_EQ(val, 10);
            break;
        case 'b':
        case 'B':
            EXPECT_EQ(val, 11);
            break;
        case 'c':
        case 'C':
            EXPECT_EQ(val, 12);
            break;
        case 'd':
        case 'D':
            EXPECT_EQ(val, 13);
            break;
        case 'e':
        case 'E':
            EXPECT_EQ(val, 14);
            break;
        case 'f':
        case 'F':
            EXPECT_EQ(val, 15);
            break;
        default:
            EXPECT_EQ(val, 0);
            break;
        }
        if ((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f') || (ch >= 'A' && ch <= 'F'))
            EXPECT_STREQ(getMockLog(), "Empty log");
        else
            EXPECT_STRNE(getMockLog(), "Empty log");
        logEntry("Empty log", "", LOG_INFO);
    }
}

TEST(CredentialManager, StringToBytes) {
    uint8_t sig[128] = {0};

    char bytesOdd[] = "a1234";
    uint8_t bytesOddCor[128] = {0};
    bytesOddCor[127] = 52;
    bytesOddCor[126] = 18;
    bytesOddCor[125] = 10;
    logEntry("Empty log", "", LOG_INFO);
    stringToBytes(bytesOdd, strlen(bytesOdd), sig);
    EXPECT_EQ(memcmp(sig, bytesOddCor, 128), 0);
    EXPECT_STREQ(getMockLog(), "Empty log");

    char bytesEven[] = "1021001f";
    uint8_t bytesEvenCor[128] = {0};
    bytesEvenCor[127] = 31;
    bytesEvenCor[125] = 33;
    bytesEvenCor[124] = 16;
    logEntry("Empty log", "", LOG_INFO);
    stringToBytes(bytesEven, strlen(bytesEven), sig);
    EXPECT_EQ(memcmp(sig, bytesEvenCor, 128), 0);
    EXPECT_STREQ(getMockLog(), "Empty log");

    char bytesLong[300] = {0};
    for (int i = 0; i < 299; i++)
        bytesLong[i] = '!';
    logEntry("Empty log", "", LOG_INFO);
    stringToBytes(bytesLong, strlen(bytesLong), sig);
    EXPECT_STRNE(getMockLog(), "Empty log");
}

TEST(CredentialManager, HashToKey) {
    uint8_t key[128] = {0};

    uint8_t hashShort[7] = {0};
    for (int i = 0; i < 7; i++)
        hashShort[i] = i + 1;
    uint8_t keyShortCor[128] = {0};
    keyShortCor[127] = 7;
    keyShortCor[126] = 6;
    keyShortCor[125] = 5;
    keyShortCor[124] = 4;
    keyShortCor[123] = 3;
    keyShortCor[122] = 2;
    keyShortCor[121] = 1;
    logEntry("Empty log", "", LOG_INFO);
    hashToKey(hashShort, 7, key);
    EXPECT_EQ(memcmp(key, keyShortCor, 128), 0);
    EXPECT_STREQ(getMockLog(), "Empty log");

    uint8_t hashLong[130] = {0};
    for (int i = 0; i < 130; i++)
        hashLong[i] = i;
    logEntry("Empty log", "", LOG_INFO);
    hashToKey(hashLong, 130, key);
    EXPECT_STRNE(getMockLog(), "Empty log");
}

TEST(CredentialManager, BytesToString) {
    char str[257] = {0};

    uint8_t bytes[128] = {0};
    bytes[0] = 250;
    bytes[1] = 9;
    bytes[2] = 176;
    bytes[3] = 61;
    bytes[5] = 228;
    bytes[6] = 12;
    bytes[7] = 112;
    bytes[8] = 21;
    char strExp[257] = {0};
    for (int i = 0; i < 256; i++)
        strExp[i] = '0';
    strncpy(strExp, "fa09b03d00e40c7015", 18);
    logEntry("Empty log", "", LOG_INFO);
    bytesToString(bytes, str);
    EXPECT_STREQ(str, strExp);
    EXPECT_STREQ(getMockLog(), "Empty log");
}

//Flight Controller
    //isStopSymbol
    //parseInt
    //parseCommands
    //parseMission
//Logger
    //logEntry

//Autopilot Connector
    //simulator -- write to socket/read
//Credential Manager
    //RSA save/load/read/sign/etc
//Navigation System
    //hasPosition
    //setGpsInfo
    //getGpsInfo
    //setAltitude
    //setCoords
    //getCoords
//Periphery Controller
    //turnKS
    //buzzer -- sim
//Server Connector
    //simulator -- write to socket/read

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}