#include "gtest/gtest.h"

#include <fcntl.h>

#include "../include/mock_declaration.h"
#include "../../shared/include/ipc_messages_logger.h"
#include "../../flight_controller/include/mission.h"
#include "../../logger/include/logger.h"

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
TEST(FlighController, IsStopSymbol) {
    EXPECT_TRUE(isStopSymbol('_'));
    EXPECT_TRUE(isStopSymbol('&'));
    EXPECT_TRUE(isStopSymbol('#'));
    EXPECT_FALSE(isStopSymbol(' '));
    EXPECT_FALSE(isStopSymbol('!'));
    EXPECT_FALSE(isStopSymbol('"'));
    EXPECT_FALSE(isStopSymbol('$'));
    EXPECT_FALSE(isStopSymbol('%'));
    for (char s = '\''; s <= '^'; s++) //' ( ) * + , - . / 0-9 : ; < = > ? @ A-Z [ \ ] ^
        EXPECT_FALSE(isStopSymbol(s));
    for (char s = '`'; s <= '~'; s++) //` a-z { | } ~
        EXPECT_FALSE(isStopSymbol(s));
    EXPECT_FALSE(isStopSymbol('\n'));
}

TEST(FlighController, ParseInt) {
    int32_t value;
    char strGood[] = "123.456_.0546_12.32109123414_9.45_228_9__";
    char* str = strGood;

    logEntry("Empty log", "", LOG_INFO);
    EXPECT_TRUE(parseInt(str, value, 3));
    EXPECT_EQ(value, 123456);
    EXPECT_STREQ(str, ".0546_12.32109123414_9.45_228_9__");
    EXPECT_STREQ(getMockLog(), "Empty log");

    logEntry("Empty log", "", LOG_INFO);
    EXPECT_TRUE(parseInt(str, value, 4));
    EXPECT_EQ(value, 546);
    EXPECT_STREQ(str, "12.32109123414_9.45_228_9__");
    EXPECT_STREQ(getMockLog(), "Empty log");

    logEntry("Empty log", "", LOG_INFO);
    EXPECT_TRUE(parseInt(str, value, 2));
    EXPECT_EQ(value, 1232);
    EXPECT_STREQ(str, "9.45_228_9__");
    EXPECT_STREQ(getMockLog(), "Empty log");

    logEntry("Empty log", "", LOG_INFO);
    EXPECT_TRUE(parseInt(str, value, 4));
    EXPECT_EQ(value, 94500);
    EXPECT_STREQ(str, "228_9__");
    EXPECT_STREQ(getMockLog(), "Empty log");

    logEntry("Empty log", "", LOG_INFO);
    EXPECT_TRUE(parseInt(str, value, 0));
    EXPECT_EQ(value, 228);
    EXPECT_STREQ(str, "9__");
    EXPECT_STREQ(getMockLog(), "Empty log");

    logEntry("Empty log", "", LOG_INFO);
    EXPECT_TRUE(parseInt(str, value, 3));
    EXPECT_EQ(value, 9000);
    EXPECT_STREQ(str, "_");
    EXPECT_STREQ(getMockLog(), "Empty log");

    logEntry("Empty log", "", LOG_INFO);
    EXPECT_TRUE(parseInt(str, value, 2));
    EXPECT_EQ(value, 0);
    EXPECT_STREQ(str, "");
    EXPECT_STREQ(getMockLog(), "Empty log");

    char strLong[] = "0123.456789012345678901234567890123456789";
    str = strLong;
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseInt(str, value, 32));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char strLetter1[] = "12a34.56";
    str = strLetter1;
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseInt(str, value, 2));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char strLetter2[] = "1234.5b6";
    str = strLetter2;
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseInt(str, value, 2));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char strNoStop[] = "221.456";
    str = strNoStop;
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseInt(str, value, 3));
    EXPECT_STRNE(getMockLog(), "Empty log");
}

TEST(FlighController, ParseCommands) {
    char commandsGood[] = "H53.1019446_107.3774394_846.22&T5.0&W0.0_53.1020863_107.3774180_5.0&S5.0_1200.0&L53.1019446_107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_TRUE(parseCommands(commandsGood));
    EXPECT_STREQ(getMockLog(), "Empty log");

    char commandsWrongCommand[] = "H53.1019446_107.3774394_846.22&X5.0&W0.0_53.1020863_107.3774180_5.0&S5.0_1200.0&L53.1019446_107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseCommands(commandsWrongCommand));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char commandsNoCommands[] = "53.1019446_107.3774394_846.22&5.0&0.0_53.1020863_107.3774180_5.0&5.0_1200.0&53.1019446_107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseCommands(commandsNoCommands));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char commandsWrongArguments1[] = "H53.1019446_846.22&T5.0&W0.0_53.1020863_107.3774180_5.0&S5.0_1200.0&L53.1019446_107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseCommands(commandsWrongArguments1));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char commandsWrongArguments2[] = "H53.1019446_107.3774394_846.22&T5.0&W0.0_53.1020863_107.3774180&S5.0_1200.0&L53.1019446_107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseCommands(commandsWrongArguments2));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char commandsWrongArguments3[] = "H53.1019446_107.3774394_846.22&T5.0&W0.0_53.1020863_107.3774180_5.0&S1200.0&L53.1019446_107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseCommands(commandsWrongArguments3));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char commandsWrongArguments4[] = "H53.1019446_107.3774394_846.22&T5.0&W0.0_53.1020863_107.3774180_5.0&S5.0_1200.0&L107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseCommands(commandsWrongArguments4));
    EXPECT_STRNE(getMockLog(), "Empty log");
}

TEST(FlighController, ParseMission) {
    char missionGood[] = "$FlightMission H53.1019446_107.3774394_846.22&T5.0&W0.0_53.1020863_107.3774180_5.0&S5.0_1200.0&L53.1019446_107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_TRUE(parseMission(missionGood));
    EXPECT_STREQ(getMockLog(), "Empty log");

    char missionEmpty[] = "Response: $-1#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseMission(missionEmpty));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char missionNoHead[] = "H53.1019446_107.3774394_846.22&T5.0&W0.0_53.1020863_107.3774180_5.0&S5.0_1200.0&L53.1019446_107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseMission(missionNoHead));
    EXPECT_STRNE(getMockLog(), "Empty log");
}

//Logger
TEST(Logger, AddLogEntry) {
    EXPECT_TRUE(createLog());

    FILE* redir = freopen("console.txt", "w", stdout);
    addLogEntry("Unit-test entry", LogLevel::LOG_INFO);
    fclose(redir);

    int file = open("/logs/flight_controller.log", O_RDONLY);
    EXPECT_NE(file, -1);
    int i = 0;
    char fileStr[256] = {0};
    while (i < 256) {
        char letter;
        EXPECT_EQ(read(file, &letter, 1), 1);
        if ((letter == '\n') || (letter == '\0'))
            break;
        fileStr[i] = letter;
        i++;
    }
    close(file);
    char* fileStart = strstr(fileStr, "[info]");
    EXPECT_TRUE(fileStart);
    EXPECT_STREQ(fileStart, "[info]Unit-test entry");

    int console = open("console.txt", O_RDONLY);
    EXPECT_NE(console, -1);
    i = 0;
    char consoleStr[256] = {0};
    while (i < 256) {
        char letter;
        EXPECT_EQ(read(console, &letter, 1), 1);
        if ((letter == '\n') || (letter == '\0'))
            break;
        consoleStr[i] = letter;
        i++;
    }
    close(console);
    char* consoleStart = strstr(consoleStr, "[info]");
    EXPECT_TRUE(consoleStart);
    EXPECT_STREQ(consoleStart, "[info]Unit-test entry");

    EXPECT_STREQ(fileStr, consoleStr);
}

//Credential Manager
    //RSA save/load/read/sign/etc

//Navigation System
TEST(NavigationSystem, NoPosition) {
    EXPECT_FALSE(hasPosition());

    float dop = -1.0f;
    int32_t sats = -1;
    EXPECT_FALSE(getGpsInfo(dop, sats));
    EXPECT_EQ(dop, 0.0f);
    EXPECT_EQ(sats, 0);

    int32_t lat = -1, lng = -1, alt = -1;
    EXPECT_FALSE(getCoords(lat, lng, alt));
    EXPECT_EQ(lat, 0);
    EXPECT_EQ(lng, 0);
    EXPECT_EQ(alt, 0);
}

TEST(NavigationSystem, SetGetCoords) {
    int32_t testLat = 142, testLng = -546, testAlt = 11;
    setCoords(testLat, 0);
    setAltitude(testAlt);
    EXPECT_FALSE(hasPosition());

    setCoords(0, testLng);
    EXPECT_FALSE(hasPosition());

    setCoords(testLat, testLng);
    EXPECT_TRUE(hasPosition());

    int32_t lat = -1, lng = -1, alt = -1;
    EXPECT_TRUE(getCoords(lat, lng, alt));
    EXPECT_EQ(lat, testLat);
    EXPECT_EQ(lng, testLng);
    EXPECT_EQ(alt, testAlt);
}

TEST(NavigationSystem, SetGetGpsInfo) {
    float testDop = 0.7f;
    int32_t testSats = 13;
    setGpsInfo(testDop, testSats);

    float dop = -1.0f;
    int32_t sats = -1;
    EXPECT_TRUE(getGpsInfo(dop, sats));
    EXPECT_EQ(dop, testDop);
    EXPECT_EQ(sats, testSats);
}

//Periphery Controller
TEST(PeripheryController, Buzz) {
    setMockBuzzer(true);
    EXPECT_TRUE(getMockBuzzer());
    buzz();
    EXPECT_FALSE(getMockBuzzer());
}

TEST(PeripheryController, EnableBuzzer) {
    setMockBuzzer(false);
    EXPECT_FALSE(getMockBuzzer());

    EXPECT_TRUE(enableBuzzer());
    EXPECT_FALSE(enableBuzzer());

    EXPECT_TRUE(getMockBuzzer());
    sleep(3);
    EXPECT_FALSE(getMockBuzzer());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}