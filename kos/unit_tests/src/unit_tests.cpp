#include "gtest/gtest.h"

#include <fcntl.h>

#include "../include/mock.h"
#include "../../shared/include/ipc_messages_logger.h"
#include "../../credential_manager/include/credential_manager.h"
#include "../../navigation_system/include/navigation_system.h"
#include "../../periphery_controller/include/periphery_controller.h"
#include "../../logger/include/logger.h"
#include "../../flight_controller/include/flight_controller.h"

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
    stringToBytes(bytesOdd, strlen(bytesOdd), sig, 128);
    EXPECT_EQ(memcmp(sig, bytesOddCor, 128), 0);
    EXPECT_STREQ(getMockLog(), "Empty log");

    char bytesEven[] = "1021001f";
    uint8_t bytesEvenCor[128] = {0};
    bytesEvenCor[127] = 31;
    bytesEvenCor[125] = 33;
    bytesEvenCor[124] = 16;
    logEntry("Empty log", "", LOG_INFO);
    stringToBytes(bytesEven, strlen(bytesEven), sig, 128);
    EXPECT_EQ(memcmp(sig, bytesEvenCor, 128), 0);
    EXPECT_STREQ(getMockLog(), "Empty log");

    char bytesLong[300] = {0};
    for (int i = 0; i < 299; i++)
        bytesLong[i] = '!';
    logEntry("Empty log", "", LOG_INFO);
    stringToBytes(bytesLong, strlen(bytesLong), sig, 128);
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
    bytesToString(bytes, 128, str, 257);
    EXPECT_STREQ(str, strExp);
    EXPECT_STREQ(getMockLog(), "Empty log");
}

TEST(CredentialManager, RSA) {
    EXPECT_TRUE(generateRsaKey());

    char message[] = "Unit test message";
    char sign[257] = {0};
    EXPECT_TRUE(getMessageSignature(message, sign));

    char key[1024] = {0};
    snprintf(key, 1024, "$Key: %s %s", getKeyN(), getKeyE());
    EXPECT_TRUE(setRsaKey(key));

    char signedMessage[512] = {0};
    snprintf(signedMessage, 512, "%s#%s", message, sign);
    uint8_t correct = 0;
    EXPECT_TRUE(checkMessageSignature(signedMessage, correct));
    EXPECT_TRUE(correct);
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
    char commandsGood[] = "H53.1019446_107.3774394_846.22&T5.0&W53.1020863_107.3774180_5.0&D1.2&S5.0_1200.0&L53.1019446_107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_TRUE(parseCommands(commandsGood));
    EXPECT_STREQ(getMockLog(), "Empty log");

    char commandsWrongCommand[] = "H53.1019446_107.3774394_846.22&X5.0&W53.1020863_107.3774180_5.0&S5.0_1200.0&L53.1019446_107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseCommands(commandsWrongCommand));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char commandsNoCommands[] = "53.1019446_107.3774394_846.22&5.0&53.1020863_107.3774180_5.0&5.0_1200.0&53.1019446_107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseCommands(commandsNoCommands));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char commandsWrongArguments1[] = "H53.1019446_846.22&T5.0&W53.1020863_107.3774180_5.0&S5.0_1200.0&L53.1019446_107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseCommands(commandsWrongArguments1));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char commandsWrongArguments2[] = "H53.1019446_107.3774394_846.22&T5.0&W53.1020863_107.3774180&S5.0_1200.0&L53.1019446_107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseCommands(commandsWrongArguments2));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char commandsWrongArguments3[] = "H53.1019446_107.3774394_846.22&T5.0&W53.1020863_107.3774180_5.0&S1200.0&L53.1019446_107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseCommands(commandsWrongArguments3));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char commandsWrongArguments4[] = "H53.1019446_107.3774394_846.22&T5.0&W53.1020863_107.3774180_5.0&S5.0_1200.0&L107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseCommands(commandsWrongArguments4));
    EXPECT_STRNE(getMockLog(), "Empty log");
}

TEST(FlighController, ParseAreas) {
    char areasGood[] = "2&test_area_1&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299&test_area_2&3&53.1019962_107.3782709&53.1019189_107.3779812&53.1019656_107.3777157#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_TRUE(parseAreas(areasGood));
    EXPECT_STREQ(getMockLog(), "Empty log");

    char areasNoName[] = "1&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseAreas(areasNoName));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char areasWrongNum[] = "2&test_area&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseAreas(areasWrongNum));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char areasWrongPointNum1[] = "1&test_area&4&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseAreas(areasWrongPointNum1));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char areasWrongPointNum2[] = "2&test_area_1&2&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299&test_area_2&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseAreas(areasWrongPointNum2));
    EXPECT_STRNE(getMockLog(), "Empty log");
}

TEST(FlighController, ParseAreasDelta) {
    char deltaGood[] = "3&area_1&modified&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299&area_2&added&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299&area_3&deleted&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_TRUE(parseAreasDelta(deltaGood));
    EXPECT_STREQ(getMockLog(), "Empty log");

    char deltaNoName[] = "1&modified&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseAreasDelta(deltaNoName));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char deltaNoType[] = "1&area&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseAreasDelta(deltaNoType));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char deltaWrongType[] = "1&area&changed&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(parseAreasDelta(deltaWrongType));
    EXPECT_STRNE(getMockLog(), "Empty log");
}

TEST(FlighController, CoordinateToString) {
    char str1[13] = {0};
    coordToString(str1, 13, 1237654321, 7);
    EXPECT_STREQ(str1, "123.7654321");

    char str2[13] = {0};
    coordToString(str2, 13, -1237654321, 7);
    EXPECT_STREQ(str2, "-123.7654321");

    char str3[13] = {0};
    coordToString(str3, 13, 1237654000, 7);
    EXPECT_STREQ(str3, "123.7654000");

    char str4[13] = {0};
    coordToString(str4, 13, 1230000000, 7);
    EXPECT_STREQ(str4, "123.0000000");

    char str5[13] = {0};
    coordToString(str5, 13, 1000000000, 7);
    EXPECT_STREQ(str5, "100.0000000");

    char str6[13] = {0};
    coordToString(str6, 13, 240123456, 7);
    EXPECT_STREQ(str6, "24.0123456");

    char str7[10] = {0};
    coordToString(str7, 10, 100, 2);
    EXPECT_STREQ(str7, "1.00");

    char str8[10] = {0};
    coordToString(str8, 10, 5, 0);
    EXPECT_STREQ(str8, "5");

    char str9[10] = {0};
    coordToString(str9, 10, 0, 2);
    EXPECT_STREQ(str9, "0.00");

    char str10[13] = {0};
    coordToString(str10, 13, 1234, 7);
    EXPECT_STREQ(str10, "0.0001234");
}

TEST(FlighController, LoadMission) {
    char missionGood[] = "$FlightMission H53.1019446_107.3774394_846.22&T5.0&W53.1020863_107.3774180_5.0&S5.0_1200.0&L53.1019446_107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_TRUE(loadMission(missionGood));
    EXPECT_STREQ(getMockLog(), "Empty log");

    char missionEmpty[] = "Response: $-1#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(loadMission(missionEmpty));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char missionNoHead[] = "H53.1019446_107.3774394_846.22&T5.0&W53.1020863_107.3774180_5.0&S5.0_1200.0&L53.1019446_107.3774394_846.22#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(loadMission(missionNoHead));
    EXPECT_STRNE(getMockLog(), "Empty log");
}

TEST(FlighController, LoadNoFlightAreas) {
    char areasGood[] = "$ForbiddenZones 1&test_area&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_TRUE(loadNoFlightAreas(areasGood));
    EXPECT_STREQ(getMockLog(), "Empty log");

    char areasEmptyGood[] = "$ForbiddenZones 0#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_TRUE(loadNoFlightAreas(areasEmptyGood));
    EXPECT_STREQ(getMockLog(), "Empty log");

    char areasEmpty[] = "Response: $-1#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(loadNoFlightAreas(areasEmpty));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char areasNoHead[] = "1&test_area&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(loadMission(areasNoHead));
    EXPECT_STRNE(getMockLog(), "Empty log");
}

TEST(FlighController, UpdateNoFlightAreas) {
    char deltaGood[] = "$ForbiddenZonesDelta 3&area_1&modified&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299&area_2&added&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299&area_3&deleted&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_TRUE(updateNoFlightAreas(deltaGood));
    EXPECT_STREQ(getMockLog(), "Empty log");

    char deltaEmpty[] = "Response: $-1#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(updateNoFlightAreas(deltaEmpty));
    EXPECT_STRNE(getMockLog(), "Empty log");

    char deltaNoHead[] = "3&area_1&modified&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299&area_2&added&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299&area_3&deleted&3&53.1021169_107.377713&53.1022184_107.3779973&53.1022023_107.3783299#";
    logEntry("Empty log", "", LOG_INFO);
    EXPECT_FALSE(updateNoFlightAreas(deltaNoHead));
    EXPECT_STRNE(getMockLog(), "Empty log");
}

TEST(FlighController, ParseNoFlightAreasHash) {
    char hashGoodFst[65] = {0};
    logEntry("Empty log", "", LOG_INFO);
    parseNoFlightAreasHash("$ForbiddenZonesHash bdbac12490e31f4d0b3b5ee45a37dea125a35510b100e48d79e143eb3f419205$", hashGoodFst, 65);
    EXPECT_STREQ(hashGoodFst, "bdbac12490e31f4d0b3b5ee45a37dea125a35510b100e48d79e143eb3f419205");
    EXPECT_STREQ(getMockLog(), "Empty log");

    char hashGoodSnd[65] = {0};
    logEntry("Empty log", "", LOG_INFO);
    parseNoFlightAreasHash("$ForbiddenZonesHash dbac12490e31f4d0b3b5ee45a37dea125a35510b100e48d79e143eb3f419205$", hashGoodSnd, 65);
    EXPECT_STREQ(hashGoodSnd, "0dbac12490e31f4d0b3b5ee45a37dea125a35510b100e48d79e143eb3f419205");
    EXPECT_STREQ(getMockLog(), "Empty log");

    char hashEmpty[65] = {0};
    logEntry("Empty log", "", LOG_INFO);
    parseNoFlightAreasHash("Response: $-1#", hashEmpty, 65);
    EXPECT_STREQ(hashEmpty, "");
    EXPECT_STRNE(getMockLog(), "Empty log");

    char hashNoHead[65] = {0};
    logEntry("Empty log", "", LOG_INFO);
    parseNoFlightAreasHash("bdbac12490e31f4d0b3b5ee45a37dea125a35510b100e48d79e143eb3f419205$", hashNoHead, 65);
    EXPECT_STREQ(hashNoHead, "");
    EXPECT_STRNE(getMockLog(), "Empty log");

    char hashNoTail[65] = {0};
    logEntry("Empty log", "", LOG_INFO);
    parseNoFlightAreasHash("$ForbiddenZonesHash bdbac12490e31f4d0b3b5ee45a37dea125a35510b100e48d79e143eb3f419205#", hashNoTail, 65);
    EXPECT_STREQ(hashNoTail, "");
    EXPECT_STRNE(getMockLog(), "Empty log");

    char hashLong[65] = {0};
    logEntry("Empty log", "", LOG_INFO);
    parseNoFlightAreasHash("$ForbiddenZonesHash bdbac12490e31f4d0b3b5ee45a37dea125a35510b100e48d79e143eb3f419205123456$", hashLong, 65);
    EXPECT_STREQ(hashLong, "");
    EXPECT_STRNE(getMockLog(), "Empty log");

    char hashShort[65] = {0};
    logEntry("Empty log", "", LOG_INFO);
    parseNoFlightAreasHash("$ForbiddenZonesHash bdbac12490e31f4d0b3b5ee45a37dea125a35510b100e48d79e143eb3f", hashShort, 65);
    EXPECT_STREQ(hashShort, "");
    EXPECT_STRNE(getMockLog(), "Empty log");
}

//Logger
TEST(Logger, AddLogEntry) {
    EXPECT_TRUE(createLog());

    FILE* redir = freopen("console.txt", "w", stdout);
    addLogEntry("Unit test entry", LogLevel::LOG_INFO);
    fclose(redir);

    int file = open("/logs/flight_controller.log", O_RDONLY);
    EXPECT_NE(file, -1);
    int i = 0;
    char fileStr[257] = {0};
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
    EXPECT_STREQ(fileStart, "[info]Unit test entry");

    int console = open("console.txt", O_RDONLY);
    EXPECT_NE(console, -1);
    i = 0;
    char consoleStr[257] = {0};
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
    EXPECT_STREQ(consoleStart, "[info]Unit test entry");

    EXPECT_STREQ(fileStr, consoleStr);
}

//Navigation System
TEST(NavigationSystem, NoPosition) {
    EXPECT_FALSE(hasPosition());

    float dop = -1.0f;
    int32_t sats = -1;
    EXPECT_FALSE(getInfo(dop, sats));
    EXPECT_EQ(dop, 0.0f);
    EXPECT_EQ(sats, 0);

    int32_t lat = -1, lng = -1, alt = -1;
    EXPECT_FALSE(getPosition(lat, lng, alt));
    EXPECT_EQ(lat, 0);
    EXPECT_EQ(lng, 0);
    EXPECT_EQ(alt, 0);
}

TEST(NavigationSystem, DronePosition) {
    int32_t testLat = 142, testLng = -546, testAlt = 11;
    setCoords(testLat, 0);
    setAltitude(testAlt);
    EXPECT_FALSE(hasPosition());

    setCoords(0, testLng);
    EXPECT_FALSE(hasPosition());

    setCoords(testLat, testLng);
    EXPECT_TRUE(hasPosition());

    int32_t lat = -1, lng = -1, alt = -1;
    EXPECT_TRUE(getPosition(lat, lng, alt));
    EXPECT_EQ(lat, testLat);
    EXPECT_EQ(lng, testLng);
    EXPECT_EQ(alt, testAlt);
}

TEST(NavigationSystem, GPSInfo) {
    float testDop = 0.7f;
    int32_t testSats = 13;
    setInfo(testDop, testSats);

    float dop = -1.0f;
    int32_t sats = -1;
    EXPECT_TRUE(getInfo(dop, sats));
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

    EXPECT_TRUE(startBuzzer());
    EXPECT_FALSE(startBuzzer());

    EXPECT_TRUE(getMockBuzzer());
    sleep(3);
    EXPECT_FALSE(getMockBuzzer());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (!RUN_ALL_TESTS())
        fprintf(stderr, "All unit tests are passed\n");
    return EXIT_SUCCESS;
}