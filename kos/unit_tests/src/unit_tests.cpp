#include "gtest/gtest.h"

#include "../../autopilot_connector/include/autopilot_connector.h"

int logEntry(char* entry, char* entity, LogLevel level) {
    char levelText[10];
    switch (level) {
        case LOG_TRACE:
            strcpy(levelText, "Trace");
            break;
        case LOG_DEBUG:
            strcpy(levelText, "Debug");
            break;
        case LOG_INFO:
            strcpy(levelText, "Info");
            break;
        case LOG_WARNING:
            strcpy(levelText, "Warning");
            break;
        case LOG_ERROR:
            strcpy(levelText, "Error");
            break;
        case LOG_CRITICAL:
            strcpy(levelText, "Critical");
            break;
    }

    fprintf(stderr, "[%s:%s] %s\n", entity, levelText, entry);
    return 1;
}

TEST(AutopilotConnector, InitAutopilotConnector) {
    EXPECT_TRUE(initAutopilotConnector());
}

TEST(AutopilotConnector, InitConnection) {
    EXPECT_TRUE(initConnection());
}

//GetAutopilotCommand -- not possible to test -- requires some send from AP

TEST(AutopilotConnector, SendAutopilotCommand) {
    EXPECT_TRUE(sendAutopilotCommand(AutopilotCommand::ERROR));
    EXPECT_TRUE(sendAutopilotCommand(AutopilotCommand::ERROR, 0));
    EXPECT_TRUE(sendAutopilotCommand(AutopilotCommand::ERROR, 0, 0, 0));
}



int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}