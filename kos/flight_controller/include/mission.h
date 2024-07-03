#pragma once
#include <stdint.h>

enum CommandType {
    HOME,
    TAKEOFF,
    WAYPOINT,
    LAND,
    SET_SERVO
};

struct CommandTakeoff {
    int32_t altitude;

    CommandTakeoff(int32_t alt) {
        altitude = alt;
    }
};

struct CommandWaypoint {
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;

    CommandWaypoint(int32_t lat, int32_t lng, int32_t alt) {
        latitude = lat;
        longitude = lng;
        altitude = alt;
    }
};

struct CommandServo {
    int32_t number;
    int32_t pwm;

    CommandServo(int32_t num, int32_t pwm_) {
        number = num;
        pwm = pwm_;
    }
};

union CommandContent {
    CommandTakeoff takeoff;
    CommandWaypoint waypoint;
    CommandServo servo;
};

struct MissionCommand {
    CommandType type;
    CommandContent content;
};

int parseMission(char* response);
void printMission();

MissionCommand* getCommands();
uint32_t getNumCommands();
