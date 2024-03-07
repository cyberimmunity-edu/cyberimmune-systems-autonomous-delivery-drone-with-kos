#pragma once

enum CommandType {
    HOME,
    TAKEOFF,
    WAYPOINT,
    LAND,
    SET_SERVO
};

struct CommandHome {
    float latitude;
    float longitude;
    float altitude;

    CommandHome(float* values) {
        latitude = values[0];
        longitude = values[1];
        altitude = values[2];
    }
};

struct CommandTakeoff {
    float altitude;

    CommandTakeoff(float* values) {
        altitude = values[0];
    }
};

struct CommandWaypoint {
    float holdTime;
    float latitude;
    float longitude;
    float altitude;

    CommandWaypoint(float* values) {
        holdTime = values[0];
        latitude = values[1];
        longitude = values[2];
        altitude = values[3];
    }
};

struct CommandLand {
    float latitude;
    float longitude;
    float altitude;

    CommandLand(float* values) {
        latitude = values[0];
        longitude = values[1];
        altitude = values[2];
    }
};

struct CommandSetServo {
    float number;
    float pwm;

    CommandSetServo(float* values) {
        number = values[0];
        pwm = values[1];
    }
};

union CommandContent {
    CommandHome home;
    CommandTakeoff takeoff;
    CommandWaypoint waypoint;
    CommandLand land;
    CommandSetServo setServo;
};

struct MissionCommand {
    CommandType type;
    CommandContent content;
};

int parseMission(char* response);
void printMission();