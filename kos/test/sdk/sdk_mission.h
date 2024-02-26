#pragma open

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

    CommandHome(float* values);
};

struct CommandTakeoff {
    float altitude;

    CommandTakeoff(float* values);
};

struct CommandWaypoint {
    float holdTime;
    float latitude;
    float longitude;
    float altitude;

    CommandWaypoint(float* values);
};

struct CommandLand {
    float latitude;
    float longitude;
    float altitude;

    CommandLand(float* values);
};

struct CommandSetServo {
    float number;
    float pwm;

    CommandSetServo(float* values);
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

int missionAvailable();
int requestMission();
void deleteMission();
void printMission();