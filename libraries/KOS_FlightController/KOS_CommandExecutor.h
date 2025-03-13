#pragma once
#include <AP_Mission/AP_Mission.h>

class KOS_CommandExecutor {
private:
    enum FlightState {
        FlightState_Active,
        FlightState_Paused,
        FlightState_Frozen
    };

    static KOS_CommandExecutor* _singleton;
    AP_Mission* mission;

    uint16_t mission_resume_index;
    FlightState current_state;

    void drop_cargo();
    void change_speed(int32_t speed);
    void change_altitude(int32_t alt);
    void change_waypoint(int32_t lat, int32_t lng, int32_t alt);

    void pause_mission();
    void resume_mission();
    void abort_mission();
    void set_mission(AP_Mission::Mission_Command* commands, int num);

public:
    KOS_CommandExecutor();
    static KOS_CommandExecutor* get_singleton();

    bool can_execute_command();

    friend class KOS_InteractionModule;
};