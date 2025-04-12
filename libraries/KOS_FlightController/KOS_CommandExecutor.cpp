#include "KOS_CommandExecutor.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Arming/AP_Arming.h>
#include <GCS_MAVLink/GCS.h>

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#include "../../ArduCopter/Copter.h"
#elif APM_BUILD_TYPE(APM_BUILD_Rover)
#include "../../Rover/Rover.h"
#endif

KOS_CommandExecutor* KOS_CommandExecutor::_singleton;

void KOS_CommandExecutor::drop_cargo() {
    if (!mission)
        return;

    gcs().send_text(MAV_SEVERITY_INFO, "Info: Trying to drop cargo at current location");
    AP_Mission::Mission_Command cmd;
    cmd.index = 0;
    cmd.id = MAV_CMD_DO_SET_SERVO;
    cmd.content.servo.channel = 5;
    cmd.content.servo.pwm = 1200;
    mission->start_command(cmd);
}

void KOS_CommandExecutor::change_speed(int32_t speed) {
    if (!mission)
        return;

#if AP_SIM_ENABLED
    gcs().send_text(MAV_SEVERITY_INFO, "Info: Changing speed to %d cm/s", speed);
#else
    gcs().send_text(MAV_SEVERITY_INFO, "Info: Changing speed to %ld cm/s", speed);
#endif
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    copter.aparm.angle_max.set(DEFAULT_ANGLE_MAX);
#endif
    AP_Mission::Mission_Command cmd;
    cmd.index = 0;
    cmd.id = MAV_CMD_DO_CHANGE_SPEED;
    cmd.content.speed.speed_type = 0;
    cmd.content.speed.target_ms = speed / 100.0f;
    cmd.content.speed.throttle_pct = 0;
    mission->start_command(cmd);
}

void KOS_CommandExecutor::change_altitude(int32_t alt) {
    if (!mission)
        return;

#if APM_BUILD_TYPE(APM_BUILD_Rover)
    gcs().send_text(MAV_SEVERITY_INFO, "Info: Altitude change is not supported by Rover");
    return;
#endif

#if AP_SIM_ENABLED
    gcs().send_text(MAV_SEVERITY_INFO, "Info: Changing altitude to %d m", alt / 100);
#else
    gcs().send_text(MAV_SEVERITY_INFO, "Info: Changing altitude to %ld m", alt / 100);
#endif
    AP_Mission::Mission_Command cmd;
    for (int i = mission->get_current_nav_index(); i < mission->num_commands(); i++) {
        mission->read_cmd_from_storage(i, cmd);
        if (cmd.id == MAV_CMD_NAV_WAYPOINT) {
            cmd.content.location.alt = alt;
            mission->write_cmd_to_storage(cmd.index, cmd);
        }
    }
    mission->restart_current_nav_cmd();
}

void KOS_CommandExecutor::change_waypoint(int32_t lat, int32_t lng, int32_t alt) {
    if (!mission)
        return;

#if AP_SIM_ENABLED
    gcs().send_text(MAV_SEVERITY_INFO, "Info: Changing current waypoint to %d, %d, %d", lat, lng, alt);
#else
    gcs().send_text(MAV_SEVERITY_INFO, "Info: Changing current waypoint to %ld, %ld, %ld", lat, lng, alt);
#endif
    AP_Mission::Mission_Command cmd = mission->get_current_nav_cmd();
    cmd.content.location.lat = lat;
    cmd.content.location.lng = lng;
    cmd.content.location.alt = alt;
    mission->replace_cmd(cmd.index, cmd);
    mission->restart_current_nav_cmd();
}

void KOS_CommandExecutor::pause_mission() {
    gcs().send_text(MAV_SEVERITY_INFO, "Info: Request to pause flight");
    if (current_state == FlightState::FlightState_Paused) {
        gcs().send_text(MAV_SEVERITY_INFO, "Info: flight is already paused");
        return;
    }
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    if (current_state == FlightState::FlightState_Active) {
        mission_resume_index = mission->get_current_nav_index() - 1;
        mission->stop();
    }
    else if (current_state == FlightState::FlightState_Frozen)
        mission_resume_index = 0;
    copter.set_mode(Mode::Number::LAND, ModeReason::KOS_COMMAND);
    if (current_state == FlightState::FlightState_Active) {
        AP_Mission::Mission_Command cmd;
        mission->read_cmd_from_storage(mission_resume_index, cmd);
        cmd.id = MAV_CMD_NAV_TAKEOFF;
        cmd.content.location.alt = (int32_t)(copter.inertial_nav.get_position_z_up_cm());
        copter.mode_auto.mission.replace_cmd(mission_resume_index, cmd);
    }
#elif APM_BUILD_TYPE(APM_BUILD_Rover)
    if (current_state == FlightState::FlightState_Active) {
        mission_resume_index = mission->get_current_nav_index();
        mission->stop();
    }
    else if (current_state == FlightState::FlightState_Frozen)
        mission_resume_index = 0;
    AP_Arming::get_singleton()->disarm(AP_Arming::Method::SCRIPTING, false);
#else
    gcs().send_text(MAV_SEVERITY_INFO, "Info: Mission pause is not supported for current vehicle type");
    return;
#endif
    current_state = FlightState::FlightState_Paused;
}

void KOS_CommandExecutor::resume_mission() {
    if ((current_state == FlightState::FlightState_Paused) && mission_resume_index) {
        gcs().send_text(MAV_SEVERITY_INFO, "Info: Request to resume flight");
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
        AP_Arming::get_singleton()->arm(AP_Arming::Method::SCRIPTING, false);
        copter.set_mode(Mode::Number::AUTO, ModeReason::KOS_COMMAND);
        copter.set_auto_armed(true);
        mission->start_or_resume();
        mission->set_current_cmd(mission_resume_index);
#elif APM_BUILD_TYPE(APM_BUILD_Rover)
        AP_Arming::get_singleton()->arm(AP_Arming::Method::SCRIPTING, false);
        rover.set_mode(Mode::Number::AUTO, ModeReason::KOS_COMMAND);
        mission->set_current_cmd(mission_resume_index);
        mission->start_or_resume();
#else
        gcs().send_text(MAV_SEVERITY_INFO, "Info: Mission resume is not supported for current vehicle type ");
        return;
#endif
        mission_resume_index = 0;
        current_state = FlightState::FlightState_Paused;
    }
    else
        gcs().send_text(MAV_SEVERITY_INFO, "Info: Impossible to resume flight: no data about previous flight");
}

void KOS_CommandExecutor::abort_mission() {
    gcs().send_text(MAV_SEVERITY_INFO, "Info: Request to abort flight");
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    mission_resume_index = 0;
    mission->stop();
    mission->truncate(0);
    if (current_state != FlightState_Paused)
        copter.set_mode(Mode::Number::BRAKE, ModeReason::KOS_COMMAND);
#elif APM_BUILD_TYPE(APM_BUILD_Rover)
    mission_resume_index = 0;
    mission->stop();
    mission->truncate(0);
    AP_Arming::get_singleton()->disarm(AP_Arming::Method::SCRIPTING, false);
#else
    gcs().send_text(MAV_SEVERITY_INFO, "Info: Mission abort is not supported for current vehicle type ");
    return;
#endif
    if (current_state != FlightState_Paused)
        current_state = FlightState::FlightState_Frozen;
}

void KOS_CommandExecutor::set_mission(AP_Mission::Mission_Command* commands, int num) {
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_Rover)
    if (current_state == FlightState_Active)
        abort_mission();
#else
    gcs().send_text(MAV_SEVERITY_INFO, "Info: Mission change is not supported for current vehicle type");
    return;
#endif

    for (int i = 0; i < num; i++)
        mission->add_cmd(commands[i]);
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    if ((current_state == FlightState_Paused) && !AP_Arming::get_singleton()->is_armed()) {
        AP_Arming::get_singleton()->arm(AP_Arming::Method::SCRIPTING, false);
        copter.set_mode(Mode::Number::AUTO, ModeReason::KOS_COMMAND);
        copter.set_auto_armed(true);
    }
    else if (current_state == FlightState_Frozen)
        copter.set_mode(Mode::Number::AUTO, ModeReason::KOS_COMMAND);
#elif APM_BUILD_TYPE(APM_BUILD_Rover)
    if (!AP_Arming::get_singleton()->is_armed())
        AP_Arming::get_singleton()->arm(AP_Arming::Method::SCRIPTING, false);
#endif
    mission->reset();
    mission->start();
    current_state = FlightState::FlightState_Active;
    gcs().send_text(MAV_SEVERITY_INFO, "Info: Mission was successfully updated");
}

KOS_CommandExecutor::KOS_CommandExecutor() {
    _singleton = this;
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    mission = &(copter.mode_auto.mission);
#elif APM_BUILD_TYPE(APM_BUILD_Rover)
    mission = &(rover.mode_auto.mission);
#else
    mission = NULL;
    gcs().send_text(MAV_SEVERITY_INFO, "Info: Current vehicle is not supported by KOS FlightController Module");
#endif
    mission_resume_index = 0;
    current_state = FlightState::FlightState_Active;
}

KOS_CommandExecutor* KOS_CommandExecutor::get_singleton() {
    return _singleton;
}

bool KOS_CommandExecutor::can_execute_command() {
    return (bool)(mission->get_current_nav_index());
}