#pragma once
// auto generated bindings, don't manually edit.  See README.md for details.
#include <AP_Vehicle/AP_Vehicle_Type.h> // needed for APM_BUILD_TYPE #if
#include <AP_Logger/AP_Logger.h>
#include <AP_EFI/AP_EFI_config.h>
#if AP_EFI_SCRIPTING_ENABLED
#include <AP_EFI/AP_EFI_Scripting.h>
#endif // AP_EFI_SCRIPTING_ENABLED
#if HAL_EFI_ENABLED
#include <AP_EFI/AP_EFI.h>
#endif // HAL_EFI_ENABLED
#if APM_BUILD_COPTER_OR_HELI
#include <AP_Winch/AP_Winch.h>
#endif // APM_BUILD_COPTER_OR_HELI
#include <AP_Mount/AP_Mount.h>
#if APM_BUILD_TYPE(APM_BUILD_Rover)
#include <APM_Control/AR_PosControl.h>
#endif // APM_BUILD_TYPE(APM_BUILD_Rover)
#if APM_BUILD_TYPE(APM_BUILD_Rover)
#include <APM_Control/AR_AttitudeControl.h>
#endif // APM_BUILD_TYPE(APM_BUILD_Rover)
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#include <AP_Follow/AP_Follow.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#include <AP_Common/AP_FWVersion.h>
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#include <AP_Motors/AP_Motors_Class.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if defined(HAL_BUILD_AP_PERIPH)
#include <../Tools/AP_Periph/AP_Periph.h>
#endif // defined(HAL_BUILD_AP_PERIPH)
#include <AP_HAL/AP_HAL.h>
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
#include <AP_Scripting/AP_Scripting_CANSensor.h>
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
#include <AP_InertialSensor/AP_InertialSensor.h>
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#include <AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#include <AP_HAL/I2CDevice.h>
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#include <AP_Motors/AP_MotorsMatrix_6DoF_Scripting.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#include <AC_AttitudeControl/AC_AttitudeControl_Multi_6DoF.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#include <AP_Frsky_Telem/AP_Frsky_SPort.h>
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#include <AP_Motors/AP_MotorsMatrix.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#include <../ArduPlane/quadplane.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#include <AP_Notify/ScriptingLED.h>
#include <AP_Button/AP_Button.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Scripting/AP_Scripting.h>
#include <AP_Scripting/AP_Scripting_helpers.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_SerialLED/AP_SerialLED.h>
#include <AP_Vehicle/AP_Vehicle.h>
#if ENABLE_ONVIF == 1
#include <AP_ONVIF/AP_ONVIF.h>
#endif // ENABLE_ONVIF == 1
#include <GCS_MAVLink/GCS.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Proximity/AP_Proximity.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/Location.h>
#include <AP_Scripting/lua/src/lua.hpp>
#include <new>

int new_uint32_t(lua_State *L);
uint32_t * check_uint32_t(lua_State *L, int arg);
#if (AP_EFI_SCRIPTING_ENABLED == 1)
int new_EFI_State(lua_State *L);
EFI_State * check_EFI_State(lua_State *L, int arg);
#endif // (AP_EFI_SCRIPTING_ENABLED == 1)
#if (AP_EFI_SCRIPTING_ENABLED == 1)
int new_Cylinder_Status(lua_State *L);
Cylinder_Status * check_Cylinder_Status(lua_State *L, int arg);
#endif // (AP_EFI_SCRIPTING_ENABLED == 1)
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
int new_AP_HAL__CANFrame(lua_State *L);
AP_HAL::CANFrame * check_AP_HAL__CANFrame(lua_State *L, int arg);
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
int new_AP_MotorsMatrix_Scripting_Dynamic__factor_table(lua_State *L);
AP_MotorsMatrix_Scripting_Dynamic::factor_table * check_AP_MotorsMatrix_Scripting_Dynamic__factor_table(lua_State *L, int arg);
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
int new_mavlink_mission_item_int_t(lua_State *L);
mavlink_mission_item_int_t * check_mavlink_mission_item_int_t(lua_State *L, int arg);
int new_Parameter(lua_State *L);
Parameter * check_Parameter(lua_State *L, int arg);
#if (HAL_WITH_ESC_TELEM == 1)
int new_AP_ESC_Telem_Backend__TelemetryData(lua_State *L);
AP_ESC_Telem_Backend::TelemetryData * check_AP_ESC_Telem_Backend__TelemetryData(lua_State *L, int arg);
#endif // (HAL_WITH_ESC_TELEM == 1)
int new_Quaternion(lua_State *L);
Quaternion * check_Quaternion(lua_State *L, int arg);
int new_Vector2f(lua_State *L);
Vector2f * check_Vector2f(lua_State *L, int arg);
int new_Vector3f(lua_State *L);
Vector3f * check_Vector3f(lua_State *L, int arg);
int new_Location(lua_State *L);
Location * check_Location(lua_State *L, int arg);
#if (AP_EFI_SCRIPTING_ENABLED == 1)
int new_AP_EFI_Backend(lua_State *L);
AP_EFI_Backend ** check_AP_EFI_Backend(lua_State *L, int arg);
#endif // (AP_EFI_SCRIPTING_ENABLED == 1)
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
int new_ScriptingCANBuffer(lua_State *L);
ScriptingCANBuffer ** check_ScriptingCANBuffer(lua_State *L, int arg);
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
int new_AP_HAL__PWMSource(lua_State *L);
AP_HAL::PWMSource ** check_AP_HAL__PWMSource(lua_State *L, int arg);
int new_AP_HAL__AnalogSource(lua_State *L);
AP_HAL::AnalogSource ** check_AP_HAL__AnalogSource(lua_State *L, int arg);
int new_AP_HAL__I2CDevice(lua_State *L);
AP_HAL::I2CDevice ** check_AP_HAL__I2CDevice(lua_State *L, int arg);
int new_AP_HAL__UARTDriver(lua_State *L);
AP_HAL::UARTDriver ** check_AP_HAL__UARTDriver(lua_State *L, int arg);
int new_RC_Channel(lua_State *L);
RC_Channel ** check_RC_Channel(lua_State *L, int arg);
int new_AP_RangeFinder_Backend(lua_State *L);
AP_RangeFinder_Backend ** check_AP_RangeFinder_Backend(lua_State *L, int arg);
void load_generated_bindings(lua_State *L);
void load_generated_sandbox(lua_State *L);
int binding_argcheck(lua_State *L, int expected_arg_count);
lua_Integer get_integer(lua_State *L, int arg_num, lua_Integer min_val, lua_Integer max_val);
int8_t get_int8_t(lua_State *L, int arg_num);
int16_t get_int16_t(lua_State *L, int arg_num);
uint8_t get_uint8_t(lua_State *L, int arg_num);
uint16_t get_uint16_t(lua_State *L, int arg_num);
float get_number(lua_State *L, int arg_num, float min_val, float max_val);
uint32_t get_uint32(lua_State *L, int arg_num, uint32_t min_val, uint32_t max_val);
int new_ap_object(lua_State *L, size_t size, const char * name);
AP_Logger * check_AP_Logger(lua_State *L);
#if (AP_EFI_SCRIPTING_ENABLED == 1)
AP_EFI * check_AP_EFI(lua_State *L);
#endif // (AP_EFI_SCRIPTING_ENABLED == 1)
#if APM_BUILD_COPTER_OR_HELI
AP_Winch * check_AP_Winch(lua_State *L);
#endif // APM_BUILD_COPTER_OR_HELI
#if HAL_MOUNT_ENABLED == 1
AP_Mount * check_AP_Mount(lua_State *L);
#endif // HAL_MOUNT_ENABLED == 1
#if APM_BUILD_TYPE(APM_BUILD_Rover)
AR_PosControl * check_AR_PosControl(lua_State *L);
#endif // APM_BUILD_TYPE(APM_BUILD_Rover)
#if APM_BUILD_TYPE(APM_BUILD_Rover)
AR_AttitudeControl * check_AR_AttitudeControl(lua_State *L);
#endif // APM_BUILD_TYPE(APM_BUILD_Rover)
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
AC_AttitudeControl * check_AC_AttitudeControl(lua_State *L);
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
AP_Follow * check_AP_Follow(lua_State *L);
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if defined(HAL_BUILD_AP_PERIPH)
AP_Periph_FW * check_AP_Periph_FW(lua_State *L);
#endif // defined(HAL_BUILD_AP_PERIPH)
AP_InertialSensor * check_AP_InertialSensor(lua_State *L);
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
AP_MotorsMatrix_Scripting_Dynamic * check_AP_MotorsMatrix_Scripting_Dynamic(lua_State *L);
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
AP_MotorsMatrix_6DoF_Scripting * check_AP_MotorsMatrix_6DoF_Scripting(lua_State *L);
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
AC_AttitudeControl_Multi_6DoF * check_AC_AttitudeControl_Multi_6DoF(lua_State *L);
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#if AP_FRSKY_SPORT_TELEM_ENABLED
AP_Frsky_SPort * check_AP_Frsky_SPort(lua_State *L);
#endif // AP_FRSKY_SPORT_TELEM_ENABLED
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
AP_MotorsMatrix * check_AP_MotorsMatrix(lua_State *L);
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane) && HAL_QUADPLANE_ENABLED
QuadPlane * check_QuadPlane(lua_State *L);
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane) && HAL_QUADPLANE_ENABLED
ScriptingLED * check_ScriptingLED(lua_State *L);
#if HAL_BUTTON_ENABLED == 1
AP_Button * check_AP_Button(lua_State *L);
#endif // HAL_BUTTON_ENABLED == 1
#if AP_RPM_ENABLED
AP_RPM * check_AP_RPM(lua_State *L);
#endif // AP_RPM_ENABLED
AP_Mission * check_AP_Mission(lua_State *L);
AP_Scripting * check_AP_Scripting(lua_State *L);
AP_Param * check_AP_Param(lua_State *L);
#if HAL_WITH_ESC_TELEM == 1
AP_ESC_Telem * check_AP_ESC_Telem(lua_State *L);
#endif // HAL_WITH_ESC_TELEM == 1
#if AP_OPTICALFLOW_ENABLED
AP_OpticalFlow * check_AP_OpticalFlow(lua_State *L);
#endif // AP_OPTICALFLOW_ENABLED
AP_Baro * check_AP_Baro(lua_State *L);
AP_SerialManager * check_AP_SerialManager(lua_State *L);
RC_Channels * check_RC_Channels(lua_State *L);
SRV_Channels * check_SRV_Channels(lua_State *L);
AP_SerialLED * check_AP_SerialLED(lua_State *L);
AP_Vehicle * check_AP_Vehicle(lua_State *L);
#if ENABLE_ONVIF == 1
AP_ONVIF * check_AP_ONVIF(lua_State *L);
#endif // ENABLE_ONVIF == 1
#if HAL_HIGH_LATENCY2_ENABLED == 1
GCS * check_GCS(lua_State *L);
#endif // HAL_HIGH_LATENCY2_ENABLED == 1
AP_Relay * check_AP_Relay(lua_State *L);
#if defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1
AP_Terrain * check_AP_Terrain(lua_State *L);
#endif // defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1
RangeFinder * check_RangeFinder(lua_State *L);
#if HAL_PROXIMITY_ENABLED == 1
AP_Proximity * check_AP_Proximity(lua_State *L);
#endif // HAL_PROXIMITY_ENABLED == 1
AP_Notify * check_AP_Notify(lua_State *L);
AP_GPS * check_AP_GPS(lua_State *L);
AP_BattMonitor * check_AP_BattMonitor(lua_State *L);
AP_Arming * check_AP_Arming(lua_State *L);
AP_AHRS * check_AP_AHRS(lua_State *L);
