project (FlightController)

include (platform/nk)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

add_executable (FlightController "src/main.cpp" "src/flight_controller_interface.cpp" "src/mission.cpp"
    "../ipc_messages/src/transport_interface.cpp" "../ipc_messages/src/initialization_interface.cpp")
add_dependencies (FlightController flight_controller_edl_files)

target_compile_definitions (FlightController PRIVATE ENTITY_NAME="Flight Controller")
target_compile_definitions (FlightController PRIVATE BOARD_ID="id=${BOARD_ID}")