project (FlightController)

include (platform/nk)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

add_executable (FlightController "src/main.cpp" "src/flight_controller.cpp" "../shared/src/initialization_interface.cpp"
    "../shared/src/ipc_messages_initialization.cpp" "../shared/src/ipc_messages_autopilot_connector.cpp"
    "../shared/src/ipc_messages_credential_manager.cpp" "../shared/src/ipc_messages_navigation_system.cpp"
    "../shared/src/ipc_messages_periphery_controller.cpp" "../shared/src/ipc_messages_server_connector.cpp"
    "../shared/src/ipc_messages_logger.cpp")
add_dependencies (FlightController flight_controller_edl_files)

target_compile_definitions (FlightController PRIVATE ENTITY_NAME="Flight Controller")
target_link_libraries (FlightController MbedTLS::mbedtls ${CMAKE_THREAD_LIBS_INIT})