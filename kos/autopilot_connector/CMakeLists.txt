project (AutopilotConnector)

include (platform/nk)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

if (SIMULATION)
    set (AUTOPILOT_CONNECTOR_SRC "src/autopilot_connector_simulator.cpp")
else()
    set (AUTOPILOT_CONNECTOR_SRC "src/autopilot_connector_real.cpp")
endif()

add_executable (AutopilotConnector "src/main.cpp" ${AUTOPILOT_CONNECTOR_SRC} "src/autopilot_connector_interface.cpp"
    "src/autopilot_connector.cpp" "../shared/src/initialization_interface.cpp"
    "../shared/src/ipc_messages_initialization.cpp" "../shared/src/ipc_messages_logger.cpp")
add_dependencies (AutopilotConnector autopilot_connector_edl_files)

target_compile_definitions (AutopilotConnector PRIVATE ENTITY_NAME="Autopilot Connector")

if (SIMULATION)
    target_compile_definitions (AutopilotConnector PRIVATE SIMULATOR_IP="${SIMULATOR_IP}")
    target_link_libraries (AutopilotConnector ${vfs_CLIENT_LIB})
else()
    target_link_libraries (AutopilotConnector ${bsp_CLIENT_LIB} ${uart_CLIENT_LIB})
endif()