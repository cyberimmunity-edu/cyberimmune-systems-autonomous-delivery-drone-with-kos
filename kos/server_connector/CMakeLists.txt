project (ServerConnector)

include (platform/nk)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

if (SERVER)
    set (SERVER_CONNECTOR_SRC "src/server_connector_online.cpp")
else()
    set (SERVER_CONNECTOR_SRC "src/server_connector_offline.cpp")
endif()

add_executable (ServerConnector "src/main.cpp" "src/server_connector.cpp" ${SERVER_CONNECTOR_SRC}
    "src/server_connector_interface.cpp" "../shared/src/initialization_interface.cpp"
    "../shared/src/ipc_messages_initialization.cpp" "../shared/src/ipc_messages_logger.cpp")
add_dependencies (ServerConnector server_connector_edl_files)

target_compile_definitions (ServerConnector PRIVATE ENTITY_NAME="Server Connector")
target_compile_definitions (ServerConnector PRIVATE BOARD_ID="${BOARD_ID}")

if (SERVER)
    target_compile_definitions (ServerConnector PRIVATE SERVER_IP="${SERVER_IP}")
    target_link_libraries (ServerConnector ${vfs_CLIENT_LIB} ${wpa_CLIENT_LIB} ${mosquitto_CXX_LIB})
else ()
    target_compile_definitions (ServerConnector PRIVATE NO_SERVER)
endif()