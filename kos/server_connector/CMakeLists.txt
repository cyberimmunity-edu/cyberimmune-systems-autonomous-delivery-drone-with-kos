project (ServerConnector)

include (platform/nk)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

if (SERVER)
    set (SERVER_CONNECTOR_SRC "src/server_connector_online.cpp")
else()
    set (SERVER_CONNECTOR_SRC "src/server_connector_offline.cpp")
endif()

add_executable (ServerConnector "src/main.cpp" ${SERVER_CONNECTOR_SRC} "src/server_connector_interface.cpp"
    "../ipc_messages/src/transport_interface.cpp" "../ipc_messages/src/initialization_interface.cpp")
add_dependencies (ServerConnector server_connector_edl_files)

target_compile_definitions (ServerConnector PRIVATE ENTITY_NAME="Server Connector")

if (SERVER)
    target_link_libraries (ServerConnector ${vfs_CLIENT_LIB} ${wpa_CLIENT_LIB})
else ()
    target_compile_definitions (ServerConnector PRIVATE NO_SERVER)
endif()