project (NavigationSystem)

include (platform/nk)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

if (SIMULATION)
    set (NAVIGATION_SYSTEM_SRC "src/navigation_system_simulator.cpp")
else ()
    set (NAVIGATION_SYSTEM_SRC "src/navigation_system_real.cpp")
endif ()

add_executable (NavigationSystem "src/main.cpp" "src/navigation_system.cpp" ${NAVIGATION_SYSTEM_SRC}
    "src/navigation_system_interface.cpp" "../shared/src/initialization_interface.cpp"
    "../shared/src/ipc_messages_server_connector.cpp" "../shared/src/ipc_messages_initialization.cpp"
    "../shared/src/ipc_messages_logger.cpp")
add_dependencies (NavigationSystem navigation_system_edl_files)

target_compile_definitions (NavigationSystem PRIVATE ENTITY_NAME="Navigation System")

if (SIMULATION)
    target_compile_definitions (NavigationSystem PRIVATE SIMULATOR_IP="${SIMULATOR_IP}")
    target_link_libraries (NavigationSystem ${vfs_CLIENT_LIB})
else ()
    target_compile_definitions (NavigationSystem PRIVATE COORD_SRC=${COORD_SRC} ALT_SRC=${ALT_SRC})
    target_link_libraries (NavigationSystem ${bsp_CLIENT_LIB} ${uart_CLIENT_LIB} ${i2c_CLIENT_LIB})
endif ()
target_link_libraries (NavigationSystem ${CMAKE_THREAD_LIBS_INIT})