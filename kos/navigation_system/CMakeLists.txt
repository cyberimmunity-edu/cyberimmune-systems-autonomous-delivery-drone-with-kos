project (NavigationSystem)

include (platform/nk)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

if (SIMULATION)
    set (NAVIGATION_SYSTEM_SRC "src/navigation_system_simulator.cpp")
else ()
    set (NAVIGATION_SYSTEM_SRC "src/navigation_system_real.cpp")
endif ()

add_executable (NavigationSystem "src/main.cpp" ${NAVIGATION_SYSTEM_SRC} "src/navigation_system_interface.cpp"
    "../ipc_messages/src/transport_interface.cpp" "../ipc_messages/src/initialization_interface.cpp")
add_dependencies (NavigationSystem navigation_system_edl_files)

target_compile_definitions (NavigationSystem PRIVATE ENTITY_NAME="Navigation System")

if (SIMULATION)
    target_compile_definitions (NavigationSystem PRIVATE SIMULATOR_IP="${SIMULATOR_IP}")
    target_link_libraries (NavigationSystem ${vfs_CLIENT_LIB} ${CMAKE_THREAD_LIBS_INIT})
else ()
    target_link_libraries (NavigationSystem ${bsp_CLIENT_LIB} ${uart_CLIENT_LIB})
endif ()