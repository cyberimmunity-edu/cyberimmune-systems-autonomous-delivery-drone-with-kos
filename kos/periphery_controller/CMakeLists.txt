project (PeripheryController)

include (platform/nk)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

if (SIMULATION)
    set (PERIPHERY_CONTROLLER_SRC "src/periphery_controller_simulator.cpp")
else()
    set (PERIPHERY_CONTROLLER_SRC "src/periphery_controller_real.cpp" "src/gpio.cpp")
endif()

add_executable (PeripheryController "src/main.cpp" ${PERIPHERY_CONTROLLER_SRC} "src/periphery_controller_interface.cpp"
    "../ipc_messages/src/transport_interface.cpp" "../ipc_messages/src/initialization_interface.cpp")
add_dependencies (PeripheryController periphery_controller_edl_files)

target_compile_definitions (PeripheryController PRIVATE ENTITY_NAME="Periphery Controller")

target_link_libraries (PeripheryController ${CMAKE_THREAD_LIBS_INIT})

if (SIMULATION)
    target_compile_definitions (PeripheryController PRIVATE SIMULATOR_IP="${SIMULATOR_IP}")
    target_link_libraries (PeripheryController ${vfs_CLIENT_LIB})
else ()
    target_link_libraries (PeripheryController ${bsp_CLIENT_LIB} ${gpio_CLIENT_LIB})
endif()