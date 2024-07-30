project (UnitTests)

include (platform/nk)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

set (AUTOPILOT_CONNECTOR_DIR ${CMAKE_SOURCE_DIR}/autopilot_connector)
set (CREDENTIAL_MANAGER_DIR ${CMAKE_SOURCE_DIR}/credential_manager)
set (NAVIGATION_SYSTEM_DIR ${CMAKE_SOURCE_DIR}/navigation_system)
set (PERIPHERY_CONTROLLER_DIR ${CMAKE_SOURCE_DIR}/periphery_controller)
set (SERVER_CONNECTOR_DIR ${CMAKE_SOURCE_DIR}/server_connector)
set (LOGGER_DIR ${CMAKE_SOURCE_DIR}/logger)
set (FLIGHT_CONTROLLER_DIR ${CMAKE_SOURCE_DIR}/flight_controller)

if (SIMULATION)
    set (UNIT_TESTS_SIM_REAL_SRC "${AUTOPILOT_CONNECTOR_DIR}/src/autopilot_connector_simulator.cpp"
        "${NAVIGATION_SYSTEM_DIR}/src/navigation_system_simulator.cpp"
        "${PERIPHERY_CONTROLLER_DIR}/src/periphery_controller_simulator.cpp")
else ()
    set (UNIT_TESTS_SIM_REAL_SRC "${AUTOPILOT_CONNECTOR_DIR}/src/autopilot_connector_real.cpp"
        "${NAVIGATION_SYSTEM_DIR}/src/navigation_system_real.cpp"
        "${PERIPHERY_CONTROLLER_DIR}/src/periphery_controller_real.cpp")
endif ()

if (SERVER)
    set (UNIT_TESTS_OFF_ON_SRC "${CREDENTIAL_MANAGER_DIR}/src/credential_manager_online.cpp"
        "${SERVER_CONNECTOR_DIR}/src/server_connector_online.cpp")
else ()
    set (UNIT_TESTS_OFF_ON_SRC "${CREDENTIAL_MANAGER_DIR}/src/credential_manager_offline.cpp"
        "${SERVER_CONNECTOR_DIR}/src/server_connector_offline.cpp")
endif ()

add_executable (UnitTests "src/unit_tests.cpp" ${UNIT_TESTS_SIM_REAL_SRC} ${UNIT_TESTS_OFF_ON_SRC}
    "${CREDENTIAL_MANAGER_DIR}/src/credential_manager_shared.cpp"
    "${NAVIGATION_SYSTEM_DIR}/src/navigation_system_shared.cpp"
    "${PERIPHERY_CONTROLLER_DIR}/src/periphery_controller_shared.cpp"
    "${LOGGER_DIR}/src/logger.cpp" "${FLIGHT_CONTROLLER_DIR}/src/mission.cpp")

add_dependencies (UnitTests unit_tests_edl_file)

target_compile_definitions (UnitTests PRIVATE ENTITY_NAME="Unit Tests")
target_compile_definitions (UnitTests PRIVATE BOARD_ID="id=${BOARD_ID}")

target_compile_features (UnitTests PRIVATE cxx_std_17)

if (SIMULATION)
    target_compile_definitions (UnitTests PRIVATE SIMULATOR_IP="${SIMULATOR_IP}")
else ()
    target_link_libraries (UnitTests ${bsp_CLIENT_LIB} ${uart_CLIENT_LIB} ${i2c_CLIENT_LIB} ${gpio_CLIENT_LIB})
endif ()

if (SERVER)
    target_compile_definitions (UnitTests PRIVATE SERVER_IP="${SERVER_IP}")
    target_link_libraries (UnitTests ${wpa_CLIENT_LIB})
else ()
    target_compile_definitions (UnitTests PRIVATE NO_SERVER)
endif()

target_link_libraries (UnitTests MbedTLS::mbedtls gtest ${vfs_CLIENT_LIB} ${CMAKE_THREAD_LIBS_INIT} ${logger_LIB})