project (UnitTests)

include (platform/nk)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

set (CREDENTIAL_MANAGER_DIR ${CMAKE_SOURCE_DIR}/credential_manager)
set (LOGGER_DIR ${CMAKE_SOURCE_DIR}/logger)
set (FLIGHT_CONTROLLER_DIR ${CMAKE_SOURCE_DIR}/flight_controller)
set (NAVIGATION_SYSTEM_DIR ${CMAKE_SOURCE_DIR}/navigation_system)
set (PERIPHERY_CONTROLLER_DIR ${CMAKE_SOURCE_DIR}/periphery_controller)

add_executable (UnitTests "src/unit_tests.cpp" "src/mock_definition.cpp"
    "${CREDENTIAL_MANAGER_DIR}/src/credential_manager_online.cpp"
    "${CREDENTIAL_MANAGER_DIR}/src/credential_manager_shared.cpp"
    "${LOGGER_DIR}/src/logger.cpp" "${FLIGHT_CONTROLLER_DIR}/src/mission.cpp"
    "${NAVIGATION_SYSTEM_DIR}/src/navigation_system_shared.cpp"
    "${PERIPHERY_CONTROLLER_DIR}/src/periphery_controller_shared.cpp")

add_dependencies (UnitTests unit_tests_edl_file)

target_compile_definitions (UnitTests PRIVATE ENTITY_NAME="Unit Tests")
target_compile_definitions (UnitTests PRIVATE BOARD_ID="id=${BOARD_ID}")

target_compile_features (UnitTests PRIVATE cxx_std_17)

target_link_libraries (UnitTests MbedTLS::mbedtls gtest ${CMAKE_THREAD_LIBS_INIT} ${logger_LIB} ${vfs_CLIENT_LIB})