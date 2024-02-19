project (test)

# Toolchain for working with the NK-parser.
include (platform/nk)

# Set compiler flags.
project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:YES")

nk_build_edl_files (test_edl_files NK_MODULE "sdk" EDL "${CMAKE_SOURCE_DIR}/resources/Test.edl")

set (SDK_FILES "sdk/sdk_firmware.cpp" "sdk/sdk_gpio.cpp" "sdk/sdk_net.cpp"
    "sdk/sdk_autopilot_communication.cpp" "sdk/sdk_compass_qmc5883.cpp"
    "sdk/sdk_mpu_6050.cpp" "sdk/sdk_mission.cpp" "sdk/sdk_authenticity.cpp"
    "sdk/sdk_light.cpp" "sdk/sdk_kill_switch.cpp")

if (FOR_SITL)
    set (SITL_FILES "sdk/sdk_sim_sensors.cpp")
endif()

add_executable (Test "src/main.cpp" ${SDK_FILES} ${EDL_FILES} ${SITL_FILES})
add_dependencies (Test test_edl_files)
target_compile_features (Test PRIVATE cxx_std_17)

if (FOR_SITL)
    target_compile_definitions (Test PRIVATE "-DFOR_SITL")
endif()

if ("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm|aarch64")
    target_link_libraries (Test ${bsp_CLIENT_LIB})
endif ()

find_package (Threads)

target_link_libraries (
    Test m
    ${vfs_CLIENT_LIB}
    ${wpa_CLIENT_LIB}
    ${gpio_CLIENT_LIB}
    ${uart_CLIENT_LIB}
    ${i2c_CLIENT_LIB}
    MbedTLS::mbedtls
    ${CMAKE_THREAD_LIBS_INIT}
)