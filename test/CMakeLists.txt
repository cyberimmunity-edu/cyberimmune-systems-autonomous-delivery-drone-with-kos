project (test)

# Toolchain for working with the NK-parser.
include (platform/nk)

# Set compiler flags.
project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:YES")

nk_build_edl_files (test_edl_files NK_MODULE "sdk" EDL "${CMAKE_SOURCE_DIR}/resources/Test.edl")

add_executable (Test "src/main.cpp" "sdk/sdk_net.h" "sdk/sdk_net.cpp" "sdk/sdk_gpio.h" "sdk/sdk_gpio.cpp" "sdk/sdk_uart.h" "sdk/sdk_uart.cpp" "sdk/sdk_compass.h" "sdk/sdk_compass.cpp" "sdk/sdk_mpu.h" "sdk/sdk_mpu.cpp" ${EDL_FILES})
add_dependencies (Test test_edl_files)
target_compile_features (Test PRIVATE cxx_std_17)

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
    ${CMAKE_THREAD_LIBS_INIT}
)