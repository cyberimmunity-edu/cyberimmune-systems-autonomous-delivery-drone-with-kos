cmake_minimum_required (VERSION 3.12)

project (sdk)

# Initialize CMake library for KasperskyOS SDK.
include (platform)
initialize_platform (FORCE_STATIC)

if (PLATFORM_SUPPORTS_DYNAMIC_LINKING)
    message (FATAL_ERROR "The solution only supports static libraries")
endif ()

# Toolchain for working with the NK-parser.
include (platform/nk)

# Add functions for installing artefacts.
include (platform/install)

find_package (bsp REQUIRED)
include_directories (${bsp_INCLUDE})

# Add package for working with the virtual file system.
find_package (vfs REQUIRED)
include_directories (${vfs_INCLUDE})

# Add a package with the dhcpcd program implementation.
find_package (rump REQUIRED COMPONENTS DHCPCD_ENTITY)
include_directories (${rump_INCLUDE})

# Add package with VFS program implementations.
find_package (precompiled_vfs REQUIRED)
include_directories (${precompiled_vfs_INCLUDE})

# Add package with wpa_supplicant program implementations.
find_package (wpa REQUIRED COMPONENTS SUPPLICANT_ENTITY)
include_directories (${wpa_INCLUDE})

find_package (MbedTLS REQUIRED COMPONENTS)
include_directories (${MbedTLS_INCLUDE})

if (NOT FOR_SITL)
    # Add a package with the GPIO client library.
    find_package (gpio REQUIRED COMPONENTS CLIENT_LIB ENTITY)
    include_directories (${gpio_INCLUDE})

    find_package (uart REQUIRED)
    include_directories (${uart_INCLUDE})

    find_package (i2c REQUIRED)
    include_directories (${i2c_INCLUDE})
endif()

# Recommended flags against vulnerabilities.
add_compile_options (-Wall -Wextra -Wconversion
                     -fPIE -pie -D_FORTIFY_SOURCE=2 -O2
                     -fstack-protector-strong -fstack-clash-protection
                     -mbranch-protection=standard
                     -Wsign-conversion -Wformat=2 -Wformat-security -Werror=format-security
                     -fsanitize=undefined -fsanitize-undefined-trap-on-error)

add_link_options (LINKER:-z,noexecstack
                  LINKER:-z,separate-code
                  LINKER:-z,now)

add_subdirectory (test)
add_subdirectory (einit)
