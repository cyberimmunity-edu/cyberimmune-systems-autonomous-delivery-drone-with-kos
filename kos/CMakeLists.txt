cmake_minimum_required (VERSION 3.12)

project (DroneController)

include (platform)
initialize_platform (FORCE_STATIC)
if (PLATFORM_SUPPORTS_DYNAMIC_LINKING)
    message (FATAL_ERROR "The solution only supports static libraries")
endif ()

include (platform/nk)
include (platform/install)

set (RESOURCES ${CMAKE_SOURCE_DIR}/resources)

# Add a package importing components for building an audit program and
# connecting to it.
find_package (klog REQUIRED)
include_directories (${klog_INCLUDE})

# Add the package that imports klog_storage_entity.
find_package (klog_storage REQUIRED)
include_directories (${klog_storage_INCLUDE})

find_package (vfs REQUIRED)
include_directories (${vfs_INCLUDE})
find_package (precompiled_vfs REQUIRED)
include_directories (${precompiled_vfs_INCLUDE})

if (SIMULATION OR SERVER)
    find_package (rump REQUIRED COMPONENTS DHCPCD_ENTITY)
    include_directories (${rump_INCLUDE})
endif ()

if (NOT SIMULATION)
    if (SERVER)
        find_package (wpa REQUIRED COMPONENTS SUPPLICANT_ENTITY)
        find_package (dnet_brcmfmac REQUIRED)
        include_directories (${wpa_INCLUDE})
    endif ()
endif ()

if (SERVER)
    find_package (mosquitto REQUIRED COMPONENTS CXX_LIB)
    include_directories (${mosquitto_INCLUDE})
endif ()

if (NOT SIMULATION)
    find_package (bsp REQUIRED)
    include_directories (${bsp_INCLUDE})
    find_package (gpio REQUIRED COMPONENTS CLIENT_LIB ENTITY)
    include_directories (${gpio_INCLUDE})
    find_package (uart REQUIRED)
    include_directories (${uart_INCLUDE})
    find_package (i2c REQUIRED)
    include_directories (${i2c_INCLUDE})
endif ()

find_package (MbedTLS REQUIRED COMPONENTS)
include_directories (${MbedTLS_INCLUDE})

set (fmt_USE_STATIC ON)
set (spdlog_USE_STATIC ON)
set (fdn_USE_STATIC ON)
set (logger_USE_STATIC ON)
find_package (spdlog REQUIRED)
include_directories (${koslogger_INCLUDE})
find_package (logger REQUIRED)

if (UNIT_TESTS)
    nk_build_edl_files (unit_tests_edl_file NK_MODULE "drone_controller" EDL "resources/UnitTests.edl")
else ()
    nk_build_idl_files (initialization_idl_files NK_MODULE "drone_controller" IDL "resources/Initialization.idl")

    nk_build_idl_files (logger_idl_files DEPENDS initialization_idl_files NK_MODULE "drone_controller" IDL "resources/LoggerInterface.idl")

    nk_build_idl_files (autopilot_connector_idl_files DEPENDS logger_idl_files NK_MODULE "drone_controller" IDL "resources/AutopilotConnectorInterface.idl")
    nk_build_idl_files (credential_manager_idl_files DEPENDS logger_idl_files NK_MODULE "drone_controller" IDL "resources/CredentialManagerInterface.idl")
    nk_build_idl_files (navigation_system_idl_files DEPENDS logger_idl_files NK_MODULE "drone_controller" IDL "resources/NavigationSystemInterface.idl")
    nk_build_idl_files (periphery_controller_idl_files DEPENDS logger_idl_files NK_MODULE "drone_controller" IDL "resources/PeripheryControllerInterface.idl")
    nk_build_idl_files (server_connector_idl_files DEPENDS logger_idl_files NK_MODULE "drone_controller" IDL "resources/ServerConnectorInterface.idl")

    nk_build_edl_files (logger_edl_files IDL_TARGET logger_idl_files NK_MODULE "drone_controller" EDL "resources/Logger.edl")
    nk_build_edl_files (autopilot_connector_edl_files IDL_TARGET autopilot_connector_idl_files NK_MODULE "drone_controller" EDL "resources/AutopilotConnector.edl")
    nk_build_edl_files (credential_manager_edl_files IDL_TARGET credential_manager_idl_files NK_MODULE "drone_controller" EDL "resources/CredentialManager.edl")
    nk_build_edl_files (navigation_system_edl_files IDL_TARGET navigation_system_idl_files NK_MODULE "drone_controller" EDL "resources/NavigationSystem.edl")
    nk_build_edl_files (periphery_controller_edl_files IDL_TARGET periphery_controller_idl_files NK_MODULE "drone_controller" EDL "resources/PeripheryController.edl")
    nk_build_edl_files (server_connector_edl_files IDL_TARGET server_connector_idl_files NK_MODULE "drone_controller" EDL "resources/ServerConnector.edl")

    nk_build_edl_files (flight_controller_edl_files DEPENDS autopilot_connector_idl_files DEPENDS credential_manager_idl_files DEPENDS navigation_system_idl_files DEPENDS periphery_controller_idl_files DEPENDS server_connector_idl_files NK_MODULE "drone_controller" EDL "resources/FlightController.edl")

    nk_build_edl_files (klog_edl_files NK_MODULE "drone_controller" EDL "resources/KlogEntity.edl")
    nk_build_edl_files (klog_storage_edl_files NK_MODULE "drone_controller" EDL "resources/KlogStorageEntity.edl" NK_FLAGS "--extended-errors")

endif ()

add_compile_options (-Wall -Wextra -Wconversion
                     -fPIE -D_FORTIFY_SOURCE=2 -O2
                     -fstack-protector-strong
                     -mbranch-protection=standard
                     -Wsign-conversion -Wformat=2 -Wformat-security -Werror=format-security
                     -fsanitize=undefined -fsanitize-undefined-trap-on-error)

add_link_options (LINKER:-z,noexecstack
                  LINKER:-z,separate-code
                  LINKER:-z,now)

if (UNIT_TESTS)
    add_subdirectory (unit_tests)
    add_subdirectory (einit/unit_tests)
else ()
    add_subdirectory (logger)
    add_subdirectory (autopilot_connector)
    add_subdirectory (credential_manager)
    add_subdirectory (navigation_system)
    add_subdirectory (periphery_controller)
    add_subdirectory (server_connector)
    add_subdirectory (flight_controller)
    add_subdirectory (klog_storage)
    if (PAL_TESTS)
        add_subdirectory (einit/pal_tests)
    else ()
        add_subdirectory (einit/drone_controller)
    endif ()
endif ()