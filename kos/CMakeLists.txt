cmake_minimum_required (VERSION 3.12)

project (DroneController)

include (platform)
initialize_platform (FORCE_STATIC)
if (PLATFORM_SUPPORTS_DYNAMIC_LINKING)
    message (FATAL_ERROR "The solution only supports static libraries")
endif ()

include (platform/nk)
include (platform/install)

find_package (vfs REQUIRED)
include_directories (${vfs_INCLUDE})
find_package (precompiled_vfs REQUIRED)
include_directories (${precompiled_vfs_INCLUDE})

if (SIMULATION OR SERVER)
    find_package (rump REQUIRED COMPONENTS DHCPCD_ENTITY)
    include_directories (${rump_INCLUDE})
endif ()

if (SERVER)
    find_package (wpa REQUIRED COMPONENTS SUPPLICANT_ENTITY)
    include_directories (${wpa_INCLUDE})
endif ()

if (NOT SIMULATION)
    find_package (bsp REQUIRED)
    include_directories (${bsp_INCLUDE})
    find_package (gpio REQUIRED COMPONENTS CLIENT_LIB ENTITY)
    include_directories (${gpio_INCLUDE})
    find_package (uart REQUIRED)
    include_directories (${uart_INCLUDE})
endif ()

find_package (MbedTLS REQUIRED COMPONENTS)
include_directories (${MbedTLS_INCLUDE})

nk_build_idl_files (initialization_idl_files NK_MODULE "drone_controller" IDL "resources/Initialization.idl")

nk_build_idl_files (autopilot_connector_idl_files DEPENDS initialization_idl_files NK_MODULE "drone_controller" IDL "resources/AutopilotConnectorInterface.idl")
nk_build_idl_files (credential_manager_idl_files DEPENDS initialization_idl_files NK_MODULE "drone_controller" IDL "resources/CredentialManagerInterface.idl")
nk_build_idl_files (navigation_system_idl_files DEPENDS initialization_idl_files NK_MODULE "drone_controller" IDL "resources/NavigationSystemInterface.idl")
nk_build_idl_files (periphery_controller_idl_files DEPENDS initialization_idl_files NK_MODULE "drone_controller" IDL "resources/PeripheryControllerInterface.idl")
nk_build_idl_files (server_connector_idl_files DEPENDS initialization_idl_files NK_MODULE "drone_controller" IDL "resources/ServerConnectorInterface.idl")

nk_build_edl_files (autopilot_connector_edl_files IDL_TARGET autopilot_connector_idl_files NK_MODULE "drone_controller" EDL "resources/AutopilotConnector.edl")
nk_build_edl_files (credential_manager_edl_files IDL_TARGET credential_manager_idl_files NK_MODULE "drone_controller" EDL "resources/CredentialManager.edl")
nk_build_edl_files (navigation_system_edl_files IDL_TARGET navigation_system_idl_files NK_MODULE "drone_controller" EDL "resources/NavigationSystem.edl")
nk_build_edl_files (periphery_controller_edl_files IDL_TARGET periphery_controller_idl_files NK_MODULE "drone_controller" EDL "resources/PeripheryController.edl")
nk_build_edl_files (server_connector_edl_files IDL_TARGET server_connector_idl_files NK_MODULE "drone_controller" EDL "resources/ServerConnector.edl")

nk_build_edl_files (flight_controller_edl_files DEPENDS autopilot_connector_idl_files DEPENDS credential_manager_idl_files DEPENDS navigation_system_idl_files DEPENDS periphery_controller_idl_files DEPENDS server_connector_idl_files NK_MODULE "drone_controller" EDL "resources/FlightController.edl")

add_compile_options (-Wall -Wextra -Wconversion
                     -fPIE -pie -D_FORTIFY_SOURCE=2 -O2
                     -fstack-protector-strong -fstack-clash-protection
                     -mbranch-protection=standard
                     -Wsign-conversion -Wformat=2 -Wformat-security -Werror=format-security
                     -fsanitize=undefined -fsanitize-undefined-trap-on-error)

add_link_options (LINKER:-z,noexecstack
                  LINKER:-z,separate-code
                  LINKER:-z,now)

add_subdirectory (autopilot_connector)
add_subdirectory (credential_manager)
add_subdirectory (flight_controller)
add_subdirectory (navigation_system)
add_subdirectory (periphery_controller)
add_subdirectory (server_connector)

if (SIMULATION)
    if (SERVER)
        add_subdirectory (einit/simulator_server)
    else ()
        add_subdirectory (einit/simulator_offline)
    endif ()
else ()
    if (SERVER)
        add_subdirectory (einit/real_server)
    else ()
        add_subdirectory (einit/real_offline)
    endif ()
endif ()