project (einit)

include (platform/image)
include (${KL_SDK_ROOT_PATH}/common/build-sd-image.cmake)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:YES")

set_target_properties (${precompiled_vfsVfsSdCardFs} PROPERTIES
    EXTRA_ARGS "\
    - -l
    - devfs /dev devfs 0")

set (ENTITIES AutopilotConnector CredentialManager FlightController NavigationSystem PeripheryController ServerConnector
    Logger ${precompiled_vfsVfsSdCardFs})

build_kos_hw_image (kos-image
                    EINIT_ENTITY EinitHw
                    CONNECTIONS_CFG "src/init.yaml.in"
                    SECURITY_PSL "src/security.psl.in"
                    IMAGE_FILES ${ENTITIES})