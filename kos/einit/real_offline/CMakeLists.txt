project (einit)

include (platform/image)
include (${KL_SDK_ROOT_PATH}/common/build-sd-image.cmake)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:YES")

set_target_properties (${precompiled_vfsVfsSdCardFs} PROPERTIES
    EXTRA_ARGS "\
    - -l
    - devfs /dev devfs 0")

if (UNIT_TESTS)
    set (ENTITIES UnitTests ${precompiled_vfsVfsSdCardFs})
    set (CONNECTIONS_CFG_FILE "../unit_tests/init.yaml.in")
    set (SECURITY_PSL_FILE "../unit_tests/security.psl.in")
else ()
    set (ENTITIES AutopilotConnector CredentialManager FlightController NavigationSystem PeripheryController ServerConnector
        Logger ${precompiled_vfsVfsSdCardFs})
    set (CONNECTIONS_CFG_FILE "src/init.yaml.in")
    set (SECURITY_PSL_FILE "src/security.psl.in")
endif ()

build_kos_hw_image (kos-image
                    EINIT_ENTITY EinitHw
                    CONNECTIONS_CFG ${CONNECTIONS_CFG_FILE}
                    SECURITY_PSL ${SECURITY_PSL_FILE}
                    IMAGE_FILES ${ENTITIES})