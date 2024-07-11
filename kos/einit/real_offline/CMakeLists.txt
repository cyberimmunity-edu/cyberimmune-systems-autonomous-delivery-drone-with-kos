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

#build_sd_image (sd-image
#                KOS_IMAGE_TARGET kos-image
#                IMAGE_FS ${IMAGE_FS})

set (QEMU_FLAGS "--nographic -monitor none -netdev user,id=net0,dhcpstart=10.0.2.15")
set (QEMU_ENV "QEMU_AUDIO_DRV=none")

build_kos_qemu_image (kos-qemu-image
                    EINIT_ENTITY EinitQemu
                    QEMU_FLAGS ${QEMU_FLAGS}
                    QEMU_ENV ${QEMU_ENV}
                    CONNECTIONS_CFG "src/init.yaml.in"
                    SECURITY_PSL "src/security.psl.in"
                    IMAGE_FILES ${ENTITIES})