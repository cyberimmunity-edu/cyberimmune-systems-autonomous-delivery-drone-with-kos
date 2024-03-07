project (einit)

include (platform/image)
include (${KL_SDK_ROOT_PATH}/common/build-sd-image.cmake)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:YES")

set_target_properties (${precompiled_vfsVfsNet} PROPERTIES
    EXTRA_ARGS "\
    - -l
    - devfs /dev devfs rw
    - -l
    - ramfs /tmp ramfs rw
    - -l
    - romfs /etc romfs ro")
set_target_properties (${rump_DHCPCD_ENTITY} PROPERTIES
    DEPENDS_ON_ENTITY ""
    EXTRA_CONNECTIONS "\
  - target: kl.VfsNet
    id: kl.VfsNet"
    EXTRA_ENV "\
    VFS_FILESYSTEM_BACKEND: client:kl.VfsNet
    VFS_NETWORK_BACKEND: client:kl.VfsNet"
    EXTRA_ARGS "\
    - '-4'
    - -f
    - /etc/dhcpcd.conf")

set (ENTITIES AutopilotConnector CredentialManager FlightController NavigationSystem PeripheryController ServerConnector
    ${precompiled_vfsVfsNet} ${rump_DHCPCD_ENTITY})

set (FILES ${CMAKE_SOURCE_DIR}/resources/hdd/etc/dhcpcd.conf)

build_kos_hw_image (kos-image
                    EINIT_ENTITY EinitHw
                    CONNECTIONS_CFG "src/init.yaml.in"
                    SECURITY_PSL "src/security.psl.in"
                    IMAGE_FILES ${ENTITIES} ${FILES})

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
                    IMAGE_FILES ${ENTITIES} ${FILES})