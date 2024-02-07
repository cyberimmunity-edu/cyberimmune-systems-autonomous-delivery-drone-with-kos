project (einit)

# Tools for working with KasperskyOS images.
include (platform/image)
include (${KL_SDK_ROOT_PATH}/common/build-sd-image.cmake)

# Set compile flags.
project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:YES")

# Set the arguments for the VFS program.
set (VFS_NET_MOUNT_FS "\
    - -l
    - devfs /dev devfs rw
    - -l
    - ramfs /tmp ramfs rw
    - -l
    - romfs /etc romfs ro"
)

set_target_properties (${precompiled_vfsVfsNet} PROPERTIES
    EXTRA_ARGS "${VFS_NET_MOUNT_FS}"
)

set (ENTITIES
     Test
     ${precompiled_vfsVfsNet}
     ${wpa_SUPPLICANT_ENTITY}
     ${rump_DHCPCD_ENTITY})

set (SECURITY_PSL_FILE "security.psl.in")

set_target_properties (${wpa_SUPPLICANT_ENTITY} PROPERTIES
    DEPENDS_ON_ENTITY ""
    EXTRA_CONNECTIONS "\
  - target: kl.VfsNet
    id: kl.VfsNet"
    EXTRA_ENV "\
    VFS_FILESYSTEM_BACKEND: client:kl.VfsNet
    VFS_NETWORK_BACKEND: client:kl.VfsNet"
    EXTRA_ARGS "\
    - '-dddddd'
    - '-ien1'
    - '-c/etc/wpa_supplicant.conf'")

# Redefine DHCPCD entity connections
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

set (FILES ${CMAKE_SOURCE_DIR}/resources/hdd/etc/wpa_supplicant.conf
            ${CMAKE_SOURCE_DIR}/resources/hdd/etc/dhcpcd.conf)

# KasperskyOS image for the target hardware platform.
build_kos_hw_image (kos-image
                    EINIT_ENTITY EinitHw
                    CONNECTIONS_CFG "init.yaml.in"
                    SECURITY_PSL ${SECURITY_PSL_FILE}
                    IMAGE_FILES ${ENTITIES} ${FILES})

# SD card image with KasperskyOS for the target hardware platform.
build_sd_image (sd-image
                KOS_IMAGE_TARGET kos-image
                IMAGE_FS         ${IMAGE_FS})