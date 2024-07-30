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
set_target_properties (${precompiled_vfsVfsSdCardFs} PROPERTIES
    EXTRA_ARGS "\
    - -l
    - devfs /dev devfs 0")

if (UNIT_TESTS)
    set (ENTITIES UnitTests ${precompiled_vfsVfsNet} ${rump_DHCPCD_ENTITY} ${wpa_SUPPLICANT_ENTITY} ${precompiled_vfsVfsSdCardFs})
    set (CONNECTIONS_CFG_FILE "../unit_tests/init.yaml.in")
    set (SECURITY_PSL_FILE "../unit_tests/security.psl.in")
else ()
    set (ENTITIES AutopilotConnector CredentialManager FlightController NavigationSystem PeripheryController ServerConnector
        Logger ${precompiled_vfsVfsNet} ${rump_DHCPCD_ENTITY} ${wpa_SUPPLICANT_ENTITY} ${precompiled_vfsVfsSdCardFs})
    set (CONNECTIONS_CFG_FILE "src/init.yaml.in")
    set (SECURITY_PSL_FILE "src/security.psl.in")
endif ()

set (FILES ${CMAKE_SOURCE_DIR}/resources/hdd/etc/wpa_supplicant.conf ${CMAKE_SOURCE_DIR}/resources/hdd/etc/dhcpcd.conf)

build_kos_hw_image (kos-image
                    EINIT_ENTITY EinitHw
                    CONNECTIONS_CFG ${CONNECTIONS_CFG_FILE}
                    SECURITY_PSL ${SECURITY_PSL_FILE}
                    IMAGE_FILES ${ENTITIES} ${FILES})