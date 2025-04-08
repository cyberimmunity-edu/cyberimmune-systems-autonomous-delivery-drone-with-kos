project (einit)

include (platform/image)
include (${KL_SDK_ROOT_PATH}/common/build-sd-image.cmake)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:YES")

find_package(kss REQUIRED)

nk_build_edl_files (klog_edl_files
                    NK_MODULE "drone_controller"
                    EDL "../../resources/KlogEntity.edl")

add_executable (KlogEntityHw "../../klog/src/klog_entity.c")
target_link_libraries (KlogEntityHw ${klog_SYSTEM_AUDIT_LIB})
add_dependencies (KlogEntityHw klog_edl_files)

add_executable (KlogEntityQemu "../../klog/src/klog_entity.c")
target_link_libraries (KlogEntityQemu ${klog_SYSTEM_AUDIT_LIB})
add_dependencies (KlogEntityQemu klog_edl_files)

set (ENTITIES
        AutopilotConnector
        CredentialManager
        FlightController
        NavigationSystem
        PeripheryController
        ServerConnector
        Logger
        ${precompiled_vfsVfsSdCardFs}
        KlogStorageEntity)
set (FILES)

set_target_properties (${precompiled_vfsVfsSdCardFs} PROPERTIES
    EXTRA_ARGS "\
    - -l
    - devfs /dev devfs 0")

set (DHCPCD_ENV "
    VFS_FILESYSTEM_BACKEND: client{fs->net}:kl.VfsSdCardFs
    VFS_NETWORK_BACKEND: client:kl.VfsNet")

set (DHCPCD_ARGS "\
    - '-4'
    - '-f'
    - /etc/dhcpcd.conf")

if (SIMULATION OR SERVER)
    set_target_properties (${precompiled_vfsVfsNet} PROPERTIES
        EXTRA_ARGS "\
        - -l
        - devfs /dev devfs rw
        - -l
        - ramfs /tmp ramfs rw
        - -l
        - romfs /etc romfs ro
        - -l
        - ramfs /var ramfs rw")
     set_target_properties (${precompiled_vfsVfsSdCardFs} PROPERTIES
        EXTRA_ARGS "\
        - -l
        - devfs /dev devfs rw
        - -l
        - ramfs /tmp ramfs rw
        - -l
        - romfs /etc romfs ro
        - -l
        - ramfs /var ramfs rw")

    set_target_properties (${rump_DHCPCD_ENTITY} PROPERTIES
        DEPENDS_ON_ENTITY "${precompiled_vfsVfsSdCardFs};${precompiled_vfsVfsNet}"
        EXTRA_ENV ${DHCPCD_ENV}
        EXTRA_ARGS ${DHCPCD_ARGS})
    list(APPEND ENTITIES ${precompiled_vfsVfsSdCardFs} ${precompiled_vfsVfsNet} ${rump_DHCPCD_ENTITY})
    list(APPEND FILES ${CMAKE_SOURCE_DIR}/resources/hdd/etc/dhcpcd.conf ${CMAKE_SOURCE_DIR}/resources/hdd/var/db/dhcpcd/.gitkeep)
endif ()

if (SERVER)
    if (NOT SIMULATION)
        set_target_properties (${rump_DHCPCD_ENTITY} PROPERTIES
            DEPENDS_ON_ENTITY ""
            EXTRA_CONNECTIONS "\
        - target: kl.VfsNet
          id: kl.VfsNet"
            EXTRA_ENV "\
            VFS_FILESYSTEM_BACKEND: client:kl.VfsNet
            VFS_NETWORK_BACKEND: client:kl.VfsNet"
            EXTRA_ARGS ${DHCPCD_ARGS})
        set_target_properties (${precompiled_vfsVfsNet} PROPERTIES
        DEPENDS_ON_ENTITY "${entropy_ENTITY};dnet_brcmfmac::server;${dnet_ENTITY}")
        set_target_properties (${wpa_SUPPLICANT_ENTITY} PROPERTIES
            DEPENDS_ON_ENTITY ""
            EXTRA_CONNECTIONS "\
        - target: kl.VfsNet
          id: kl.VfsNet"
            EXTRA_ENV "\
            VFS_FILESYSTEM_BACKEND: client:kl.VfsNet
            VFS_NETWORK_BACKEND: client:kl.VfsNet"
            EXTRA_ARGS "\
            - '-iwl0'
            - '-c/etc/wpa_supplicant.conf'")
        list(APPEND ENTITIES ${wpa_SUPPLICANT_ENTITY})
        list(APPEND FILES ${CMAKE_SOURCE_DIR}/resources/hdd/etc/wpa_supplicant.conf)
        add_custom_command (OUTPUT ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.bin
                    COMMAND cp /lib/firmware/brcm/brcmfmac43455-sdio.bin ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.bin)
        add_custom_command (OUTPUT ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.txt
                    COMMAND cp /lib/firmware/brcm/brcmfmac43455-sdio.raspberrypi,4-model-b.txt ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.txt)
        add_custom_command (OUTPUT ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.clm_blob
                    COMMAND cp /lib/firmware/brcm/brcmfmac43455-sdio.clm_blob ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.clm_blob)
        set (FIRMWARE_FILES ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.bin
                    ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.txt
                    ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.clm_blob)
    endif()
endif ()

if (SIMULATION)
    set (FS_IMAGE mmc0.img)
    if(EXISTS "${CMAKE_BINARY_DIR}/einit/drone_controller/${FS_IMAGE}")
      add_custom_target (${FS_IMAGE})
    else()
      add_custom_target (${FS_IMAGE} COMMAND ${KL_SDK_ROOT_PATH}/common/prepare_hdd_img.sh -s 128 -f fat32 -img ${FS_IMAGE})
    endif()
    list(APPEND ENTITIES ${FS_IMAGE})
endif ()

if (SIMULATION)
    if (SERVER)
        set (CONNECITONS_FILE "src/sim_online.yaml.in")
        set (SECURITY_FILE "src/sim_online.psl.in")
    else ()
        set (CONNECITONS_FILE "src/sim_offline.yaml.in")
        set (SECURITY_FILE "src/sim_offline.psl.in")
    endif ()
else ()
    if (SERVER)
        set (CONNECITONS_FILE "src/real_online.yaml.in")
        set (SECURITY_FILE "src/real_online.psl.in")
    else ()
        set (CONNECITONS_FILE "src/real_offline.yaml.in")
        set (SECURITY_FILE "src/real_offline.psl.in")
    endif ()
endif()

if (SIMULATION)
    set (INIT_KlogEntity_PATH "KlogEntityQemu")
    set (QEMU_FLAGS "-drive file=${FS_IMAGE},if=sd,format=raw")
    set (QEMU_ENV "QEMU_AUDIO_DRV=none")
    build_kos_qemu_image (kos-qemu-image
                        EINIT_ENTITY EinitQemu
                        QEMU_FLAGS ${QEMU_FLAGS}
                        QEMU_ENV ${QEMU_ENV}
                        QEMU_DEPENDENCIES ${FS_IMAGE}
                        CONNECTIONS_CFG ${CONNECITONS_FILE}
                        SECURITY_PSL ${SECURITY_FILE}
                        KLOG_ENTITY KlogEntityQemu
                        IMAGE_FILES ${ENTITIES} ${FILES})
    target_link_libraries (KlogEntityQemu kssaudit)
else ()
    set (INIT_KlogEntity_PATH "KlogEntityHw")
    build_kos_hw_image (kos-image
                        EINIT_ENTITY EinitHw
                        CONNECTIONS_CFG ${CONNECITONS_FILE}
                        SECURITY_PSL ${SECURITY_FILE}
                        KLOG_ENTITY KlogEntityHw
                        IMAGE_FILES ${ENTITIES} ${FIRMWARE_FILES} ${FILES})
    target_link_libraries (KlogEntityHw kssaudit)
endif()