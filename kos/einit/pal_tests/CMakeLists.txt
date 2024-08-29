project (einit)

include (platform/image)
include (${KL_SDK_ROOT_PATH}/common/build-sd-image.cmake)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:YES")

set (ENTITIES AutopilotConnector CredentialManager FlightController NavigationSystem PeripheryController ServerConnector
    Logger ${precompiled_vfsVfsSdCardFs} ${precompiled_vfsVfsNet} ${rump_DHCPCD_ENTITY} ${wpa_SUPPLICANT_ENTITY})

if (SIMULATION)
    if (SERVER)
        set (CONNECITONS_FILE "../drone_controller/src/sim_online.yaml.in")
        set (SECURITY_FILE "../drone_controller/src/sim_online.psl.in")
    else ()
        set (CONNECITONS_FILE "../drone_controller/src/sim_offline.yaml.in")
        set (SECURITY_FILE "../drone_controller/src/sim_offline.psl.in")
    endif ()
else ()
    if (SERVER)
        set (CONNECITONS_FILE "../drone_controller/src/real_online.yaml.in")
        set (SECURITY_FILE "../drone_controller/src/real_online.psl.in")
    else ()
        set (CONNECITONS_FILE "../drone_controller/src/real_offline.yaml.in")
        set (SECURITY_FILE "../drone_controller/src/real_offline.psl.in")
    endif ()
endif()

set (QEMU_FLAGS "--nographic -monitor none -netdev user,id=net0,dhcpstart=10.0.2.15")
set (QEMU_ENV "QEMU_AUDIO_DRV=none")
build_kos_qemu_image (kos-qemu-image
                    EINIT_ENTITY EinitQemu
                    QEMU_FLAGS ${QEMU_FLAGS}
                    QEMU_ENV ${QEMU_ENV}
                    CONNECTIONS_CFG ${CONNECITONS_FILE}
                    SECURITY_PSL ${SECURITY_FILE}
                    IMAGE_FILES ${ENTITIES})

add_kss_pal_qemu_tests (
    PSL_FILES "../drone_controller/src/pal_tests.psl.in"
    DEPENDS kos-qemu-image
    ENTITIES ${ENTITIES})