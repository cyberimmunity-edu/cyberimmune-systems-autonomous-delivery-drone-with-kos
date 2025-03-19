project (einit)

include (platform/image)
include (test-generator/add_kss_pal_qemu_tests)

include (${KL_SDK_ROOT_PATH}/common/build-sd-image.cmake)

project_header_default ("STANDARD_GNU_14:YES" "STRICT_WARNINGS:YES")

set (ENTITIES
        AutopilotConnector
        CredentialManager
        FlightController
        NavigationSystem
        PeripheryController
        ServerConnector
        Logger
        ${precompiled_vfsVfsSdCardFs}
        ${precompiled_vfsVfsNet}
        ${rump_DHCPCD_ENTITY}
        ${wpa_SUPPLICANT_ENTITY}
        KlogStorageEntity
)


# Generate PAL tests.
set_target_properties (${precompiled_vfsVfsSdCardFs}
PROPERTIES
    EXTRA_ENV "
        VFS_FILESYSTEM_BACKEND: server:kl.VfsSdCardFs"
    EXTRA_ARGS "
        - -f
        - fstab"
)
add_kss_pal_qemu_tests (
    PSL_FILES "../drone_controller/src/pal_tests.psl.in"
    DEPENDS
        logger_edl_files
        autopilot_connector_edl_files
        credential_manager_edl_files
        navigation_system_edl_files
        periphery_controller_edl_files
        server_connector_edl_files
        flight_controller_edl_files
        klog_storage_edl_files
        klog_edl_files
    ENTITIES ${ENTITIES}
    CUSTOM_VFS
        ${precompiled_vfsVfsSdCardFs}
    ENV_VARS
        VFS_FILESYSTEM_BACKEND=client:kl.VfsSdCardFs
    )

add_custom_target (pal-test0 DEPENDS kos-qemu-image-PalTest0-sim)