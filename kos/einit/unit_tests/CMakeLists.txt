project (einit)

include (platform/image)
include (${KL_SDK_ROOT_PATH}/common/build-sd-image.cmake)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:YES")

set_target_properties (${precompiled_vfsVfsSdCardFs} PROPERTIES
    EXTRA_ARGS "\
    - -l
    - devfs /dev devfs 0")

set (FS_IMAGE mmc0.img)
if(EXISTS "${CMAKE_BINARY_DIR}/einit/unit_tests/${FS_IMAGE}")
  add_custom_target (${FS_IMAGE})
else()
  add_custom_target (${FS_IMAGE} COMMAND ${KL_SDK_ROOT_PATH}/common/prepare_hdd_img.sh -s 128 -f fat32 -img ${FS_IMAGE})
endif()

set (ENTITIES UnitTests ${precompiled_vfsVfsSdCardFs})

set (QEMU_FLAGS "--nographic -monitor none -netdev user,id=net0,dhcpstart=10.0.2.15 -drive file=${FS_IMAGE},if=sd,format=raw")
set (QEMU_ENV "QEMU_AUDIO_DRV=none")
build_kos_qemu_image (kos-qemu-image
                    EINIT_ENTITY EinitQemu
                    QEMU_FLAGS ${QEMU_FLAGS}
                    QEMU_ENV ${QEMU_ENV}
                    QEMU_DEPENDENCIES ${FS_IMAGE}
                    CONNECTIONS_CFG "src/unit_tests.yaml.in"
                    SECURITY_PSL "src/unit_tests.psl.in"
                    IMAGE_FILES ${ENTITIES} ${FILES})