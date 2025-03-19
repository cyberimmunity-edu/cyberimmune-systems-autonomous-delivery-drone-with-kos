project (klog_storage)

set (ENTITIES ${ENTITIES} KlogStorageEntity PARENT_SCOPE)

add_nk_edl (klog_edl "${RESOURCES}/KlogEntity.edl"
            NK_MODULE       "drone_controller"
            LANG            "C")

add_nk_edl (klog_storage_edl "${RESOURCES}/KlogStorageEntity.edl"
            NK_MODULE       "drone_controller"
            LANG            "C")

add_nk_edl (kss_unit_test_edl "${RESOURCES}/KSSUnitTest.edl"
            NK_MODULE       "drone_controller"
            LANG            "C")


add_executable (KlogStorageEntity ${EDL_FILES} "src/klog_storage_entity.cpp")
target_link_libraries (KlogStorageEntity ${klog_storage_SERVER_LIB})
add_dependencies (KlogStorageEntity klog_edl klog_storage_edl)
