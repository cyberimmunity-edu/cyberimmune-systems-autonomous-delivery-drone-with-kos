project (Logger)

include (platform/nk)

project_header_default ("STANDARD_GNU_17:YES" "STRICT_WARNINGS:NO")

add_executable (Logger "src/main.cpp" "src/logger.cpp" "src/logger_interface.cpp"
    "../shared/src/initialization_interface.cpp" "../shared/src/ipc_messages_logger.cpp")
add_dependencies (Logger logger_edl_files)

target_compile_definitions (Logger PRIVATE ENTITY_NAME="Logger")
target_compile_features (Logger PRIVATE cxx_std_17)
target_link_libraries (Logger ${vfs_CLIENT_LIB} ${logger_LIB})