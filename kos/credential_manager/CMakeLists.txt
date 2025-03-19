project (CredentialManager)

include (platform/nk)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

if (SERVER)
    set (CREDENTIAL_MANAGER_SRC "src/credential_manager_online.cpp")
else()
    set (CREDENTIAL_MANAGER_SRC "src/credential_manager_offline.cpp")
endif()

add_executable (CredentialManager "src/main.cpp" "src/credential_manager.cpp"
    ${CREDENTIAL_MANAGER_SRC} "src/credential_manager_interface.cpp"
    "../shared/src/initialization_interface.cpp" "../shared/src/ipc_messages_server_connector.cpp"
    "../shared/src/ipc_messages_initialization.cpp" "../shared/src/ipc_messages_logger.cpp")
add_dependencies (CredentialManager credential_manager_edl_files)
add_dependencies (CredentialManager server_connector_idl_files)

target_compile_definitions (CredentialManager PRIVATE ENTITY_NAME="Credential Manager")
target_link_libraries (CredentialManager ${vfs_CLIENT_LIB} MbedTLS::mbedtls ${CMAKE_THREAD_LIBS_INIT})