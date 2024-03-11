project (CredentialManager)

include (platform/nk)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

if (SERVER)
    set (CREDENTIAL_MANAGER_SRC "src/credential_manager_online.cpp")
else()
    set (CREDENTIAL_MANAGER_SRC "src/credential_manager_offline.cpp")
endif()

add_executable (CredentialManager "src/main.cpp" ${CREDENTIAL_MANAGER_SRC}
    "src/credential_manager_shared.cpp" "src/credential_manager_interface.cpp"
    "../ipc_messages/src/transport_interface.cpp" "../ipc_messages/src/initialization_interface.cpp")
add_dependencies (CredentialManager credential_manager_edl_files)

target_compile_definitions (CredentialManager PRIVATE ENTITY_NAME="Credential Manager")
target_compile_definitions (CredentialManager PRIVATE BOARD_ID="id=${BOARD_ID}")
target_link_libraries (CredentialManager ${vfs_CLIENT_LIB} MbedTLS::mbedtls ${CMAKE_THREAD_LIBS_INIT})