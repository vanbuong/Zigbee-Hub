# nanopb_generate_c is provided by the nanopb managed component.
# It must run before idf_component_register so PROTO_SRCS is populated.
nanopb_generate_c(PROTO_SRCS PROTO_HDRS
    RELPATH "${CMAKE_CURRENT_SOURCE_DIR}/proto"
    "${CMAKE_CURRENT_SOURCE_DIR}/proto/zb_device.proto"
)

idf_component_register(
    SRCS
        "zb_storage.c"
        "zb_storage_config.c"
        "zb_storage_device.c"
        ${PROTO_SRCS}
    INCLUDE_DIRS
        "include"
        "${CMAKE_CURRENT_BINARY_DIR}"   # for generated zb_device.pb.h
    REQUIRES
        json
        nanopb
        esp_vfs
    PRIV_REQUIRES
        log
        esp_common
)

# Suppress nanopb-generated unused-variable warnings
set_source_files_properties(${PROTO_SRCS}
    PROPERTIES COMPILE_FLAGS "-Wno-unused-variable"
)
