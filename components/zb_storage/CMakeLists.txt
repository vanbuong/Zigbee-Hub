# Pre-generated nanopb files live in proto/ alongside the .proto source.
# Regenerate with:
#   python -m nanopb.generator.nanopb_generator components/zb_storage/proto/zb_device.proto
# (requires: pip install nanopb protobuf>=6.31.1)

idf_component_register(
    SRCS
        "zb_storage.c"
        "zb_storage_config.c"
        "zb_storage_device.c"
        "proto/zb_device.pb.c"
    INCLUDE_DIRS
        "include"
        "proto"                 # for zb_device.pb.h
    REQUIRES
        json
        vfs
        livekit__nanopb
    PRIV_REQUIRES
        log
        esp_common
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-variable)
