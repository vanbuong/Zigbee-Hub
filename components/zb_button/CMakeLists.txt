idf_component_register(
    SRCS "zb_button.c"
    INCLUDE_DIRS "include"
    REQUIRES
        zb_device_mgr
        driver
        freertos
        log
)
