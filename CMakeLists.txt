idf_component_register(
    SRCS "hello_world_main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        driver
        esp_timer
        freertos
        log
        esp_common
        esp_hw_support
)
# Se o warning do driver legacy virar erro:
# target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=cpp)
