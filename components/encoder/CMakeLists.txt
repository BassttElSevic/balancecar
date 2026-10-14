idf_component_register(
    SRCS "encoder.c"
    INCLUDE_DIRS "include"
    REQUIRES i2c_bus
    PRIV_REQUIRES esp_timer
)
