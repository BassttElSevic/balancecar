idf_component_register(
    SRCS "current_sense.c"
    INCLUDE_DIRS "include"
    REQUIRES board
    PRIV_REQUIRES esp_adc
)
