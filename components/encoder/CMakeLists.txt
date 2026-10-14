idf_component_register(
    SRCS "encoder.c"
    INCLUDE_DIRS "include"
    REQUIRES board
    # esp_timer 用于测速的时间基准，esp_driver_i2c 用于 AS5600 通信
    PRIV_REQUIRES esp_driver_i2c esp_timer
)
