idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES board current motor
                       PRIV_REQUIRES esp_adc esp_driver_gpio)
