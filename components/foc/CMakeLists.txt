idf_component_register(
    SRCS "foc_motor.c" "foc_hw.c"
    INCLUDE_DIRS "include"
    REQUIRES foc_math foc_ctrl board motor encoder current
    PRIV_REQUIRES esp_timer
)
