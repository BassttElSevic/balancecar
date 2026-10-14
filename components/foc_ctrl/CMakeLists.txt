idf_component_register(
    SRCS "foc_pid.c" "foc_lpf.c"
    INCLUDE_DIRS "include"
    REQUIRES foc_math
    PRIV_REQUIRES esp_timer
)
