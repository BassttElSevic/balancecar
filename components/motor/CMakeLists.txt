idf_component_register(
    SRCS "motor.c"
    INCLUDE_DIRS "include"
    # board_pins.h 来自 board 组件
    REQUIRES board
    PRIV_REQUIRES esp_driver_mcpwm esp_driver_gpio
)
