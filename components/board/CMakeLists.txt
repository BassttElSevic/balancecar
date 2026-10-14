idf_component_register(
    # 纯头文件的板级定义组件
    SRCS ""
    INCLUDE_DIRS "include"
    # board_pins.h 里用了 GPIO_NUM_xx，属于公开头文件，消费者也要能看到
    REQUIRES esp_driver_gpio
)
