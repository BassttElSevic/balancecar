idf_component_register(
    # 纯头文件的板级定义组件
    SRCS ""
    INCLUDE_DIRS "include"
    # board_pins.h 里用了 GPIO_NUM_xx / ADC_UNIT_x / ADC_CHANNEL_x 等枚举，
    # 都是公开头文件，消费者也要能看到，所以这两个依赖必须公开
    REQUIRES esp_driver_gpio esp_hal_ana_conv
)
