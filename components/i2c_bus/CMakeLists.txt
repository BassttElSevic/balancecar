idf_component_register(
    SRCS "i2c_bus.c"
    INCLUDE_DIRS "include"
    # i2c_bus.h 里暴露了 i2c_master_bus_handle_t / i2c_port_num_t，
    # 消费者也要能看到，所以必须公开依赖
    REQUIRES esp_driver_i2c
)
