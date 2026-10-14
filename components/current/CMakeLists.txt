idf_component_register(
    SRCS "current_sense.c"
    INCLUDE_DIRS "include"
    # current_sense.h 里暴露了 adc_oneshot_unit_handle_t / adc_cali_handle_t，
    # 消费者也要能看到，所以必须公开依赖
    REQUIRES board esp_adc
)
