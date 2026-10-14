idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       # 自己的组件
                       REQUIRES board motor encoder current
                       # 不在 IDF 公共依赖列表里，需显式声明
                       PRIV_REQUIRES spi_flash esp_timer esp_app_format)
