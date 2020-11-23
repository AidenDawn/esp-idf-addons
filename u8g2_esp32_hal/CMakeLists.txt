set(srcs
    "app/app_gpio_events.c"
    "app/app_ssd1306.c"
    "u8g2_esp32_hal.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "include"
                    REQUIRES u8g2
                    REQUIRES ds3231)
