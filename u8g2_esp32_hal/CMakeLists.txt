set(srcs
    "u8g2_esp32_hal.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "include"
                    REQUIRES u8g2)
