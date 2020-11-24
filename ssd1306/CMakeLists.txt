set(srcs 
    "ssd1306.c"
    "ssd1306_spi.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "include")
