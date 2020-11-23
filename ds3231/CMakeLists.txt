set(srcs
    "ds3231.c")
set(COMPONENT_NAME "ds3231")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "include"
                    REQUIRES i2c_bus)
