set(srcs
    "i2c_bus.c")
set(COMPONENT_NAME "i2c_bus")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "include")
