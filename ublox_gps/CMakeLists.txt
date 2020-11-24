set(srcs
    "ublox_gps.c")
set(COMPONENT_NAME "ublox_gps")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "include"
                    REQUIRES i2c_bus)
