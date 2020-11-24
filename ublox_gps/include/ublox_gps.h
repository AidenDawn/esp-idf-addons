#ifndef UBLOX_GPS_H_
#define UBLOX_GPS_H_

#include <time.h>

#include "i2c_bus.h"

#define GPS_RST     26
#define GPS_SAFE    25
#define GPS_INT     33
#define GPS_PPS     27
#define GPS_ON      32
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPS_RST) | (1ULL<<GPS_SAFE) | (1ULL<<GPS_INT) | (1ULL<<GPS_ON))
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPS_PPS))
#define ESP_INTR_FLAG_DEFAULT 0

void ublox_gps_init();

#endif //UBLOX_GPS_H_