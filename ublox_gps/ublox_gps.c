#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_log.h"

#include "i2c_bus.h"
#include "ublox_gps.h"

static char tag[] = "UBLOX_GPS";

void ublox_gps_init(){
    gpio_config_t gps_output_conf = {
        .intr_type = GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&gps_output_conf);

    gpio_config_t gps_input_conf = {
        .intr_type = GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPS_PPS),
        .pull_down_en = 1,
        .pull_up_en = 0,
    };
    gpio_config(&gps_input_conf);

    gpio_set_level(GPS_SAFE, 1);
    gpio_set_level(GPS_ON, 1);

    gpio_set_level(GPS_RST, 0);
    vTaskDelay(100);
    gpio_set_level(GPS_RST, 1);
}