#ifndef MAIN_DS3231_H_
#define MAIN_DS3231_H_

#include <time.h>

#include "i2c_bus.h"

#define TEST_LED 16
#define ESP_INTR_FLAG_DEFAULT 0

typedef enum
{
    one = (0x00),
    onek = (0x08),
    fourk = (0x10),
    eightk = (0x18),
} sqwPinMode_t;

esp_err_t ds3231_init(uint8_t int_pin);
esp_err_t ds3231_set_time(i2c_dev_t *dev, struct tm *time);
esp_err_t ds3231_get_time(i2c_dev_t *dev, struct tm *time);

esp_err_t ds3231_enable_alarm_interrupts(i2c_dev_t *dev);
esp_err_t ds3231_disable_alarm_interrupts(i2c_dev_t *dev);
esp_err_t ds3231_set_sqw_mode(i2c_dev_t *dev, sqwPinMode_t mode);

bool ds3231_power_lost(i2c_dev_t *dev);
bool ds3231_clk_status(i2c_dev_t *dev);
esp_err_t ds3231_enable_clk_out(i2c_dev_t *dev);
esp_err_t ds3231_disable_clk_out(i2c_dev_t *dev);

#endif //MAIN_DS3231_H_