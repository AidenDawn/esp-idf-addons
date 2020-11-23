#ifndef MAIN_DS3231_H_
#define MAIN_DS3231_H_

#include <time.h>

#include "i2c_bus.h"

#define TEST_LED 16
#define ESP_INTR_FLAG_DEFAULT               0

#define DS3231_I2C_ADDRESS_DEFAULT  		(0x68)   	// The device's I2C address is 0x68.

#define DS3231_REGISTER_SECOND				(0x00)
#define DS3231_REGISTER_MINUTE				(0x01)
#define DS3231_REGISTER_HOUR				(0x02)
#define DS3231_REGISTER_DAY					(0x03)
#define DS3231_REGISTER_DATE				(0x04)
#define DS3231_REGISTER_MONTH				(0x05)
#define DS3231_REGISTER_YEAR				(0x06)

#define DS3231_REGISTER_ALARM1_SECOND		(0x07)
#define DS3231_REGISTER_ALARM1_MINUTE		(0x08)
#define DS3231_REGISTER_ALARM1_HOUR			(0X09)
#define DS3231_REGISTER_ALARM1_DAY			(0X0A)
#define DS3231_REGISTER_ALARM2_MINUTES		(0X0B)
#define DS3231_REGISTER_ALARM2_HOURS		(0X0C)
#define DS3231_REGISTER_ALARM2_DAY			(0X0D)

#define DS3231_REGISTER_CONTROL				(0X0E)
#define DS3231_REGISTER_STATUS				(0X0F)
#define DS3231_REGISTER_OFFSET				(0X10)
#define DS3231_REGISTER_TEMP_MSB			(0X11)
#define DS3231_REGISTER_TEMP_LSB			(0X12)

#define DS3231_HOUR_12HOUR_FLAG  	        (0x40)
#define DS3231_HOUR_12HOUR_MASK  	        (0x1f)
#define DS3231_HOUR_PM_FLAG      	        (0x20)
#define DS3231_MONTH_CENTURY_MASK           (0x1f)
#define DS3231_MONTH_CENTURY_FLAG           (0x80)

#define DS3231_STATUS_OSCILLATOR            (0x80)
#define DS3231_STATUS_32KHZ                 (0x08)
#define DS3231_STATUS_ALARM_2               (0x02)
#define DS3231_STATUS_ALARM_1               (0x01)

#define DS3231_CONTROL_OSCILLATOR           (0x80)
#define DS3231_CONTROL_TEMPCONV             (0x20)
#define DS3231_CONTROL_ALARM_INTS           (0x04)
#define DS3231_CONTROL_ALARM2_INT           (0x02)
#define DS3231_CONTROL_ALARM1_INT           (0x01)

typedef enum {
    one     = (0x00),
    onek    = (0x08),
    fourk   = (0x10),
    eightk  = (0x18),
} sqwPinMode_t;

esp_err_t ds3231_init();
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