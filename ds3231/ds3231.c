#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_log.h"

#include "i2c_bus.h"
#include "ds3231.h"

static char tag[] = "DS3231";

static uint8_t DS3231_I2C_ADDRESS_DEFAULT = (0x68); // The device's I2C address is 0x68.

static uint8_t DS3231_REGISTER_SECOND = (0x00);
static uint8_t DS3231_REGISTER_MINUTE = (0x01);
static uint8_t DS3231_REGISTER_HOUR = (0x02);
static uint8_t DS3231_REGISTER_DAY = (0x03);
static uint8_t DS3231_REGISTER_DATE = (0x04);
static uint8_t DS3231_REGISTER_MONTH = (0x05);
static uint8_t DS3231_REGISTER_YEAR = (0x06);

static uint8_t DS3231_REGISTER_ALARM1_SECOND = (0x07);
static uint8_t DS3231_REGISTER_ALARM1_MINUTE = (0x08);
static uint8_t DS3231_REGISTER_ALARM1_HOUR = (0X09);
static uint8_t DS3231_REGISTER_ALARM1_DAY = (0X0A);
static uint8_t DS3231_REGISTER_ALARM2_MINUTES = (0X0B);
static uint8_t DS3231_REGISTER_ALARM2_HOURS = (0X0C);
static uint8_t DS3231_REGISTER_ALARM2_DAY = (0X0D);

static uint8_t DS3231_REGISTER_CONTROL = (0X0E);
static uint8_t DS3231_REGISTER_STATUS = (0X0F);
static uint8_t DS3231_REGISTER_OFFSET = (0X10);
static uint8_t DS3231_REGISTER_TEMP_MSB = (0X11);
static uint8_t DS3231_REGISTER_TEMP_LSB = (0X12);

static uint8_t DS3231_HOUR_12HOUR_FLAG = (0x40);
static uint8_t DS3231_HOUR_12HOUR_MASK = (0x1f);
static uint8_t DS3231_HOUR_PM_FLAG = (0x20);
static uint8_t DS3231_MONTH_CENTURY_MASK = (0x1f);
static uint8_t DS3231_MONTH_CENTURY_FLAG = (0x80);

static uint8_t DS3231_STATUS_OSCILLATOR = (0x80);
static uint8_t DS3231_STATUS_32KHZ = (0x08);
static uint8_t DS3231_STATUS_ALARM_2 = (0x02);
static uint8_t DS3231_STATUS_ALARM_1 = (0x01);

static uint8_t DS3231_CONTROL_OSCILLATOR = (0x80);
static uint8_t DS3231_CONTROL_TEMPCONV = (0x20);
static uint8_t DS3231_CONTROL_ALARM_INTS = (0x04);
static uint8_t DS3231_CONTROL_ALARM2_INT = (0x02);
static uint8_t DS3231_CONTROL_ALARM1_INT = (0x01);

uint8_t bcd2dec(uint8_t val){
    return (val >> 4) * 10 + (val & 0x0f);
}

uint8_t dec2bcd(uint8_t val){
    return ((val / 10) << 4) + (val % 10);
}

static volatile bool status = false;

static void IRAM_ATTR rtc_pps_handler(void* arg){
    status = !status;
    gpio_set_level(TEST_LED, status);
}

esp_err_t ds3231_init(void* arg){
    esp_err_t ret = ESP_OK;
    uint8_t DS3231_INT_GPIO = arg;

	gpio_config_t rtc_int = {
        .intr_type = GPIO_PIN_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << DS3231_INT_GPIO),
        .pull_down_en = true,
        .pull_up_en = false,
    };
    ret |= gpio_config(&rtc_int);
    gpio_set_direction(TEST_LED, GPIO_MODE_OUTPUT);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(CONFIG_DS3231_INT_GPIO, rtc_pps_handler, (void*) 0);

    return ret;    
}

esp_err_t ds3231_set_time(i2c_dev_t *dev, struct tm *time){
    uint8_t data[7];

    data[0] = dec2bcd(time->tm_sec);
    data[1] = dec2bcd(time->tm_min);
    data[2] = dec2bcd(time->tm_hour);
    data[3] = dec2bcd(time->tm_wday + 1); // tm weekday[0-6] -> DS3231 weekday[1-7]
    data[4] = dec2bcd(time->tm_mday);
    if(time->tm_year > 2000){
        data[5] = dec2bcd(time->tm_mon + 1) & DS3231_MONTH_CENTURY_FLAG; // tm month[0-11] -> DS3231 month[1-12] + CENTURY
    } else {
        data[5] = dec2bcd(time->tm_mon + 1); // tm month[0-11] -> DS3231 month[1-12]
    }
    data[6] = dec2bcd(time->tm_year);

    return i2c_bus_write_bytes(dev->port, dev->addr, &DS3231_REGISTER_SECOND, 1, data, sizeof(data));
}

esp_err_t ds3231_get_time(i2c_dev_t *dev, struct tm *time){
    esp_err_t ret = ESP_FAIL;

    uint8_t data[7];
    if( i2c_bus_read_bytes(dev->port, dev->addr, &DS3231_REGISTER_SECOND, 1, data, sizeof(data)) == ESP_FAIL){
        return ret;
    }

    time->tm_sec = bcd2dec(data[0]);
    time->tm_min = bcd2dec(data[1]);
    if (data[2] & DS3231_HOUR_12HOUR_FLAG){
        time->tm_hour = bcd2dec(data[2] & DS3231_HOUR_12HOUR_MASK) - 1; // 12 Hour Mode
        if (data[2] & DS3231_HOUR_PM_FLAG){
            time->tm_hour += 12; //isPM: add 12
        }
    }
    else time->tm_hour = bcd2dec(data[2]); /* 24H */
    time->tm_wday = bcd2dec(data[3]) - 1;
    time->tm_mday = bcd2dec(data[4]);
    if(data[5] & DS3231_MONTH_CENTURY_FLAG){
        data[5] &= ~DS3231_MONTH_CENTURY_FLAG;
        time->tm_mon = bcd2dec(data[5]) - 1; // tm month[0-11] -> DS3231 month[1-12] - CENTURY
    } else {
        time->tm_mon = bcd2dec(data[5]) - 1; // tm month[0-11] -> DS3231 month[1-12]
    }
    time->tm_year = bcd2dec(data[6]);    

    // Could process setting of TZs and DST here!

    return ESP_OK;
}

bool ds3231_power_lost(i2c_dev_t *dev){
    uint8_t data;
    if(i2c_bus_read_bytes(dev->port, dev->addr, &DS3231_REGISTER_STATUS, 1, &data, 1) == ESP_FAIL){
        return false;
    }
    if(data & DS3231_STATUS_OSCILLATOR){
        data &= ~DS3231_STATUS_OSCILLATOR;
        i2c_bus_write_bytes(dev->port, dev->addr, &DS3231_REGISTER_STATUS, 1, &data, 1);
        return true;
    }

    return false;
}

bool ds3231_clk_status(i2c_dev_t *dev){

    uint8_t data;
    if(i2c_bus_read_bytes(dev->port, dev->addr, &DS3231_REGISTER_STATUS, 1, &data, 1) == ESP_FAIL){
        return false;
    }

    return (data & DS3231_STATUS_32KHZ) ? true : false;
}

esp_err_t ds3231_enable_clk_out(i2c_dev_t *dev){
    esp_err_t ret = ESP_OK;
    uint8_t data;
    if(!ds3231_clk_status(dev)){
        i2c_bus_read_bytes(dev->port, dev->addr, &DS3231_REGISTER_STATUS, 1, &data, 1);
        data |= DS3231_STATUS_32KHZ;
        ret = i2c_bus_write_bytes(dev->port, dev->addr, &DS3231_REGISTER_CONTROL, 1, &data, 1);
    }
    ESP_LOGI(tag, "32kHz output enabled");

    return ret;
}

esp_err_t ds3231_disable_clk_out(i2c_dev_t *dev){
    esp_err_t ret = ESP_OK;

    uint8_t data;
    if(ds3231_clk_status(dev)){
        i2c_bus_read_bytes(dev->port, dev->addr, &DS3231_REGISTER_STATUS, 1, &data, 1);
        data &= ~DS3231_STATUS_32KHZ;
        ret = i2c_bus_write_bytes(dev->port, dev->addr, &DS3231_REGISTER_CONTROL, 1, &data, 1);
    }
    ESP_LOGI(tag, "32kHz output disabled");

    return ret;
}

esp_err_t ds3231_enable_alarm_interrupts(i2c_dev_t *dev){
    esp_err_t ret = ESP_OK;

    uint8_t data;
    ret |= i2c_bus_read_bytes(dev->port, dev->addr, &DS3231_REGISTER_CONTROL, 1, &data, 1);
    data |= DS3231_CONTROL_ALARM_INTS;
    ret |= i2c_bus_write_bytes(dev->port, dev->addr, &DS3231_REGISTER_CONTROL, 1, &data, 1);

    return ret;
}

esp_err_t ds3231_disable_alarm_interrupts(i2c_dev_t *dev){
    esp_err_t ret = ESP_OK;

    uint8_t data;
    i2c_bus_read_bytes(dev->port, dev->addr, &DS3231_REGISTER_CONTROL, 1, &data, 1);
    data &= ~DS3231_CONTROL_ALARM_INTS;
    ret = i2c_bus_write_bytes(dev->port, dev->addr, &DS3231_REGISTER_CONTROL, 1, &data, 1);

    return ret;
}

esp_err_t ds3231_set_sqw_mode(i2c_dev_t *dev, sqwPinMode_t mode){
    esp_err_t ret = ESP_OK;

    uint8_t data;
    ret |= i2c_bus_read_bytes(dev->port, dev->addr, &DS3231_REGISTER_CONTROL, 1, &data, 1);
    data &= ~(eightk);
    data |= mode;
    ret |= i2c_bus_write_bytes(dev->port, dev->addr, &DS3231_REGISTER_CONTROL, 1, &data, 1);

    return ret;
}