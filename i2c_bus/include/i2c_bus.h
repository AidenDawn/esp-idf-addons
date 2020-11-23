#ifndef _I2C_BUS_H_
#define _I2C_BUS_H_
#include "driver/i2c.h"

typedef void *i2c_bus_handle_t;
typedef uint8_t i2c_addr_t;


typedef struct {
    i2c_config_t i2c_conf;   /*!<I2C bus parameters*/
    i2c_port_t i2c_port;     /*!<I2C port number */
} i2c_bus_t;

typedef struct {
    i2c_port_t port;
    i2c_addr_t addr;
} i2c_dev_t;

/**
 * @brief Create and init I2C bus and return a I2C bus handle
 *
 * @param port I2C port number
 * @param conf Pointer to I2C parameters
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
i2c_bus_handle_t i2c_bus_create(i2c_port_t port, i2c_config_t *conf);


esp_err_t i2c_bus_write_bytes(i2c_port_t port, i2c_addr_t addr, uint8_t *reg, int reglen, uint8_t *data, int datalen);
esp_err_t i2c_bus_write_data(i2c_port_t port, i2c_addr_t addr, uint8_t *data, int datalen);
esp_err_t i2c_bus_read_bytes(i2c_port_t port, i2c_addr_t addr,  uint8_t *reg, int reglen, uint8_t *data, int datalen);
esp_err_t i2c_bus_read_data(i2c_port_t port, i2c_addr_t addr, uint8_t *data, int datalen);

/**
 * @brief Delete and release the I2C bus object
 *
 * @param bus I2C bus handle
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t i2c_bus_delete(i2c_bus_handle_t bus);

/**
 * @brief I2C start sending buffered commands
 *
 * @param bus I2C bus handle
 * @param cmd I2C cmd handle
 * @param ticks_to_wait Maximum blocking time
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t i2c_bus_cmd_begin(i2c_bus_handle_t bus, i2c_cmd_handle_t cmd, portBASE_TYPE ticks_to_wait);
#endif

