/*
 * u8g2_esp32_hal.h
 *
 *  Created on: Feb 12, 2017
 *      Author: kolban
 */

#ifndef COMPONENTS_GLCD_U8G2_ESP32_HAL_H_
#define COMPONENTS_GLCD_U8G2_ESP32_HAL_H_

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "u8g2.h"

#ifdef __cplusplus
extern "C" {
#endif

#define U8G2_ESP32_HAL_UNDEFINED (GPIO_NUM_0)

#define ACK_CHECK_EN 0x1   //  I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0  //  I2C master will not check ack from slave

typedef struct {
    gpio_num_t clk;
    gpio_num_t mosi;
    gpio_num_t sda;  // data for I²C
    gpio_num_t scl;  // clock for I²C
    gpio_num_t cs;
    gpio_num_t reset;
    gpio_num_t dc;
} u8g2_esp32_hal_t;

#define U8G2_ESP32_HAL_DEFAULT                                                                                  \
    {                                                                                                           \
        U8G2_ESP32_HAL_UNDEFINED, U8G2_ESP32_HAL_UNDEFINED, U8G2_ESP32_HAL_UNDEFINED, U8G2_ESP32_HAL_UNDEFINED, \
            U8G2_ESP32_HAL_UNDEFINED, U8G2_ESP32_HAL_UNDEFINED, U8G2_ESP32_HAL_UNDEFINED                        \
    }

void u8g2_esp32_hal_init(u8g2_esp32_hal_t u8g2_esp32_hal_param);
uint8_t u8g2_esp32_spi_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

#ifdef __cplusplus
}
#endif

#endif  // COMPONENTS_GLCD_U8G2_ESP32_HAL_H_
