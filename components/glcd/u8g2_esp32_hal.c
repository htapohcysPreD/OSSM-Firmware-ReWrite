#include "u8g2_esp32_hal.h"

#include <stdio.h>
#include <string.h>

#include "../../main/ossm_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "u8g2_hal";
static const unsigned int I2C_TIMEOUT_MS = 1000;

static spi_device_handle_t handle_spi;   // SPI handle.
static i2c_cmd_handle_t handle_i2c;      // I2C handle.
static u8g2_esp32_hal_t u8g2_esp32_hal;  // HAL state data.

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)                         \
    do {                                           \
        esp_err_t rc = (x);                        \
        if (rc != ESP_OK) {                        \
            ESP_LOGE("err", "esp_err_t = %d", rc); \
            assert(0 && #x);                       \
        }                                          \
    } while (0);

/*
 * Initialze the ESP32 HAL.
 */
void u8g2_esp32_hal_init(u8g2_esp32_hal_t u8g2_esp32_hal_param) {
    u8g2_esp32_hal = u8g2_esp32_hal_param;
}  // u8g2_esp32_hal_init

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is invoked
 * to handle SPI communications.
 */
uint8_t u8g2_esp32_spi_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_BYTE_SET_DC:
            if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.dc, arg_int);
            }
            break;

        case U8X8_MSG_BYTE_INIT: {
            if (u8g2_esp32_hal.clk == U8G2_ESP32_HAL_UNDEFINED || u8g2_esp32_hal.mosi == U8G2_ESP32_HAL_UNDEFINED ||
                u8g2_esp32_hal.cs == U8G2_ESP32_HAL_UNDEFINED) {
                break;
            }

            spi_bus_config_t bus_config;
            memset(&bus_config, 0, sizeof(spi_bus_config_t));
            bus_config.sclk_io_num = u8g2_esp32_hal.clk;   // CLK
            bus_config.mosi_io_num = u8g2_esp32_hal.mosi;  // MOSI
            bus_config.miso_io_num = -1;                   // MISO
            bus_config.quadwp_io_num = -1;                 // Not used
            bus_config.quadhd_io_num = -1;                 // Not used
            ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &bus_config, 1));

            spi_device_interface_config_t dev_config;
            dev_config.address_bits = 0;
            dev_config.command_bits = 0;
            dev_config.dummy_bits = 0;
            dev_config.mode = 0;
            dev_config.duty_cycle_pos = 0;
            dev_config.cs_ena_posttrans = 0;
            dev_config.cs_ena_pretrans = 0;
            dev_config.clock_speed_hz = 10000;
            dev_config.spics_io_num = u8g2_esp32_hal.cs;
            dev_config.flags = 0;
            dev_config.queue_size = 200;
            dev_config.pre_cb = NULL;
            dev_config.post_cb = NULL;
            ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev_config, &handle_spi));

            break;
        }

        case U8X8_MSG_BYTE_SEND: {
            spi_transaction_t trans_desc;
            trans_desc.addr = 0;
            trans_desc.cmd = 0;
            trans_desc.flags = 0;
            trans_desc.length = 8 * arg_int;  // Number of bits NOT number of bytes.
            trans_desc.rxlength = 0;
            trans_desc.tx_buffer = arg_ptr;
            trans_desc.rx_buffer = NULL;

            // ESP_LOGI(TAG, "... Transmitting %d bytes.", arg_int);
            ESP_ERROR_CHECK(spi_device_transmit(handle_spi, &trans_desc));
            break;
        }
    }
    return 0;
}  // u8g2_esp32_spi_byte_cb

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is invoked
 * to handle I2C communications.
 */
uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_BYTE_SET_DC:  // Set the level of the data/command pin. arg_int contains the expected output
                                    // level. Use u8x8_gpio_SetDC(u8x8, arg_int) to send a message to the GPIO
                                    // procedure.
        {
            if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.dc, arg_int);
            }
            break;
        }
        case U8X8_MSG_BYTE_INIT:  // Send once during the init phase of the display.
        {
            if (u8g2_esp32_hal.sda == U8G2_ESP32_HAL_UNDEFINED || u8g2_esp32_hal.scl == U8G2_ESP32_HAL_UNDEFINED) {
                break;
            }

            i2c_config_t i2c_conf = {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = REMOTE_LCD_SDA,
                .scl_io_num = REMOTE_LCD_CLK,
                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                .master.clk_speed = REMOTE_LCD_CLOCKSPEED,
            };
            ESP_ERROR_CHECK(i2c_param_config(REMOTE_LCD_I2C_HOST, &i2c_conf));
            ESP_ERROR_CHECK(i2c_driver_install(REMOTE_LCD_I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));
            break;
        }

        case U8X8_MSG_BYTE_SEND:  // Send one or more bytes, located at arg_ptr, arg_int contains the number of bytes.
        {
            uint8_t *data_ptr = (uint8_t *)arg_ptr;  // NOLINT(readability/casting)
            ESP_LOG_BUFFER_HEXDUMP(TAG, data_ptr, arg_int, ESP_LOG_VERBOSE);

            while (arg_int > 0) {
                ESP_ERROR_CHECK(i2c_master_write_byte(handle_i2c, *data_ptr, ACK_CHECK_EN));
                data_ptr++;
                arg_int--;
            }
            break;
        }

        case U8X8_MSG_BYTE_START_TRANSFER:  // Set the chip select line here. u8x8->display_info->chip_enable_level
                                            // contains the expected level. Use u8x8_gpio_SetCS(u8x8,
                                            // u8x8->display_info->chip_enable_level) to call the GPIO procedure.
        {
            uint8_t i2c_address = u8x8_GetI2CAddress(u8x8);
            handle_i2c = i2c_cmd_link_create();
            ESP_ERROR_CHECK(i2c_master_start(handle_i2c));
            ESP_ERROR_CHECK(i2c_master_write_byte(handle_i2c, i2c_address | I2C_MASTER_WRITE, ACK_CHECK_EN));
            break;
        }

        case U8X8_MSG_BYTE_END_TRANSFER:  // Unselect the device. Use the CS level from here:
                                          // u8x8->display_info->chip_disable_level.
        {
            ESP_ERROR_CHECK(i2c_master_stop(handle_i2c));
            ESP_ERROR_CHECK(i2c_master_cmd_begin(REMOTE_LCD_I2C_HOST, handle_i2c, I2C_TIMEOUT_MS / portTICK_PERIOD_MS));
            i2c_cmd_link_delete(handle_i2c);
            break;
        }
    }
    return 0;
}  // u8g2_esp32_i2c_byte_cb

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is invoked
 * to handle callbacks for GPIO and delay functions.
 */
uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
            // Initialize the GPIO and DELAY HAL functions.  If the pins for DC and RESET have been
            // specified then we define those pins as GPIO outputs.
        case U8X8_MSG_GPIO_AND_DELAY_INIT: {
            uint64_t bitmask = 0;
            if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
                bitmask = bitmask | (1ull << u8g2_esp32_hal.dc);
            }
            if (u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED) {
                bitmask = bitmask | (1ull << u8g2_esp32_hal.reset);
            }
            if (u8g2_esp32_hal.cs != U8G2_ESP32_HAL_UNDEFINED) {
                bitmask = bitmask | (1ull << u8g2_esp32_hal.cs);
            }

            if (bitmask == 0) {
                break;
            }
            gpio_config_t gpioConfig;
            gpioConfig.pin_bit_mask = bitmask;
            gpioConfig.mode = GPIO_MODE_OUTPUT;
            gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
            gpioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
            gpioConfig.intr_type = GPIO_INTR_DISABLE;
            gpio_config(&gpioConfig);
            break;
        }

            // Set the GPIO reset pin to the value passed in through arg_int.
        case U8X8_MSG_GPIO_RESET:
            if (u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.reset, arg_int);
            }
            break;
            // Set the GPIO client select pin to the value passed in through arg_int.
        case U8X8_MSG_GPIO_CS:
            if (u8g2_esp32_hal.cs != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.cs, arg_int);
            }
            break;
            // Set the Software I²C pin to the value passed in through arg_int.
        case U8X8_MSG_GPIO_I2C_CLOCK:
            if (u8g2_esp32_hal.scl != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.scl, arg_int);
            }
            break;
            // Set the Software I²C pin to the value passed in through arg_int.
        case U8X8_MSG_GPIO_I2C_DATA:
            if (u8g2_esp32_hal.sda != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.sda, arg_int);
            }
            break;

            // Delay for the number of milliseconds passed in through arg_int.
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(arg_int / portTICK_PERIOD_MS);
            break;
    }
    return 0;
}  // u8g2_esp32_gpio_and_delay_cb
