/**
 ******************************************************************************
 *  file           : led.h
 *  brief          : LED Control
 ******************************************************************************
 */

#ifndef COMPONENTS_LED_LED_H_
#define COMPONENTS_LED_LED_H_

#include <esp_err.h>  // NOLINT
#include <esp_log.h>  // NOLINT
#include <string.h>   // NOLINT

#include "driver/gpio.h"        // NOLINT
#include "driver/rmt_tx.h"      // NOLINT
#include "freertos/FreeRTOS.h"  // NOLINT
#include "freertos/task.h"      // NOLINT

class LEDObject {
 public:
    /**
     * @brief Construct a new LED Object object
     *
     * @param number Number of LEDs
     * @param pin Output Pin
     */
    LEDObject(const uint16_t number, const gpio_num_t pin);
    virtual ~LEDObject();

    /**
     * @brief Create LED Object
     */
    esp_err_t Create();

    /**
     * @brief Set the color of a LED
     *
     * @param index Index 0..number
     * @param r RED Value
     * @param g GREEN Value
     * @param b BLUE Value
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t SetLED(const uint16_t index, const uint8_t red, const uint8_t green, const uint8_t blue);

    /**
     * @brief Helper to convert HSV to RGB
     * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
     * @param h Hue [In]
     * @param s Saturation [In]
     * @param v Vue [In]
     * @param r Red [Out]
     * @param g Green [Out]
     * @param b Blue [Out]
     */
    void hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b);

 private:
    uint16_t NumLeds;   // Number of LEDs
    gpio_num_t OutPin;  // Output Pin
    uint8_t *pLedData;  // The LED Data
    rmt_channel_handle_t led_chan = NULL;
    rmt_encoder_handle_t led_encoder = {};
};

#endif  // COMPONENTS_LED_LED_H_
