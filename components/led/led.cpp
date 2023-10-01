/**
 ******************************************************************************
 *  file           : led.cpp
 *  brief          : LED Control
 ******************************************************************************
 */

/****************************** Includes  */
#include "led.h"

#include "led_strip_encoder.h"

/****************************** Configuration */
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000  // 10MHz resolution

/****************************** Statics */
static const char *TAG = "LED";

/****************************** Functions */

void LEDObject::hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b) {
    h %= 360;  // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
        case 0:
            *r = rgb_max;
            *g = rgb_min + rgb_adj;
            *b = rgb_min;
            break;
        case 1:
            *r = rgb_max - rgb_adj;
            *g = rgb_max;
            *b = rgb_min;
            break;
        case 2:
            *r = rgb_min;
            *g = rgb_max;
            *b = rgb_min + rgb_adj;
            break;
        case 3:
            *r = rgb_min;
            *g = rgb_max - rgb_adj;
            *b = rgb_max;
            break;
        case 4:
            *r = rgb_min + rgb_adj;
            *g = rgb_min;
            *b = rgb_max;
            break;
        default:
            *r = rgb_max;
            *g = rgb_min;
            *b = rgb_max - rgb_adj;
            break;
    }
}
esp_err_t LEDObject::SetLED(const uint16_t index, const uint8_t red, const uint8_t green, const uint8_t blue) {
    rmt_transmit_config_t tx_config = {};
    tx_config.loop_count = 0;  // no transfer loop

    if (index >= NumLeds) return ESP_ERR_INVALID_ARG;

    // WS2813 byte ordering seems to be GRB
    pLedData[index * 3 + 0] = green;
    pLedData[index * 3 + 1] = red;
    pLedData[index * 3 + 2] = blue;

    // Send values
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, pLedData, NumLeds * 3, &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

    return ESP_OK;
}
LEDObject::LEDObject(const uint16_t number, const gpio_num_t pin) {
    NumLeds = number;
    OutPin = pin;

    pLedData = reinterpret_cast<uint8_t *>(malloc(number * 3));
}
LEDObject::~LEDObject() {
    free(pLedData);
}
esp_err_t LEDObject::Create() {
    led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {};
    tx_chan_config.gpio_num = OutPin;
    tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;  // select source clock
    tx_chan_config.resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ;
    tx_chan_config.mem_block_symbols = 64;
    tx_chan_config.trans_queue_depth = 4;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    ESP_LOGI(TAG, "Object created for %d LEDs", NumLeds);

    return ESP_OK;
}
