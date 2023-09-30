/**
 ******************************************************************************
 *  file           : encoder.cpp
 *  brief          : The encoder
 ******************************************************************************
 */

/****************************** Includes  */
#include "encoder.h"

/****************************** Configuration */
#define ENC_HIGH_LIMIT 2
#define ENC_LOW_LIMIT -2

/****************************** Statics */
static const char *TAG = "ENC";
static uint16_t EncoderValue;
QueueHandle_t xButtonQ = NULL;
QueueHandle_t xEncoderQ = NULL;

/****************************** Functions */

/**
 * @brief ISR handler for pressed encoder button
 */
static void IRAM_ATTR button_isr_handler(void *arg)
{
    static int presstime = 0;
    int payload = 0;

    if (gpio_get_level(ENCODER_SW_PIN) == 1) // Falling edge
    {
        presstime = esp_timer_get_time();
    }
    else // Rising edge
    {
        payload = (esp_timer_get_time() - presstime) / 1000;
        presstime = 0;
    }
    // On press: Send 0
    // On release: Send pressed time
    if (NULL != xButtonQ) xQueueOverwrite(xButtonQ, &payload);
}
/**
 * @brief ISR handler when reaching watchpoints
 */
static bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    if ((edata->watch_point_value < 0) && (EncoderValue > 0)) EncoderValue--;
    if ((edata->watch_point_value > 0) && (EncoderValue < 100)) EncoderValue++;
    if (NULL != xEncoderQ) xQueueOverwrite(xEncoderQ, &EncoderValue);
    return false;
}

Encoder::Encoder(const gpio_num_t button, const gpio_num_t pinA, const gpio_num_t pinB)
{
    pinButton = button;
    pinEncA = pinA;
    pinEncB = pinB;
    pcnt_unit = NULL;
    xButtonQ = NULL;
    xEncoderQ = NULL;
}
Encoder::~Encoder() {}
int Encoder::GetCount(void)
{
    return EncoderValue;
}
esp_err_t Encoder::Create(QueueHandle_t &xQueueE, QueueHandle_t &xQueueB)
{
    // Set up the encoders button
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << pinButton);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_EDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(pinButton, button_isr_handler, NULL));

    // The PCNT unit
    pcnt_unit_config_t unit_config = {};
    unit_config.high_limit = ENC_HIGH_LIMIT;
    unit_config.low_limit = ENC_LOW_LIMIT;
    unit_config.flags.accum_count = 1;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    // Glitch Filter
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    // PCNT Channels
    pcnt_chan_config_t chan_a_config = {};
    chan_a_config.edge_gpio_num = pinEncB;
    chan_a_config.level_gpio_num = pinEncA;

    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {};
    chan_b_config.edge_gpio_num = pinEncA;
    chan_b_config.level_gpio_num = pinEncB;

    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    // Edge and level actions
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Reconfigure inputs to Pulldowns
    ESP_ERROR_CHECK(gpio_set_pull_mode(pinEncA, GPIO_PULLDOWN_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode(pinEncB, GPIO_PULLDOWN_ONLY));

    // Watchpoints
    int watch_points[] = {ENC_LOW_LIMIT, 0, ENC_HIGH_LIMIT};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++)
    {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
    }
    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, NULL));

    // Start PCNT
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    // Queue Creation
    xQueueE = xQueueCreate(1, sizeof(uint16_t));
    xEncoderQ = xQueueE;
    xQueueB = xQueueCreate(1, sizeof(int));
    xButtonQ = xQueueB;

    ESP_LOGI(TAG, "Object created");

    return ESP_OK;
}
