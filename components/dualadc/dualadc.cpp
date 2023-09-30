/**
 ******************************************************************************
 *  file           : dualadc.cpp
 *  brief          : Analog inputs
 ******************************************************************************
 */

/****************************** Includes  */
#include "dualadc.h"

/****************************** Configuration */
#define BITWITH ADC_BITWIDTH_12
#define ATTEN ADC_ATTEN_DB_11
/****************************** Statics */
static const char* TAG = "ADC";

adc_oneshot_unit_handle_t adc_handle = NULL;
adc_channel_t AdcChan0 = ADC_CHANNEL_0;
adc_channel_t AdcChan1 = ADC_CHANNEL_0;
adc_unit_t UnitId = ADC_UNIT_1;
QueueHandle_t xADCQueue1 = NULL;
QueueHandle_t xADCQueue2 = NULL;
TaskHandle_t xHandle = NULL;
/****************************** Functions */

ADCObject::ADCObject(const int Ana1, const int Ana2)
{
    anaIn1 = Ana1;
    anaIn2 = Ana2;
    AdcChan0 = ADC_CHANNEL_0;
    AdcChan1 = ADC_CHANNEL_0;
    UnitId = ADC_UNIT_1;
    adc_handle = NULL;
}

ADCObject::~ADCObject() {}

// The Worker Task
static void vWorker(void* pvParameter)
{
    while (1)
    {
        int adc_raw[2];
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, AdcChan0, &adc_raw[0]));
        xQueueOverwrite(xADCQueue1, &adc_raw[0]);

        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, AdcChan1, &adc_raw[0]));
        xQueueOverwrite(xADCQueue2, &adc_raw[0]);

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

esp_err_t ADCObject::Create(const UBaseType_t TaskPrio, QueueHandle_t& xQueue1, QueueHandle_t& xQueue2)
{
    adc_unit_t UnitId0 = ADC_UNIT_1;
    adc_unit_t UnitId1 = ADC_UNIT_1;

    // Get channels and units for IOs
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(anaIn1, &UnitId0, &AdcChan0));
    ESP_LOGI(TAG, "IO = %d Unit=%d, Chan=%d", anaIn1, (int)UnitId0, (int)AdcChan0);

    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(anaIn2, &UnitId1, &AdcChan1));
    ESP_LOGI(TAG, "IO = %d Unit=%d, Chan=%d", anaIn2, (int)UnitId1, (int)AdcChan1);

    if (UnitId0 != UnitId1)
    {
        ESP_LOGE(TAG, "ADC units must match!");
        return ESP_FAIL;
    }
    UnitId = UnitId0;

    // ADC Unit
    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = UnitId;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // ADC Configuration
    adc_oneshot_chan_cfg_t config = {};
    config.bitwidth = BITWITH;
    config.atten = ATTEN;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, AdcChan0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, AdcChan1, &config));

    // Queue Creation
    xQueue1 = xQueueCreate(1, sizeof(int));
    xADCQueue1 = xQueue1;
    xQueue2 = xQueueCreate(1, sizeof(int));
    xADCQueue2 = xQueue2;

    xTaskCreate(vWorker, "ADCWorker", 2048, this, TaskPrio, &xHandle);
    if (NULL == xHandle) return ESP_FAIL;

    return ESP_OK;
}
