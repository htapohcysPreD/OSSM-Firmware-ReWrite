/**
 ******************************************************************************
 *  file           : dualadc.h
 *  brief          : Analog inputs and filtering
 ******************************************************************************
 */

#ifndef COMPONENTS_DUALADC_DUALADC_H_
#define COMPONENTS_DUALADC_DUALADC_H_

#include "esp_adc/adc_cali.h"         // NOLINT
#include "esp_adc/adc_cali_scheme.h"  // NOLINT
#include "esp_adc/adc_oneshot.h"      // NOLINT
#include "esp_err.h"                  // NOLINT
#include "esp_log.h"                  // NOLINT
#include "freertos/FreeRTOS.h"        // NOLINT
#include "freertos/queue.h"           // NOLINT
#include "freertos/task.h"            // NOLINT

class ADCObject {
 public:
    ADCObject(const int Ana1, const int Ana2);
    virtual ~ADCObject();

    /**
     * @brief Create a ADC Object for a pin
     *
     * @param anaIn Input Pin Number
     * @return esp_err_t
     */
    esp_err_t Create(const UBaseType_t TaskPrio, QueueHandle_t& xQueue1,  // NOLINT(runtime/references)
                     QueueHandle_t& xQueue2);                             // NOLINT(runtime/references)

    /**
     * @brief Get the percentage of the Input
     */
    uint16_t GetPercentage(const uint8_t Channel, const uint8_t Samples);

    /**
     * @brief Get the averaged Voltage in mV
     */
    uint16_t GetVoltage(const uint8_t Channel, const uint8_t Samples);

 private:
    int anaIn1;
    int anaIn2;
};

#endif  // COMPONENTS_DUALADC_DUALADC_H_
