
/**
 ******************************************************************************
 *  file           : encoder.h
 *  brief          : The encoder
 ******************************************************************************
 */

#ifndef COMPONENTS_ENCODER_ENCODER_H_
#define COMPONENTS_ENCODER_ENCODER_H_

#include <esp_err.h> // NOLINT
#include <stdint.h>  // NOLINT
#include <stdio.h>

#include "../../main/ossm_config.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h" // NOLINT
#include "freertos/queue.h"    // NOLINT
#include "freertos/task.h"
#include "sdkconfig.h"

class Encoder
{
   public:
    /**
     * @brief Construct a new Encoder
     *
     * @param button GPIO of the button
     * @param pinA GPIO A-Line
     * @param pinB GPIO B-Line
     */
    Encoder(const gpio_num_t button, const gpio_num_t pinA, const gpio_num_t pinB);
    virtual ~Encoder();

    /**
     * @brief Create and Start the Encoder
     *
     * @param xQueue1 Queue for the Encoder [Out]
     * @param xQueue2 Queue for the Button [Out]
     * @return esp_err_t
     */
    esp_err_t Create(QueueHandle_t& xQueueE, QueueHandle_t& xQueueB);

    /**
     * @brief Return the current counter value
     *
     * @return int
     */
    int GetCount(void);

    /**
     * @brief Get the Queue to the Button
     *
     * @return QueueHandle_t
     */
    QueueHandle_t GetButtonQueue();

   private:
    pcnt_unit_handle_t pcnt_unit = NULL;
    gpio_num_t pinButton;
    gpio_num_t pinEncA;
    gpio_num_t pinEncB;
};

#endif // COMPONENTS_ENCODER_ENCODER_H_
