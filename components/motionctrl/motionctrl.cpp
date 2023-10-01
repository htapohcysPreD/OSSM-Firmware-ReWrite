/**
 ******************************************************************************
 *  file           : motionctrl.cpp
 *  brief          : The Motion Controller
 ******************************************************************************
 */

/****************************** Includes  */
#include "motionctrl.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "sdkconfig.h"

/****************************** Configuration */

/****************************** Statics */
static const char* TAG = "MOTION";

QueueHandle_t xADCQueue;            // The Queue to receive ADC Values from
TaskHandle_t xWorkerHandle = NULL;  // The Worker Task Handle

/****************************** Functions */

MotionCtrl::MotionCtrl(const gpio_num_t EnaPin, const gpio_num_t AlmPin, const gpio_num_t STPPin,
                       const gpio_num_t DIRPin) {
    xADCQueue = NULL;
    xWorkerHandle = NULL;
    RunMode = Modes::NONE;
    ReqMode = Modes::NONE;
    MotorEnaPin = EnaPin;
    MotorAlmPin = AlmPin;
    MotorStepPin = STPPin;
    MotorDirPin = DIRPin;
}

MotionCtrl::~MotionCtrl() {
    if (xWorkerHandle != NULL) {
        vTaskDelete(xWorkerHandle);
    }
}

// Alarm Interrupt Handler
static void IRAM_ATTR alarm_isr_handler(void* arg) {
    MotionCtrl* pMotionCtrl = reinterpret_cast<MotionCtrl*>(arg);
    pMotionCtrl->RequestModeSet(MotionCtrl::Modes::EMERGENCYSTOP);
}

// Enable Motor On/Off
static inline void MotorOnOff(MotionCtrl* pCtrl, const bool On) {
    gpio_set_level(pCtrl->getEnaPin(), (On) ? 0 : 1);  // Low Active!
}

// Generate a specific number of steps
static uint16_t makeSteps(gpio_num_t pin, uint16_t steps, bool GoFast = false) {
    uint16_t delay = 1250;
    if (GoFast) delay = delay / 2;

    for (uint16_t i = 0; i < steps; i++) {
        gpio_set_level(pin, 1);
        usleep(delay);
        gpio_set_level(pin, 0);
        usleep(delay);
    }
    return steps;
}

// Run until endpoint is reached
static uint32_t gotoEndpoint(MotionCtrl* pCtrl, uint32_t MaxSteps, uint32_t cOffset, uint32_t cLimit) {
    const uint32_t StepsPerLoop = 10;
    uint32_t StepsDone = 0;
    const uint32_t stepsPerMM =
        pCtrl->getMotorData().StepsPerRev / (pCtrl->getMotorData().PulleyTeeth * pCtrl->getMotorData().BeltPitch);

    for (uint32_t Step = 0; Step < MaxSteps; Step += StepsPerLoop) {
        int adcval;
        StepsDone += makeSteps(pCtrl->getStepPin(), StepsPerLoop);

        if (xQueueReceive(xADCQueue, &adcval, portMAX_DELAY) == pdTRUE) {
            uint32_t current = abs(static_cast<int>(adcval - cOffset));
            if (current >= cLimit) {
                ESP_LOGI(TAG, "Endpoint reached, Steps=%ld", StepsDone);
                return StepsDone;
            }
        }
    }
    ESP_LOGW(TAG, "Endpoint not detected after %ld steps!", StepsDone);
    return StepsDone;
}

/**
 * @brief Execute sensorless homing. Blocking
 *
 * @return float Stroke in mm
 */
static float do_Homing(MotionCtrl* pCtrl) {
    const uint8_t numAverage = 100;     // Number of Samples for Averaging
    const uint16_t maxLength = 300;     // Maximum Length in mm
    const uint32_t currentLimit = 200;  // Delta of Current Offset to detect limit
    const uint32_t stepsPerMM =
        pCtrl->getMotorData().StepsPerRev / (pCtrl->getMotorData().PulleyTeeth * pCtrl->getMotorData().BeltPitch);
    const uint32_t MaxSteps = maxLength * stepsPerMM;
    const uint32_t EndpointDistance = 5 * stepsPerMM;  // 5mm distance in Steps // TODO Make changeable
    uint32_t strokeSteps = 0;
    float strokeDistance = 0.0;

    uint32_t CurrentOffset = 0;

    ESP_LOGI(TAG, "Homing...");

    MotorOnOff(pCtrl, 0);  // Turn Motor OFF
    vTaskDelay(500 / portTICK_PERIOD_MS);
    MotorOnOff(pCtrl, 1);  // Turn Motor ON
    vTaskDelay(100 / portTICK_PERIOD_MS);

    for (uint8_t i = 0; i < numAverage; i++) {
        uint16_t adcval;
        if (xQueueReceive(xADCQueue, &adcval, portMAX_DELAY) == pdTRUE) {
            CurrentOffset += adcval;
        }
    }
    CurrentOffset = CurrentOffset / numAverage;
    ESP_LOGI(TAG, "CurrentOffset Avg =  %ld", CurrentOffset);

    // Find forward Limit
    ESP_LOGI(TAG, "Forward Limit...");
    gpio_set_level(pCtrl->getDirPin(), 0);
    gotoEndpoint(pCtrl, MaxSteps, CurrentOffset, currentLimit);

    // Move back 5mm from endpoint
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(pCtrl->getDirPin(), 1);
    makeSteps(pCtrl->getStepPin(), EndpointDistance);

    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Find reverse Limit
    ESP_LOGI(TAG, "Reverse Limit...");
    gpio_set_level(pCtrl->getDirPin(), 1);
    strokeSteps = gotoEndpoint(pCtrl, MaxSteps, CurrentOffset, currentLimit);

    // Move back 5mm from endpoint
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(pCtrl->getDirPin(), 0);
    makeSteps(pCtrl->getStepPin(), EndpointDistance);
    strokeSteps = strokeSteps - EndpointDistance;

    strokeDistance = (static_cast<float>(strokeSteps) / stepsPerMM);
    ESP_LOGI(TAG, "Homing done, distance is %ld steps or %.2f mm", strokeSteps, strokeDistance);

    return strokeDistance;
}

// The Worker Task
static void vWorker(void* pvParameter) {
    MotionCtrl::Modes currentMode = MotionCtrl::Modes::NONE;
    MotionCtrl* pCtrl = reinterpret_cast<MotionCtrl*>(pvParameter);
    float detectedStroke = 0.0;  // Stroke in mm

    while (1) {
        // Handle State Change Request
        if (pCtrl->RequestModeGet() != MotionCtrl::Modes::NONE) {
            ESP_LOGI(TAG, "Requested Mode: %d", (int)pCtrl->RequestModeGet());

            switch (pCtrl->RequestModeGet()) {
                case MotionCtrl::Modes::NONE:
                    // Nothing to do
                    break;
                case MotionCtrl::Modes::HOMING:
                    currentMode = MotionCtrl::Modes::HOMING;  // Allowed from every state
                    break;
                case MotionCtrl::Modes::READY:
                    // Not allowed manually
                    break;
                case MotionCtrl::Modes::STROKE:  // Only allowed from READY
                    if (currentMode == MotionCtrl::Modes::READY) {
                        currentMode = MotionCtrl::Modes::STROKE;
                    }
                    break;
                case MotionCtrl::Modes::EMERGENCYSTOP:
                    currentMode = MotionCtrl::Modes::EMERGENCYSTOP;
                    break;
            }
            ESP_LOGI(TAG, "New Mode: %d", (int)currentMode);
        }
        pCtrl->RequestModeSet(MotionCtrl::Modes::NONE);  // Clear request
        pCtrl->CurrentModeSet(currentMode);

        // Do the Stuff
        switch (currentMode) {
            case MotionCtrl::Modes::NONE:
                MotorOnOff(pCtrl, 0);  // Turn Motor OFF
                break;
            case MotionCtrl::Modes::HOMING:
                detectedStroke = do_Homing(pCtrl);
                if (detectedStroke > 0) {
                    currentMode = MotionCtrl::Modes::READY;
                }
                break;
            case MotionCtrl::Modes::READY:
                MotorOnOff(pCtrl, 0);  // Turn Motor OFF // TODO REMOVE ME
                break;
            case MotionCtrl::Modes::STROKE:
                break;
            case MotionCtrl::Modes::EMERGENCYSTOP:
                MotorOnOff(pCtrl, 0);  // Turn Motor OFF
                break;
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }  // while 1
}

void MotionCtrl::RequestModeSet(Modes Mode) {
    if (Mode == MotionCtrl::Modes::EMERGENCYSTOP) {
        ESP_LOGI(TAG, "Emergency Stop!");
        gpio_set_level(MotorEnaPin, 1);
        RunMode = MotionCtrl::Modes::EMERGENCYSTOP;
        ReqMode = MotionCtrl::Modes::EMERGENCYSTOP;
        vTaskSuspend(xWorkerHandle);
    }
    ReqMode = Mode;
}

esp_err_t MotionCtrl::Create(const UBaseType_t TaskPrio, QueueHandle_t xQueue, sMotorData MData) {
    xADCQueue = xQueue;
    MotorData = MData;

    // Motor Pin Enable, Step, Dir
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << MotorEnaPin) | (1ULL << MotorStepPin) | (1ULL << MotorDirPin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Motor AlarmPin
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << MotorAlmPin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    gpio_isr_handler_add(MotorAlmPin, alarm_isr_handler, reinterpret_cast<void*>(this));

    // Worker Task
    // xTaskCreate(vWorker, "MotionCtrl", 2048, this, TaskPrio, &xWorkerHandle);
    xTaskCreatePinnedToCore(vWorker, "MotionCtrl", 2048, this, TaskPrio, &xWorkerHandle, 1);
    if (NULL == xWorkerHandle) return ESP_FAIL;

    return ESP_OK;
}
