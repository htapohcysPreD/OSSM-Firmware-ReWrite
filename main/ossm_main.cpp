/**
 ******************************************************************************
 *  file           : ossm_main.cpp
 *  brief          : OSSM Main
 ******************************************************************************
 */

/****************************** Includes  */
#include <inttypes.h>  // NOLINT
#include <stdio.h>     // NOLINT
#include <string.h>    // NOLINT

#include "../components/dualadc/dualadc.h"
#include "../components/encoder/encoder.h"
#include "../components/glcd/glcd.h"
#include "../components/led/led.h"
#include "../components/motionctrl/motionctrl.h"
#include "../components/storage/storage.h"
#include "./ossm_config.h"
#include "esp_chip_info.h"      // NOLINT
#include "esp_flash.h"          // NOLINT
#include "esp_log.h"            // NOLINT
#include "esp_ota_ops.h"        // NOLINT
#include "esp_partition.h"      // NOLINT
#include "freertos/FreeRTOS.h"  // NOLINT
#include "freertos/task.h"      // NOLINT

/****************************** Configuration */

#define SUPERVIS_OVERCUR_CNT 5  // Number of detected Overcurrents to trigger Emergency

/****************************** Statics */
static const char* TAG = "MAIN";

LEDObject TheLEDs(LED_COUNT, LED_PIN);                                              // The LED(s)
OSSMLCD TheGLCD(REMOTE_LCD_SDA, REMOTE_LCD_CLK, REMOTE_LCD_ADDRESS);                // The GLCD on the wired Remote
Encoder TheEncoder(ENCODER_SW_PIN, ENCODER_A_PIN, ENCODER_B_PIN);                   // The Encoder on the wired Remote
ADCObject TheADCs(ANA2_PIN, MOTOR_CURRENT_PIN);                                     // ADC Inputs
MotionCtrl TheMCtrl(MOTOR_ENA_PIN, MOTOR_ALARM_PIN, MOTOR_STP_PIN, MOTOR_DIR_PIN);  // The Motion Controller

QueueHandle_t xAdcQueuePoti = NULL;     // Poti ADC values
QueueHandle_t xAdcQueueMCur = NULL;     // Motor Current Values
QueueHandle_t xEncQueueButton = NULL;   // Push Button
QueueHandle_t xEncQueueEncoder = NULL;  // Encoder Value

TaskHandle_t xSupervisorHandle = NULL;

/****************************** Functions */

/**
 * @brief Dev Function: Print Task Statistics
 *
 */
static void print_task_stats() {
    static int64_t lastrun = 0;
    const int64_t now = esp_timer_get_time() / 1000;

    if ((now - lastrun) > 60000) {
        ESP_LOGI(TAG, "Task statistics:");

        TaskStatus_t* pxTaskStatusArray;
        UBaseType_t uxArraySize, x;
        uint32_t ulTotalRunTime, ulStatsAsPercentage;

        // Take a snapshot of the number of tasks in case it changes while this
        // function is executing.
        uxArraySize = uxTaskGetNumberOfTasks();

        // Allocate a TaskStatus_t structure for each task.  An array could be
        // allocated statically at compile time.
        pxTaskStatusArray = reinterpret_cast<TaskStatus_t*>(pvPortMalloc(uxArraySize * sizeof(TaskStatus_t)));

        if (pxTaskStatusArray != NULL) {
            // Generate raw status information about each task.
            uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);

            // For percentage calculations.
            ulTotalRunTime /= 100UL;

            // Avoid divide by zero errors.
            if (ulTotalRunTime > 0) {
                // For each populated position in the pxTaskStatusArray array,
                // format the raw data as human readable ASCII data
                ESP_LOGI(TAG, "            Name\t   RunTime\t Prc\tPrio\tWatermark");
                for (x = 0; x < uxArraySize; x++) {
                    // What percentage of the total run time has the task used?
                    // This will always be rounded down to the nearest integer.
                    // ulTotalRunTimeDiv100 has already been divided by 100.
                    ulStatsAsPercentage = pxTaskStatusArray[x].ulRunTimeCounter / ulTotalRunTime;

                    ESP_LOGI(TAG, "%16s\t%10lu\t%3lu%%\t%3d\t%3lu", pxTaskStatusArray[x].pcTaskName,
                             pxTaskStatusArray[x].ulRunTimeCounter, ulStatsAsPercentage,
                             pxTaskStatusArray[x].uxCurrentPriority, pxTaskStatusArray[x].usStackHighWaterMark);
                }
            }

            // The array is no longer needed, free the memory it consumes.
            vPortFree(pxTaskStatusArray);
        }

        lastrun = now;
    }
}

// The Supervisor Task
static void vSupervisor(void* pvParameter) {
    uint8_t OverCurrCnt = 0;
    ESP_LOGI(TAG, "System Supervisor started");
    QueueHandle_t xQueue = (QueueHandle_t)pvParameter;

    while (1) {
        int adcval;

        // Check Motor Current
        if (xQueuePeek(xQueue, &adcval, 0) == pdTRUE) {
            if (adcval >= default_EmergencyAdc) {
                OverCurrCnt++;

                if (OverCurrCnt > SUPERVIS_OVERCUR_CNT) {
                    ESP_LOGW(TAG, "System Supervisor gets ADC = %d", adcval);
                    TheMCtrl.RequestModeSet(MotionCtrl::Modes::EMERGENCYSTOP);
                }
            } else {
                OverCurrCnt = 0;
            }
        }

        // TODO Check Emergency Off Switch

        vTaskDelay(25 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Entry and main
 */
extern "C" void app_main() {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    uint32_t flash_size;

    ESP_LOGI(TAG, "------------------------------------------ System Information");
    ESP_LOGI(TAG, "%s chip with %d CPU cores, WiFi%s%s ", CONFIG_IDF_TARGET, chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    ESP_LOGI(TAG, "Silicon rev %d ", chip_info.revision);
    ESP_LOGI(TAG, "Reset reason: %d", esp_reset_reason());
    ESP_LOGI(TAG, "------------------------------------------ Memory");
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        ESP_LOGE(TAG, "Get flash size failed");
        return;
    }
    ESP_LOGI(TAG, "%lu MB %s flash", flash_size / (1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    ESP_LOGI(TAG, "Heap: %lu", esp_get_free_heap_size());

    ESP_LOGI(TAG, "------------------------------------------ Partitions");
    const esp_partition_t* part_info;
    esp_ota_img_states_t ota_state;
    part_info = esp_ota_get_boot_partition();
    esp_ota_get_state_partition(part_info, &ota_state);
    if (NULL != part_info) {
        ESP_LOGI(TAG, "Boot partition:");
        ESP_LOGI(TAG, "  Address=0x%lx, size=0x%lx", part_info->address, part_info->size);
        ESP_LOGI(TAG, "  Label = %s", part_info->label);
        ESP_LOGI(TAG, "  State = %d", ota_state);
    }
    ESP_LOGI(TAG, "------------------------------------------");

    // Setup the modules
    STOR_Init();  // Non-Volatile Storage

    // TODO Read Motordata from NVS
    MotionCtrl::sMotorData MotorData = {};
    MotorData.BeltPitch = default_beltPitch;
    MotorData.PulleyTeeth = default_pulleyCnt;
    MotorData.StepsPerRev = default_motorStepPerRev;

    TheLEDs.Create();                                            // LED
    TheLEDs.SetLED(0, 0xFF, 0xA0, 0);                            // Orange on boot/init
    TheGLCD.Create();                                            // UI/GLCD
    TheGLCD.DrawScreen(OSSMLCD::Screens::BOOTING);               // Initial Screen
    TheEncoder.Create(xEncQueueEncoder, xEncQueueButton);        // Encoder
    TheADCs.Create(TASKPRIO_ADC, xAdcQueuePoti, xAdcQueueMCur);  // Analog inputs
    TheMCtrl.Create(TASKPRIO_MOTION, xAdcQueueMCur, MotorData);  // Motion control
    // WiFi_Init();                  // WiFi (if configured)

    if (NULL == xEncQueueEncoder) ESP_LOGE(TAG, "Queue Encoder is NULL!");
    if (NULL == xEncQueueButton) ESP_LOGE(TAG, "Queue Button is NULL!");
    if (NULL == xAdcQueuePoti) ESP_LOGE(TAG, "Queue Poti is NULL!");
    if (NULL == xAdcQueueMCur) ESP_LOGE(TAG, "Queue MCurrent is NULL!");

    xTaskCreate(vSupervisor, "Supervisor", 2048, xAdcQueueMCur, (configMAX_PRIORITIES - 1), &xSupervisorHandle);

    // --- Main Loop
    ESP_LOGW(TAG, "Main IDLE started on core id %d", esp_cpu_get_core_id());
    while (1) {
        print_task_stats();
#if 0  // LED Testing
        static uint8_t rval = 0;
        static bool goUp = true;
        TheLEDs.SetLED(0, rval, 0, 0);
        (goUp) ? ((rval == 0xFF) ? (goUp = false) : (rval++)) : ((rval == 0) ? (goUp = true) : (rval--));
#endif
#if 0  // ADC Testing
        int ADCVal;
        if (xQueueReceive(xAdcQueuePoti, &ADCVal, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Poti    = %x / %d", ADCVal, ADCVal);
        }
        if (xQueueReceive(xAdcQueueMCur, &ADCVal, 0) == pdTRUE) {
            ESP_LOGI(TAG, "MotorCur= %x / %d", ADCVal, ADCVal);
        }
#endif
#if 0  // Encoder Testing
        if (NULL != xEncQueueButton) {
            int ButtonTime;
            if (xQueueReceive(xEncQueueButton, &ButtonTime, (TickType_t)0) == pdPASS) {
                if (ButtonTime == 0) {
                    ESP_LOGI(TAG, "Button pressed");
                } else {
                    ESP_LOGI(TAG, "Button released after %d", ButtonTime);
                }
            }
        }
        if (NULL != xEncQueueEncoder) {
            int EncoderVal;
            if (xQueueReceive(xEncQueueEncoder, &EncoderVal, (TickType_t)0) == pdPASS) {
                ESP_LOGI(TAG, "Encoder Value changed to %d", EncoderVal);
            }
        }
#endif

        switch (TheMCtrl.CurrentModeGet()) {
            case MotionCtrl::Modes::NONE:  // Stay in NONE for 2sec, then switch to Homing
                if ((esp_timer_get_time() / 1000) > 2000) {
                    TheMCtrl.RequestModeSet(MotionCtrl::Modes::HOMING);
                }
                break;
            case MotionCtrl::Modes::HOMING:  // Wait until homing is complete
                TheGLCD.DrawScreen(OSSMLCD::Screens::HOMING);
                break;
            case MotionCtrl::Modes::READY:
                TheLEDs.SetLED(0, 0, 0xFF, 0);
                TheGLCD.DrawScreen(OSSMLCD::Screens::READY);
                break;
            case MotionCtrl::Modes::STROKE:
                TheGLCD.DrawScreen(OSSMLCD::Screens::STROKE);
                break;
            case MotionCtrl::Modes::EMERGENCYSTOP:
                TheGLCD.DrawScreen(OSSMLCD::Screens::EMERGENCY);
                break;
        }  // switch (TheMController.CurrentModeGet())
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    esp_restart();  // unreachable
}
