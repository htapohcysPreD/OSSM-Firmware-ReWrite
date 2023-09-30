/**
 ******************************************************************************
 *  file           : ossm_config.h
 *  brief          : Global configuration: Settings, pinnings, etc
 ******************************************************************************
 */

#ifndef MAIN_OSSM_CONFIG_H_
#define MAIN_OSSM_CONFIG_H_

#define SW_VERSION "0.01"          // The SW Version
#define LED_PIN GPIO_NUM_25        // GPIO for the WS2813 RGB LED
#define LED_COUNT 1                // Number of LEDs
#define ANA2_PIN 34                // Wired Controller: Speed Poti
#define ENCODER_SW_PIN GPIO_NUM_35 // Wired Controller: Encoder Switch
#define ENCODER_A_PIN GPIO_NUM_18  // Wired Controller: Encoder A
#define ENCODER_B_PIN GPIO_NUM_5   // Wired Controller: Encoder B
#define MOTOR_CURRENT_PIN 36       // Motor current sensing
#define MOTOR_ENA_PIN GPIO_NUM_26  // Motor Enable
#define MOTOR_STP_PIN GPIO_NUM_14  // Motor Step
// #define MOTOR_PED_PIN GPIO_NUM_14       // Motor PED Pin
#define MOTOR_DIR_PIN GPIO_NUM_27       // Motor Direction
#define MOTOR_ALARM_PIN GPIO_NUM_13     // Motor Alarm
#define REMOTE_LCD_SDA GPIO_NUM_21      // GLCD SDA Pin
#define REMOTE_LCD_CLK GPIO_NUM_19      // GLCD CLK Pin
#define REMOTE_LCD_ADDRESS 0x3c         // GLCD Address
#define REMOTE_LCD_I2C_HOST 0           // GLCD I2C Host to use
#define REMOTE_LCD_CLOCKSPEED (1000000) // GLCD I2C Clock Speed (Max 1MHz)
#define REMOTE_LCD_X 128                // GLCD Width
#define REMOTE_LCD_Y 64                 // GLCD Height
#define NVS_NAMESPACE "NVSStorage"      // Namespace of the NVS
// #define STATS_DISPLAY (5 * 1000)        // Rate of printing stats on console
// #define STATS_NVSTORE (1 * 60 * 1000)   // Rate of writing stats to NV storage
#define SYSTEM_HOSTNAME "OSSM" // Hostname for WiFi

// System configuration
static const uint16_t default_EmergencyAdc = 4000;   // ADC Value for triggering emergency
static const uint16_t default_motorStepPerRev = 800; // Default steps per rev
static const uint8_t default_pulleyCnt = 20;         // Teeth Count of the pulley
static const float default_beltPitch = 2;            // Pitch of the belt in mm
// static const float default_strokeZeroOffsetmm = 6.0f;        // Distance from the endstops to avoid collission
// static const float default_commandDeadzonePercentage = 1.0f; // Deadzone for encoder ant pot
// static const float default_maxStrokeLengthMm = 75.f;         // Initial stroke length
// static const float default_maxSpeedMmPerSecond = 600.0f;     // Max. linear speed
// static const float default_accelerationScaling = 100.0f;     // Acceleration/Deceleration scaling

// Task priorities
#define TASKPRIO_GLCD (tskIDLE_PRIORITY + 1)   // GLCD Worker (Core 0)
#define TASKPRIO_MOTION (tskIDLE_PRIORITY + 2) // Motion Worker (Core 1)
#define TASKPRIO_ADC (tskIDLE_PRIORITY)        // WiFi Worker (Core 0)

#endif // MAIN_OSSM_CONFIG_H_
