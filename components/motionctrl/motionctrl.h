/**
 ******************************************************************************
 *  file           : motionctrl.h
 *  brief          : The Motion Controller
 ******************************************************************************
 */

#ifndef COMPONENTS_MOTIONCTRL_MOTIONCTRL_H_
#define COMPONENTS_MOTIONCTRL_MOTIONCTRL_H_

#include <esp_err.h> // NOLINT

#include "driver/gpio.h"       // NOLINT
#include "esp_log.h"           // NOLINT
#include "freertos/FreeRTOS.h" // NOLINT
#include "freertos/queue.h"    // NOLINT
#include "freertos/task.h"     // NOLINT
class MotionCtrl
{
   public:
    // Possible states of the motion controller
    enum class Modes
    {
        NONE,         // Starting position
        HOMING,       // Homing in progress
        READY,        // Homed and ready
        STROKE,       // Stroke mode
        EMERGENCYSTOP // Full Stop / Abort
    };

    struct sMotorData
    {
        uint16_t StepsPerRev; // Steps of the Motor per revolution
        uint8_t PulleyTeeth;  // Number of Teeth of the Pulley
        float BeltPitch;      // Pitch of the belt in mm
    };

    /**
     * @brief Construct a new MotionCtrl
     *
     * @param EnaPin Motor Enable
     * @param AlmPin Motor Alarm
     * @param STPPin Step/Pulse
     * @param DIRPin Direction
     */
    MotionCtrl(const gpio_num_t EnaPin, const gpio_num_t AlmPin, const gpio_num_t STPPin, const gpio_num_t DIRPin);
    virtual ~MotionCtrl();

    /**
     * @brief Create the Motion Controller
     *
     * @param TaskPrio Task Priority
     * @param EnaPin The Motor Enable Pin (Out)
     * @param AlmPin The Motor Alarm Pin (In)
     * @return esp_err_t
     */
    esp_err_t Create(const UBaseType_t TaskPrio, QueueHandle_t xQueue, sMotorData MData);

    /**
     * @brief Get the currently running Mode
     */
    Modes CurrentModeGet()
    {
        return RunMode;
    }
    /**
     * @brief Set the currently running Mode
     */
    void CurrentModeSet(Modes Mode)
    {
        RunMode = Mode;
    }
    /**
     * @brief Request a new Mode
     */
    void RequestModeSet(Modes Mode);
    /**
     * @brief Get the requested Mode
     */
    Modes RequestModeGet()
    {
        return ReqMode;
    };
    /**
     * @brief Return the Motor Enable Pin
     */
    gpio_num_t getEnaPin()
    {
        return MotorEnaPin;
    }
    /**
     * @brief Return the Motor Step Pin
     */
    gpio_num_t getStepPin()
    {
        return MotorStepPin;
    }
    /**
     * @brief Return the Motor Direction Pin
     */
    gpio_num_t getDirPin()
    {
        return MotorDirPin;
    }
    sMotorData getMotorData()
    {
        return MotorData;
    };

   private:
    gpio_num_t MotorEnaPin;      // Configured Motor Ena Pin
    gpio_num_t MotorAlmPin;      // Configured Motor Alarm Pin
    gpio_num_t MotorStepPin;     // Configured Motor Step Pin
    gpio_num_t MotorDirPin;      // Configured Motor Dir Pin
    Modes RunMode = Modes::NONE; // Current Run Mode
    Modes ReqMode = Modes::NONE; // Requested Run Mode
    sMotorData MotorData;        // The motors mechanical Data
};

#endif // COMPONENTS_MOTIONCTRL_MOTIONCTRL_H_
