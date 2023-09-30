/**
 ******************************************************************************
 *  file           : glcd.h
 *  brief          : The GLCD for the OSSM
 ******************************************************************************
 */

#ifndef COMPONENTS_GLCD_GLCD_H_
#define COMPONENTS_GLCD_GLCD_H_

#include <esp_err.h>  // NOLINT
#include <inttypes.h> // NOLINT

#include "../../main/ossm_config.h"
#include "driver/gpio.h" // NOLINT
#include "u8g2_esp32_hal.h"

class OSSMLCD
{
   public:
    enum class Screens
    {
        NONE,
        BOOTING,
        HOMING,
        READY,
        STROKE,
        REMCTRL,
        EMERGENCY
    };

    /**
     * @brief Construct a new OSSMLCD object
     *
     * @param sda GPIO
     * @param clk GPIO
     * @param address I2C Address
     */
    OSSMLCD(const gpio_num_t sda, const gpio_num_t clk, const uint8_t address);
    virtual ~OSSMLCD();

    /**
     * @brief Create and set up the GLCD
     * @return esp_err_t
     */
    esp_err_t Create();

    /**
     * @brief Fully draw a specific screen
     *
     * @param Screen
     */
    void DrawScreen(Screens Screen);

   private:
    u8g2_t u8g2;                                      // All the data for one display
    Screens ScreenCurrent = OSSMLCD::Screens::NONE;   // Current Screen
    Screens ScreenRequested = OSSMLCD::Screens::NONE; // Requested Screen

    gpio_num_t sdaPin;
    gpio_num_t clkPin;
    uint8_t lcdAddress;

    void DrawScreenNone();                                 // Draw 'NONE' Screen
    void DrawScreenBooting();                              // Draw 'BOOTING' Screen
    void DrawScreenHoming();                               // Draw 'HOMING' Screen
    void DrawScreenReady();                                // Draw 'READY' Screen
    void DrawScreenStroke();                               // Draw 'STROKE' Screen
    void DrawScreenEmergency();                            // Draw 'EMERGENCY' Screen
    void DrawScreenRemoteCtrl();                           // Draw 'EMERGENCY' Screen
    void PrintStrCenterAligned(int y, const char *string); // Print a string center aligned
};

#endif // COMPONENTS_GLCD_GLCD_H_
