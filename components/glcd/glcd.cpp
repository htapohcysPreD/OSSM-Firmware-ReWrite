/**
 ******************************************************************************
 *  file           : glcd.cpp
 *  brief          : The OSSMs GLCD
 ******************************************************************************
 */

/****************************** Includes  */
#include "glcd.h"

#include <esp_err.h>  // NOLINT
#include <esp_log.h>  // NOLINT
#include <freertos/FreeRTOS.h>
#include <stdio.h>

/****************************** Configuration */

/****************************** Statics */
static const char *TAG = "GLCD";

// The KM logo
static const uint8_t km_logo[] = {
    0xF8, 0xFF, 0xFF, 0xFF, 0x1F, 0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0x87, 0xFF, 0x3F, 0xF8, 0xFF, 0x87, 0xFF, 0x1F, 0xFC, 0xFF, 0x87, 0xFF, 0x0F, 0xFE, 0xFF, 0x87, 0xFF, 0x07,
    0xFF, 0xFF, 0x87, 0xFF, 0x83, 0xFF, 0xFF, 0x87, 0xFF, 0xC1, 0xFF, 0xFF, 0x87, 0xFF, 0xE0, 0xFF, 0xFF, 0x87, 0x7F,
    0x80, 0xFF, 0xE1, 0x87, 0x3F, 0x80, 0xFF, 0xE1, 0x87, 0x1F, 0x84, 0xFF, 0xE1, 0x87, 0x1F, 0x06, 0xFF, 0xE0, 0x87,
    0x0F, 0x06, 0xFF, 0xE4, 0x87, 0x87, 0x07, 0xFE, 0xE4, 0x87, 0x83, 0x6F, 0x7E, 0xE4, 0x87, 0xC1, 0x67, 0x7E, 0xE6,
    0x07, 0xE0, 0xC7, 0x3C, 0xE6, 0x07, 0xF0, 0xE7, 0x3C, 0xE7, 0x87, 0xE0, 0xC7, 0x38, 0xE7, 0x87, 0xC1, 0xCF, 0x99,
    0xE7, 0x87, 0x83, 0xE7, 0x91, 0xE7, 0x87, 0x07, 0xC7, 0xC3, 0xE7, 0x87, 0x0F, 0xEE, 0xC3, 0xE7, 0x87, 0x1F, 0xC4,
    0xC7, 0xE7, 0x87, 0x1F, 0xE0, 0xE7, 0xE7, 0x87, 0x7F, 0xE0, 0xE7, 0xE7, 0x87, 0xFF, 0xE0, 0xFF, 0xFF, 0x87, 0xFF,
    0xC1, 0xFF, 0xFF, 0x87, 0xFF, 0x81, 0xFF, 0xFF, 0x87, 0xFF, 0x07, 0xFF, 0xFF, 0x87, 0xFF, 0x07, 0xFE, 0xFF, 0x87,
    0xFF, 0x1F, 0xFC, 0xFF, 0x87, 0xFF, 0x1F, 0xF8, 0xFF, 0x87, 0xFF, 0x7F, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F,
    0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0xF8, 0xFF, 0xFF, 0xFF, 0x3F,
};

/****************************** Functions */
OSSMLCD::OSSMLCD(const gpio_num_t sda, const gpio_num_t clk, const uint8_t address) {
    sdaPin = sda;
    clkPin = clk;
    lcdAddress = address;
}
OSSMLCD::~OSSMLCD() {}
esp_err_t OSSMLCD::Create() {
    // Init LCD and u8g2 lib
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda = sdaPin;
    u8g2_esp32_hal.scl = clkPin;
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_esp32_i2c_byte_cb,
                                           u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
    u8x8_SetI2CAddress(&u8g2.u8x8, (lcdAddress << 1));

    u8g2_InitDisplay(&u8g2);      // send init sequence to the display, display is in sleep mode after this,
    u8g2_SetPowerSave(&u8g2, 0);  // wake up display
    u8g2_ClearBuffer(&u8g2);

    ScreenCurrent = OSSMLCD::Screens::NONE;
    ScreenRequested = OSSMLCD::Screens::NONE;

    DrawScreen(OSSMLCD::Screens::NONE);

    ESP_LOGI(TAG, "Object created");

    return ESP_OK;
}
void OSSMLCD::DrawScreen(Screens Screen) {
    switch (Screen) {
        default:
        case OSSMLCD::Screens::NONE:
            DrawScreenNone();
            break;
        case OSSMLCD::Screens::BOOTING:
            DrawScreenBooting();
            break;
        case OSSMLCD::Screens::HOMING:
            DrawScreenHoming();
            break;
        case OSSMLCD::Screens::READY:
            DrawScreenReady();
            break;
        case OSSMLCD::Screens::STROKE:
            DrawScreenStroke();
            break;
        case OSSMLCD::Screens::REMCTRL:
            DrawScreenRemoteCtrl();
            break;
        case OSSMLCD::Screens::EMERGENCY:
            DrawScreenEmergency();
            break;
    }
}
void OSSMLCD::DrawScreenNone() {
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawXBM(&u8g2, 44, 6, 40, 40, km_logo);
    u8g2_UpdateDisplay(&u8g2);
}
void OSSMLCD::DrawScreenBooting() {
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g_font_profont12);
    u8g2_DrawStr(&u8g2, 4, 1 + (1 * 14), "Version " SW_VERSION);
    u8g2_DrawStr(&u8g2, 4, 1 + (2 * 14), __DATE__);
    u8g2_DrawStr(&u8g2, 4, 1 + (3 * 14), __TIME__);
    u8g2_DrawStr(&u8g2, 4, 1 + (4 * 14), "Booting...");
    u8g2_UpdateDisplay(&u8g2);
}
void OSSMLCD::DrawScreenHoming() {
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_lucasarts_scumm_subtitle_r_tr);
    PrintStrCenterAligned(32, "Homing!");
    u8g2_UpdateDisplay(&u8g2);
}
void OSSMLCD::DrawScreenReady() {
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_streamline_all_t);
    u8g2_DrawUTF8(&u8g2, (REMOTE_LCD_X - 21) / 2, 32, "\u0119");
    u8g2_SetFont(&u8g2, u8g2_font_lucasarts_scumm_subtitle_r_tr);
    PrintStrCenterAligned(46, "Ready!");
    u8g2_UpdateDisplay(&u8g2);
}
void OSSMLCD::DrawScreenStroke() {
    // TODO Write me!
}
void OSSMLCD::DrawScreenEmergency() {
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_streamline_all_t);
    u8g2_DrawUTF8(&u8g2, (REMOTE_LCD_X - 21) / 2, 32, "\u01e3");
    u8g2_SetFont(&u8g2, u8g_font_profont12);
    PrintStrCenterAligned(46, "Emergency stop");
    u8g2_UpdateDisplay(&u8g2);
}
void OSSMLCD::DrawScreenRemoteCtrl() {
    u8g2_SetFont(&u8g2, u8g2_font_streamline_all_t);
    u8g2_DrawUTF8(&u8g2, (REMOTE_LCD_X - 21) / 2, 32, "\u020d");
    u8g2_SetFont(&u8g2, u8g_font_profont12);
    PrintStrCenterAligned(46, "Remote Controlled");
    u8g2_UpdateDisplay(&u8g2);
}

void OSSMLCD::PrintStrCenterAligned(int y, const char *string) {
    const int W = u8g2_GetStrWidth(&u8g2, string);
    u8g2_DrawStr(&u8g2, (REMOTE_LCD_X - W) / 2, y, string);
}
