idf_component_register(SRCS "glcd.cpp" "u8g2_esp32_hal.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_lcd u8g2 driver)
