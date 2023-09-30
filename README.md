# OSSM Firmware

This is my version of a firmware for the OSSM. The original can be found in the [OSSM repository](https://github.com/KinkyMakers/OSSM-hardware)

I wanted to implement some modifications there, but in the end I decided to drop PlatformIO/Arduino and create a FW based directly on Espressifs ESP-IDF

## TODO List

- [X] Basic ESP-IDF Frame
- [X] NV Storage
- [X] RGB-LED(s) with RMT
- [X] Encoder and Encoder Button
- [X] Sensorless homing
- [X] GLCD
- [ ] Provisioning [Doc](https://docs.espressif.com/projects/esp-idf/en/v5.1.1/esp32/api-reference/provisioning/wifi_provisioning.html)
- [ ] Stroke Engine
- [ ] Stroke Engine: Usage Stats
- [ ] WiFi
- [ ] Webserver
- [ ] Webserver: Build info and current status
- [ ] Webserver: Remote control by webpage
- [ ] Webserver: Firmware update
- [ ] Add emergency off switch
- [ ] GLCD: Use Bitmaps instead of UTF8-Chars
- [ ] ADC: Change to continuous read
- [ ] Button: Click/Doubleclick/Hold detection
- [ ] Storage: Replace with Cpp-Class
- [ ] Make all hardcoded_ values changeable by NVS values
- [ ] Remote Control by Bluetooth?
- [ ] MQTT to local broker or AWS?

## Hardware Info

The FW is written for the OSSM Reference PCB v2.1 with a ESP32-WROOM-32D (4GB Flash, no SPIRAM).

Older/Other PCBs are not supported. Also only sensorless homeing is supported, no limit switches

## Libraries

| Lib       | Version    | Link      |
|-----------|------------|-----------|
| ESP-IDF   | v5.1.1     | [Doc](https://docs.espressif.com/projects/esp-idf/en/v5.1.1/esp32/index.html)  |
| U8G2      | Sept.2023  | [GitHub](https://github.com/olikraus/u8g2) |

## Workflow Status

![Build Workflow](https://github.com/htapohcysPreD/OSSM-FIRMWARE/actions/workflows/main.yml/badge.svg)
