#pragma once

#include <Arduino.h>

// Hardware constants
#define OUTPUT_ACTIVE_LOW 1
#define DEFAULT_MANUAL_SECS 900
#define MAX_CONCURRENT_VALVES 4
#define BLE_DEVICE_NAME "GreenhouseControl"
#define BTN_DEBOUNCE_MS 60

// GPIO assignments (ESP32 DOIT DevKit v1 headers)
// LEFT header: buttons & analog sensors (USB jack up orientation)
constexpr uint8_t PIN_BTN_LINE1 = 25;
constexpr uint8_t PIN_BTN_LINE2 = 26;
constexpr uint8_t PIN_BTN_LINE3 = 27;
constexpr uint8_t PIN_BTN_MISTER = 14;
constexpr uint8_t PIN_BTN_FAN = 32;
constexpr uint8_t PIN_BTN_LIGHTS = 33;

constexpr uint8_t PIN_SOIL1 = 34; // ADC1
constexpr uint8_t PIN_SOIL2 = 35; // ADC1

// RIGHT header: drivers, data, and I2C
constexpr uint8_t PIN_OUT_LINE1 = 4;
constexpr uint8_t PIN_OUT_LINE2 = 5;
constexpr uint8_t PIN_OUT_LINE3 = 18;
constexpr uint8_t PIN_OUT_MISTER = 19;
constexpr uint8_t PIN_OUT_FAN = 16;
constexpr uint8_t PIN_OUT_LIGHTS = 17;

constexpr uint8_t PIN_DHT_DATA = 23;
constexpr uint8_t PIN_I2C_SDA = 21;
constexpr uint8_t PIN_I2C_SCL = 22;
