#pragma once

#include <Arduino.h>
#include <cstddef>

// Use 0xFF to denote that a pin is intentionally left unused.
static constexpr uint8_t GREENHOUSE_UNUSED_PIN = 0xFF;

// High-level wiring for the climate sensor (e.g. DHT22).
// - dataPin: digital GPIO connected to the DHT signal pin.
// - sensorType: one of the DHT library constants (DHT11, DHT22, etc.).
// - pollIntervalMs: how often to poll the sensor for new readings.
struct ClimateSensorConfig {
    uint8_t dataPin;
    uint8_t sensorType;
    uint32_t pollIntervalMs;
};

// Temperature-controlled exhaust/circulation fans (values in Fahrenheit).
// - driverPin: GPIO linked to the fan driver (relay or MOSFET).
// - activeLow: set true when the driver energises on LOW.
// - temperatureOnF / temperatureOffF: hysteresis window for fan control.
// - buttonPin / buttonActiveLow: optional manual override switch (GREENHOUSE_UNUSED_PIN to disable).
struct FanConfig {
    uint8_t driverPin;
    bool activeLow;
    float temperatureOnF;
    float temperatureOffF;
    uint8_t buttonPin;
    bool buttonActiveLow;
};

// Optional real-time clock (RTC) support.
// - enabled: set true when an RTC (e.g. DS3231) is present on the I2C bus.
struct RtcConfig {
    bool enabled;
};

// Optional OLED display over I2C (e.g. SSD1306 backpack).
// - enabled: set true to initialise the display.
// - address: I2C address of the OLED controller.
// - width / height: pixel dimensions passed to the driver.
// - resetPin: optional hardware reset pin (-1 when unused).
struct DisplayConfig {
    bool enabled;
    uint8_t address;
    uint8_t width;
    uint8_t height;
    int8_t resetPin;
};

// Soil moisture sampling options for a single zone (capacitive sensors assumed).
// - analogPin: ADC-capable GPIO tied to the sensor output.
// - dryThreshold: raw ADC value that triggers irrigation (tune via Serial prints).
// - sampleCount: number of quick samples to average each time.
struct SoilSensorConfig {
    uint8_t analogPin;
    uint16_t dryThreshold;
    uint8_t sampleCount;
};

// Per-zone hardware and behaviour settings.
// - name: friendly label for logging/debugging.
// - driverPin: GPIO that drives the solenoid MOSFET/relay gate.
// - driverActiveLow: set true if the driver turns on when driven LOW.
// - buttonPin: optional manual override button (use GREENHOUSE_UNUSED_PIN if none).
// - buttonActiveLow: true for pull-up wiring where LOW means pressed.
// - soilSensor: moisture sampling config for this zone.
// - wateringDurationMs: maximum time to keep the valve open per activation.
// - minimumRestMs: enforced cooldown before automatic watering can resume.
struct SolenoidConfig {
    const char *name;
    uint8_t driverPin;
    bool driverActiveLow;
    uint8_t buttonPin;
    bool buttonActiveLow;
    SoilSensorConfig soilSensor;
    uint32_t wateringDurationMs;
    uint32_t minimumRestMs;
};

// Climate guard rails for pausing irrigation under specific conditions.
// - maxHumidityPercent: skip auto watering when humidity rises above this.
// - minTemperatureF: skip auto watering if temperature falls below this.
struct ClimateHoldConfig {
    float maxHumidityPercent;
    float minTemperatureF;
};

// Global automation behaviour.
// - moisturePollIntervalMs: delay between soil sampling passes per zone.
// - buttonDebounceMs: debounce window for manual override buttons.
struct AutomationConfig {
    uint32_t moisturePollIntervalMs;
    uint32_t buttonDebounceMs;
};

// Bundle all greenhouse settings together for easy sharing with the controller.
struct GreenhouseConfig {
    ClimateSensorConfig climateSensor;
    RtcConfig rtc;
    DisplayConfig display;
    const FanConfig *fans;
    size_t fanCount;
    ClimateHoldConfig climateHold;
    AutomationConfig automation;
    const SolenoidConfig *zones;
    size_t zoneCount;
};

extern const SolenoidConfig SOLENOID_CONFIG[];
extern const GreenhouseConfig greenhouseConfig;
