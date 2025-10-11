#include "GreenhouseConfig.h"

#include <DHT.h>

// Wiring guide (ESP32 DOIT DevKit v1, viewed with USB jack up):
//   * LEFT header (buttons & analog sensors):
//       - Zone buttons: GPIO 25, 26, 27, 14 (normally-open to ground, INPUT_PULLUP)
//       - Fan buttons: GPIO 32, 33 (toggle overrides, INPUT_PULLUP)
//       - Soil sensors: GPIO 34, 35 (analog inputs for capacitive probes)
//   * RIGHT header (drivers, data, and I2C):
//       - Irrigation solenoids: GPIO 4, 5, 18, 19 (active-low MOSFET/relay inputs)
//       - Fans: GPIO 16 (RX2) and 17 (TX2) (active-low relay/MOSFET inputs)
//       - DHT22 data: GPIO 23
//       - I2C bus (OLED + RTC): SDA 21 / SCL 22
//
// Update the four entries below to match your valve wiring and capacitive moisture sensors.
// Duplicate or remove entries if the number of irrigation zones changes.
const SolenoidConfig SOLENOID_CONFIG[] = {
    {
        "North Bed",    // friendly name shown in debug prints
        4,              // driverPin: GPIO driving the MOSFET/relay for the North solenoid
        true,           // driverActiveLow: set false if HIGH turns the solenoid on
        25,             // buttonPin: manual override button (GREENHOUSE_UNUSED_PIN to disable)
        true,           // buttonActiveLow: true for pull-up wiring (LOW when pressed)
        {34, 2400, 5},  // soilSensor: {analogPin, dryThreshold, sampleCount}
        120000UL,       // wateringDurationMs: 2 minutes maximum water time
        900000UL        // minimumRestMs: 15 minutes rest before auto start
    },
    {
        "Center Bed",
        5,
        true,
        26,
        true,
        {35, 2400, 5},
        120000UL,
        900000UL
    },
    {
        "South Bed",
        18,
        true,
        27,
        true,
        {GREENHOUSE_UNUSED_PIN, 0, 0}, // manual-only zone (no soil sensor)
        120000UL,
        900000UL
    },
    {
        "Herb Planter",
        19,
        true,
        14,
        true,
        {GREENHOUSE_UNUSED_PIN, 0, 0}, // manual-only zone (no soil sensor)
        120000UL,
        900000UL
    }
};

// Dual-fan configuration with manual override buttons.
const FanConfig FAN_CONFIG[] = {
    {
        16,    // driverPin: exhaust fan 1 controller (GPIO16 / RX2)
        true,  // activeLow: LOW energises the relay/MOSFET
        89.6f, // temperatureOnF
        82.4f, // temperatureOffF
        32,    // buttonPin: manual toggle button (LOW when pressed)
        true   // buttonActiveLow: uses INPUT_PULLUP wiring
    },
    {
        17,    // driverPin: circulation fan 2 controller (GPIO17 / TX2)
        true,
        89.6f,
        82.4f,
        33,    // buttonPin: manual toggle button (LOW when pressed)
        true
    }
};

// High-level greenhouse settings. Adjust sensor pins/thresholds here instead of in the logic.
const GreenhouseConfig greenhouseConfig = {
    {23, DHT22, 15000UL},     // climateSensor: {dataPin, sensorType, pollIntervalMs}
    {true},                   // rtc: DS3231 present on the I2C bus
    {true, 0x3C, 128, 64, -1}, // display: SSD1306 OLED @ 0x3C, 128x64, no reset pin
    FAN_CONFIG,
    sizeof(FAN_CONFIG) / sizeof(FAN_CONFIG[0]),
    {90.0f, 39.2f},           // climateHold: pause auto watering above 90% RH or below 39.2 deg F
    {10000UL, 50UL},          // automation: {moisturePollIntervalMs, buttonDebounceMs}
    SOLENOID_CONFIG,
    sizeof(SOLENOID_CONFIG) / sizeof(SOLENOID_CONFIG[0])
};
