#pragma once

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <RTClib.h>
#include <WebServer.h>
#include <memory>
#include <vector>

#include "GreenhouseConfig.h"

// Coordinates sensor polling, button overrides, valve control, RTC synchronisation, and fan/display automation.
class GreenhouseController {
  public:
    explicit GreenhouseController(const GreenhouseConfig &config);

    // Call once from setup() after Serial is ready.
    void begin();
    // Call frequently from loop(); handles all automation tasks.
    void loop();

    float humidityPercent() const { return lastHumidity_; }
    float temperatureF() const { return lastTemperatureF_; }

  private:
    struct ZoneState {
        bool watering = false;             // true while the driver is actively watering
        bool manual = false;               // true if the current run was triggered by a button
        unsigned long wateringStartMs = 0; // millis() when watering began
        unsigned long lastStopMs = 0;      // millis() when watering last ended
        unsigned long lastMoistureReadMs = 0; // timestamp of previous moisture sample
        uint16_t lastMoisture = 0;         // cached moisture ADC reading for debugging
        bool buttonStableState = true;     // debounced button state
        bool buttonLastReading = true;     // most recent raw button read
        unsigned long lastButtonChangeMs = 0; // last time the button reading toggled
    };

    const GreenhouseConfig &config_;
    DHT dht_;
    std::vector<ZoneState> zoneStates_;
    struct FanState {
        bool enabled = false;             // true while the fan driver is energised
        bool manual = false;              // true when a button press forces the fan on
        bool buttonStableState = true;    // debounced button state
        bool buttonLastReading = true;    // most recent raw button read
        unsigned long lastButtonChangeMs = 0; // last time the button reading toggled
    };
    struct FanRuntimeConfig {
        float temperatureOnF = 0.0f;
        float temperatureOffF = 0.0f;
    };
    std::vector<FanState> fanStates_;
    std::vector<FanRuntimeConfig> fanRuntime_;
    RTC_DS3231 rtc_;
    bool rtcAvailable_ = false;
    std::unique_ptr<Adafruit_SSD1306> display_;
    bool displayReady_ = false;
    bool updatingDisplay_ = false;
    String displayStatus_ = F("No WiFi");
    WebServer server_{80};
    unsigned long lastClimateReadMs_ = 0;
    bool climateHoldActive_ = false;
    float lastHumidity_ = NAN;
    float lastTemperatureF_ = NAN;
    float climateHoldMinTempF_ = -200.0f;

    void pollClimate(unsigned long now);
    void pollButtons(unsigned long now);
    void pollFanButtons(unsigned long now);
    void pollMoisture(unsigned long now);
    void updateFans(unsigned long now);
    void updateDisplay();
    void setDisplayStatus(const String &status);
    void initialiseNetwork();
    void setupWebServer();
    void handleWebServer();
    void sendRootPage(const String &statusMessage);
    void setFanThreshold(size_t index, float onF, float offF);
    float fanOnThreshold(size_t index) const;
    float fanOffThreshold(size_t index) const;
    void setClimateHoldMinTempF(float value);
    float climateHoldMinTempF() const;
    void startWatering(size_t zoneIndex, bool manual, unsigned long now);
    void stopWatering(size_t zoneIndex, unsigned long now);
    bool canStartWatering(size_t zoneIndex, bool manual, unsigned long now) const;
    bool readButtonRaw(const SolenoidConfig &zone) const;
    bool isButtonActive(const SolenoidConfig &zone, bool reading) const;
    void setDriverState(uint8_t pin, bool activeLow, bool enable);
    uint16_t sampleSoilMoisture(const SoilSensorConfig &sensor) const;
};
