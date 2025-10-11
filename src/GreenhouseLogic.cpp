#include "GreenhouseLogic.h"

#include <cmath>
#include <WiFi.h>
#include <Wire.h>

GreenhouseController::GreenhouseController(const GreenhouseConfig &config)
    : config_(config),
      dht_(config.climateSensor.dataPin, config.climateSensor.sensorType),
      zoneStates_(config.zoneCount),
      fanStates_(config.fanCount) {}

void GreenhouseController::begin() {
    dht_.begin();
    Wire.begin();

    if (config_.rtc.enabled) {
        if (rtc_.begin()) {
            rtcAvailable_ = true;
            Serial.println(F("[RTC] DS3231 initialised"));
            if (rtc_.lostPower()) {
                rtc_.adjust(DateTime(F(__DATE__), F(__TIME__)));
                Serial.println(F("[RTC] Power loss detected, setting time to compile timestamp."));
            }
        } else {
            Serial.println(F("[RTC] DS3231 not detected on I2C bus."));
        }
    }

    if (config_.display.enabled) {
        display_.reset(new Adafruit_SSD1306(config_.display.width,
                                            config_.display.height,
                                            &Wire,
                                            config_.display.resetPin));
        if (!display_->begin(SSD1306_SWITCHCAPVCC, config_.display.address)) {
            Serial.println(F("[Display] SSD1306 initialisation failed."));
            display_.reset();
            displayReady_ = false;
        } else {
            displayReady_ = true;
            display_->setRotation(2); // flip the screen so the yellow strip sits at the bottom
            display_->clearDisplay();
            display_->setTextColor(SSD1306_WHITE);
            display_->setTextSize(2);
            display_->setCursor(0, 0);
            display_->println(F("Greenhouse"));
            display_->display();
        }
    }

    // Prepare each zone's driver, button, and timing state.
    for (size_t i = 0; i < config_.zoneCount; ++i) {
        const auto &zone = config_.zones[i];
        auto &state = zoneStates_[i];

        pinMode(zone.driverPin, OUTPUT);
        setDriverState(zone.driverPin, zone.driverActiveLow, false); // ensure valves start closed

        if (zone.buttonPin != GREENHOUSE_UNUSED_PIN) {
            // Configure manual override button with optional pull-up.
            if (zone.buttonActiveLow) {
                pinMode(zone.buttonPin, INPUT_PULLUP);
            } else {
                pinMode(zone.buttonPin, INPUT);
            }
            const bool reading = readButtonRaw(zone);
            state.buttonStableState = reading;
            state.buttonLastReading = reading;
            state.lastButtonChangeMs = millis();
        }

        state.lastStopMs = millis();
    }

    if (config_.fans != nullptr && config_.fanCount > 0) {
        fanRuntime_.resize(config_.fanCount);
        for (size_t i = 0; i < config_.fanCount; ++i) {
            const auto &fan = config_.fans[i];
            auto &fanState = fanStates_[i];
            fanRuntime_[i].temperatureOnF = fan.temperatureOnF;
            fanRuntime_[i].temperatureOffF = fan.temperatureOffF;

            if (fan.driverPin != GREENHOUSE_UNUSED_PIN) {
                pinMode(fan.driverPin, OUTPUT);
                setDriverState(fan.driverPin, fan.activeLow, false);
                fanState.enabled = false;
            }

            if (fan.buttonPin != GREENHOUSE_UNUSED_PIN) {
                if (fan.buttonActiveLow) {
                    pinMode(fan.buttonPin, INPUT_PULLUP);
                } else {
                    pinMode(fan.buttonPin, INPUT);
                }
                const bool reading = digitalRead(fan.buttonPin) == HIGH;
                fanState.buttonStableState = reading;
                fanState.buttonLastReading = reading;
                fanState.lastButtonChangeMs = millis();
            }
        }
    }

    lastClimateReadMs_ = millis();
    climateHoldMinTempF_ = config_.climateHold.minTemperatureF;

    initialiseNetwork();
    setupWebServer();
    updateDisplay();
}

void GreenhouseController::loop() {
    const unsigned long now = millis();
    pollClimate(now);   // capture temp/humidity, update hold state, and manage climate devices
    pollButtons(now);   // check irrigation manual override buttons with debounce
    pollFanButtons(now); // manage fan manual overrides with debounce
    pollMoisture(now);  // run moisture-driven irrigation automation
    updateFans(now);    // drive fan outputs after manual/automation decisions
    handleWebServer();
}

void GreenhouseController::pollClimate(unsigned long now) {
    if (now - lastClimateReadMs_ < config_.climateSensor.pollIntervalMs) {
        return;
    }

    lastClimateReadMs_ = now;

    const float humidity = dht_.readHumidity();
    const float temperature = dht_.readTemperature(true); // request Fahrenheit

    if (!std::isnan(humidity)) {
        lastHumidity_ = humidity;
    }
    if (!std::isnan(temperature)) {
        lastTemperatureF_ = temperature;
    }

    // Emit the latest readings for visibility over Serial Monitor.
    Serial.print(F("[Climate] Humidity: "));
    if (std::isnan(lastHumidity_)) {
        Serial.print(F("N/A"));
    } else {
        Serial.print(lastHumidity_, 1);
        Serial.print(F("%"));
    }
    Serial.print(F("  Temperature: "));
    if (std::isnan(lastTemperatureF_)) {
        Serial.println(F("N/A"));
    } else {
        Serial.print(lastTemperatureF_, 1);
        Serial.println(F(" deg F"));
    }

    // Decide whether the climate should temporarily pause automatic watering.
    bool hold = false;
    if (!std::isnan(lastHumidity_) && config_.climateHold.maxHumidityPercent > 0.0f) {
        hold |= lastHumidity_ > config_.climateHold.maxHumidityPercent;
    }
    if (!std::isnan(lastTemperatureF_) && config_.climateHold.minTemperatureF > -200.0f) {
        hold |= lastTemperatureF_ < config_.climateHold.minTemperatureF;
    }
    climateHoldActive_ = hold;

    updateDisplay();
}

void GreenhouseController::pollButtons(unsigned long now) {
    const uint32_t debounce = config_.automation.buttonDebounceMs;

    for (size_t i = 0; i < config_.zoneCount; ++i) {
        const auto &zone = config_.zones[i];
        if (zone.buttonPin == GREENHOUSE_UNUSED_PIN) {
            continue;
        }

        auto &state = zoneStates_[i];
        const bool reading = readButtonRaw(zone);

        if (reading != state.buttonLastReading) {
            state.buttonLastReading = reading;
            state.lastButtonChangeMs = now;
        }

        // When the reading stabilises for the debounce window, treat it as a press.
        if ((now - state.lastButtonChangeMs) >= debounce && reading != state.buttonStableState) {
            state.buttonStableState = reading;

            if (isButtonActive(zone, reading)) {
                if (state.watering) {
                    stopWatering(i, now);  // button stops the current irrigation run
                } else if (canStartWatering(i, true, now)) {
                    startWatering(i, true, now); // manual start ignores climate hold
                }
            }
        }
    }
}

void GreenhouseController::pollFanButtons(unsigned long now) {
    if (config_.fans == nullptr || config_.fanCount == 0) {
        return;
    }

    const uint32_t debounce = config_.automation.buttonDebounceMs;

    for (size_t i = 0; i < config_.fanCount; ++i) {
        const auto &fan = config_.fans[i];
        if (fan.buttonPin == GREENHOUSE_UNUSED_PIN) {
            continue;
        }

        auto &fanState = fanStates_[i];
        const bool reading = digitalRead(fan.buttonPin) == HIGH;

        if (reading != fanState.buttonLastReading) {
            fanState.buttonLastReading = reading;
            fanState.lastButtonChangeMs = now;
        }

        if ((now - fanState.lastButtonChangeMs) >= debounce && reading != fanState.buttonStableState) {
            fanState.buttonStableState = reading;

            const bool active = fan.buttonActiveLow ? !reading : reading;
            if (active) {
                fanState.manual = !fanState.manual;
            }
        }
    }
}

void GreenhouseController::pollMoisture(unsigned long now) {
    for (size_t i = 0; i < config_.zoneCount; ++i) {
        const auto &zone = config_.zones[i];
        auto &state = zoneStates_[i];

        if (state.watering) {
            // Shut off the valve once the programmed duration elapses.
            if (now - state.wateringStartMs >= zone.wateringDurationMs) {
                stopWatering(i, now);
            }
            continue;
        }

        if (zone.soilSensor.analogPin == GREENHOUSE_UNUSED_PIN) {
            continue;
        }

        if (now - state.lastMoistureReadMs < config_.automation.moisturePollIntervalMs) {
            continue;
        }

        state.lastMoistureReadMs = now;
        state.lastMoisture = sampleSoilMoisture(zone.soilSensor);

        // If the soil is drier than the tuned threshold, request a watering cycle.
        if (state.lastMoisture >= zone.soilSensor.dryThreshold) {
            if (canStartWatering(i, false, now)) {
                startWatering(i, false, now);
            }
        }
    }
}

void GreenhouseController::updateFans(unsigned long now) {
    (void)now;

    if (config_.fans == nullptr || config_.fanCount == 0) {
        return;
    }

    const bool tempValid = !std::isnan(lastTemperatureF_);

    for (size_t i = 0; i < config_.fanCount; ++i) {
        const auto &fan = config_.fans[i];
        auto &fanState = fanStates_[i];

        if (fan.driverPin == GREENHOUSE_UNUSED_PIN) {
            continue;
        }

        bool request = fanState.enabled;
        const auto &runtime = fanRuntime_[i];

        if (fanState.manual) {
            request = true;
        } else {
            bool autoRequest = fanState.enabled;

            if (!fanState.enabled) {
                const bool tempDemand = tempValid && runtime.temperatureOnF > -200.0f && lastTemperatureF_ >= runtime.temperatureOnF;
                autoRequest = tempDemand;
            } else {
                const bool tempLow = !tempValid || runtime.temperatureOffF <= -200.0f || lastTemperatureF_ <= runtime.temperatureOffF;
                autoRequest = !tempLow;
            }

            request = autoRequest;
        }

        if (request != fanState.enabled) {
            setDriverState(fan.driverPin, fan.activeLow, request);
            fanState.enabled = request;
        }
    }
}

void GreenhouseController::updateDisplay() {
    if (!displayReady_ || !display_) {
        return;
    }

    updatingDisplay_ = true;
    display_->clearDisplay();
    display_->setTextColor(SSD1306_WHITE);
    display_->setTextSize(2);

    display_->setCursor(0, 0);
    display_->print(F("T:"));
    if (std::isnan(lastTemperatureF_)) {
        display_->print(F("--.-"));
    } else {
        display_->print(lastTemperatureF_, 1);
    }
    display_->print(F("F"));

    display_->setCursor(0, 32);
    display_->print(F("H:"));
    if (std::isnan(lastHumidity_)) {
        display_->print(F("--.-"));
    } else {
        display_->print(lastHumidity_, 1);
        display_->print(F("%"));
    }

    display_->setTextSize(1);
    display_->fillRect(0, display_->height() - 10, display_->width(), 10, SSD1306_BLACK);
    display_->setCursor(0, display_->height() - 8);
    display_->print(displayStatus_);

    display_->display();
    updatingDisplay_ = false;
}

void GreenhouseController::setDisplayStatus(const String &status) {
    displayStatus_ = status;
    if (displayStatus_.length() > 21) {
        displayStatus_ = displayStatus_.substring(0, 21);
    }
    if (displayReady_ && !updatingDisplay_) {
        updateDisplay();
    }
}

void GreenhouseController::initialiseNetwork() {
    const char *staSsid = "chicken24";
    const char *staPass = "1d0ntkn0w";
    const char *apSsid = "greenhouse";
    const char *apPass = "1d0ntkn0w";

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin(staSsid, staPass);
    setDisplayStatus(F("Connecting WiFi..."));

    unsigned long start = millis();
    unsigned long lastUpdate = 0;
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < 25000UL) {
        unsigned long elapsed = (millis() - start) / 1000UL;
        if (millis() - lastUpdate >= 500UL) {
            setDisplayStatus(String(F("STA connect ")) + String(elapsed) + F("s"));
            lastUpdate = millis();
        }
        delay(100);
        yield();
    }

    if (WiFi.status() == WL_CONNECTED) {
        IPAddress ip = WiFi.localIP();
        String label = ip.toString();
        setDisplayStatus(String(F("WEB ")) + label);
        Serial.print(F("[WiFi] Connected to "));
        Serial.print(staSsid);
        Serial.print(F(" IP: "));
        Serial.println(label);
    } else {
        setDisplayStatus(F("STA failed -> AP"));
        WiFi.disconnect(true);
        WiFi.mode(WIFI_AP);
        if (WiFi.softAP(apSsid, apPass)) {
            delay(100);
            IPAddress ip = WiFi.softAPIP();
            String label = ip.toString();
            setDisplayStatus(String(F("AP  ")) + label);
            Serial.print(F("[WiFi] Started AP "));
            Serial.print(apSsid);
            Serial.print(F(" IP: "));
            Serial.println(label);
        } else {
            Serial.println(F("[WiFi] Failed to start access point"));
            setDisplayStatus(F("AP start failed"));
        }
    }
}

void GreenhouseController::setupWebServer() {
    server_.on("/", HTTP_GET, [this]() {
        this->sendRootPage(String());
    });

    server_.on("/update", HTTP_POST, [this]() {
        bool changed = false;
        for (size_t i = 0; i < fanRuntime_.size(); ++i) {
            const String onKey = String(F("fan")) + String(i + 1) + F("_on");
            const String offKey = String(F("fan")) + String(i + 1) + F("_off");
            float onValue = fanRuntime_[i].temperatureOnF;
            float offValue = fanRuntime_[i].temperatureOffF;
            bool provided = false;
            if (server_.hasArg(onKey)) {
                onValue = server_.arg(onKey).toFloat();
                provided = true;
            }
            if (server_.hasArg(offKey)) {
                offValue = server_.arg(offKey).toFloat();
                provided = true;
            }
            if (provided) {
                setFanThreshold(i, onValue, offValue);
                changed = true;
            }
        }

        if (server_.hasArg("climate_min")) {
            float value = server_.arg("climate_min").toFloat();
            setClimateHoldMinTempF(value);
            changed = true;
        }

        sendRootPage(changed ? F("Settings updated.") : F("No changes detected."));
    });

    server_.onNotFound([this]() {
        server_.send(404, "text/plain", "Not Found");
    });

    server_.begin();
    Serial.println(F("[HTTP] Web server started on port 80"));
}

void GreenhouseController::handleWebServer() {
    server_.handleClient();
}

void GreenhouseController::sendRootPage(const String &statusMessage) {
    String html;
    html.reserve(2048);
    html += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
    html += F("<title>Greenhouse Monitor</title>");
    html += F("<style>body{font-family:system-ui,Segoe UI,sans-serif;margin:0;padding:1.5rem;background:#101820;color:#f5f5f5;}h1{margin-top:0;}table{width:100%;border-collapse:collapse;margin:0.5rem 0;}th,td{padding:0.4rem;border-bottom:1px solid #233;}fieldset{border:1px solid #233;border-radius:8px;margin-bottom:1rem;padding:0.8rem;}legend{padding:0 0.4rem;}label{display:block;margin:0.4rem 0;}input{width:100%;padding:0.45rem;border-radius:6px;border:1px solid #344;background:#0f1a22;color:#f5f5f5;}button{padding:0.6rem 1.5rem;border:none;border-radius:6px;background:#2ecc71;color:#031f15;font-size:1rem;font-weight:600;margin-top:0.5rem;cursor:pointer;}button:hover{background:#29b866;} .card{background:#16222b;border-radius:12px;padding:1rem;margin-bottom:1.2rem;box-shadow:0 0 18px rgba(0,0,0,0.25);} .status{color:#2ecc71;font-weight:600;}</style></head><body>");
    html += F("<h1>Greenhouse Monitor</h1>");

    if (statusMessage.length()) {
        html += F("<p class='status'>");
        html += statusMessage;
        html += F("</p>");
    }

    html += F("<div class='card'><h2>Current Readings</h2><table>");
    html += F("<tr><th>Temperature</th><td>");
    if (std::isnan(lastTemperatureF_)) {
        html += F("N/A");
    } else {
        html += String(lastTemperatureF_, 1);
        html += F(" &deg;F");
    }
    html += F("</td></tr><tr><th>Humidity</th><td>");
    if (std::isnan(lastHumidity_)) {
        html += F("N/A");
    } else {
        html += String(lastHumidity_, 1);
        html += F(" %");
    }
    html += F("</td></tr><tr><th>Climate Hold</th><td>");
    html += climateHoldActive_ ? F("Active") : F("Inactive");
    html += F("</td></tr></table></div>");

    if (!fanStates_.empty()) {
        html += F("<div class='card'><h2>Fan Status</h2><table>");
        for (size_t i = 0; i < fanStates_.size(); ++i) {
            html += F("<tr><th>Fan ");
            html += String(i + 1);
            html += F("</th><td>");
            html += fanStates_[i].enabled ? F("Running") : F("Idle");
            if (fanStates_[i].manual) {
                html += F(" (Manual)");
            }
            html += F("</td></tr>");
        }
        html += F("</table></div>");
    }

    html += F("<div class='card'><h2>Automation Settings</h2><form method='POST' action='/update'>");
    for (size_t i = 0; i < fanRuntime_.size(); ++i) {
        html += F("<fieldset><legend>Fan ");
        html += String(i + 1);
        html += F("</legend>");
        html += F("<label>On Threshold (&deg;F)<input type='number' step='0.1' name='fan");
        html += String(i + 1);
        html += F("_on' value='");
        html += String(fanRuntime_[i].temperatureOnF, 1);
        html += F("'></label>");
        html += F("<label>Off Threshold (&deg;F)<input type='number' step='0.1' name='fan");
        html += String(i + 1);
        html += F("_off' value='");
        html += String(fanRuntime_[i].temperatureOffF, 1);
        html += F("'></label>");
        html += F("</fieldset>");
    }

    html += F("<fieldset><legend>Climate Hold</legend>");
    html += F("<label>Minimum Temperature for Watering (&deg;F)<input type='number' step='0.1' name='climate_min' value='");
    html += String(climateHoldMinTempF_, 1);
    html += F("'></label></fieldset>");
    html += F("<button type='submit'>Save Settings</button></form></div>");

    html += F("<footer><p>Wi-Fi Status: ");
    html += displayStatus_;
    html += F("</p></footer></body></html>");

    server_.send(200, "text/html", html);
}

void GreenhouseController::setFanThreshold(size_t index, float onF, float offF) {
    if (index >= fanRuntime_.size()) {
        return;
    }
    if (std::isnan(onF) || std::isnan(offF)) {
        return;
    }
    if (offF >= onF) {
        offF = onF - 1.0f;
    }
    fanRuntime_[index].temperatureOnF = onF;
    fanRuntime_[index].temperatureOffF = offF;
    Serial.print(F("[Config] Fan "));
    Serial.print(index + 1);
    Serial.print(F(" thresholds updated to on="));
    Serial.print(onF, 1);
    Serial.print(F(" off="));
    Serial.println(offF, 1);
}

float GreenhouseController::fanOnThreshold(size_t index) const {
    if (index >= fanRuntime_.size()) {
        return NAN;
    }
    return fanRuntime_[index].temperatureOnF;
}

float GreenhouseController::fanOffThreshold(size_t index) const {
    if (index >= fanRuntime_.size()) {
        return NAN;
    }
    return fanRuntime_[index].temperatureOffF;
}

void GreenhouseController::setClimateHoldMinTempF(float value) {
    climateHoldMinTempF_ = value;
    Serial.print(F("[Config] Climate hold min temperature set to "));
    Serial.print(value, 1);
    Serial.println(F(" F"));
}

float GreenhouseController::climateHoldMinTempF() const {
    return climateHoldMinTempF_;
}

void GreenhouseController::startWatering(size_t zoneIndex, bool manual, unsigned long now) {
    auto &state = zoneStates_[zoneIndex];
    const auto &zone = config_.zones[zoneIndex];

    state.watering = true;
    state.manual = manual;
    state.wateringStartMs = now;
    setDriverState(zone.driverPin, zone.driverActiveLow, true);
}

void GreenhouseController::stopWatering(size_t zoneIndex, unsigned long now) {
    auto &state = zoneStates_[zoneIndex];
    const auto &zone = config_.zones[zoneIndex];

    setDriverState(zone.driverPin, zone.driverActiveLow, false);
    state.watering = false;
    state.manual = false;
    state.lastStopMs = now;
}

bool GreenhouseController::canStartWatering(size_t zoneIndex, bool manual, unsigned long now) const {
    const auto &state = zoneStates_[zoneIndex];
    const auto &zone = config_.zones[zoneIndex];

    if (state.watering) {
        return false;
    }

    if (!manual) {
        // Automatic runs honour climate holds and rest windows.
        if (climateHoldActive_) {
            return false;
        }
        if (zone.minimumRestMs > 0 && (now - state.lastStopMs) < zone.minimumRestMs) {
            return false;
        }
    }

    return true;
}

bool GreenhouseController::readButtonRaw(const SolenoidConfig &zone) const {
    if (zone.buttonPin == GREENHOUSE_UNUSED_PIN) {
        // Return an inactive state when no button hardware is defined.
        return zone.buttonActiveLow ? HIGH : LOW;
    }
    return digitalRead(zone.buttonPin) == HIGH;
}

bool GreenhouseController::isButtonActive(const SolenoidConfig &zone, bool reading) const {
    if (zone.buttonActiveLow) {
        return !reading;
    }
    return reading;
}

void GreenhouseController::setDriverState(uint8_t pin, bool activeLow, bool enable) {
    if (pin == GREENHOUSE_UNUSED_PIN) {
        return;
    }
    const bool level = activeLow ? !enable : enable;
    digitalWrite(pin, level ? HIGH : LOW);
}

uint16_t GreenhouseController::sampleSoilMoisture(const SoilSensorConfig &sensor) const {
    if (sensor.analogPin == GREENHOUSE_UNUSED_PIN || sensor.sampleCount == 0) {
        return 0;
    }

    uint32_t total = 0;
    for (uint8_t i = 0; i < sensor.sampleCount; ++i) {
        total += static_cast<uint32_t>(analogRead(sensor.analogPin));
        delayMicroseconds(200); // brief pause for ADC settle between samples
    }

    return static_cast<uint16_t>(total / sensor.sampleCount);
}

