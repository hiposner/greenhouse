#include <Arduino.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <ctype.h>
#include <cmath>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <RTClib.h>
#include <time.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "config.h"

enum Zone {
    Z_LINE1 = 0,
    Z_LINE2,
    Z_LINE3,
    Z_MISTER,
    Z_FAN,
    Z_LIGHTS,
    Z_COUNT
};

enum ControlMode {
    MODE_OFF = 0,
    MODE_AUTO
};

enum DeviceMode {
    DEVICE_MODE_SCHEDULE = 0,
    DEVICE_MODE_SCHEDULE_WITH_LOGIC,
    DEVICE_MODE_LOGIC_ONLY
};

enum RuleType {
    RULE_INHIBIT_ON = 0,
    RULE_FORCE_ON
};

enum RuleOp {
    OP_GT = 0,
    OP_LT,
    OP_GTE,
    OP_LTE
};

static const uint8_t ZONE_OUTPUT_PINS[Z_COUNT] = {
    PIN_OUT_LINE1,
    PIN_OUT_LINE2,
    PIN_OUT_LINE3,
    PIN_OUT_MISTER,
    PIN_OUT_FAN,
    PIN_OUT_LIGHTS
};

static const uint8_t ZONE_BUTTON_PINS[Z_COUNT] = {
    PIN_BTN_LINE1,
    PIN_BTN_LINE2,
    PIN_BTN_LINE3,
    PIN_BTN_MISTER,
    PIN_BTN_FAN,
    PIN_BTN_LIGHTS
};

static const char *ZONE_KEYS[Z_COUNT] = {
    "line1", "line2", "line3", "mister", "fan", "lights"
};

static const char *ZONE_LABELS[Z_COUNT] = {
    "Red (Line 1)", "Green (Line 2)", "Yellow (Line 3)", "Blue (Mister)", "Fan", "Grow Lights"
};

static constexpr int DISPLAY_WIDTH = 128;
static constexpr int DISPLAY_HEIGHT = 64;
static constexpr uint8_t DISPLAY_ADDRESS = 0x3C;
static constexpr unsigned long DISPLAY_REFRESH_MS = 5000;
static Adafruit_SSD1306 display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, -1);
static bool displayReady = false;
static unsigned long lastDisplayMs = 0;

static constexpr unsigned long CLIMATE_POLL_MS = 15000;
static constexpr unsigned long SOIL_POLL_MS = 10000;
static constexpr unsigned long SENSOR_PUBLISH_MIN_MS = 10000;
static constexpr float CLIMATE_DELTA_MIN = 0.2f;
static constexpr float SOIL_DELTA_MIN = 10.0f;
static DHT dht(PIN_DHT_DATA, DHT22);
static unsigned long lastClimatePollMs = 0;
static unsigned long lastSoilPollMs = 0;
static unsigned long lastSensorPublishMs = 0;
static float lastTempF = NAN;
static float lastHumidity = NAN;
static float lastSoil1 = NAN;
static float lastSoil2 = NAN;

static uint32_t defaultManualDuration(Zone zone) {
    switch (zone) {
    case Z_LINE1:
    case Z_LINE2:
    case Z_LINE3:
    case Z_MISTER:
        return 900; // 15 minutes
    default:
        return 0; // relays stay on until toggled off
    }
}

struct ZoneState {
    bool on = false;
    bool manual = false;
    uint32_t safetyUntilMs = 0;
    bool buttonStableState = true;
    bool buttonLastReading = true;
    unsigned long lastButtonChangeMs = 0;
};

static ZoneState zoneStates[Z_COUNT];

static NimBLEServer *bleServer = nullptr;
static NimBLECharacteristic *txCharacteristic = nullptr;
static NimBLECharacteristic *rxCharacteristic = nullptr;
static bool bleConnected = false;
static ControlMode currentMode = MODE_AUTO;

struct ScheduleEntry {
    String id;
    Zone zone = Z_LINE1;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint32_t durationSeconds = 0;
    uint8_t daysMask = 0;
    int lastRunYDay = -1;
};

struct LogicRule {
    RuleType type = RULE_INHIBIT_ON;
    RuleOp op = OP_GT;
    String sensor;
    float onThreshold = NAN;
    float offThreshold = NAN; // optional hysteresis
    bool active = false;      // hysteresis latch
};

static std::vector<ScheduleEntry> schedules;
static std::vector<DeviceMode> deviceModes(Z_COUNT, DEVICE_MODE_SCHEDULE);
static std::vector<std::vector<LogicRule>> deviceLogic(Z_COUNT);
static Preferences preferences;
static bool preferencesReady = false;
static RTC_DS3231 rtc;
static bool rtcReady = false;
static bool timeSynced = false;
static int64_t timeSyncEpoch = 0;
static unsigned long timeSyncMillis = 0;
static int lastSchedulerMinute = -1;
static std::unordered_map<std::string, float> sensorReadings; // name -> last value
static constexpr const char *PERSIST_NAMESPACE = "ghctl";
static constexpr const char *PERSIST_KEY_STATE = "state";

static const NimBLEUUID SERVICE_UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static const NimBLEUUID RX_UUID("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
static const NimBLEUUID TX_UUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e");

// Forward declarations
static void publishState();
static void publishScheduleEntry(const ScheduleEntry &entry);
static void publishScheduleDelete(const String &id);
static void publishAllSchedules();
static void publishDeviceMode(Zone zone);
static void publishAllDeviceModes();
static void publishLogicRules(Zone zone);
static void publishAllLogicRules();
static void publishPong();
static void handleBleCommand(const std::string &payload);
static bool loadPersistentState();
static bool savePersistentState();
static void initRtc();
static void saveTimeToRtc(uint32_t epochSeconds);
static bool setZone(Zone zone, bool on, uint32_t durationSeconds = 0, bool manual = false);
static void digitalWriteActive(uint8_t pin, bool on);
static bool parseZoneKey(const char *key, Zone &zone);
static const char *modeToString(ControlMode mode);
static bool equalsIgnoreCase(const char *a, const char *b);
static void pollZoneButtons(unsigned long now);
static bool parseDaysMask(const JsonVariantConst &value, uint8_t &mask);
static void daysMaskToJson(uint8_t mask, JsonArray &out);
static void evaluateSchedules();
static bool addOrUpdateSchedule(const ScheduleEntry &entry);
static bool removeScheduleById(const String &id);
static time_t currentEpoch();
static void syncTime(uint32_t epochSeconds);
static void setTimeOffsetMinutes(int32_t minutes);
static int32_t timeOffsetSeconds = 0;
static bool manualOverrideActive();
struct LogicResult { bool forceOn = false; bool inhibitOn = false; };
static LogicResult evaluateLogic(size_t zoneIndex);
static bool evaluateRule(const LogicRule &rule, float value);
static void applyModeAndPriority(unsigned long now);
static float getSensorValue(const String &name, bool &valid);
static bool parseDeviceMode(const char *text, DeviceMode &outMode);
static const char *deviceModeToString(DeviceMode mode);
static const char *ruleTypeToString(RuleType type);
static const char *ruleOpToString(RuleOp op);
static bool parseLogicRule(const JsonVariantConst &obj, LogicRule &outRule);
static void pollSensors(unsigned long now);
static void pollClimateSensors(unsigned long now, bool &shouldPublish);
static void pollSoilSensors(unsigned long now, bool &shouldPublish);
static bool valueChanged(float previous, float current, float delta);
static void updateDisplay(bool force);

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
        (void)server;
        (void)connInfo;
        bleConnected = true;
        publishState();
        publishAllSchedules();
        publishAllDeviceModes();
        publishAllLogicRules();
    }

    void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
        (void)server;
        (void)connInfo;
        (void)reason;
        bleConnected = false;
        NimBLEDevice::startAdvertising();
    }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &connInfo) override {
        (void)connInfo;
        std::string payload = characteristic->getValue();
        if (!payload.empty()) {
            handleBleCommand(payload);
        }
    }
};

static ServerCallbacks serverCallbacks;
static CommandCallbacks commandCallbacks;

static const char *modeToString(ControlMode mode) {
    switch (mode) {
    case MODE_OFF:
        return "OFF";
    case MODE_AUTO:
    default:
        return "AUTO";
    }
}

static bool equalsIgnoreCase(const char *a, const char *b) {
    if (!a || !b) {
        return false;
    }
    while (*a && *b) {
        if (tolower(*a) != tolower(*b)) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

static bool parseZoneKey(const char *key, Zone &zone) {
    if (!key) {
        return false;
    }
    for (size_t i = 0; i < Z_COUNT; ++i) {
        if (equalsIgnoreCase(key, ZONE_KEYS[i])) {
            zone = static_cast<Zone>(i);
            return true;
        }
    }
    return false;
}

static void digitalWriteActive(uint8_t pin, bool on) {
#if OUTPUT_ACTIVE_LOW
    digitalWrite(pin, on ? LOW : HIGH);
#else
    digitalWrite(pin, on ? HIGH : LOW);
#endif
}

static void applyZoneOutput(Zone zone, bool on) {
    digitalWriteActive(ZONE_OUTPUT_PINS[zone], on);
    zoneStates[zone].on = on;
}

static bool setZone(Zone zone, bool on, uint32_t durationSeconds, bool manual) {
    bool stateChanged = false;
    unsigned long now = millis();
    // Interlock removed: multiple irrigation lines may run simultaneously.

    if (zoneStates[zone].on != on) {
        Serial.printf("Zone %s -> %s\n", ZONE_KEYS[zone], on ? "ON" : "OFF");
        applyZoneOutput(zone, on);
        stateChanged = true;
    }

    if (on) {
        zoneStates[zone].manual = manual;
        uint32_t targetSeconds = durationSeconds;
        if (targetSeconds == 0 && manual) {
            targetSeconds = defaultManualDuration(zone);
        }
        uint32_t newDeadline = 0;
        if (targetSeconds > 0) {
            newDeadline = now + (targetSeconds * 1000UL);
        }
        if (zoneStates[zone].safetyUntilMs != newDeadline) {
            zoneStates[zone].safetyUntilMs = newDeadline;
            stateChanged = true;
        }
    } else {
        if (zoneStates[zone].safetyUntilMs != 0) {
            zoneStates[zone].safetyUntilMs = 0;
            stateChanged = true;
        }
        if (zoneStates[zone].manual) {
            zoneStates[zone].manual = false;
            stateChanged = true;
        }
    }

    if (stateChanged) {
        publishState();
    }
    return stateChanged;
}

static const char *DAY_NAMES[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

static void publishState() {
    // Compact state payload (arrays) to stay under conservative MTU limits.
    JsonDocument doc;
    const unsigned long nowMs = millis();
    doc["e"] = "s"; // event: state
    doc["m"] = (currentMode == MODE_AUTO) ? 1 : 0; // mode: 1=auto,0=off
    doc["o"] = manualOverrideActive() ? 1 : 0;     // any manual override active
    doc["t"] = nowMs;                              // timestamp
    if (timeSynced) {
        doc["c"] = static_cast<uint32_t>(currentEpoch()); // current epoch seconds
        doc["x"] = 1;                                      // time synced flag
    } else {
        doc["c"] = 0;
        doc["x"] = 0;
    }

    JsonArray states = doc["z"].to<JsonArray>(); // zone states
    JsonArray remaining = doc["r"].to<JsonArray>();
    JsonArray overrides = doc["v"].to<JsonArray>();
    for (size_t i = 0; i < Z_COUNT; ++i) {
        states.add(zoneStates[i].on ? 1 : 0);
        if (zoneStates[i].on && zoneStates[i].safetyUntilMs > nowMs) {
            remaining.add((zoneStates[i].safetyUntilMs - nowMs) / 1000UL);
        } else {
            remaining.add(0);
        }
        overrides.add(zoneStates[i].manual ? 1 : 0);
    }

    // Include sensor readings when available so the UI can display them.
    if (!sensorReadings.empty()) {
        JsonObject sensors = doc["sensors"].to<JsonObject>();
        for (const auto &kv : sensorReadings) {
            if (!std::isnan(kv.second)) {
                sensors[kv.first.c_str()] = kv.second;
            }
        }
    }

    std::string payload;
    serializeJson(doc, payload);

    if (bleConnected && txCharacteristic != nullptr) {
        txCharacteristic->setValue(reinterpret_cast<const uint8_t *>(payload.data()), payload.size());
        txCharacteristic->notify();
    }

    Serial.print("State event: ");
    Serial.println(payload.c_str());
}

static void publishPong() {
    JsonDocument doc;
    doc["evt"] = "pong";
    doc["ts"] = millis();
    char buffer[96];
    size_t len = serializeJson(doc, buffer, sizeof(buffer));

    if (bleConnected && txCharacteristic != nullptr) {
        txCharacteristic->setValue(reinterpret_cast<const uint8_t *>(buffer), len);
        txCharacteristic->notify();
    }

    Serial.println("Responded with pong");
}

static void publishScheduleEntry(const ScheduleEntry &entry) {
    JsonDocument doc;
    doc["e"] = "sc"; // schedule create/update
    doc["i"] = entry.id;
    doc["z"] = ZONE_KEYS[entry.zone];
    doc["h"] = entry.hour;
    doc["m"] = entry.minute;
    doc["d"] = entry.durationSeconds;
    doc["n"] = (entry.durationSeconds + 59U) / 60U; // minutes, rounded up
    doc["w"] = entry.daysMask;

    std::string payload;
    serializeJson(doc, payload);

    if (bleConnected && txCharacteristic != nullptr) {
        txCharacteristic->setValue(reinterpret_cast<const uint8_t *>(payload.data()), payload.size());
        txCharacteristic->notify();
    }

    Serial.print("Schedule event: ");
    Serial.println(payload.c_str());
}

static void publishScheduleDelete(const String &id) {
    JsonDocument doc;
    doc["e"] = "sd";
    doc["i"] = id;
    std::string payload;
    serializeJson(doc, payload);
    if (bleConnected && txCharacteristic != nullptr) {
        txCharacteristic->setValue(reinterpret_cast<const uint8_t *>(payload.data()), payload.size());
        txCharacteristic->notify();
    }
    Serial.print("Schedule delete: ");
    Serial.println(id);
}

static void publishAllSchedules() {
    for (const auto &entry : schedules) {
        publishScheduleEntry(entry);
    }
}

static const char *deviceModeToString(DeviceMode mode) {
    switch (mode) {
    case DEVICE_MODE_SCHEDULE:
        return "SCHEDULE";
    case DEVICE_MODE_SCHEDULE_WITH_LOGIC:
        return "SCHEDULE_WITH_LOGIC";
    case DEVICE_MODE_LOGIC_ONLY:
        return "LOGIC_ONLY";
    default:
        return "SCHEDULE";
    }
}

static const char *ruleTypeToString(RuleType type) {
    switch (type) {
    case RULE_INHIBIT_ON:
        return "INHIBIT_ON";
    case RULE_FORCE_ON:
        return "FORCE_ON";
    default:
        return "INHIBIT_ON";
    }
}

static const char *ruleOpToString(RuleOp op) {
    switch (op) {
    case OP_GT:
        return ">";
    case OP_LT:
        return "<";
    case OP_GTE:
        return ">=";
    case OP_LTE:
        return "<=";
    default:
        return ">";
    }
}

static void publishDeviceMode(Zone zone) {
    JsonDocument doc;
    doc["e"] = "dm";
    doc["z"] = ZONE_KEYS[zone];
    doc["m"] = deviceModeToString(deviceModes[zone]);

    std::string payload;
    serializeJson(doc, payload);

    if (bleConnected && txCharacteristic != nullptr) {
        txCharacteristic->setValue(reinterpret_cast<const uint8_t *>(payload.data()), payload.size());
        txCharacteristic->notify();
    }
}

static void publishAllDeviceModes() {
    for (size_t i = 0; i < Z_COUNT; ++i) {
        publishDeviceMode(static_cast<Zone>(i));
    }
}

static void publishLogicRules(Zone zone) {
    JsonDocument clearDoc;
    clearDoc["e"] = "lc";
    clearDoc["z"] = ZONE_KEYS[zone];

    std::string clearPayload;
    serializeJson(clearDoc, clearPayload);

    if (bleConnected && txCharacteristic != nullptr) {
        txCharacteristic->setValue(reinterpret_cast<const uint8_t *>(clearPayload.data()), clearPayload.size());
        txCharacteristic->notify();
    }

    const auto &rules = deviceLogic[zone];
    for (size_t i = 0; i < rules.size(); ++i) {
        const auto &rule = rules[i];
        JsonDocument doc;
        doc["e"] = "la";
        doc["z"] = ZONE_KEYS[zone];
        doc["i"] = static_cast<uint16_t>(i);
        doc["type"] = ruleTypeToString(rule.type);
        doc["sensor"] = rule.sensor;
        doc["op"] = ruleOpToString(rule.op);
        doc["on"] = rule.onThreshold;
        if (!std::isnan(rule.offThreshold)) {
            doc["off"] = rule.offThreshold;
        }

        std::string payload;
        serializeJson(doc, payload);

        if (bleConnected && txCharacteristic != nullptr) {
            txCharacteristic->setValue(reinterpret_cast<const uint8_t *>(payload.data()), payload.size());
            txCharacteristic->notify();
        }
    }
}

static void publishAllLogicRules() {
    for (size_t i = 0; i < Z_COUNT; ++i) {
        publishLogicRules(static_cast<Zone>(i));
    }
}

static bool savePersistentState() {
    if (!preferencesReady) {
        return false;
    }

    JsonDocument doc;
    doc["mode"] = modeToString(currentMode);
    doc["tzm"] = timeOffsetSeconds / 60;

    JsonArray schedulesArr = doc["s"].to<JsonArray>();
    for (const auto &entry : schedules) {
        JsonObject item = schedulesArr.add<JsonObject>();
        item["i"] = entry.id;
        item["z"] = ZONE_KEYS[entry.zone];
        item["h"] = entry.hour;
        item["m"] = entry.minute;
        item["d"] = entry.durationSeconds;
        item["w"] = entry.daysMask;
    }

    JsonArray modesArr = doc["dm"].to<JsonArray>();
    for (size_t i = 0; i < Z_COUNT; ++i) {
        modesArr.add(deviceModeToString(deviceModes[i]));
    }

    JsonObject logicObj = doc["l"].to<JsonObject>();
    for (size_t i = 0; i < Z_COUNT; ++i) {
        JsonArray rulesArr = logicObj[ZONE_KEYS[i]].to<JsonArray>();
        for (const auto &rule : deviceLogic[i]) {
            JsonObject ruleObj = rulesArr.add<JsonObject>();
            ruleObj["type"] = ruleTypeToString(rule.type);
            ruleObj["sensor"] = rule.sensor;
            ruleObj["op"] = ruleOpToString(rule.op);
            ruleObj["on"] = rule.onThreshold;
            if (!std::isnan(rule.offThreshold)) {
                ruleObj["off"] = rule.offThreshold;
            }
        }
    }

    std::string payload;
    serializeJson(doc, payload);
    size_t written = preferences.putBytes(PERSIST_KEY_STATE, payload.data(), payload.size());
    if (written != payload.size()) {
        Serial.println("Persistent save failed");
        return false;
    }

    Serial.printf("Persistent state saved (%u bytes)\n", static_cast<unsigned>(written));
    return true;
}

static bool loadPersistentState() {
    if (!preferencesReady) {
        return false;
    }

    size_t len = preferences.getBytesLength(PERSIST_KEY_STATE);
    if (len == 0) {
        Serial.println("No persisted state found");
        return false;
    }

    std::vector<char> buffer(len + 1, '\0');
    size_t read = preferences.getBytes(PERSIST_KEY_STATE, buffer.data(), len);
    if (read != len) {
        Serial.println("Persistent load failed");
        return false;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, buffer.data(), len);
    if (err) {
        Serial.print("Persistent JSON parse error: ");
        Serial.println(err.f_str());
        return false;
    }

    const char *modeStr = doc["mode"];
    if (modeStr) {
        if (equalsIgnoreCase(modeStr, "AUTO")) {
            currentMode = MODE_AUTO;
        } else if (equalsIgnoreCase(modeStr, "OFF")) {
            currentMode = MODE_OFF;
        }
    }

    int32_t tzOffsetMin = doc["tzm"] | 0;
    setTimeOffsetMinutes(tzOffsetMin);

    schedules.clear();
    JsonArrayConst schedulesArr = doc["s"].as<JsonArrayConst>();
    for (JsonVariantConst item : schedulesArr) {
        if (schedules.size() >= 16) {
            break;
        }
        const char *idStr = item["i"];
        const char *zoneKey = item["z"];
        int hour = item["h"] | -1;
        int minute = item["m"] | -1;
        uint32_t durationSeconds = item["d"] | 0;
        uint8_t daysMask = item["w"] | 0;

        Zone zone;
        if (!idStr || !zoneKey || !parseZoneKey(zoneKey, zone)) {
            continue;
        }
        if (hour < 0 || hour > 23 || minute < 0 || minute > 59 || durationSeconds == 0 || daysMask == 0) {
            continue;
        }

        ScheduleEntry entry;
        entry.id = idStr;
        entry.zone = zone;
        entry.hour = static_cast<uint8_t>(hour);
        entry.minute = static_cast<uint8_t>(minute);
        entry.durationSeconds = durationSeconds;
        entry.daysMask = daysMask;
        entry.lastRunYDay = -1;
        schedules.push_back(entry);
    }

    deviceModes.assign(Z_COUNT, DEVICE_MODE_SCHEDULE);
    JsonArrayConst modesArr = doc["dm"].as<JsonArrayConst>();
    if (!modesArr.isNull()) {
        size_t idx = 0;
        for (JsonVariantConst modeVar : modesArr) {
            if (idx >= Z_COUNT) {
                break;
            }
            const char *modeText = modeVar.as<const char *>();
            DeviceMode parsedMode;
            if (parseDeviceMode(modeText, parsedMode)) {
                deviceModes[idx] = parsedMode;
            }
            ++idx;
        }
    }

    deviceLogic.assign(Z_COUNT, {});
    JsonObjectConst logicObj = doc["l"].as<JsonObjectConst>();
    if (!logicObj.isNull()) {
        for (size_t i = 0; i < Z_COUNT; ++i) {
            JsonArrayConst rulesArr = logicObj[ZONE_KEYS[i]].as<JsonArrayConst>();
            if (rulesArr.isNull()) {
                continue;
            }
            for (JsonVariantConst ruleVar : rulesArr) {
                LogicRule rule;
                if (parseLogicRule(ruleVar, rule)) {
                    deviceLogic[i].push_back(rule);
                }
            }
        }
    }

    Serial.printf("Restored %u schedules from persistence\n", static_cast<unsigned>(schedules.size()));
    return true;
}

static void initRtc() {
    if (!rtc.begin()) {
        Serial.println("RTC not detected on I2C");
        return;
    }
    rtcReady = true;
    if (rtc.lostPower()) {
        Serial.println("RTC reports power loss; waiting for clock sync");
        return;
    }
    DateTime now = rtc.now();
    uint32_t epoch = now.unixtime();
    if (epoch > 946684800UL) { // Jan 1, 2000 UTC
        syncTime(epoch);
        Serial.print("Time restored from RTC: ");
        Serial.println(epoch);
    } else {
        Serial.println("RTC time invalid; waiting for clock sync");
    }
}

static void saveTimeToRtc(uint32_t epochSeconds) {
    if (!rtcReady || epochSeconds == 0) {
        return;
    }
    rtc.adjust(DateTime(epochSeconds));
}

static void handleBleCommand(const std::string &payload) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (err) {
        Serial.print("JSON parse error: ");
        Serial.println(err.f_str());
        return;
    }

    const char *cmd = doc["cmd"];
    if (!cmd) {
        Serial.println("Missing cmd field");
        return;
    }

    if (equalsIgnoreCase(cmd, "setValve")) {
        const char *zoneKey = doc["zone"];
        const char *stateStr = doc["state"];
        uint32_t durationSeconds = doc["duration_s"] | 0;
        Zone zone;
        if (!parseZoneKey(zoneKey, zone)) {
            Serial.println("Invalid zone key");
            return;
        }
        if (!stateStr) {
            Serial.println("Missing state in setValve");
            return;
        }
        bool turnOn = equalsIgnoreCase(stateStr, "ON");
        setZone(zone, turnOn, durationSeconds, true);
    } else if (equalsIgnoreCase(cmd, "setMode")) {
        const char *modeStr = doc["mode"];
        if (!modeStr) {
            Serial.println("Missing mode value");
            return;
        }
        ControlMode nextMode = currentMode;
        if (equalsIgnoreCase(modeStr, "AUTO")) {
            nextMode = MODE_AUTO;
        } else if (equalsIgnoreCase(modeStr, "OFF") || equalsIgnoreCase(modeStr, "MANUAL")) {
            // treat legacy "MANUAL" command the same as OFF (manual-only control)
            nextMode = MODE_OFF;
        } else {
            Serial.println("Unknown mode value");
            return;
        }
        if (nextMode != currentMode) {
            currentMode = nextMode;
            savePersistentState();
            publishState();
        }
    } else if (equalsIgnoreCase(cmd, "setSchedule")) {
        const char *idStr = doc["id"];
        const char *zoneKey = doc["zone"];
        int hour = doc["hour"] | -1;
        int minute = doc["minute"] | -1;
        uint32_t durationSeconds = doc["duration_s"] | 0;
        uint32_t durationMinutes = doc["duration_m"] | 0;
        JsonVariantConst daysVar = doc["days"];

        if (durationSeconds == 0 && durationMinutes > 0) {
            durationSeconds = durationMinutes * 60U;
        }

        Zone zone;
        if (!parseZoneKey(zoneKey, zone)) {
            Serial.println("Invalid schedule zone");
            return;
        }
        if (hour < 0 || hour > 23 || minute < 0 || minute > 59 || durationSeconds == 0) {
            Serial.println("Invalid schedule timings");
            return;
        }
        uint8_t daysMask = 0;
        if (!parseDaysMask(daysVar, daysMask)) {
            Serial.println("Invalid schedule days");
            return;
        }
        String scheduleId = idStr && idStr[0] ? String(idStr) : String(millis());
        ScheduleEntry entry;
        entry.id = scheduleId;
        entry.zone = zone;
        entry.hour = static_cast<uint8_t>(hour);
        entry.minute = static_cast<uint8_t>(minute);
        entry.durationSeconds = durationSeconds;
        entry.daysMask = daysMask;
        entry.lastRunYDay = -1;
        if (addOrUpdateSchedule(entry)) {
            Serial.print("Schedule saved: ");
            Serial.println(scheduleId);
            savePersistentState();
            publishState();
            publishScheduleEntry(entry);
        }
    } else if (equalsIgnoreCase(cmd, "deleteSchedule")) {
        const char *idStr = doc["id"];
        if (!idStr) {
            Serial.println("Missing schedule id");
            return;
        }
        if (removeScheduleById(String(idStr))) {
            Serial.print("Schedule deleted: ");
            Serial.println(idStr);
            savePersistentState();
            publishState();
            publishScheduleDelete(String(idStr));
        }
    } else if (equalsIgnoreCase(cmd, "setTime")) {
        uint32_t epoch = doc["epoch"] | 0;
        int32_t tzOffsetMin = doc["tzOffsetMin"] | 0;
        if (epoch > 0) {
            syncTime(epoch);
            setTimeOffsetMinutes(tzOffsetMin);
            saveTimeToRtc(epoch);
            savePersistentState();
            publishState();
        } else {
            Serial.println("Invalid epoch");
        }
    } else if (equalsIgnoreCase(cmd, "setDeviceMode")) {
        const char *zoneKey = doc["zone"];
        const char *modeStr = doc["mode"];
        Zone zone;
        if (!parseZoneKey(zoneKey, zone) || !modeStr) {
            Serial.println("Invalid device mode payload");
            return;
        }
        DeviceMode mode;
        if (!parseDeviceMode(modeStr, mode)) {
            Serial.println("Unknown device mode");
            return;
        }
        deviceModes[zone] = mode;
        Serial.printf("Mode for %s set to %s\n", zoneKey, modeStr);
        savePersistentState();
        publishDeviceMode(zone);
    } else if (equalsIgnoreCase(cmd, "setLogicRules")) {
        const char *zoneKey = doc["zone"];
        Zone zone;
        if (!parseZoneKey(zoneKey, zone)) {
            Serial.println("Invalid zone in setLogicRules");
            return;
        }
        JsonArrayConst rulesArr = doc["rules"].as<JsonArrayConst>();
        if (rulesArr.isNull()) {
            Serial.println("Missing rules array");
            return;
        }
        std::vector<LogicRule> newRules;
        for (JsonVariantConst item : rulesArr) {
            LogicRule rule;
            if (parseLogicRule(item, rule)) {
                newRules.push_back(rule);
            }
        }
        deviceLogic[zone] = std::move(newRules);
        Serial.printf("Loaded %u logic rules for %s\n", static_cast<unsigned>(deviceLogic[zone].size()), zoneKey);
        savePersistentState();
        publishLogicRules(zone);
    } else if (equalsIgnoreCase(cmd, "setSensor")) {
        const char *name = doc["name"];
        float value = doc["value"] | NAN;
        if (!name || std::isnan(value)) {
            Serial.println("Invalid sensor payload");
            return;
        }
        sensorReadings[name] = value;
        Serial.printf("Sensor %s updated to %.2f\n", name, value);
    } else if (equalsIgnoreCase(cmd, "syncState")) {
        publishState();
        publishAllSchedules();
        publishAllDeviceModes();
        publishAllLogicRules();
    } else if (equalsIgnoreCase(cmd, "ping")) {
        publishPong();
    } else {
        Serial.print("Unknown command: ");
        Serial.println(cmd);
    }
}

static bool parseDaysMask(const JsonVariantConst &value, uint8_t &mask) {
    mask = 0;
    if (value.is<const char *>()) {
        const char *text = value.as<const char *>();
        if (text && equalsIgnoreCase(text, "daily")) {
            mask = 0x7F;
            return true;
        }
    }
    if (!value.is<JsonArrayConst>()) {
        return false;
    }
    for (JsonVariantConst item : value.as<JsonArrayConst>()) {
        const char *name = item.as<const char *>();
        if (!name) {
            continue;
        }
        for (size_t i = 0; i < 7; ++i) {
            if (equalsIgnoreCase(name, DAY_NAMES[i])) {
                mask |= (1 << i);
                break;
            }
        }
    }
    return mask != 0;
}

static void daysMaskToJson(uint8_t mask, JsonArray &out) {
    if (mask == 0) {
        return;
    }
    for (size_t i = 0; i < 7; ++i) {
        if (mask & (1 << i)) {
            out.add(DAY_NAMES[i]);
        }
    }
}

static bool manualOverrideActive() {
    for (size_t i = 0; i < Z_COUNT; ++i) {
        if (zoneStates[i].manual && zoneStates[i].on) {
            return true;
        }
    }
    return false;
}

static float getSensorValue(const String &name, bool &valid) {
    auto it = sensorReadings.find(name.c_str());
    if (it == sensorReadings.end()) {
        valid = false;
        return NAN;
    }
    valid = true;
    return it->second;
}

static bool evaluateRule(const LogicRule &rule, float value) {
    switch (rule.op) {
    case OP_GT: return value > rule.onThreshold;
    case OP_LT: return value < rule.onThreshold;
    case OP_GTE: return value >= rule.onThreshold;
    case OP_LTE: return value <= rule.onThreshold;
    default: return false;
    }
}

static LogicResult evaluateLogic(size_t zoneIndex) {
    LogicResult result;
    if (zoneIndex >= deviceLogic.size()) {
        return result;
    }
    const auto &rules = deviceLogic[zoneIndex];
    for (auto &rule : const_cast<std::vector<LogicRule>&>(rules)) {
        bool valid = false;
        float value = getSensorValue(rule.sensor, valid);
        if (!valid || std::isnan(rule.onThreshold)) {
            continue;
        }

        bool match = false;
        // Hysteresis: use offThreshold if provided to release the latch.
        if (!std::isnan(rule.offThreshold)) {
            if (!rule.active) {
                match = evaluateRule(rule, value);
                if (match) {
                    rule.active = true;
                }
            } else {
                // Currently active; check if we should clear it.
                switch (rule.op) {
                case OP_GT:  rule.active = value > rule.offThreshold; break;
                case OP_GTE: rule.active = value >= rule.offThreshold; break;
                case OP_LT:  rule.active = value < rule.offThreshold; break;
                case OP_LTE: rule.active = value <= rule.offThreshold; break;
                default: break;
                }
                match = rule.active;
            }
        } else {
            match = evaluateRule(rule, value);
            rule.active = match;
        }

        if (match) {
            if (rule.type == RULE_FORCE_ON) {
                result.forceOn = true;
            } else if (rule.type == RULE_INHIBIT_ON) {
                result.inhibitOn = true;
            }
        }
    }
    return result;
}

static bool parseDeviceMode(const char *text, DeviceMode &outMode) {
    if (!text) return false;
    if (equalsIgnoreCase(text, "SCHEDULE")) {
        outMode = DEVICE_MODE_SCHEDULE;
        return true;
    }
    if (equalsIgnoreCase(text, "SCHEDULE_WITH_LOGIC")) {
        outMode = DEVICE_MODE_SCHEDULE_WITH_LOGIC;
        return true;
    }
    if (equalsIgnoreCase(text, "LOGIC_ONLY")) {
        outMode = DEVICE_MODE_LOGIC_ONLY;
        return true;
    }
    return false;
}

static bool parseLogicRule(const JsonVariantConst &obj, LogicRule &outRule) {
    if (!obj.is<JsonObjectConst>()) return false;
    const char *type = obj["type"];
    const char *op = obj["op"];
    const char *sensor = obj["sensor"];
    if (!type || !op || !sensor) return false;

    if (equalsIgnoreCase(type, "INHIBIT_ON")) {
        outRule.type = RULE_INHIBIT_ON;
    } else if (equalsIgnoreCase(type, "FORCE_ON")) {
        outRule.type = RULE_FORCE_ON;
    } else {
        return false;
    }

    if (equalsIgnoreCase(op, ">")) {
        outRule.op = OP_GT;
    } else if (equalsIgnoreCase(op, "<")) {
        outRule.op = OP_LT;
    } else if (equalsIgnoreCase(op, ">=")) {
        outRule.op = OP_GTE;
    } else if (equalsIgnoreCase(op, "<=")) {
        outRule.op = OP_LTE;
    } else {
        return false;
    }

    outRule.sensor = sensor;
    outRule.onThreshold = obj["on"] | NAN;
    outRule.offThreshold = obj["off"] | NAN;
    outRule.active = false;
    return !std::isnan(outRule.onThreshold);
}

static void pollZoneButtons(unsigned long now) {
    for (size_t i = 0; i < Z_COUNT; ++i) {
        ZoneState &state = zoneStates[i];
        bool reading = digitalRead(ZONE_BUTTON_PINS[i]);
        if (reading != state.buttonLastReading) {
            state.buttonLastReading = reading;
            state.lastButtonChangeMs = now;
        }
        if ((now - state.lastButtonChangeMs) >= BTN_DEBOUNCE_MS) {
            if (reading != state.buttonStableState) {
                state.buttonStableState = reading;
                if (!reading) { // active low
                    bool turnOn = !state.on;
                    Serial.printf("Button toggle for %s -> %s\n", ZONE_KEYS[i], turnOn ? "ON" : "OFF");
                    setZone(static_cast<Zone>(i), turnOn, 0, true);
                }
            }
        }
    }
}

static time_t currentEpoch() {
    if (!timeSynced) {
        return 0;
    }
    unsigned long elapsed = millis() - timeSyncMillis;
    return static_cast<time_t>(timeSyncEpoch + (elapsed / 1000UL));
}

static void syncTime(uint32_t epochSeconds) {
    timeSyncEpoch = epochSeconds;
    timeSyncMillis = millis();
    timeSynced = true;
    lastSchedulerMinute = -1;
    for (auto &entry : schedules) {
        entry.lastRunYDay = -1;
    }
    Serial.print("Time synced: ");
    Serial.println(epochSeconds);
}

static void setTimeOffsetMinutes(int32_t minutes) {
    timeOffsetSeconds = minutes * 60;
    Serial.print("Time offset minutes set to ");
    Serial.println(minutes);
}

static bool addOrUpdateSchedule(const ScheduleEntry &entry) {
    for (auto &existing : schedules) {
        if (existing.id == entry.id) {
            existing = entry;
            existing.lastRunYDay = -1;
            return true;
        }
    }
    if (schedules.size() >= 16) {
        Serial.println("Schedule capacity reached");
        return false;
    }
    schedules.push_back(entry);
    return true;
}

static bool removeScheduleById(const String &id) {
    for (auto it = schedules.begin(); it != schedules.end(); ++it) {
        if (it->id == id) {
            schedules.erase(it);
            return true;
        }
    }
    return false;
}

static void evaluateSchedules() {
    if (currentMode != MODE_AUTO || !timeSynced || schedules.empty()) {
        return;
    }
    time_t now = currentEpoch();
    if (now <= 0) {
        return;
    }
    // Apply timezone offset so schedules run in local time.
    time_t localNow = now + timeOffsetSeconds;
    struct tm timeinfo;
#if defined(__XTENSA__) || defined(ESP_PLATFORM)
    gmtime_r(&localNow, &timeinfo);
#else
    gmtime_r(&localNow, &timeinfo);
#endif
    int minuteOfDay = timeinfo.tm_hour * 60 + timeinfo.tm_min;
    if (minuteOfDay == lastSchedulerMinute) {
        return;
    }
    lastSchedulerMinute = minuteOfDay;
    uint8_t dayBit = 1 << timeinfo.tm_wday;
    for (auto &entry : schedules) {
        if (!(entry.daysMask & dayBit)) {
            continue;
        }
        if (entry.hour == timeinfo.tm_hour && entry.minute == timeinfo.tm_min) {
            if (entry.lastRunYDay != timeinfo.tm_yday) {
                Serial.printf("Schedule %s running %s\n", entry.id.c_str(), ZONE_KEYS[entry.zone]);
                entry.lastRunYDay = timeinfo.tm_yday;
                setZone(entry.zone, true, entry.durationSeconds, false);
            }
        } else if (entry.lastRunYDay != timeinfo.tm_yday &&
                   (entry.hour * 60 + entry.minute) > minuteOfDay) {
            // future schedule for today; nothing to do
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {
        delay(10);
    }
    Serial.println();
    Serial.println("Greenhouse controller firmware booting");

    if (preferences.begin(PERSIST_NAMESPACE, false)) {
        preferencesReady = true;
        loadPersistentState();
    } else {
        Serial.println("Failed to initialize NVS preferences");
    }

    unsigned long now = millis();
    for (size_t i = 0; i < Z_COUNT; ++i) {
        pinMode(ZONE_OUTPUT_PINS[i], OUTPUT);
        digitalWriteActive(ZONE_OUTPUT_PINS[i], false);
        pinMode(ZONE_BUTTON_PINS[i], INPUT_PULLUP);
        bool reading = digitalRead(ZONE_BUTTON_PINS[i]);
        zoneStates[i].buttonStableState = reading;
        zoneStates[i].buttonLastReading = reading;
        zoneStates[i].lastButtonChangeMs = now;
    }

    dht.begin();
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    initRtc();
    displayReady = display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDRESS);
    if (displayReady) {
        display.setRotation(2); // 180-degree flip for upside-down mounted OLED
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("Greenhouse");
        display.println("Display ready");
        display.display();
        lastDisplayMs = millis();
    } else {
        Serial.println("SSD1306 not detected");
    }

    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    // Increase MTU/data length so state JSON notifications are not truncated.
    NimBLEDevice::setMTU(185);
    bleServer = NimBLEDevice::createServer();
    bleServer->setCallbacks(&serverCallbacks);

    NimBLEService *service = bleServer->createService(SERVICE_UUID);
    txCharacteristic = service->createCharacteristic(TX_UUID, NIMBLE_PROPERTY::NOTIFY);
    rxCharacteristic = service->createCharacteristic(RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rxCharacteristic->setCallbacks(&commandCallbacks);

    service->start();

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    NimBLEDevice::startAdvertising();

    Serial.print("BLE advertising as ");
    Serial.println(BLE_DEVICE_NAME);

    publishState();
}

void loop() {
    unsigned long now = millis();
    pollSensors(now);
    for (size_t i = 0; i < Z_COUNT; ++i) {
        uint32_t deadline = zoneStates[i].safetyUntilMs;
        if (deadline > 0 && static_cast<int32_t>(deadline - now) <= 0) {
            Serial.printf("Safety timer expired for %s\n", ZONE_KEYS[i]);
            setZone(static_cast<Zone>(i), false);
        }
    }

    evaluateSchedules();

    pollZoneButtons(now);

    applyModeAndPriority(now);

    delay(10);
}

static void applyModeAndPriority(unsigned long now) {
    (void)now;
    for (size_t i = 0; i < Z_COUNT; ++i) {
        const auto mode = deviceModes[i];
        // Manual overrides always win; skip logic when manual and ON.
        if (zoneStates[i].manual && zoneStates[i].on) {
            continue;
        }

        const bool scheduleIntent = zoneStates[i].on && !zoneStates[i].manual;
        const auto logic = evaluateLogic(i);

        bool desired = zoneStates[i].on;
        switch (mode) {
        case DEVICE_MODE_SCHEDULE:
            desired = scheduleIntent;
            break;
        case DEVICE_MODE_SCHEDULE_WITH_LOGIC:
            if (logic.forceOn) {
                desired = true;
            } else if (scheduleIntent && !logic.inhibitOn) {
                desired = true;
            } else {
                desired = false;
            }
            break;
        case DEVICE_MODE_LOGIC_ONLY:
            desired = logic.forceOn;
            break;
        default:
            desired = scheduleIntent;
            break;
        }

        if (desired != zoneStates[i].on) {
            // For logic-driven ON without a schedule intent, bound the run time.
            uint32_t duration = (desired && !scheduleIntent) ? DEFAULT_MANUAL_SECS : 0;
            setZone(static_cast<Zone>(i), desired, duration, false);
        }
    }
}

static bool valueChanged(float previous, float current, float delta) {
    if (std::isnan(previous) && std::isnan(current)) {
        return false;
    }
    if (std::isnan(previous) != std::isnan(current)) {
        return true;
    }
    return std::fabs(previous - current) >= delta;
}

static void pollClimateSensors(unsigned long now, bool &shouldPublish) {
    if (now - lastClimatePollMs < CLIMATE_POLL_MS) {
        return;
    }
    lastClimatePollMs = now;

    const float humidity = dht.readHumidity();
    const float tempF = dht.readTemperature(true); // true = Fahrenheit

    bool changed = false;
    if (!std::isnan(humidity)) {
        changed |= valueChanged(lastHumidity, humidity, CLIMATE_DELTA_MIN);
        lastHumidity = humidity;
        sensorReadings["humidity"] = humidity;
    }
    if (!std::isnan(tempF)) {
        changed |= valueChanged(lastTempF, tempF, CLIMATE_DELTA_MIN);
        lastTempF = tempF;
        sensorReadings["temp"] = tempF;
    }

    shouldPublish |= changed;
}

static void pollSoilSensors(unsigned long now, bool &shouldPublish) {
    if (now - lastSoilPollMs < SOIL_POLL_MS) {
        return;
    }
    lastSoilPollMs = now;

    // Raw ADC values (0-4095 on ESP32). These are useful for relative comparisons and logic rules.
    if (PIN_SOIL1 != 0xFF) {
        float soil1 = static_cast<float>(analogRead(PIN_SOIL1));
        bool changed = valueChanged(lastSoil1, soil1, SOIL_DELTA_MIN);
        lastSoil1 = soil1;
        sensorReadings["soil1"] = soil1;
        shouldPublish |= changed;
    }
    if (PIN_SOIL2 != 0xFF) {
        float soil2 = static_cast<float>(analogRead(PIN_SOIL2));
        bool changed = valueChanged(lastSoil2, soil2, SOIL_DELTA_MIN);
        lastSoil2 = soil2;
        sensorReadings["soil2"] = soil2;
        shouldPublish |= changed;
    }
}

static void pollSensors(unsigned long now) {
    bool shouldPublish = false;
    pollClimateSensors(now, shouldPublish);
    pollSoilSensors(now, shouldPublish);

    if (shouldPublish && (lastSensorPublishMs == 0 || (now - lastSensorPublishMs) >= SENSOR_PUBLISH_MIN_MS)) {
        publishState(); // include latest sensorReadings in the compact state payload
        lastSensorPublishMs = now;
    }

    if (shouldPublish || (displayReady && (now - lastDisplayMs) >= DISPLAY_REFRESH_MS)) {
        updateDisplay(shouldPublish);
        lastDisplayMs = now;
    }
}

static void updateDisplay(bool force) {
    (void)force;
    if (!displayReady) {
        return;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    display.print("Temp: ");
    if (std::isnan(lastTempF)) {
        display.print("--.-");
    } else {
        display.print(lastTempF, 1);
    }
    display.println(" F");

    display.print("Hum: ");
    if (std::isnan(lastHumidity)) {
        display.print("--.-");
    } else {
        display.print(lastHumidity, 1);
    }
    display.println(" %");

    display.print("Soil1: ");
    if (std::isnan(lastSoil1)) {
        display.println("--");
    } else {
        display.println(static_cast<int>(lastSoil1));
    }

    display.print("Soil2: ");
    if (std::isnan(lastSoil2)) {
        display.println("--");
    } else {
        display.println(static_cast<int>(lastSoil2));
    }

    display.print("Mode: ");
    display.println(modeToString(currentMode));

    display.display();
}
