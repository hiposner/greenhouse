#include <Arduino.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <ctype.h>
#include <time.h>
#include <string>
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

static std::vector<ScheduleEntry> schedules;
static bool timeSynced = false;
static int64_t timeSyncEpoch = 0;
static unsigned long timeSyncMillis = 0;
static int lastSchedulerMinute = -1;

static const NimBLEUUID SERVICE_UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static const NimBLEUUID RX_UUID("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
static const NimBLEUUID TX_UUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e");

// Forward declarations
static void publishState();
static void publishScheduleEntry(const ScheduleEntry &entry);
static void publishScheduleDelete(const String &id);
static void publishAllSchedules();
static void publishPong();
static void handleBleCommand(const std::string &payload);
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

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
        (void)server;
        (void)connInfo;
        bleConnected = true;
        publishState();
        publishAllSchedules();
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
    StaticJsonDocument<256> doc;
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

    JsonArray states = doc.createNestedArray("z"); // zone states
    JsonArray remaining = doc.createNestedArray("r");
    JsonArray overrides = doc.createNestedArray("v");
    for (size_t i = 0; i < Z_COUNT; ++i) {
        states.add(zoneStates[i].on ? 1 : 0);
        if (zoneStates[i].on && zoneStates[i].safetyUntilMs > nowMs) {
            remaining.add((zoneStates[i].safetyUntilMs - nowMs) / 1000UL);
        } else {
            remaining.add(0);
        }
        overrides.add(zoneStates[i].manual ? 1 : 0);
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
    StaticJsonDocument<64> doc;
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
    StaticJsonDocument<256> doc;
    doc["e"] = "sc"; // schedule create/update
    doc["i"] = entry.id;
    doc["z"] = ZONE_KEYS[entry.zone];
    doc["h"] = entry.hour;
    doc["m"] = entry.minute;
    doc["d"] = entry.durationSeconds;
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
    StaticJsonDocument<128> doc;
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
            publishState();
        }
    } else if (equalsIgnoreCase(cmd, "setSchedule")) {
        const char *idStr = doc["id"];
        const char *zoneKey = doc["zone"];
        int hour = doc["hour"] | -1;
        int minute = doc["minute"] | -1;
        uint32_t durationSeconds = doc["duration_s"] | 0;
        JsonVariantConst daysVar = doc["days"];

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
            publishState();
            publishScheduleDelete(String(idStr));
        }
    } else if (equalsIgnoreCase(cmd, "setTime")) {
        uint32_t epoch = doc["epoch"] | 0;
        int32_t tzOffsetMin = doc["tzOffsetMin"] | 0;
        if (epoch > 0) {
            syncTime(epoch);
            setTimeOffsetMinutes(tzOffsetMin);
            publishState();
        } else {
            Serial.println("Invalid epoch");
        }
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
    for (size_t i = 0; i < Z_COUNT; ++i) {
        uint32_t deadline = zoneStates[i].safetyUntilMs;
        if (deadline > 0 && static_cast<int32_t>(deadline - now) <= 0) {
            Serial.printf("Safety timer expired for %s\n", ZONE_KEYS[i]);
            setZone(static_cast<Zone>(i), false);
        }
    }

    evaluateSchedules();

    pollZoneButtons(now);

    delay(10);
}
