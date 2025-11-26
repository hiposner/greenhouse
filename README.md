# Greenhouse Controller

Complete greenhouse controller reference project that pairs an ESP32 firmware (PlatformIO / Arduino) with a lightweight progressive web app for Bluetooth control.

```
greenhouse-controller/
|-- firmware/                 # PlatformIO project (ESP32, Arduino)
|   |-- platformio.ini
|   |-- include/config.h
|   |-- src/main.cpp
|   `-- data/
|       |-- settings.json
|       `-- schedules.json
`-- app/                      # Static PWA (Web Bluetooth dashboard)
    |-- public/index.html
    |-- public/manifest.json
    `-- package.json
```

## Hardware Wiring

ESP32 DOIT DevKit v1 (USB jack up orientation). All driver outputs are active-low and all buttons use `INPUT_PULLUP`.

| Function            | GPIO | Notes                              |
|---------------------|------|------------------------------------|
| Line 1 valve out    | 4    | MOSFET/relay input (LOW = ON)      |
| Line 2 valve out    | 5    | MOSFET/relay input (LOW = ON)      |
| Line 3 valve out    | 18   | MOSFET/relay input (LOW = ON)      |
| Mister output       | 19   | MOSFET/relay input (LOW = ON)      |
| Fan output          | 16   | Active-low relay / RX2 pin         |
| Grow lights output  | 17   | Active-low relay / TX2 pin         |
| Line 1 button       | 25   | Normally-open to GND, pull-up      |
| Line 2 button       | 26   | Normally-open to GND, pull-up      |
| Line 3 button       | 27   | Normally-open to GND, pull-up      |
| Mister button       | 14   | Normally-open to GND, pull-up      |
| Fan button          | 32   | Normally-open to GND, pull-up      |
| Lights button       | 33   | Normally-open to GND, pull-up      |
| Soil sensor 1       | 34   | Capacitive probe (ADC1)            |
| Soil sensor 2       | 35   | Capacitive probe (ADC1)            |
| DHT22 data          | 23   | Sensor data line                   |
| I2C SDA / SCL       | 21/22| OLED or RTC (future use)           |

## Firmware (PlatformIO)

1. Install [PlatformIO](https://platformio.org/).
2. From the repository root run:
   ```
   cd firmware
   pio run -t build
   pio run -t upload
   ```
3. Serial monitor at `115200` baud.

Key behaviour:
- Six zones: three irrigation lines (Red/Green/Yellow for Lines 1/2/3), mister (Blue), fan, and grow lights.
- Active-low outputs (`LOW` energises the driver).
- Manual buttons run solenoids for 15 minutes (fan/lights latch until pressed again) with 60 ms debounce.
- Interlock ensures only one irrigation line (Line1/Line2/Line3) is active at a time.
- Built-in scheduler per zone with phone-driven time sync (BLE `setTime` command).
- BLE service `6e400001-b5a3-f393-e0a9-e50e24dcca9e` (Nordic UART profile).
- JSON commands/events (examples below):
  - `{"cmd":"setValve","zone":"line1","state":"ON","duration_s":300}`
  - `{"evt":"state","mode":"AUTO","zones":{"line1":"OFF",...}}`

Configuration defaults live in `include/config.h`; editable data stubs are in `data/settings.json` and `data/schedules.json`.

## Web App (PWA)

The PWA is a static site served from `app/public`. It works offline via the manifest and runs entirely in-browser.

### Development server

```
cd app
npm run start
```

This launches `python3 -m http.server 5173 -d public` for quick testing. You can also open `public/index.html` directly without a server.

- Connect via Web Bluetooth (Chrome / Edge / Android).
- Dashboard shows each zone's state and a countdown or "latched" badge when it's running.
- Tabs let you switch between Status, Schedule, and Event Log views.
- Scheduler UI to add/delete per-zone schedules (days, time, duration).
- Mode selector for Auto vs Off (manual overrides are detected automatically).
- Offline-capable PWA with a service worker (`app/public/sw.js`) so you can "Install app" on each phone and run it without a constant server.

### Install the PWA

1. Host `app/public` once from any HTTPS origin (GitHub Pages, Netlify, etc.) and open it in Chrome/Edge on the phone.
2. After the service worker registers, use the browser menu -> **Install app** (or "Add to Home Screen").
3. From then on you can launch the installed shortcut offline inside the greenhouse; it loads from cache and immediately prompts for the BLE connection.

### Scheduler & Time Sync

- The PWA automatically calls `{"cmd":"setTime","epoch":...}` on connect so the ESP32 tracks real time using the phone/desktop clock.
- From the Schedule tab you can add/delete entries (zone, time, duration, days). Entries sync to the ESP32 and run even if the phone disconnects.
- Delete schedules with the in-app button; the controller stores up to 16 entries in memory.
- Manual controls still respect the irrigation interlock and safety timers even when scheduled runs are active.
- Modes:
  - **Auto** keeps schedules active; the UI flags manual overrides while physical buttons are running.
  - **Off** disables schedules so only manual inputs (physical buttons or BLE commands) can run equipment.
- The Events tab mirrors the live BLE log (state updates, pings, errors) so you can audit recent activity.

### Pairing (Android Chrome example)

1. Build and flash the firmware; ensure the ESP32 is powered.
2. On your phone, open Chrome and navigate to the hosted `index.html`.
3. Tap **Connect via Bluetooth**, select **GreenhouseControl**, and pair.
4. The dashboard will show the current state; buttons send BLE JSON commands.

## Acceptance Checklist

- [ ] PlatformIO build succeeds and advertises as `GreenhouseControl`.
- [ ] PWA loads, connects over Web Bluetooth, and shows live state.
- [ ] ON/OFF/Quick timer buttons send commands and toggle drivers.
- [ ] Manual buttons mirror the UI via BLE notifications.
- [ ] Safety timer (180 s default) turns zones off if no duration supplied.
- [ ] Interlock prevents more than one irrigation valve from running.
- [ ] `{"cmd":"ping"}` returns `{"evt":"pong"}`.
- [ ] Manual override buttons energise solenoids for 15 minutes and relays until toggled off.
- [ ] Scheduler entries created in the PWA sync to the ESP32 and execute at the programmed times.
