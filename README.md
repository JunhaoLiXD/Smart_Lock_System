# Smart Lock System — Dual-ESP32 Bike Anti-Theft

A bicycle anti-theft system built on two ESP32 boards. The **lock node** detects when the lock
cable is cut and reports it over Bluetooth; the **bike node** senses tampering with an
accelerometer, reads GPS, and syncs status to Firebase over Wi-Fi. A Firebase Cloud Function sends a
Pushover push notification when an alarm fires, and a web dashboard shows live status and location.

Team project (Nov–Dec 2025); I was the project lead and wrote the firmware for both nodes, the Cloud Function, and the dashboard.

---

## System Overview

```text
┌──────────────── Lock node (Lock.ino) ────────────────┐
│ Cable continuity on GPIO32 (internal pull-down)      │
│ Debounced HIGH→LOW = "Cable Cut Detected"            │
└───────────────┬──────────────────────────────────────┘
                │ Bluetooth Classic (SPP)
                ▼
┌──────────────── Bike node (Bike.ino) ────────────────┐
│ Receives cut / reconnect messages from the lock      │
│ ADXL345 accelerometer → tamper (motion) detection    │
│ GPS module → latitude / longitude                    │
│ LED + buzzer local alarm                             │
│ Wi-Fi → Firebase Realtime Database every 5 s         │
└───────────────┬──────────────────────────────────────┘
                │ writes /bike_status and /device/001/alarm
                ▼
┌──────── Firebase ────────┐        ┌──── webpage.html ────┐
│ Cloud Function (index.js)│        │ Live dashboard: alarm│
│ on /device/{id}/alarm =  │        │ state, accel, GPS on │
│ true → Pushover alert    │        │ a Leaflet map        │
└──────────────────────────┘        └──────────────────────┘
```

---

## How It Works

### Lock node — cable-cut detection (`Lock.ino`)

- The lock cable closes a circuit into **GPIO32**, configured as `INPUT_PULLDOWN`. An intact cable
  reads `HIGH`; a cut cable reads `LOW`.
- State changes are debounced (50 ms). On a cut, the node lights the on-board LED and sends
  `Cable Cut Detected` over Bluetooth; on reconnection it sends `Cable Reconnected`.
- The node advertises as the Bluetooth SPP slave `BikeLock`.

### Bike node — tamper sensing, GPS, and cloud sync (`Bike.ino`)

- **Bluetooth:** connects as the SPP master to the lock node's MAC address and listens for the
  cut/reconnect messages. A cut triggers the alarm and sets the `isCut` flag.
- **Tamper detection (ADXL345 over I²C, SDA 21 / SCL 22):**
  - At startup, it averages 100 samples while the bike is still to compute a gravity offset.
  - It samples every 500 ms and computes the deviation of the acceleration magnitude from 1 g.
  - If the deviation exceeds **1.0 m/s²** at least **2 times within a 3 s window**, the alarm fires.
- **Local alarm:** the LED (GPIO26) and active buzzer (GPIO25) stay on for 5 s.
- **GPS:** reads NMEA from a GPS module on UART2 (RX 4 / TX 5, 9600 baud) with TinyGPSPlus. If there
  is no fix, it falls back to the last known location stored in Firebase.
- **Cloud sync:** every 5 s it uploads a JSON snapshot to `/bike_status` (alarm, accel x/y/z,
  battery, lat, lon, NTP Unix timestamp). If the alarm is active or the cable is cut, it also sets
  `/device/001/alarm = true`. Resetting the alarm writes `false`.

### Notifications (`index.js`)

A Firebase Cloud Function (2nd gen, `onValueWritten`) watches `/device/{id}/alarm`. When the value
becomes `true`, it sends a high-priority Pushover notification titled *Bike Anti-Theft Alert*.

### Dashboard (`webpage.html`)

A single-page monitor (Tailwind CSS + Leaflet) subscribes to `/bike_status` with the Firebase JS SDK
and shows the alarm state, last update time, accelerometer readings, battery, and the bike's GPS
position on an OpenStreetMap map.

---

## Firebase Data Layout

```text
/bike_status
    alarm      : bool
    accel_x    : float   (m/s²)
    accel_y    : float
    accel_z    : float
    battery    : float   (placeholder value in the current firmware)
    lat, lon   : float
    timestamp  : int     (Unix seconds, NTP-synced)

/device/001/alarm : bool   ← triggers the Cloud Function
```

---

## Hardware

| Node | Components |
|---|---|
| Lock | ESP32, lock cable wired as a continuity loop to GPIO32, status LED (GPIO2) |
| Bike | ESP32, ADXL345 accelerometer (I²C), GPS module (UART2), LED (GPIO26), active buzzer (GPIO25) |

---

## Repository Structure

```text
.
├── Lock.ino       # lock-node firmware: cable continuity → Bluetooth message
├── Bike.ino       # bike-node firmware: Bluetooth, ADXL345, GPS, alarm, Firebase upload
├── index.js       # Firebase Cloud Function: alarm flag → Pushover notification
└── webpage.html   # live web dashboard (Firebase JS SDK + Leaflet map)
```

---

## Setup

1. **Arduino libraries (bike node):** `Adafruit ADXL345`, `Adafruit Unified Sensor`,
   `Firebase ESP Client` (mobizt), `TinyGPSPlus`. `BluetoothSerial` and `WiFi` ship with the ESP32
   Arduino core.
2. **Flash the lock node** with `Lock.ino` and note the Bluetooth MAC address it prints on serial.
3. **Configure the bike node** by replacing the placeholders in `Bike.ino`: `lockAddress` (the lock's
   MAC), `WIFI_SSID` / `WIFI_PASSWORD`, `API_KEY` / `DATABASE_URL`. Enable **Anonymous** sign-in in
   Firebase Authentication. Then flash `Bike.ino`.
4. **Deploy the Cloud Function:** put `index.js` in a Firebase Functions project (with `axios`), set
   your Pushover `userKey` and `appToken`, and run `firebase deploy --only functions`.
5. **Open the dashboard:** point `firebaseConfig` in `webpage.html` at your own Firebase project and
   open it in a browser.

The Wi-Fi, Firebase API key, and Pushover credentials in the firmware and Cloud Function are
placeholders. Do not commit real keys.
