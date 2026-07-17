# ESP32 Smart Fan Controller

A WiFi-enabled fan controller built on ESP32 that provides a web-based control
UI (reachable at **http://fan.local**), PWM speed control for four 4-pin fans,
real-time RPM monitoring, temperature-based automation, and over-the-air
firmware updates from the browser.

![ESP32 Fan Controller](https://img.shields.io/badge/ESP32-Fan%20Controller-blue) ![PlatformIO](https://img.shields.io/badge/PlatformIO-esp32dev-orange) ![License](https://img.shields.io/badge/License-MIT-green)

## ✨ Features

- 🌐 **Web control UI** — dark, responsive, reachable at `http://fan.local` (mDNS)
- ⚡ **PWM speed control** — independent intake / exhaust duty (0–255)
- 📊 **Real-time RPM** — live tachometer readings per fan
- 🌡️ **Temperature automation** — auto mode drives speed from a DHT22 curve
- 🔄 **Browser OTA** — upload new firmware from the UI, no Arduino IDE needed
- 💾 **Persistent settings** — intake / exhaust / auto survive reboots (NVS)
- 🔒 **Secrets kept separate** — credentials live in a gitignored header

## 🛠️ Hardware

### Components
- **ESP32 Development Board** (tested with ESP32 DevKitV1)
- **4-pin PWM Fans** (tested with Noctua NF-F12)
- **DHT22 Temperature Sensor**
- **12V Power Supply** (for fans)

### Wiring

| Component | ESP32 Pin | Notes |
|-----------|-----------|-------|
| Intake Fan 1 PWM  | D25 | PWM signal |
| Intake Fan 1 Tach | D32 | RPM feedback |
| Intake Fan 2 PWM  | D26 | PWM signal |
| Intake Fan 2 Tach | D33 | RPM feedback |
| Exhaust Fan 1 PWM  | D27 | PWM signal |
| Exhaust Fan 1 Tach | D12 | RPM feedback (GPIO12 is a boot strapping pin — see `main.cpp`) |
| Exhaust Fan 2 PWM  | D14 | PWM signal |
| Exhaust Fan 2 Tach | D13 | RPM feedback |
| DHT22 Data | D4 | Temperature sensor |
| DHT22 VCC / GND | 3.3V / GND | Power |
| Fan VCC / GND | External 12V / GND | Common ground with ESP32 |

> ⚠️ Noctua fans use non-standard wire colors: Blue = PWM, Green = Tach.

All pin assignments live in [`include/config.h`](include/config.h).

## 📋 Software

Built with **PlatformIO** (ESP32 Arduino core 3.x). Dependencies are declared in
[`platformio.ini`](platformio.ini) and fetched automatically:

- `adafruit/DHT sensor library`
- `adafruit/Adafruit Unified Sensor`

Project layout (mirrors the companion 7-segment clock project):

```
platformio.ini
include/config.h        # hardware config (pins, PWM, temp curve)
include/webpage.h       # the web UI (served from flash)
include/secrets.h       # WiFi + OTA credentials (gitignored)
src/main.cpp            # firmware
```

## 🚀 Build & flash

1. **Configure credentials** — copy the template and edit it:
   ```
   cp include/secrets.h.example include/secrets.h
   # then set WIFI_SSID / WIFI_PASSWORD / OTA_PASSWORD
   ```
2. **Build**:
   ```
   pio run -e esp32dev
   ```
3. **Flash over USB** (first time):
   ```
   pio run -e esp32dev -t upload
   ```
   If the wrong serial port is picked, set `upload_port` / `monitor_port` in
   `platformio.ini`.
4. **Find the device** — open the serial monitor (115200 baud); it prints
   `mDNS: http://fan.local/` once WiFi connects. Browse to
   **http://fan.local**.

## 📱 Usage

The web UI provides:
- **Auto mode** — Off / On. When on, both banks follow the temperature curve and
  the sliders are disabled.
- **Intake / Exhaust speed** — 0–255 sliders with live RPM readouts.
- **Firmware update** — upload a `.bin` (see OTA below).

### Temperature curve (auto mode)

| Temperature | PWM |
|-------------|-----|
| < 25 °C | 50  |
| 25–30 °C | 100 |
| 30–35 °C | 180 |
| ≥ 35 °C | 255 |

Thresholds are in [`include/config.h`](include/config.h).

## 🔄 OTA updates (browser push)

After the first USB flash, update wirelessly — no Arduino IDE:

- **From the UI**: build (`pio run`), open the **Firmware update** card, choose
  `.pio/build/esp32dev/firmware.bin`, click Upload, and enter the OTA password
  (`OTA_PASSWORD` in `secrets.h`, username `admin`). The device flashes and
  reboots.
- **From the CLI**:
  ```
  curl.exe -u admin:YOUR_OTA_PASSWORD \
       -F "firmware=@.pio/build/esp32dev/firmware.bin" \
       http://fan.local/update
  ```

The upload is an outbound HTTP POST from your machine, so no PC-firewall rules
are needed.

## 🐛 Troubleshooting

- **Fan not responding to PWM** — confirm it's a 4-pin fan on the right pin; try
  `PWM_FREQ` 1000 instead of 25000 in `config.h`; check the 12V supply.
- **No RPM** — check the tach wire; exhaust fan 1 (D34) is intentionally not read
  (input-only pin) and always shows `--`.
- **`fan.local` won't resolve** — ensure your OS supports mDNS/Bonjour; fall back
  to the IP printed on the serial monitor, or `ping fan.local`.
- **OTA fails** — verify the OTA password matches `secrets.h`; make sure the
  device and your computer are on the same network.

## 📄 License

MIT — see [LICENSE](LICENSE).
