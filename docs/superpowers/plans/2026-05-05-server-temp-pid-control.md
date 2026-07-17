# ESP32 Fan Controller — Server-Temp PID Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add closed-loop fan speed control driven by CPU package temperatures scraped from a server's Prometheus metrics endpoint, with graceful fallback to the existing DHT22 ambient-air step curve when the metrics signal goes stale.

**Architecture:** Single-file Arduino sketch, additive changes only. Adds an HTTP scraper that polls `node_thermal_zone_temp_celsius{type="x86_pkg_temp"}` every 10 s, averages the two zones, and drives a small PI controller targeting a configurable setpoint (default 70 °C). Three operating modes (manual / auto-ambient / auto-server) with NVS-backed persistence. Stale-data detection (>60 s) automatically degrades to the ambient-curve fallback at a safety PWM floor.

**Tech Stack:** ESP32 Arduino Core ≥3.0, ESPAsyncWebServer, HTTPClient (ESP32 built-in), Preferences (NVS, ESP32 built-in), DHT22, ArduinoOTA. No new external libraries beyond what ships with the ESP32 core.

---

## File Structure

- **Modify:** `fan_controller_with_ota.ino` (single-file sketch — additive changes throughout)
  - New: `Preferences`-backed `Settings` struct (load on boot, save on change)
  - New: `fetchServerTemp()` HTTP scrape + `parsePrometheusBody()` line parser
  - New: `PI` controller struct and `pidStep()` function
  - New: `Mode` enum (`MODE_MANUAL`, `MODE_AUTO_AMBIENT`, `MODE_AUTO_SERVER`) replacing `bool autoMode`
  - New endpoints: `/status` (JSON), `/setpoint`, `/mode`, `/debug/server-temp`
  - Modified: HTML/JS in `/` route — adds setpoint input, 3-way mode selector, server-temp display, control-source badge, inline status messages instead of `alert()`
- **No new files.** Arduino IDE auto-concatenation makes multi-file sketches awkward; this stays single-file.

---

## Verification Strategy

This is hobby firmware on real hardware with no host-side test framework configured. The build host is a Linux server with no GUI — Arduino IDE is not used; everything is `arduino-cli`. Each task ends with **hardware-in-the-loop verification**: compile → OTA upload → exercise via browser / `curl` → observe Serial Monitor at 115200 baud. Steps explicitly state what command to run and what output to expect.

**Standard build / flash / monitor commands (used throughout):**

```bash
# Compile (subagents run this; user does not need to)
arduino-cli compile --fqbn esp32:esp32:esp32 /home/brian/esp32_fan_controller

# OTA upload (over-the-air; ESP32 must be on the network)
arduino-cli upload --fqbn esp32:esp32:esp32 \
  --port <esp32-ip>                          \
  --upload-field password=<OTA_PASSWORD>     \
  /home/brian/esp32_fan_controller

# Or, if espota is invoked directly:
~/.arduino15/packages/esp32/hardware/esp32/3.3.8/tools/espota.py \
  -i <esp32-ip> -p 3232 -a <OTA_PASSWORD> \
  -f /home/brian/esp32_fan_controller/build/esp32.esp32.esp32/esp32_fan_controller.ino.bin

# Serial monitor (USB serial — only useful if the board is also tethered;
# without USB tether on the build host, you'll need telnet logs or a
# secondary device. For now, rely on /status JSON for visible state.)
```

If serial output is unreachable from the build host (no USB tether), most verification falls back to `curl` against the HTTP endpoints. The Serial Monitor lines noted in each task are still emitted — they're useful when a USB cable is plugged in for debugging.

The Prometheus parser is factored as a pure function `parsePrometheusBody(const String&, float*)` so a host-side test can be added later if desired — out of scope for this plan.

---

## Pre-flight: branch + sanity

- [ ] **Step 0.1: Create feature branch**

```bash
git checkout -b feature/server-temp-pid
```

- [ ] **Step 0.2: Confirm metrics endpoint reachable from your laptop on the same VLAN as the ESP32**

```bash
curl -s http://192.168.0.110:9101/metrics | grep '^node_thermal_zone_temp_celsius'
```
Expected: at least two lines, e.g.
```
node_thermal_zone_temp_celsius{zone="0",type="x86_pkg_temp"} 50.000
node_thermal_zone_temp_celsius{zone="1",type="x86_pkg_temp"} 49.000
```
If this fails, stop — fix routing/firewall before continuing.

- [ ] **Step 0.3: Commit baseline**

```bash
git status   # should be clean
```
No commit needed; just confirming we're starting from a known state.

---

## Task 1: Add NVS-backed Settings struct

**Goal:** persist `intakePWM`, `exhaustPWM`, `mode`, `setpointC`, `pwmMin` across reboots. No behavior change yet beyond persistence.

**Files:**
- Modify: `fan_controller_with_ota.ino` — add `<Preferences.h>` include, `Settings` struct, `loadSettings()`/`saveSettings()`, replace globals.

- [ ] **Step 1.1: Add include and Settings struct (top of file, after existing includes)**

Add immediately after `#include "secrets.h"`:

```cpp
#include <Preferences.h>

Preferences prefs;

struct Settings {
  uint8_t intakePWM = 128;
  uint8_t exhaustPWM = 128;
  uint8_t mode       = 0;   // 0 = manual, 1 = auto-ambient, 2 = auto-server
  uint8_t setpointC  = 70;
  uint8_t pwmMin     = 60;
};
Settings settings;

void loadSettings() {
  prefs.begin("fanctl", true);  // read-only
  settings.intakePWM  = prefs.getUChar("ipwm", 128);
  settings.exhaustPWM = prefs.getUChar("epwm", 128);
  settings.mode       = prefs.getUChar("mode", 0);
  settings.setpointC  = prefs.getUChar("set",  70);
  settings.pwmMin     = prefs.getUChar("pmin", 60);
  prefs.end();
  Serial.printf("[settings] loaded ipwm=%u epwm=%u mode=%u set=%u pmin=%u\n",
                settings.intakePWM, settings.exhaustPWM, settings.mode,
                settings.setpointC, settings.pwmMin);
}

void saveSettings() {
  prefs.begin("fanctl", false);
  prefs.putUChar("ipwm", settings.intakePWM);
  prefs.putUChar("epwm", settings.exhaustPWM);
  prefs.putUChar("mode", settings.mode);
  prefs.putUChar("set",  settings.setpointC);
  prefs.putUChar("pmin", settings.pwmMin);
  prefs.end();
}
```

- [ ] **Step 1.2: Replace existing globals**

Delete the existing lines:
```cpp
int intakePWM = 128;
int exhaustPWM = 128;
bool autoMode = false;
```
We are replacing these with `settings.*` fields. The `autoMode` boolean becomes `settings.mode != 0` for now (Task 4 introduces the proper enum).

- [ ] **Step 1.3: Update all references**

Search-and-replace inside `fan_controller_with_ota.ino`:
- `intakePWM`  → `settings.intakePWM`
- `exhaustPWM` → `settings.exhaustPWM`
- `autoMode`   → `(settings.mode != 0)` for read sites
- For write sites (e.g. `autoMode = request->getParam("mode")->value() == "1";`), replace with:
  ```cpp
  settings.mode = (request->getParam("mode")->value() == "1") ? 1 : 0;
  saveSettings();
  ```
- For `intakePWM = val;` and `exhaustPWM = val;` in `/set`, append `saveSettings();`

- [ ] **Step 1.4: Call `loadSettings()` early in `setup()`**

In `setup()`, immediately after `Serial.println("\n=== ESP32 Fan Controller Starting ===");`, add:

```cpp
loadSettings();
```

- [ ] **Step 1.5: Build and OTA-flash**

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 /home/brian/esp32_fan_controller
arduino-cli upload --fqbn esp32:esp32:esp32 \
  --port <esp32-ip> \
  --upload-field password=<OTA_PASSWORD> \
  /home/brian/esp32_fan_controller
```
Expected: upload succeeds, board reboots. If a USB tether is available on the build host, Serial Monitor (115200 baud) shows:
```
[settings] loaded ipwm=128 epwm=128 mode=0 set=70 pmin=60
```
Without a tether, confirm the board reconnects to WiFi (it should respond at `http://<esp32-ip>/`).

- [ ] **Step 1.6: Verify persistence**

In a browser, hit `http://<esp32-ip>/set?fan=intake&val=200`. Then power-cycle the ESP32 (yank USB or kill 12 V). On reboot, Serial Monitor should show `ipwm=200`. Hit `/` and confirm the intake slider reads 200. (If the slider initial value is hardcoded to 128, that's a known UI gap — Task 7 fixes it.)

- [ ] **Step 1.7: Commit**

```bash
git add fan_controller_with_ota.ino
git commit -m "feat: persist fan controller settings to NVS"
```

---

## Task 2: Add `/status` JSON endpoint

**Goal:** consolidate status into one endpoint with one JSON shape. Existing `/temp` and `/rpm` stay for back-compat.

**Files:**
- Modify: `fan_controller_with_ota.ino` — register a new handler for `/status`.

- [ ] **Step 2.1: Add `/status` handler inside `setup()`**

Place this immediately after the existing `server.on("/rpm", ...)` block:

```cpp
server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
  String json = "{";
  json += "\"ambientC\":" + String(temperatureC, 1) + ",";
  json += "\"rpm\":[" + String(rpm1) + "," + String(rpm2) + "," + String(rpm3) + "," + String(rpm4) + "],";
  json += "\"intakePWM\":" + String(settings.intakePWM) + ",";
  json += "\"exhaustPWM\":" + String(settings.exhaustPWM) + ",";
  json += "\"mode\":" + String(settings.mode) + ",";
  json += "\"setpointC\":" + String(settings.setpointC) + ",";
  json += "\"pwmMin\":" + String(settings.pwmMin);
  json += "}";
  request->send(200, "application/json", json);
});
```

(Server-temp / control-source fields will be added in Tasks 3 and 6.)

- [ ] **Step 2.2: Build, OTA-flash, verify**

```bash
curl -s http://<esp32-ip>/status
```
Expected output (values may vary):
```
{"ambientC":22.4,"rpm":[850,860,0,855],"intakePWM":128,"exhaustPWM":128,"mode":0,"setpointC":70,"pwmMin":60}
```

- [ ] **Step 2.3: Commit**

```bash
git add fan_controller_with_ota.ino
git commit -m "feat: add /status JSON endpoint"
```

---

## Task 3: Scrape Prometheus metrics endpoint

**Goal:** pull the two `x86_pkg_temp` zone values from `http://192.168.0.110:9101/metrics`, average them, expose via `/debug/server-temp`. No control-loop wiring yet.

**Files:**
- Modify: `fan_controller_with_ota.ino` — add `<HTTPClient.h>`, `parsePrometheusBody()`, `fetchServerTemp()`, scheduled poll in `loop()`, debug endpoint.

- [ ] **Step 3.1: Add include and constants**

Add after the `#include <Preferences.h>` line:
```cpp
#include <HTTPClient.h>

const char* METRICS_URL = "http://192.168.0.110:9101/metrics";
const unsigned long METRICS_POLL_MS    = 10000;  // 10 s
const unsigned long METRICS_STALE_MS   = 60000;  // 60 s before considered stale

struct ServerTemp {
  bool          valid;
  float         celsius;
  unsigned long timestampMs;
};
ServerTemp serverTemp = {false, 0.0f, 0};
unsigned long lastMetricsPoll = 0;
```

- [ ] **Step 3.2: Add the parser as a pure function**

Add above `setup()`:

```cpp
// Parses a Prometheus text-format body and writes the average of all
// node_thermal_zone_temp_celsius{...,type="x86_pkg_temp"} values to *avgOut.
// Returns true if at least one matching line was found.
bool parsePrometheusBody(const String& body, float* avgOut) {
  int idx = 0, found = 0;
  float sum = 0;
  const int len = body.length();
  while (idx < len) {
    int eol = body.indexOf('\n', idx);
    if (eol < 0) eol = len;
    // Skip comments / empty
    if (eol > idx && body.charAt(idx) != '#') {
      // Cheap startsWith without copying
      if (body.indexOf("node_thermal_zone_temp_celsius{", idx) == idx) {
        int lineEnd = eol;
        // Look for x86_pkg_temp within this line only
        int ttIdx = body.indexOf("x86_pkg_temp", idx);
        if (ttIdx >= 0 && ttIdx < lineEnd) {
          // Value follows the last space on the line
          int sp = body.lastIndexOf(' ', lineEnd - 1);
          if (sp > idx) {
            String valStr = body.substring(sp + 1, lineEnd);
            float v = valStr.toFloat();
            sum += v;
            found++;
          }
        }
      }
    }
    idx = eol + 1;
  }
  if (found == 0) return false;
  *avgOut = sum / found;
  return true;
}
```

- [ ] **Step 3.3: Add the fetch wrapper**

Add directly below `parsePrometheusBody`:

```cpp
bool fetchServerTemp(float* out) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[metrics] wifi not connected");
    return false;
  }
  HTTPClient http;
  http.setConnectTimeout(2000);
  http.setTimeout(3000);
  if (!http.begin(METRICS_URL)) {
    Serial.println("[metrics] http.begin failed");
    return false;
  }
  int code = http.GET();
  if (code != 200) {
    Serial.printf("[metrics] HTTP %d\n", code);
    http.end();
    return false;
  }
  String body = http.getString();
  http.end();
  if (!parsePrometheusBody(body, out)) {
    Serial.println("[metrics] no x86_pkg_temp lines parsed");
    return false;
  }
  return true;
}
```

- [ ] **Step 3.4: Add scheduled poll in `loop()`**

In `loop()`, immediately after the `calculateRPM();` line, add:

```cpp
if (millis() - lastMetricsPoll >= METRICS_POLL_MS) {
  lastMetricsPoll = millis();
  float t;
  if (fetchServerTemp(&t)) {
    serverTemp.valid       = true;
    serverTemp.celsius     = t;
    serverTemp.timestampMs = millis();
    Serial.printf("[metrics] server temp avg = %.1f C\n", t);
  }
}
```

- [ ] **Step 3.5: Add debug endpoint**

Inside `setup()`, after the `/status` handler:

```cpp
server.on("/debug/server-temp", HTTP_GET, [](AsyncWebServerRequest *request){
  String body;
  if (!serverTemp.valid) {
    body = "{\"valid\":false}";
  } else {
    unsigned long age = millis() - serverTemp.timestampMs;
    body = "{\"valid\":true,\"celsius\":" + String(serverTemp.celsius, 1) +
           ",\"ageMs\":" + String(age) + "}";
  }
  request->send(200, "application/json", body);
});
```

- [ ] **Step 3.6: Build, OTA-flash, verify**

Wait at least 15 s after the board boots, then:
```bash
curl -s http://<esp32-ip>/debug/server-temp
```
Expected (values will vary):
```
{"valid":true,"celsius":49.5,"ageMs":4200}
```
Serial Monitor should show a `[metrics] server temp avg = ...` line every 10 s.

- [ ] **Step 3.7: Negative test — block the metrics host**

Either disconnect the metrics host from the network, or temporarily change `METRICS_URL` to a bogus IP and re-flash. Expected: Serial Monitor shows `[metrics] HTTP -1` (or connect failure), `/debug/server-temp` `ageMs` keeps growing. Restore `METRICS_URL` and re-flash before continuing.

- [ ] **Step 3.8: Commit**

```bash
git add fan_controller_with_ota.ino
git commit -m "feat: scrape server CPU temps from Prometheus endpoint"
```

---

## Task 4: Replace `autoMode` boolean with three-mode enum

**Goal:** introduce `MODE_MANUAL` / `MODE_AUTO_AMBIENT` / `MODE_AUTO_SERVER` as the canonical mode state.

**Files:**
- Modify: `fan_controller_with_ota.ino` — define enum, add `/mode` endpoint, update `applyFanSpeeds()` to switch on mode (server branch is a stub for now).

- [ ] **Step 4.1: Add the enum**

Add above the `Settings` struct (so it can be referenced):

```cpp
enum Mode : uint8_t {
  MODE_MANUAL        = 0,
  MODE_AUTO_AMBIENT  = 1,
  MODE_AUTO_SERVER   = 2,
};
```

- [ ] **Step 4.2: Update `applyFanSpeeds()`**

Replace the function body with:

```cpp
void applyFanSpeeds() {
  int intakeValue, exhaustValue;
  switch (settings.mode) {
    case MODE_AUTO_AMBIENT: {
      int v = computePWMFromTemp();
      intakeValue = exhaustValue = v;
      break;
    }
    case MODE_AUTO_SERVER: {
      // Stub — Task 5 wires up the PID. For now, fall through to ambient.
      int v = computePWMFromTemp();
      intakeValue = exhaustValue = v;
      break;
    }
    case MODE_MANUAL:
    default:
      intakeValue  = settings.intakePWM;
      exhaustValue = settings.exhaustPWM;
      break;
  }

  ledcWrite(intakeFan1,  intakeValue);
  ledcWrite(intakeFan2,  intakeValue);
  ledcWrite(exhaustFan1, exhaustValue);
  ledcWrite(exhaustFan2, exhaustValue);

  Serial.printf("[fans] mode=%u tempA=%.1f intake=%d exhaust=%d\n",
                settings.mode, temperatureC, intakeValue, exhaustValue);
}
```

- [ ] **Step 4.3: Replace the existing `/auto` handler with `/mode`**

Delete the entire existing `server.on("/auto", ...)` block. Add in its place:

```cpp
server.on("/mode", HTTP_GET, [](AsyncWebServerRequest *request){
  if (request->hasParam("m")) {
    String m = request->getParam("m")->value();
    if      (m == "manual")  settings.mode = MODE_MANUAL;
    else if (m == "ambient") settings.mode = MODE_AUTO_AMBIENT;
    else if (m == "server")  settings.mode = MODE_AUTO_SERVER;
    else { request->send(400, "text/plain", "bad mode"); return; }
    saveSettings();
    applyFanSpeeds();
  }
  request->send(200, "text/plain", "OK");
});
```

- [ ] **Step 4.4: Update `/status` to also include the mode name (optional but helpful)**

Add this line just before the closing `}` of the JSON in the `/status` handler (so it sits after `pwmMin`):

```cpp
json += ",\"modeName\":\"";
switch (settings.mode) {
  case MODE_MANUAL:       json += "manual";  break;
  case MODE_AUTO_AMBIENT: json += "ambient"; break;
  case MODE_AUTO_SERVER:  json += "server";  break;
}
json += "\"";
```

- [ ] **Step 4.5: Build, OTA-flash, verify**

```bash
curl -s 'http://<esp32-ip>/mode?m=ambient'
curl -s http://<esp32-ip>/status
```
Expected: `/status` shows `"mode":1,"modeName":"ambient"`. Power-cycle the board; Serial Monitor reports `mode=1` on boot.

```bash
curl -s 'http://<esp32-ip>/mode?m=manual'
```
Confirm `/status` returns to `"mode":0,"modeName":"manual"`.

- [ ] **Step 4.6: Commit**

```bash
git add fan_controller_with_ota.ino
git commit -m "feat: replace autoMode bool with three-mode enum"
```

---

## Task 5: PI controller for AUTO_SERVER mode

**Goal:** wire the server-temp signal through a small PI controller targeting `settings.setpointC`. Output drives both intake and exhaust to the same PWM (v1).

**Files:**
- Modify: `fan_controller_with_ota.ino` — add `PI` struct + `pidStep()`, update `applyFanSpeeds()`, add `/setpoint` endpoint, expose values in `/status`.

- [ ] **Step 5.1: Add the controller state**

Add above `applyFanSpeeds()`:

```cpp
struct PI {
  float kp;
  float ki;
  float integral;
  float outMin;
  float outMax;
};

// Conservative starting gains. Setpoint of 70 C, error in C, output in PWM (0-255).
// kp=12: every 1 C above setpoint adds 12 to PWM.
// ki=0.2 with 10 s loop: 1 C error for 10 s adds 2 to PWM. Slow trim.
PI piCtl = {12.0f, 0.2f, 0.0f, 60.0f, 255.0f};

unsigned long lastPidStepMs = 0;

float pidStep(PI* c, float error, float dtSec) {
  c->integral += error * dtSec;
  // Anti-windup: clamp integral so ki*integral can never push past output range.
  float iMax = (c->outMax - c->outMin) / c->ki;
  if (c->integral > iMax)  c->integral = iMax;
  if (c->integral < 0.0f)  c->integral = 0.0f;  // never accumulate cooling credit
  float out = c->outMin + c->kp * error + c->ki * c->integral;
  if (out < c->outMin) out = c->outMin;
  if (out > c->outMax) out = c->outMax;
  return out;
}

void resetPid() {
  piCtl.integral = 0.0f;
  lastPidStepMs  = 0;
}
```

- [ ] **Step 5.2: Track most recent control-source decision**

Add a global next to the controller:

```cpp
const char* controlSource = "manual";  // updated each applyFanSpeeds() call
int         lastAppliedPWM = 0;        // most recent PID/curve output
```

- [ ] **Step 5.3: Update `applyFanSpeeds()` AUTO_SERVER branch**

Replace the `case MODE_AUTO_SERVER` block in `applyFanSpeeds()` with:

```cpp
case MODE_AUTO_SERVER: {
  if (serverTemp.valid &&
      (millis() - serverTemp.timestampMs) <= METRICS_STALE_MS) {
    // Fresh signal: run PID
    piCtl.outMin = settings.pwmMin;
    unsigned long now = millis();
    float dtSec = (lastPidStepMs == 0) ? 1.0f : (now - lastPidStepMs) / 1000.0f;
    if (dtSec > 30.0f) dtSec = 30.0f;  // clamp first call after long gap
    lastPidStepMs = now;
    float error = serverTemp.celsius - (float)settings.setpointC;
    float out = pidStep(&piCtl, error, dtSec);
    intakeValue = exhaustValue = (int)out;
    controlSource = "server";
    lastAppliedPWM = (int)out;
  } else {
    // Stale or missing — fallback handled in Task 6. For now, ambient curve.
    int v = computePWMFromTemp();
    intakeValue = exhaustValue = v;
    controlSource = "fallback-stale";
    resetPid();
  }
  break;
}
```

Also update the other branches to set `controlSource`:
- `MODE_AUTO_AMBIENT`: append `controlSource = "ambient";`
- `MODE_MANUAL`: append `controlSource = "manual";`

- [ ] **Step 5.4: Add `/setpoint` endpoint**

After the `/mode` handler in `setup()`:

```cpp
server.on("/setpoint", HTTP_GET, [](AsyncWebServerRequest *request){
  if (request->hasParam("val")) {
    int v = request->getParam("val")->value().toInt();
    v = constrain(v, 40, 95);
    settings.setpointC = (uint8_t)v;
    saveSettings();
    resetPid();           // start fresh after setpoint change
    applyFanSpeeds();
  }
  request->send(200, "text/plain", "OK");
});
```

- [ ] **Step 5.5: Update `loop()` so each mode is driven by the right tick**

The existing `loop()` only fires `applyFanSpeeds()` on the 3 s DHT22 tick. We need:
- `MODE_AUTO_AMBIENT` driven by the 3 s tick (DHT22 is its input).
- `MODE_AUTO_SERVER` driven by the 10 s metrics tick — **regardless of whether the fetch succeeded**, so the stale-fallback in Task 6 actually triggers.

Replace the body of `loop()` with:

```cpp
void loop() {
  ArduinoOTA.handle();

  static unsigned long lastTempRead = 0;
  if (millis() - lastTempRead > 3000) {
    lastTempRead = millis();
    readTemp();
    if (settings.mode == MODE_AUTO_AMBIENT) {
      applyFanSpeeds();
    }
  }

  if (millis() - lastMetricsPoll >= METRICS_POLL_MS) {
    lastMetricsPoll = millis();
    float t;
    if (fetchServerTemp(&t)) {
      serverTemp.valid       = true;
      serverTemp.celsius     = t;
      serverTemp.timestampMs = millis();
      Serial.printf("[metrics] server temp avg = %.1f C\n", t);
    }
    if (settings.mode == MODE_AUTO_SERVER) {
      applyFanSpeeds();
    }
  }

  calculateRPM();
}
```

This **replaces** the `loop()` body and the metrics-poll snippet you added in Task 3.4 — they're consolidated here.

- [ ] **Step 5.6: Expose new fields via `/status`**

Update the `/status` handler. Just before the closing `}` of the JSON (after `modeName`):

```cpp
json += ",\"serverTemp\":";
if (serverTemp.valid) json += String(serverTemp.celsius, 1);
else                  json += "null";
json += ",\"serverTempAgeMs\":";
json += (serverTemp.valid ? String(millis() - serverTemp.timestampMs) : String("null"));
json += ",\"controlSource\":\"" + String(controlSource) + "\"";
json += ",\"appliedPWM\":" + String(lastAppliedPWM);
json += ",\"pidIntegral\":" + String(piCtl.integral, 2);
```

- [ ] **Step 5.7: Build, OTA-flash, verify**

```bash
curl -s 'http://<esp32-ip>/mode?m=server'
curl -s 'http://<esp32-ip>/setpoint?val=45'
sleep 12
curl -s http://<esp32-ip>/status
```
With CPUs at ~50 °C and setpoint=45, error ≈ 5 °C. Expected `appliedPWM` rises from 60 toward 60 + 12*5 = 120 within a few cycles. `controlSource` should read `"server"`.

```bash
curl -s 'http://<esp32-ip>/setpoint?val=80'
sleep 12
curl -s http://<esp32-ip>/status
```
With CPUs at ~50 °C and setpoint=80, error is negative; PI clamps to `pwmMin` (60) and `pidIntegral` stays at 0. Listen — fans should drop to a quiet idle.

Restore a sensible setpoint:
```bash
curl -s 'http://<esp32-ip>/setpoint?val=70'
```

- [ ] **Step 5.8: Commit**

```bash
git add fan_controller_with_ota.ino
git commit -m "feat: PI controller drives fans from server CPU temperatures"
```

---

## Task 6: Stale-data fallback with safety floor

**Goal:** when in `MODE_AUTO_SERVER` and the metrics signal is older than `METRICS_STALE_MS`, drop to the ambient curve **but raise the PWM floor to 128** so a dead Prometheus endpoint never leaves the box under-cooled.

**Files:**
- Modify: `fan_controller_with_ota.ino` — refine the stale branch in `applyFanSpeeds()`.

- [ ] **Step 6.1: Define the safety floor constant**

Add near the other metrics constants:

```cpp
const int FALLBACK_PWM_MIN = 128;  // safety floor when server signal is stale
```

- [ ] **Step 6.2: Update the stale branch**

In `applyFanSpeeds()`, replace the `else` block of the `MODE_AUTO_SERVER` case with:

```cpp
} else {
  int v = computePWMFromTemp();
  if (v < FALLBACK_PWM_MIN) v = FALLBACK_PWM_MIN;
  intakeValue = exhaustValue = v;
  controlSource = serverTemp.valid ? "fallback-stale" : "fallback-no-signal";
  resetPid();
}
```

- [ ] **Step 6.3: Build, OTA-flash, verify**

Set server mode and confirm normal operation:
```bash
curl -s 'http://<esp32-ip>/mode?m=server'
curl -s http://<esp32-ip>/status   # expect controlSource=server
```

Now break the metrics feed (kill node_exporter on the server, or unplug it). Wait 70 s, then:
```bash
curl -s http://<esp32-ip>/status
```
Expected: `"controlSource":"fallback-stale"` and `"appliedPWM"` ≥ 128. Restore the metrics feed; within 10 s `controlSource` should return to `"server"`.

- [ ] **Step 6.4: Commit**

```bash
git add fan_controller_with_ota.ino
git commit -m "feat: stale-server-signal fallback at safety PWM floor"
```

---

## Task 7: Web UI — expose mode, setpoint, server temp, control source

**Goal:** make the new state visible and controllable in the browser. Replace `alert()` calls with inline status text. Keep the page minimal — no full restyle in this plan.

**Files:**
- Modify: `fan_controller_with_ota.ino` — replace the `R"rawliteral(...)"` HTML inside the `/` handler.

- [ ] **Step 7.1: Replace the HTML/JS block**

Replace the entire HTML string assigned to `html` inside `server.on("/", HTTP_GET, ...)` with the following:

```cpp
String html = R"rawliteral(
<!DOCTYPE html><html><head>
<title>Fan Controller</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { font-family: system-ui, sans-serif; margin: 16px; max-width: 700px; }
  .card { border: 1px solid #ccc; border-radius: 6px; margin: 12px 0; padding: 14px; }
  .row { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; }
  .rpm { font-weight: bold; color: #007acc; }
  .badge { display: inline-block; padding: 2px 8px; border-radius: 10px; font-size: 12px; }
  .badge-ok    { background: #d6f5d6; color: #186218; }
  .badge-warn  { background: #fff2cc; color: #7a5b00; }
  .badge-err   { background: #f7d4d4; color: #8a1a1a; }
  .status-line { font-size: 12px; color: #666; min-height: 1em; }
  input[type=range] { flex: 1; }
  button { padding: 6px 12px; }
</style>
</head><body>
<h1>Fan Controller</h1>

<div class="card">
  <div class="row">
    <strong>Mode:</strong>
    <select id="mode" onchange="setMode(this.value)">
      <option value="manual">Manual</option>
      <option value="ambient">Auto (ambient DHT22)</option>
      <option value="server">Auto (server CPU temp)</option>
    </select>
    <span id="srcBadge" class="badge badge-ok">--</span>
  </div>
  <p>Server temp avg: <span id="srvTemp">--</span> &deg;C
     <small id="srvAge"></small></p>
  <p>Ambient temp: <span id="ambTemp">--</span> &deg;C</p>
  <div class="row">
    <strong>Setpoint:</strong>
    <input type="number" id="setpoint" min="40" max="95" value="70">
    <button onclick="setSetpoint()">Set</button>
    <span>&deg;C</span>
  </div>
  <p class="status-line" id="statusLine"></p>
</div>

<div class="card">
  <h2>Intake fans</h2>
  <div class="row">
    <input type="range" min="0" max="255" value="128" id="intakeSlider"
           oninput="document.getElementById('intVal').textContent=this.value"
           onchange="updateFan('intake', this.value)">
    <span>PWM <span id="intVal">128</span></span>
  </div>
  <p>RPM: <span id="rpm1" class="rpm">--</span> / <span id="rpm2" class="rpm">--</span></p>
</div>

<div class="card">
  <h2>Exhaust fans</h2>
  <div class="row">
    <input type="range" min="0" max="255" value="128" id="exhaustSlider"
           oninput="document.getElementById('extVal').textContent=this.value"
           onchange="updateFan('exhaust', this.value)">
    <span>PWM <span id="extVal">128</span></span>
  </div>
  <p>RPM: <span id="rpm3" class="rpm">--</span> / <span id="rpm4" class="rpm">--</span></p>
</div>

<div class="card">
  <h2>Diagnostics</h2>
  <button onclick="runPWMTest()">Quick PWM Test</button>
  <div class="row" style="margin-top:8px;">
    <input type="number" id="manualPWM" min="0" max="255" value="128">
    <button onclick="setManualPWM()">Set PWM (pin 25)</button>
  </div>
  <p class="status-line" id="diagLine"></p>
</div>

<script>
function status(msg, target) {
  const el = document.getElementById(target || 'statusLine');
  el.textContent = msg;
  if (msg) setTimeout(() => { if (el.textContent === msg) el.textContent = ''; }, 4000);
}
function updateFan(type, val) {
  fetch('/set?fan=' + type + '&val=' + val)
    .then(() => status(type + ' set to ' + val))
    .catch(() => status('failed', 'statusLine'));
}
function setMode(m) {
  fetch('/mode?m=' + m).then(() => status('mode: ' + m));
}
function setSetpoint() {
  const v = document.getElementById('setpoint').value;
  fetch('/setpoint?val=' + v).then(() => status('setpoint: ' + v + ' C'));
}
function runPWMTest() {
  fetch('/test').then(() => status('PWM test running', 'diagLine'));
}
function setManualPWM() {
  const v = document.getElementById('manualPWM').value;
  fetch('/manual?pwm=' + v).then(() => status('manual PWM ' + v, 'diagLine'));
}
function refresh() {
  fetch('/status').then(r => r.json()).then(s => {
    document.getElementById('ambTemp').textContent = s.ambientC.toFixed(1);
    document.getElementById('srvTemp').textContent =
      (s.serverTemp === null) ? '--' : s.serverTemp.toFixed(1);
    document.getElementById('srvAge').textContent =
      (s.serverTempAgeMs === null) ? '' : '(' + Math.round(s.serverTempAgeMs/1000) + 's old)';
    document.getElementById('rpm1').textContent = s.rpm[0];
    document.getElementById('rpm2').textContent = s.rpm[1];
    document.getElementById('rpm3').textContent = s.rpm[2];
    document.getElementById('rpm4').textContent = s.rpm[3];
    document.getElementById('mode').value = s.modeName;
    document.getElementById('setpoint').value = s.setpointC;
    document.getElementById('intakeSlider').value = s.intakePWM;
    document.getElementById('intVal').textContent = s.intakePWM;
    document.getElementById('exhaustSlider').value = s.exhaustPWM;
    document.getElementById('extVal').textContent = s.exhaustPWM;
    const badge = document.getElementById('srcBadge');
    badge.textContent = s.controlSource + ' / pwm ' + s.appliedPWM;
    badge.className = 'badge ' +
      (s.controlSource === 'server' ? 'badge-ok' :
       s.controlSource.startsWith('fallback') ? 'badge-warn' : 'badge-ok');
  }).catch(() => {
    document.getElementById('srcBadge').className = 'badge badge-err';
    document.getElementById('srcBadge').textContent = 'offline';
  });
}
refresh();
setInterval(refresh, 5000);
</script>
</body></html>
)rawliteral";
```

- [ ] **Step 7.2: Build, OTA-flash, verify**

Open `http://<esp32-ip>/` in a browser. Verify:
1. Mode dropdown reflects current mode after a refresh.
2. Server temp updates within 10 s and shows age.
3. Sliders show PWM values pulled from `/status` (so persistence is now visible).
4. Changing the setpoint and clicking Set triggers a status line update; reload the page and confirm the new value sticks.
5. Switching mode to "server" updates the badge to green `server / pwm <n>`.
6. Pull network on the metrics host: within 70 s the badge turns yellow `fallback-stale / pwm 128+`.

Open the page from a phone too — layout should be readable, no horizontal scroll.

- [ ] **Step 7.3: Commit**

```bash
git add fan_controller_with_ota.ino
git commit -m "feat: web UI exposes mode, setpoint, server temp, control source"
```

---

## Task 8: README update

**Goal:** document the new feature so future-you remembers what `MODE_AUTO_SERVER` does.

**Files:**
- Modify: `README.md`

- [ ] **Step 8.1: Add a "Server-Temp Auto Mode" section**

Insert after the existing "Temperature Thresholds (Auto Mode)" table:

```markdown
### Server-Temp Auto Mode

If you set `Mode` to "Auto (server CPU temp)", the controller scrapes
`http://192.168.0.110:9101/metrics` every 10 seconds, averages the two
`node_thermal_zone_temp_celsius{type="x86_pkg_temp"}` zones, and runs a
small PI controller that targets the configured **setpoint** (default
70 °C). PWM output is clamped to `[pwmMin, 255]`.

Edit `METRICS_URL` near the top of `fan_controller_with_ota.ino` to
match your own metrics endpoint.

If the metrics endpoint stops responding for more than 60 seconds, the
controller falls back to the ambient (DHT22) step curve with a safety
PWM floor of 128 and surfaces `controlSource: "fallback-stale"` in
`/status`.
```

- [ ] **Step 8.2: Add new endpoints to a small reference**

Append below the "Features Available" list:

```markdown
### HTTP API
- `GET /status` — JSON snapshot of all state
- `GET /mode?m=manual|ambient|server` — set operating mode
- `GET /setpoint?val=N` — set PI target (40–95 °C)
- `GET /set?fan=intake|exhaust&val=N` — set manual PWM
- `GET /debug/server-temp` — last server-temp scrape result
```

- [ ] **Step 8.3: Commit**

```bash
git add README.md
git commit -m "docs: server-temp auto mode + HTTP API reference"
```

---

## Final integration smoke test

- [ ] **Step F.1: Cold boot from manual mode**

Power-cycle the board. `/status` should show `mode=0`, `controlSource=manual`, fans at last-saved PWM.

- [ ] **Step F.2: Switch to server mode and watch one full reaction cycle**

```bash
curl -s 'http://<esp32-ip>/mode?m=server'
curl -s 'http://<esp32-ip>/setpoint?val=45'   # force fans to ramp up
```
Wait 30 s. Listen for fan speed increase. `/status` `appliedPWM` should rise above `pwmMin`.

```bash
curl -s 'http://<esp32-ip>/setpoint?val=80'
```
Wait 30 s. Fans drop to `pwmMin` = 60. `pidIntegral` should be ~0.

- [ ] **Step F.3: Stale-feed test on real hardware**

Block the metrics host (firewall rule, or `sudo systemctl stop` the exporter). Watch `/status`. Within 70 s, `controlSource` flips to `fallback-stale` and `appliedPWM ≥ 128`. Restore the exporter; within 10 s `controlSource` returns to `server`.

- [ ] **Step F.4: Power-cycle persistence test**

Change setpoint to 65, mode to `server`, intake PWM to 200 (in manual mode). Power-cycle. After reboot, `/status` should show those exact values.

- [ ] **Step F.5: Open feature PR or merge**

Use `superpowers:finishing-a-development-branch` to choose between PR or direct merge. Branch is `feature/server-temp-pid` — confirm `git log main..HEAD` lists 8 commits before finishing.
