// =====================================================================
//  ESP32 Server-Cabinet Fan Controller — firmware
//  Joins WiFi, serves a control web UI (reachable at http://fan.local),
//  drives four 4-pin PWM fans (intake x2, exhaust x2), reads their tachs
//  for RPM, reads a DHT22 for cabinet temperature, and supports an
//  optional temperature-driven auto mode. Firmware updates over the air
//  via an HTTP push to /update (browser card in the UI).
//
//  Structure mirrors the companion "7-segment clock" project.
// =====================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <Preferences.h>
#include <esp_system.h>
#include <DHT.h>

#include "config.h"
#include "secrets.h"
#include "webpage.h"

DHT       dht(DHT_PIN, DHT_TYPE);
WebServer server(80);
Preferences prefs;

// ---- Persisted user settings ----
int  intakePWM  = DEFAULT_INTAKE_PWM;    // 0-255
int  exhaustPWM = DEFAULT_EXHAUST_PWM;   // 0-255
bool autoMode   = DEFAULT_AUTO_MODE;

// ---- Live sensor state ----
float temperatureC = NAN;                // NAN until first valid DHT read

// ---- Tachometers ----
volatile unsigned long tachCount1 = 0, tachCount2 = 0, tachCount3 = 0, tachCount4 = 0;
volatile unsigned long lastTachUs1 = 0, lastTachUs2 = 0, lastTachUs3 = 0, lastTachUs4 = 0;
unsigned long rpm1 = 0, rpm2 = 0, rpm3 = 0, rpm4 = 0;
unsigned long lastRPMCalc = 0;

// Debounced tach ISRs: a real pulse can't arrive faster than TACH_DEBOUNCE_US,
// so edges closer than that are ringing/noise and are dropped (see config.h).
void IRAM_ATTR tachISR1() { unsigned long n = micros(); if (n - lastTachUs1 >= TACH_DEBOUNCE_US) { lastTachUs1 = n; tachCount1++; } }
void IRAM_ATTR tachISR2() { unsigned long n = micros(); if (n - lastTachUs2 >= TACH_DEBOUNCE_US) { lastTachUs2 = n; tachCount2++; } }
void IRAM_ATTR tachISR3() { unsigned long n = micros(); if (n - lastTachUs3 >= TACH_DEBOUNCE_US) { lastTachUs3 = n; tachCount3++; } }
void IRAM_ATTR tachISR4() { unsigned long n = micros(); if (n - lastTachUs4 >= TACH_DEBOUNCE_US) { lastTachUs4 = n; tachCount4++; } }

// Settings are committed to flash lazily: a change marks them dirty and the
// loop commits once things settle, so a burst of slider /set requests doesn't
// hammer NVS (and block the loop) on every request.
bool     settingsDirty   = false;
uint32_t settingsDirtyMs = 0;
void markSettingsDirty() { settingsDirty = true; settingsDirtyMs = millis(); }

// Why the chip last reset — reported on serial at boot and exposed via /state,
// so a fault that only happens off-USB can still be diagnosed after the fact.
const char* gResetReason = "?";
const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "poweron";
    case ESP_RST_EXT:       return "external";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "sdio";
    default:                return "unknown";
  }
}

// ---------------------------------------------------------------------
//  Fan control
// ---------------------------------------------------------------------
// Auto-mode duty as a function of temperature (see thresholds in config.h).
// Defined before applyFanSpeeds() — .cpp has no Arduino auto-prototyping.
int computePWMFromTemp() {
  if (isnan(temperatureC)) return AUTO_PWM_1;   // no reading yet: keep it gentle
  if (temperatureC < AUTO_TEMP_1) return AUTO_PWM_1;
  if (temperatureC < AUTO_TEMP_2) return AUTO_PWM_2;
  if (temperatureC < AUTO_TEMP_3) return AUTO_PWM_3;
  return AUTO_PWM_4;
}

void applyFanSpeeds() {
  int intakeValue  = autoMode ? computePWMFromTemp() : intakePWM;
  int exhaustValue = autoMode ? computePWMFromTemp() : exhaustPWM;

  ledcWrite(INTAKE_FAN1_CH,  intakeValue);
  ledcWrite(INTAKE_FAN2_CH,  intakeValue);
  ledcWrite(EXHAUST_FAN1_CH, exhaustValue);
  ledcWrite(EXHAUST_FAN2_CH, exhaustValue);

  Serial.printf("Fans: auto=%s temp=%.1f intake=%d->%d exhaust=%d->%d\n",
                autoMode ? "ON" : "OFF", temperatureC,
                intakePWM, intakeValue, exhaustPWM, exhaustValue);
}

void setupPWM(int pin, int channel) {
  // ESP32 Arduino core 2.x channel-based LEDC API.
  ledcSetup(channel, PWM_FREQ, PWM_RES);
  ledcAttachPin(pin, channel);
  ledcWrite(channel, 0);
  Serial.printf("PWM setup pin %d -> channel %d\n", pin, channel);
}

void readTemp() {
  float t = dht.readTemperature();
  if (!isnan(t)) temperatureC = t;
}

void calculateRPM() {
  // Most PC fans output 2 pulses per revolution; recompute every ~3 s.
  unsigned long elapsed = millis() - lastRPMCalc;
  if (elapsed < 3000) return;

  rpm1 = (tachCount1 * 60000) / (elapsed * TACH_PULSES_PER_REV);
  rpm2 = (tachCount2 * 60000) / (elapsed * TACH_PULSES_PER_REV);
  rpm3 = (tachCount3 * 60000) / (elapsed * TACH_PULSES_PER_REV);
  rpm4 = (tachCount4 * 60000) / (elapsed * TACH_PULSES_PER_REV);

  tachCount1 = tachCount2 = tachCount3 = tachCount4 = 0;
  lastRPMCalc = millis();
}

// ---------------------------------------------------------------------
//  Settings (persisted to NVS)
// ---------------------------------------------------------------------
void loadSettings() {
  prefs.begin("fan", true);
  intakePWM  = prefs.getInt ("intake",  intakePWM);
  exhaustPWM = prefs.getInt ("exhaust", exhaustPWM);
  autoMode   = prefs.getBool("auto",    autoMode);
  prefs.end();
}

void saveSettings() {
  prefs.begin("fan", false);
  prefs.putInt ("intake",  intakePWM);
  prefs.putInt ("exhaust", exhaustPWM);
  prefs.putBool("auto",    autoMode);
  prefs.end();
}

// ---------------------------------------------------------------------
//  Web interface
// ---------------------------------------------------------------------
void sendState() {
  char tempField[12];
  if (isnan(temperatureC)) strcpy(tempField, "null");
  else snprintf(tempField, sizeof(tempField), "%.1f", temperatureC);

  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"intake\":%d,\"exhaust\":%d,\"auto\":%d,\"temp\":%s,"
           "\"rpm1\":%lu,\"rpm2\":%lu,\"rpm3\":%lu,\"rpm4\":%lu,"
           "\"rst\":\"%s\",\"up\":%lu}",
           intakePWM, exhaustPWM, autoMode ? 1 : 0, tempField,
           rpm1, rpm2, rpm3, rpm4,
           gResetReason, (unsigned long)(millis() / 1000));
  server.send(200, "application/json", buf);
}

void handleSet() {
  if (server.hasArg("intake"))
    intakePWM = constrain(server.arg("intake").toInt(), 0, 255);
  if (server.hasArg("exhaust"))
    exhaustPWM = constrain(server.arg("exhaust").toInt(), 0, 255);
  if (server.hasArg("auto"))
    autoMode = (server.arg("auto").toInt() == 1);

  applyFanSpeeds();      // instant live response
  markSettingsDirty();   // flash commit deferred to the loop
  sendState();
}

// ---------------------------------------------------------------------
//  OTA — HTTP firmware push to /update (multipart POST, HTTP basic auth).
//  Outbound from the uploader's side, so no PC-firewall involvement.
// ---------------------------------------------------------------------
bool otaAuthed = false;

// Receives the uploaded .bin in chunks and streams it to flash.
void handleUpdateUpload() {
  HTTPUpload& up = server.upload();
  switch (up.status) {
    case UPLOAD_FILE_START:
      otaAuthed = server.authenticate("admin", OTA_PASSWORD);
      if (!otaAuthed) return;                 // gate before touching flash
      Serial.printf("OTA(http): start %s\n", up.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
      break;

    case UPLOAD_FILE_WRITE:
      if (!otaAuthed) return;
      if (Update.write(up.buf, up.currentSize) != up.currentSize)
        Update.printError(Serial);
      else {
        static uint32_t lastShown = 0;
        if (up.totalSize - lastShown >= 24000) {
          lastShown = up.totalSize;
          Serial.printf("OTA(http): %u bytes\r", up.totalSize);
        }
      }
      break;

    case UPLOAD_FILE_END:
      if (!otaAuthed) return;
      if (Update.end(true)) Serial.printf("\nOTA(http): done, %u bytes\n", up.totalSize);
      else                  Update.printError(Serial);
      break;

    default: break;
  }
}

// Runs after the upload completes: reports result and reboots on success.
void handleUpdateDone() {
  if (!otaAuthed) { server.requestAuthentication(); return; }
  bool ok = !Update.hasError();
  server.sendHeader("Connection", "close");
  server.send(200, "text/plain", ok ? "OK - rebooting" : "FAIL");
  if (ok) { delay(700); ESP.restart(); }
}

void startWebServer() {
  server.on("/", []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/state", sendState);
  server.on("/set", handleSet);
  server.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  server.begin();
  Serial.printf("Web UI: http://%s/\n", WiFi.localIP().toString().c_str());
}

// ---------------------------------------------------------------------
//  WiFi
// ---------------------------------------------------------------------
void connectWiFi() {
  Serial.printf("WiFi: connecting to \"%s\"", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(300);
    Serial.print('.');
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\nWiFi: connected, IP %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println(F("\nWiFi: FAILED (will keep retrying in background)"));
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n\nESP32 fan controller booting..."));
  gResetReason = resetReasonStr(esp_reset_reason());
  Serial.printf("Last reset reason: %s\n", gResetReason);

  loadSettings();

  // Fan PWM outputs.
  setupPWM(INTAKE_FAN1_PIN,  INTAKE_FAN1_CH);
  setupPWM(INTAKE_FAN2_PIN,  INTAKE_FAN2_CH);
  setupPWM(EXHAUST_FAN1_PIN, EXHAUST_FAN1_CH);
  setupPWM(EXHAUST_FAN2_PIN, EXHAUST_FAN2_CH);
  applyFanSpeeds();

  // Temperature sensor.
  dht.begin();

  // Tachometer inputs + interrupts. Note: EXHAUST_FAN1_TACH is GPIO12, a boot
  // strapping pin (MTDI, selects flash voltage). We only enable its pull-up here
  // in setup() — after boot — so it doesn't affect strapping; the board has
  // been observed booting cleanly with the exhaust fan wired to it. If you ever
  // see boot failures, move that tach to another free GPIO.
  pinMode(INTAKE_FAN1_TACH,  INPUT_PULLUP);
  pinMode(INTAKE_FAN2_TACH,  INPUT_PULLUP);
  pinMode(EXHAUST_FAN1_TACH, INPUT_PULLUP);
  pinMode(EXHAUST_FAN2_TACH, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTAKE_FAN1_TACH),  tachISR1, FALLING);
  attachInterrupt(digitalPinToInterrupt(INTAKE_FAN2_TACH),  tachISR2, FALLING);
  attachInterrupt(digitalPinToInterrupt(EXHAUST_FAN1_TACH), tachISR3, FALLING);
  attachInterrupt(digitalPinToInterrupt(EXHAUST_FAN2_TACH), tachISR4, FALLING);
  lastRPMCalc = millis();

  connectWiFi();

  if (WiFi.status() == WL_CONNECTED) {
    if (MDNS.begin(MDNS_HOSTNAME)) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("mDNS: http://%s.local/\n", MDNS_HOSTNAME);
    }
    startWebServer();
  }

  Serial.println(F("=== Setup complete ==="));
}

void loop() {
  server.handleClient();

  // Commit settings to flash once changes have settled (see markSettingsDirty).
  if (settingsDirty && millis() - settingsDirtyMs > 1500) {
    settingsDirty = false;
    saveSettings();
  }

  // Keep WiFi alive.
  static uint32_t lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 5000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  }

  // Temperature read every ~3 s; re-apply fan speeds in auto mode so the curve
  // tracks temperature.
  static uint32_t lastTempRead = 0;
  if (millis() - lastTempRead > 3000) {
    lastTempRead = millis();
    readTemp();
    if (autoMode) applyFanSpeeds();
  }

  calculateRPM();
}
