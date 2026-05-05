#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include <DHT.h>
#include "secrets.h"  // Include our secrets file
#include <Preferences.h>
#include <HTTPClient.h>

struct PICtrl {
  float kp;
  float ki;       // must be > 0; pidStep divides by ki for anti-windup
  float integral;
  float outMin;
  float outMax;
};

const char* METRICS_URL = "http://192.168.0.110:9101/metrics";
const unsigned long METRICS_POLL_MS    = 10000;  // 10 s
const unsigned long METRICS_STALE_MS   = 60000;  // 60 s before considered stale
const int           FALLBACK_PWM_MIN   = 128;    // safety floor when server signal is stale

struct ServerTemp {
  bool          valid;
  float         celsius;
  unsigned long timestampMs;
};
ServerTemp serverTemp = {false, 0.0f, 0};
unsigned long lastMetricsPoll = 0;

Preferences prefs;

enum Mode : uint8_t {
  MODE_MANUAL        = 0,
  MODE_AUTO_AMBIENT  = 1,
  MODE_AUTO_SERVER   = 2,
};

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

#define PWM_FREQ     25000
#define PWM_RES      8

// Try 1kHz if 25kHz doesn't work with your fan
// #define PWM_FREQ     1000

// PWM fan pins
const int intakeFan1 = 25;
const int intakeFan2 = 26;
const int exhaustFan1 = 27;
const int exhaustFan2 = 14;

// PWM channels for each fan
const int intakeFan1Channel = 0;
const int intakeFan2Channel = 1;
const int exhaustFan1Channel = 2;
const int exhaustFan2Channel = 3;

// Tachometer pins - using accessible pins on your board
const int intakeFan1Tach = 32;
const int intakeFan2Tach = 33;
const int exhaustFan1Tach = 13;  // moved from D34 (D34 is input-only, no pullup)
const int exhaustFan2Tach = 12;  // D12 — strapping pin, OK as input after boot

// Tachometer variables
volatile unsigned long tachCount1 = 0;
volatile unsigned long tachCount2 = 0;
volatile unsigned long tachCount3 = 0;
volatile unsigned long tachCount4 = 0;

unsigned long rpm1 = 0, rpm2 = 0, rpm3 = 0, rpm4 = 0;
unsigned long lastRPMCalc = 0;

// Temp sensor
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

AsyncWebServer server(80);

// WiFi credentials (now from secrets.h)
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

float temperatureC = 0;

// Tachometer interrupt functions.
// Real PC fans top out around 3000 RPM × 2 pulses/rev = 100 Hz at the tach line
// (10 ms between pulses). 5 ms debounce still admits any real fan up to 6000
// RPM (no PC fan reaches that) but rejects PWM crosstalk and ringing.
// micros() rolls over at ~70 minutes; the unsigned subtraction handles wrap.
volatile unsigned long lastTach1Us = 0, lastTach2Us = 0, lastTach3Us = 0, lastTach4Us = 0;
const unsigned long TACH_DEBOUNCE_US = 5000;

void IRAM_ATTR tachISR1() {
  unsigned long now = micros();
  if (now - lastTach1Us >= TACH_DEBOUNCE_US) { tachCount1++; lastTach1Us = now; }
}
void IRAM_ATTR tachISR2() {
  unsigned long now = micros();
  if (now - lastTach2Us >= TACH_DEBOUNCE_US) { tachCount2++; lastTach2Us = now; }
}
void IRAM_ATTR tachISR3() {
  unsigned long now = micros();
  if (now - lastTach3Us >= TACH_DEBOUNCE_US) { tachCount3++; lastTach3Us = now; }
}
void IRAM_ATTR tachISR4() {
  unsigned long now = micros();
  if (now - lastTach4Us >= TACH_DEBOUNCE_US) { tachCount4++; lastTach4Us = now; }
}

void setupPWM(int pin) {
  // Updated for ESP32 Arduino Core 3.2+ - channels are auto-assigned
  bool success = ledcAttach(pin, PWM_FREQ, PWM_RES);
  Serial.println("PWM setup for pin " + String(pin) + ": " + (success ? "SUCCESS" : "FAILED"));
  
  // Set initial low value to test
  ledcWrite(pin, 0);
  Serial.println("Initial PWM set to 0 for pin " + String(pin));
}

// Conservative starting gains. Setpoint of 70 C, error in C, output in PWM (0-255).
// kp=12: every 1 C above setpoint adds 12 to PWM.
// ki=0.2 with 10 s loop: 1 C error for 10 s adds 2 to PWM. Slow trim.
PICtrl piCtl = {12.0f, 0.2f, 0.0f, 60.0f, 255.0f};

unsigned long lastPidStepMs = 0;

float pidStep(PICtrl* c, float error, float dtSec) {
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

const char* controlSource = "manual";  // updated each applyFanSpeeds() call
int         lastAppliedPWM = 0;        // most recent PID/curve output

void applyFanSpeeds() {
  int intakeValue, exhaustValue;
  switch (settings.mode) {
    case MODE_AUTO_AMBIENT: {
      int v = computePWMFromTemp();
      intakeValue = exhaustValue = v;
      controlSource = "ambient";
      lastAppliedPWM = v;
      break;
    }
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
        int v = computePWMFromTemp();
        if (v < FALLBACK_PWM_MIN) v = FALLBACK_PWM_MIN;
        intakeValue = exhaustValue = v;
        controlSource = serverTemp.valid ? "fallback-stale" : "fallback-no-signal";
        lastAppliedPWM = v;
        resetPid();
      }
      break;
    }
    case MODE_MANUAL:
    default:
      intakeValue  = settings.intakePWM;
      exhaustValue = settings.exhaustPWM;
      controlSource = "manual";
      lastAppliedPWM = intakeValue;
      break;
  }

  ledcWrite(intakeFan1,  intakeValue);
  ledcWrite(intakeFan2,  intakeValue);
  ledcWrite(exhaustFan1, exhaustValue);
  ledcWrite(exhaustFan2, exhaustValue);

  Serial.printf("[fans] mode=%u tempA=%.1f intake=%d exhaust=%d\n",
                settings.mode, temperatureC, intakeValue, exhaustValue);
}

int computePWMFromTemp() {
  if (temperatureC < 25) return 50;
  if (temperatureC < 30) return 100;
  if (temperatureC < 35) return 180;
  return 255;
}

void readTemp() {
  float t = dht.readTemperature();
  if (!isnan(t)) {
    temperatureC = t;
  }
}

void calculateRPM() {
  // Calculate RPM every 3 seconds (most PC fans output 2 pulses per revolution)
  unsigned long elapsed = millis() - lastRPMCalc;
  if (elapsed >= 3000) {
    rpm1 = (tachCount1 * 60000) / (elapsed * 2); // 2 pulses per revolution
    rpm2 = (tachCount2 * 60000) / (elapsed * 2);
    rpm3 = (tachCount3 * 60000) / (elapsed * 2);
    rpm4 = (tachCount4 * 60000) / (elapsed * 2);
    
    // Debug output
    Serial.println("Tach counts - Fan1: " + String(tachCount1) + ", Fan2: " + String(tachCount2) + 
                   ", Fan3: " + String(tachCount3) + ", Fan4: " + String(tachCount4));
    Serial.println("RPM - Fan1: " + String(rpm1) + ", Fan2: " + String(rpm2) + 
                   ", Fan3: " + String(rpm3) + ", Fan4: " + String(rpm4));
    
    // Reset counters
    tachCount1 = tachCount2 = tachCount3 = tachCount4 = 0;
    lastRPMCalc = millis();
  }
}

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
          // Value follows the last space on the line.
          // Assumes no per-sample timestamp column (standard node_exporter default).
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

bool fetchServerTemp(float* out) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[metrics] wifi not connected");
    return false;
  }
  HTTPClient http;
  http.setConnectTimeout(2000);
  http.setTimeout(3000);
  // http.begin() on ESP32 core only parses the URL; connection happens in GET().
  // Failures surface as a non-200 response code below.
  http.begin(METRICS_URL);
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

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32 Fan Controller Starting ===");
  loadSettings();
  Serial.println("Fan connected to PWM pin: " + String(intakeFan1));
  Serial.println("Fan connected to TACH pin: " + String(intakeFan1Tach));

  // Init PWM for fans - channels are auto-assigned in 3.2+
  setupPWM(intakeFan1);
  setupPWM(intakeFan2);
  setupPWM(exhaustFan1);
  setupPWM(exhaustFan2);
  
  // Set initial fan speeds
  applyFanSpeeds();
  Serial.println("PWM initialized and initial speeds set");

  // Init temp sensor
  dht.begin();

  // Setup tachometer pins and interrupts
  Serial.println("Setting up tachometer pins...");
  
  // Setup pins with pull-up capability
  pinMode(intakeFan1Tach, INPUT_PULLUP);  // D32
  pinMode(intakeFan2Tach, INPUT_PULLUP);  // D33
  pinMode(exhaustFan1Tach, INPUT_PULLUP); // D13 (moved from D34)
  pinMode(exhaustFan2Tach, INPUT_PULLUP); // D12

  attachInterrupt(digitalPinToInterrupt(intakeFan1Tach),  tachISR1, FALLING);
  attachInterrupt(digitalPinToInterrupt(intakeFan2Tach),  tachISR2, FALLING);
  attachInterrupt(digitalPinToInterrupt(exhaustFan1Tach), tachISR3, FALLING);
  attachInterrupt(digitalPinToInterrupt(exhaustFan2Tach), tachISR4, FALLING);

  lastRPMCalc = millis();
  Serial.println("Tachometer interrupts setup complete");
  Serial.println("Active tach pins: 32, 33, 13, 12");

  // Connect WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected!");
  Serial.println("IP address: " + WiFi.localIP().toString());
  Serial.println("Go to: http://" + WiFi.localIP().toString());
  Serial.println("=== Setup Complete ===\n");

  // Setup ArduinoOTA
  ArduinoOTA.setHostname("ESP32-FanController");
  ArduinoOTA.setPassword(OTA_PASSWORD);  // Now from secrets.h
  
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
  Serial.println("ArduinoOTA Ready");

  // Web server endpoints
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
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
  fetch('/mode?m=' + m)
    .then(() => status('mode: ' + m))
    .catch(() => status('failed'));
}
function setSetpoint() {
  const v = document.getElementById('setpoint').value;
  fetch('/setpoint?val=' + v)
    .then(() => status('setpoint: ' + v + ' C'))
    .catch(() => status('failed'));
}
function runPWMTest() {
  fetch('/test')
    .then(() => status('PWM test running', 'diagLine'))
    .catch(() => status('failed', 'diagLine'));
}
function setManualPWM() {
  const v = document.getElementById('manualPWM').value;
  fetch('/manual?pwm=' + v)
    .then(() => status('manual PWM ' + v, 'diagLine'))
    .catch(() => status('failed', 'diagLine'));
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
    request->send(200, "text/html", html);
  });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("fan") && request->hasParam("val")) {
      String fan = request->getParam("fan")->value();
      int val = constrain(request->getParam("val")->value().toInt(), 0, 255);
      Serial.println("Setting " + fan + " fan to " + String(val));
      if (fan == "intake") settings.intakePWM = val;
      else if (fan == "exhaust") settings.exhaustPWM = val;
      saveSettings();
      applyFanSpeeds();
    }
    request->send(200, "text/plain", "OK");
  });

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

  server.on("/temp", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(temperatureC));
  });

  server.on("/test", HTTP_GET, [](AsyncWebServerRequest *request){
    // Quick PWM test without blocking delays
    Serial.println("=== PWM TEST SEQUENCE ===");
    Serial.println("Testing fan on pin " + String(intakeFan1));
    
    // Test different PWM values quickly
    ledcWrite(intakeFan1, 0);
    Serial.println("Set PWM to 0 (minimum)");
    
    // Short non-blocking delay
    yield();
    
    ledcWrite(intakeFan1, 128);
    Serial.println("Set PWM to 128 (50% speed)");
    
    yield();
    
    ledcWrite(intakeFan1, 255);
    Serial.println("Set PWM to 255 (100% speed)");
    
    yield();
    
    // Back to current setting
    applyFanSpeeds();
    Serial.println("=== TEST COMPLETE - Back to normal operation ===");
    
    request->send(200, "text/plain", "PWM test complete - values set to 0, 128, 255, then back to normal");
  });

  server.on("/manual", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("pwm")) {
      int pwmValue = request->getParam("pwm")->value().toInt();
      pwmValue = constrain(pwmValue, 0, 255);  // Ensure valid range
      Serial.println("=== Manual PWM Control ===");
      Serial.println("Setting pin " + String(intakeFan1) + " to PWM value: " + String(pwmValue));
      ledcWrite(intakeFan1, pwmValue);
      Serial.println("PWM command sent. Fan should respond if wiring is correct.");
      request->send(200, "text/plain", "PWM set to " + String(pwmValue));
    } else {
      request->send(400, "text/plain", "Missing pwm parameter. Use: /manual?pwm=128");
    }
  });

  server.on("/rpm", HTTP_GET, [](AsyncWebServerRequest *request){
    String rpmData = String(rpm1) + "," + String(rpm2) + "," + String(rpm3) + "," + String(rpm4);
    request->send(200, "text/plain", rpmData);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{";
    json += "\"ambientC\":" + String(temperatureC, 1) + ",";
    json += "\"rpm\":[" + String(rpm1) + "," + String(rpm2) + "," + String(rpm3) + "," + String(rpm4) + "],";
    json += "\"intakePWM\":" + String(settings.intakePWM) + ",";
    json += "\"exhaustPWM\":" + String(settings.exhaustPWM) + ",";
    json += "\"mode\":" + String(settings.mode) + ",";
    json += "\"setpointC\":" + String(settings.setpointC) + ",";
    json += "\"pwmMin\":" + String(settings.pwmMin);
    json += ",\"modeName\":\"";
    switch (settings.mode) {
      case MODE_MANUAL:       json += "manual";  break;
      case MODE_AUTO_AMBIENT: json += "ambient"; break;
      case MODE_AUTO_SERVER:  json += "server";  break;
      default:                json += "unknown"; break;
    }
    json += "\"";
    json += ",\"serverTemp\":";
    if (serverTemp.valid) json += String(serverTemp.celsius, 1);
    else                  json += "null";
    json += ",\"serverTempAgeMs\":";
    json += (serverTemp.valid ? String(millis() - serverTemp.timestampMs) : String("null"));
    json += ",\"controlSource\":\"" + String(controlSource) + "\"";
    json += ",\"appliedPWM\":" + String(lastAppliedPWM);
    json += ",\"pidIntegral\":" + String(piCtl.integral, 2);
    json += "}";
    request->send(200, "application/json", json);
  });

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

  server.begin();
}

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
      // Write data fields before publishing valid=true so a concurrent reader
      // on the AsyncWebServer task can't see valid=true with stale celsius.
      serverTemp.celsius     = t;
      serverTemp.timestampMs = millis();
      serverTemp.valid       = true;
      Serial.printf("[metrics] server temp avg = %.1f C\n", t);
    }
    if (settings.mode == MODE_AUTO_SERVER) {
      applyFanSpeeds();
    }
  }

  calculateRPM();
}