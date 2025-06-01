#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include <DHT.h>
#include "secrets.h"  // Include our secrets file

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
const int exhaustFan1Tach = 34;
const int exhaustFan2Tach = 13;  // Changed to D13 - easily accessible

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

// Fan speed (0-255)
int intakePWM = 128;
int exhaustPWM = 128;
bool autoMode = false;

float temperatureC = 0;

// Tachometer interrupt functions
void IRAM_ATTR tachISR1() { tachCount1++; }
void IRAM_ATTR tachISR2() { tachCount2++; }
void IRAM_ATTR tachISR3() { tachCount3++; }
void IRAM_ATTR tachISR4() { tachCount4++; }

void setupPWM(int pin) {
  // Updated for ESP32 Arduino Core 3.2+ - channels are auto-assigned
  bool success = ledcAttach(pin, PWM_FREQ, PWM_RES);
  Serial.println("PWM setup for pin " + String(pin) + ": " + (success ? "SUCCESS" : "FAILED"));
  
  // Set initial low value to test
  ledcWrite(pin, 0);
  Serial.println("Initial PWM set to 0 for pin " + String(pin));
}

void applyFanSpeeds() {
  int intakeValue = autoMode ? computePWMFromTemp() : intakePWM;
  int exhaustValue = autoMode ? computePWMFromTemp() : exhaustPWM;

  ledcWrite(intakeFan1, intakeValue);
  ledcWrite(intakeFan2, intakeValue);
  ledcWrite(exhaustFan1, exhaustValue);
  ledcWrite(exhaustFan2, exhaustValue);
  
  // Debug output with more detail
  Serial.println("=== PWM Update ===");
  Serial.println("Auto mode: " + String(autoMode ? "ON" : "OFF"));
  Serial.println("Temperature: " + String(temperatureC) + "°C");
  Serial.println("Intake PWM target: " + String(intakePWM) + " -> Applied: " + String(intakeValue));
  Serial.println("Exhaust PWM target: " + String(exhaustPWM) + " -> Applied: " + String(exhaustValue));
  Serial.println("Pin " + String(intakeFan1) + " set to: " + String(intakeValue));
  Serial.println("==================");
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

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32 Fan Controller Starting ===");
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
  pinMode(intakeFan1Tach, INPUT_PULLUP);  // Pin 32 - OK
  pinMode(intakeFan2Tach, INPUT_PULLUP);  // Pin 33 - OK  
  pinMode(exhaustFan1Tach, INPUT_PULLUP); // Pin 34 - input only, no pullup but try anyway
  pinMode(exhaustFan2Tach, INPUT_PULLUP); // Pin 13 - OK
  
  attachInterrupt(digitalPinToInterrupt(intakeFan1Tach), tachISR1, FALLING);
  attachInterrupt(digitalPinToInterrupt(intakeFan2Tach), tachISR2, FALLING);
  // Skip pin 34 for now since it's input-only and problematic
  // attachInterrupt(digitalPinToInterrupt(exhaustFan1Tach), tachISR3, FALLING);
  attachInterrupt(digitalPinToInterrupt(exhaustFan2Tach), tachISR4, FALLING);
  
  lastRPMCalc = millis();
  Serial.println("Tachometer interrupts setup complete");
  Serial.println("Active tach pins: 32, 33, 13 (pin 34 skipped)");

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
      <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .fan-section { border: 1px solid #ccc; margin: 10px 0; padding: 15px; }
        .rpm-display { font-weight: bold; color: #007acc; }
      </style>
      </head><body>
      <h1>Fan Controller</h1>
      <div class="fan-section">
        <h2>Intake Fans</h2>
        <h3>Speed: <span id="intVal">128</span></h3>
        <input type="range" min="0" max="255" value="128" id="intakeSlider" onchange="updateFan('intake', this.value)">
        <p>Fan 1 RPM: <span id="rpm1" class="rpm-display">--</span></p>
        <p>Fan 2 RPM: <span id="rpm2" class="rpm-display">--</span></p>
      </div>
      
      <div class="fan-section">
        <h2>Exhaust Fans</h2>
        <h3>Speed: <span id="extVal">128</span></h3>
        <input type="range" min="0" max="255" value="128" id="exhaustSlider" onchange="updateFan('exhaust', this.value)">
        <p>Fan 3 RPM: <span id="rpm3" class="rpm-display">--</span></p>
        <p>Fan 4 RPM: <span id="rpm4" class="rpm-display">--</span></p>
      </div>
      
      <div class="fan-section">
        <h2>Troubleshooting</h2>
        <button onclick="runPWMTest()">Quick PWM Test</button>
        <p><small>Quickly cycles through 0, 128, 255 PWM values</small></p>
        
        <h3>Manual PWM Control</h3>
        <input type="number" id="manualPWM" min="0" max="255" value="128" placeholder="0-255">
        <button onclick="setManualPWM()">Set PWM</button>
        <p><small>Directly set PWM value (0-255) for testing</small></p>
        
        <h3>OTA Updates</h3>
        <p><strong>Hostname:</strong> ESP32-FanController</p>
        <p><strong>Password:</strong> (stored in secrets.h)</p>
        <p><small>Use Arduino IDE → Tools → Port → Network Ports to upload wirelessly</small></p>
      </div>
      
      <div class="fan-section">
        <h2>Auto Mode: <input type="checkbox" id="autoToggle" onchange="toggleAuto(this.checked)"></h2>
        <p>Temperature: <span id="temp">--</span> °C</p>
      </div>
      
      <script>
        function updateFan(type, val) {
          document.getElementById(type === 'intake' ? 'intVal' : 'extVal').textContent = val;
          fetch('/set?fan=' + type + '&val=' + val);
        }
        function toggleAuto(state) {
          fetch('/auto?mode=' + (state ? '1' : '0'));
        }
        function runPWMTest() {
          fetch('/test');
          alert('Quick PWM test running - check serial monitor');
        }
        function setManualPWM() {
          const pwmValue = document.getElementById('manualPWM').value;
          fetch('/manual?pwm=' + pwmValue);
          alert('PWM set to ' + pwmValue + ' - check serial monitor');
        }
        setInterval(() => {
          fetch('/temp').then(res => res.text()).then(t => document.getElementById('temp').textContent = t);
          fetch('/rpm').then(res => res.text()).then(data => {
            const rpms = data.split(',');
            document.getElementById('rpm1').textContent = rpms[0];
            document.getElementById('rpm2').textContent = rpms[1];
            document.getElementById('rpm3').textContent = rpms[2];
            document.getElementById('rpm4').textContent = rpms[3];
          });
        }, 3000);
      </script>
      </body></html>
    )rawliteral";
    request->send(200, "text/html", html);
  });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("fan") && request->hasParam("val")) {
      String fan = request->getParam("fan")->value();
      int val = request->getParam("val")->value().toInt();
      Serial.println("Setting " + fan + " fan to " + String(val));
      if (fan == "intake") intakePWM = val;
      else if (fan == "exhaust") exhaustPWM = val;
      applyFanSpeeds();
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/auto", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("mode")) {
      autoMode = request->getParam("mode")->value() == "1";
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

  server.begin();
}

void loop() {
  ArduinoOTA.handle();  // Handle OTA updates
  
  static unsigned long lastTempRead = 0;
  if (millis() - lastTempRead > 3000) {
    lastTempRead = millis();
    readTemp();
    if (autoMode) {
      applyFanSpeeds();
    }
  }
  
  calculateRPM();
}