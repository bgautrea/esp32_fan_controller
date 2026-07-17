#pragma once
#include <Arduino.h>

// =====================================================================
//  ESP32 Server-Cabinet Fan Controller — hardware configuration
//  Everything that depends on how the board is wired lives here. Ported
//  from the original single-file Arduino sketch.
// =====================================================================

// ---------- PWM ----------
#define PWM_FREQ  25000     // 25 kHz — above audible, standard for 4-pin PC fans
#define PWM_RES   8         // 8-bit duty (0-255)
// Some fans don't like 25 kHz; 1 kHz is the usual fallback:
// #define PWM_FREQ  1000

// ---------- Fan PWM pins + LEDC channels ----------
// This project builds against ESP32 Arduino core 2.x (see platformio.ini),
// whose LEDC API is channel-based: ledcSetup(ch)/ledcAttachPin(pin,ch)/
// ledcWrite(ch). Each fan output gets its own channel.
#define INTAKE_FAN1_PIN   25
#define INTAKE_FAN2_PIN   26
#define EXHAUST_FAN1_PIN  27
#define EXHAUST_FAN2_PIN  14

#define INTAKE_FAN1_CH    0
#define INTAKE_FAN2_CH    1
#define EXHAUST_FAN1_CH   2
#define EXHAUST_FAN2_CH   3

// ---------- Tachometer pins ----------
// Standard 4-pin fan: green wire = tach (sense). Most PC fans emit 2 pulses
// per revolution. All four are real GPIOs with internal pull-ups and interrupt
// support (the original sketch had exhaust fan 1 on the input-only GPIO34 and
// skipped it — the green wire actually lands on GPIO12).
#define INTAKE_FAN1_TACH   32
#define INTAKE_FAN2_TACH   33
#define EXHAUST_FAN1_TACH  12   // NOTE: GPIO12 is a boot strapping pin (see main.cpp)
#define EXHAUST_FAN2_TACH  13
#define TACH_PULSES_PER_REV 2

// Ignore tach edges closer together than this. A fan's open-collector tach has
// slow/ringing edges that make the ESP32's edge interrupt fire several times
// per real pulse, inflating the count. The fans here are Noctua NF-F12 PWM,
// max 1500 RPM = 2 pulses/rev = 50 Hz = a 20 ms pulse spacing, so an 8 ms window
// blocks the spurious double-edges while never dropping a real pulse (that would
// need >3750 RPM). Empirically: 1 ms window read ~4x high, 2.5 ms ~2x, 8 ms ~1x.
#define TACH_DEBOUNCE_US  8000

// ---------- Temperature sensor ----------
#define DHT_PIN   4
#define DHT_TYPE  DHT22

// ---------- Auto-mode temperature curve ----------
// computePWMFromTemp(): PWM duty as a function of temperature (°C).
// Below the first threshold uses AUTO_PWM_MIN; each threshold bumps the duty.
#define AUTO_TEMP_1  25   // < 25 °C
#define AUTO_TEMP_2  30   // < 30 °C
#define AUTO_TEMP_3  35   // < 35 °C
#define AUTO_PWM_1   50   // duty below AUTO_TEMP_1
#define AUTO_PWM_2   100  // duty below AUTO_TEMP_2
#define AUTO_PWM_3   180  // duty below AUTO_TEMP_3
#define AUTO_PWM_4   255  // duty at/above AUTO_TEMP_3

// ---------- Defaults ----------
#define DEFAULT_INTAKE_PWM   220
#define DEFAULT_EXHAUST_PWM  220
#define DEFAULT_AUTO_MODE    false

// mDNS hostname -> http://fan.local/
#define MDNS_HOSTNAME  "fan"
