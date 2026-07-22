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
// skipped it — the green wires actually land on GPIO13 and GPIO12). Pin mapping
// matches the canonical assignment on the Arduino main branch.
#define INTAKE_FAN1_TACH   32
#define INTAKE_FAN2_TACH   33
#define EXHAUST_FAN1_TACH  13
#define EXHAUST_FAN2_TACH  12   // NOTE: GPIO12 is a boot strapping pin (see main.cpp)
#define TACH_PULSES_PER_REV 2

// Ignore tach edges closer together than this. At low PWM duty, PSU ripple
// (120 Hz US mains, 100 Hz EU) couples onto the open-collector tach line and
// fires the edge interrupt several times per real pulse, inflating the count.
// The fans are Noctua NF-F12 PWM, max 1500 RPM = 2 pulses/rev = 50 Hz = 20 ms
// spacing, so a 10 ms window rejects the ripple (8-10 ms apart) while never
// dropping a real pulse (that would need >3000 RPM). Empirically: 1 ms read ~4x
// high, 2.5 ms ~2x, 8-10 ms ~1x. True hardware fix: 100 nF from each tach pin
// to GND. (Root cause + 10 ms value carried over from the Arduino main branch.)
#define TACH_DEBOUNCE_US  10000

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
