#pragma once
#include <Arduino.h>

// =============================================================================
//  calibration.h — SINGLE SOURCE OF ELECTRICAL TRUTH for the Karen valve system
//  Rev K · post short-fix review (2026-07-27)
//
//  Every build (deploy / bench / calibrate) #includes this. If a value lives
//  here it must NOT be redefined in any main.cpp — change it once, rebuild all
//  three. This is what stops a "95°" in one firmware from meaning something
//  different in another.
//
//  CHANGES in this revision:
//    • railOn()/railOff() track rail state in a variable instead of
//      digitalRead() on an OUTPUT pin (unreliable across ESP32 cores; a false
//      "off" would have re-run the old pulse dance and cut the rail mid-hold).
//    • railOn() is now a single clean edge + RAIL_SETTLE_MS. The old
//      prime/rest/go pulse sequence was a workaround for the SERVO_RAIL short
//      (rail sat at ~3V and servos half-booted); root cause is fixed, and
//      re-pulsing doubles inrush events. railCycle() remains for deliberate
//      power-cycle tests from the calibrate CLI.
//    • Shared ≤2-servo concurrency tracker (design rule: valve + common,
//      never more). All builds must route drive/release through it.
//    • latchOff() added for the deploy build's real mission-final shutdown.
//    • Fail-safe note corrected: Rev K already has the float-window pulldowns
//      in copper (R15, R13) — verify with a meter, no bodges required.
// =============================================================================

// ---- I2C + PCA9685 boards ---------------------------------------------------
constexpr uint8_t I2C_SDA   = 21;
constexpr uint8_t I2C_SCL   = 22;
constexpr uint8_t PCA0_ADDR = 0x40;    // channels 0..15
constexpr uint8_t PCA1_ADDR = 0x41;    // channels 16..31 (local = ch - 16)

// ---- Servo PWM --------------------------------------------------------------
constexpr uint8_t  SERVO_FREQ_HZ   = 50;
constexpr uint16_t PULSE_MIN_COUNT = 102;   // ~500 us   (0 deg)
constexpr uint16_t PULSE_MAX_COUNT = 512;   // ~2500 us  (180 deg)

// Degrees -> PCA9685 count. THE mapping. Do not diverge anywhere.
// (1 count = 20 ms / 4096 ≈ 4.88 us at 50 Hz; 307 ≈ 1500 us = 90 deg.)
inline uint16_t degToPulse(uint8_t deg) {
  return (uint16_t)map(deg, 0, 180, PULSE_MIN_COUNT, PULSE_MAX_COUNT);
}

// ---- Valve channel layout ---------------------------------------------------
constexpr uint8_t MAIN_SERVO_CH      = 20;   // main intake valve
constexpr uint8_t SAMPLE_SERVO_COUNT = 20;   // sample valves = Ch0..19
constexpr uint8_t SERVO_CHANNELS[SAMPLE_SERVO_COUNT] =
  {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};

// ---- Calibration result — PER LANDER ----------------------------------------
//  Open angles are trimmed to each INDIVIDUAL servo (visual centre, ~90°) and
//  therefore differ per physical lander. Each lander carries its own 21-value
//  row in CH_OPEN_DEG_ALL, selected at runtime from the unit ID stored in NVS
//  (see the assembly build's loadUnitId()). ONE binary ships to all four units.
//
//  Closed = open + CLOSE_OFFSET_DEG, clamped to 180 in the build. 65° is the
//  max offset that keeps every channel at/under 180: Ch13 opens at 115°, so
//  115 + 65 = 180 exactly — Ch13 is the binding channel that sets this ceiling.
//
//  WORKFLOW per unit:  `setid <n>` on the calibrate build (stamps NVS) →
//  calibrate the 21 servos → paste that unit's open angles into row (n-1)
//  below → flip CAL_COMMITTED[n-1] to true → commit → flash the assembly build.
//  A unit whose row is still a placeholder (CAL_COMMITTED false) REFUSES TO ARM.
constexpr uint8_t CLOSE_OFFSET_DEG = 65;     // !! MAX 65 — Ch13 (115°) hits 180 !!
constexpr uint8_t LANDER_COUNT     = 4;

//  Rows are indexed [unitID - 1].  Unit IDs are 1..4  →  SSID LanderController1..4
constexpr uint8_t CH_OPEN_DEG_ALL[LANDER_COUNT][21] = {
  // ── Lander 1 ──  existing calibrated set
  { 100, 110, 100,  95,  95, 105,  95, 110,    // Ch0-7
     95, 105, 100, 105,  95, 115, 110,  95,    // Ch8-15
    100,  95,  95,  90,  90 },                 // Ch16-20 (Ch20 = main)
  // ── Lander 2 ──  PLACEHOLDER (copy of L1) — REPLACE with L2 bench angles
  { 100, 110, 100,  95,  95, 105,  95, 110,
     95, 105, 100, 105,  95, 115, 110,  95,
    100,  95,  95,  90,  90 },
  // ── Lander 3 ──  PLACEHOLDER (copy of L1) — REPLACE with L3 bench angles
  { 100, 110, 100,  95,  95, 105,  95, 110,
     95, 105, 100, 105,  95, 115, 110,  95,
    100,  95,  95,  90,  90 },
  // ── Lander 4 ──  PLACEHOLDER (copy of L1) — REPLACE with L4 bench angles
  { 100, 110, 100,  95,  95, 105,  95, 110,
     95, 105, 100, 105,  95, 115, 110,  95,
    100,  95,  95,  90,  90 },
};

//  Flip a row's flag to true ONLY after that physical unit's angles are
//  bench-confirmed and pasted above. Placeholder rows stay false → the assembly
//  build refuses to arm them. (L1 is marked true as the existing set; if that
//  set is generic or belongs to a different physical unit, set this false and
//  re-confirm it like the others.)
constexpr bool CAL_COMMITTED[LANDER_COUNT] = { true, false, false, false };

//  Bounds-checked row accessor. id is 1..LANDER_COUNT; returns nullptr when the
//  ID is unset (0) or out of range, so callers can fail safe.
inline const uint8_t* chOpenRow(uint8_t id) {
  return (id >= 1 && id <= LANDER_COUNT) ? CH_OPEN_DEG_ALL[id - 1] : nullptr;
}
inline bool calCommitted(uint8_t id) {
  return (id >= 1 && id <= LANDER_COUNT) && CAL_COMMITTED[id - 1];
}

// Mechanical floor for any commanded angle. Valve linkages bind below this;
// the calibrate CLI refuses smaller values. (Bench "raw" mode should too.)
constexpr uint8_t SERVO_MIN_DEG = 70;

// ---- Concurrency rule -------------------------------------------------------
//  Design rule (Howard): only one or two servos move/hold at a time — the
//  selected valve plus Common, never more. This is enforced structurally at
//  the drive-primitive level so NO code path (UI, CLI, state machine, park
//  routine) can stampede the servo rail.
constexpr uint8_t MAX_ACTIVE_SERVOS = 2;

//  Shared active-channel tracker. Every build must:
//    • call servoGuardAllows(ch) before driving a channel, and refuse if false
//    • call servoMarkActive(ch, true)  after driving
//    • call servoMarkActive(ch, false) after releasing (PWM off / limp)
//  (Function-local static in an inline function = exactly one instance across
//  all translation units, no C++17 required.)
inline bool* _servoActiveArr() { static bool a[32] = {false}; return a; }

inline uint8_t servoActiveCount() {
  uint8_t n = 0; bool* a = _servoActiveArr();
  for (uint8_t i = 0; i < 32; i++) if (a[i]) n++;
  return n;
}
inline bool servoIsActive(uint8_t ch) { return (ch < 32) && _servoActiveArr()[ch]; }
inline void servoMarkActive(uint8_t ch, bool on) { if (ch < 32) _servoActiveArr()[ch] = on; }

// Re-commanding an already-active channel is always allowed; adding a NEW
// active channel is allowed only below the limit.
inline bool servoGuardAllows(uint8_t ch) {
  if (ch >= 32) return false;
  if (_servoActiveArr()[ch]) return true;
  return servoActiveCount() < MAX_ACTIVE_SERVOS;
}
// Print the active list, e.g. "Ch0, Ch20" (or "none"), for guard messages.
inline void printActiveServos(Stream& s) {
  bool* a = _servoActiveArr(); bool first = true;
  for (uint8_t i = 0; i < 32; i++)
    if (a[i]) { s.printf(first ? "Ch%u" : ", Ch%u", i); first = false; }
  if (first) s.print("none");
}

// ---- Pins -------------------------------------------------------------------
constexpr uint8_t LED_PIN             = 27;      // GPIO27 -> R16 2.2k -> LED1
constexpr uint8_t VBATT_SENSE_PIN     = 34;      // GPIO34, ADC1_CH6
constexpr float   VBATT_DIVIDER_RATIO = 0.3125f; // 220k top / 100k bottom

// ---- Hardware-gated power (Rev K) -------------------------------------------
//  SERVO_EN (GPIO25): HIGH -> R1 -> Q1 -> Q2 gate low -> SERVO_RAIL energized.
//                     LOW = rail off.
//  SHUTDOWN (GPIO26): -> R12 -> U10 OFF. Drive HIGH (pulse) to kill the latch
//                     and power the whole system down. Keep LOW otherwise.
//
//  Float-window fail-safes — ALREADY IN COPPER ON REV K (verify with meter):
//    • R15 100k (Q1 base -> GND) holds the rail OFF while GPIO25 is high-Z
//      (power-on, deep sleep, brownout restart).
//    • R13 100k (U10 OFF -> GND) keeps the latch ALIVE while GPIO26 is high-Z.
//  So the board fails safe in both directions with no firmware running and no
//  bodge pulldowns. Firmware still drives both pins to defined levels ASAP.
constexpr uint8_t SERVO_EN_PIN = 25;
constexpr uint8_t SHUTDOWN_PIN = 26;

constexpr uint16_t RAIL_SETTLE_MS = 250;   // cap-bank charge + servo boot time

// Rail state tracker (single instance across all builds; see note above).
inline bool& _railState() { static bool on = false; return on; }
inline bool  railIsOn()   { return _railState(); }

// Call ONCE at the very top of setup(), before Serial, before anything.
// SHUTDOWN first (stay powered), then rail off at a defined level.
inline void initPowerPins() {
  pinMode(SHUTDOWN_PIN, OUTPUT); digitalWrite(SHUTDOWN_PIN, LOW);   // stay powered
  pinMode(SERVO_EN_PIN, OUTPUT); digitalWrite(SERVO_EN_PIN, LOW);   // rail off, defined
  _railState() = false;
}

// Single clean rising edge + settle. Call AFTER the PWM signal is set so the
// servo sees a valid pulse train the instant power arrives. No-op if already on.
inline void railOn() {
  if (_railState()) return;
  digitalWrite(SERVO_EN_PIN, HIGH);
  _railState() = true;
  delay(RAIL_SETTLE_MS);
}
inline void railOff() {
  digitalWrite(SERVO_EN_PIN, LOW);
  _railState() = false;
}
// Deliberate power-cycle (calibrate CLI `cycle` command / edge testing).
inline void railCycle(uint32_t offMs = 2000) {
  railOff(); delay(offMs); railOn();
}

// Mission-final shutdown: pulse U10 OFF via R12. On battery power, execution
// ends inside the delay (the latch cuts our own supply). The tail only runs
// if we're being kept alive by USB — caller should deep-sleep afterwards.
inline void latchOff() {
  digitalWrite(SHUTDOWN_PIN, HIGH);
  delay(1000);
  digitalWrite(SHUTDOWN_PIN, LOW);
}