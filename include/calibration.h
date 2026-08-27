#pragma once
#include <Arduino.h>

// =============================================================================
//  calibration.h — electrical truth for the Karen valve system (Rev K).
//  Included by deploy / minideploy / calibrate. If a value lives here, do not
//  redefine it in a main.cpp. Mission timing lives in schedule.h.
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
//  Declared here because SAMPLE_ORDER_ALL below needs the bound; the
//  authoritative LANDER_COUNT sits with the calibration table and is
//  static_asserted equal to it.
constexpr uint8_t LANDER_COUNT_FWD   = 4;
constexpr uint8_t MAIN_SERVO_CH      = 20;   // main intake valve
constexpr uint8_t SAMPLE_SERVO_COUNT = 20;   // sample valves = Ch0..19
//  The SET of sample channels, ascending. Use this wherever every sample valve
//  must be touched and the order does not matter (e.g. the calibrate build's
//  `rest` sweep). It is NOT the mission order — see SAMPLE_ORDER_ALL.
constexpr uint8_t SERVO_CHANNELS[SAMPLE_SERVO_COUNT] =
  {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};

// ---- Mission sampling ORDER — PER LANDER ------------------------------------
//  Which physical valve each scheduled event drives. Event i of the schedule
//  drives SAMPLE_ORDER_ALL[unitID-1][i]. The TIMES are fixed by schedule.h and
//  are the same on every unit; this table decides which valve gets which
//  timepoint, and it differs per unit because the manifolds are not all
//  plumbed the same way round.
//
//  Every row must be a permutation of 0..SAMPLE_SERVO_COUNT-1 — each sample
//  valve used exactly once. _calOrderAllOk() fails the BUILD otherwise, which
//  is the check that catches a duplicated channel (one valve sampled twice,
//  another never opened — invisible at runtime on open-loop servos).
//
//  Angles are NOT reordered with this: CH_OPEN_DEG_ALL stays indexed by
//  physical channel, so a channel keeps its own calibration wherever it lands
//  in the order.
//
//  CHANGING AN ORDER MID-MISSION MIS-MAPS EVERYTHING. st.evtIdx is an event
//  index, so a unit that has already taken samples would resume against the new
//  order. Change these only pre-flight, and `clearmission` afterwards.
constexpr uint8_t SAMPLE_ORDER_ALL[LANDER_COUNT_FWD][SAMPLE_SERVO_COUNT] = {
  // unit 1 - Lander D : default, ascending
  {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19},
  // unit 2 - Lander A : default, ascending
  {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19},
  // unit 3 - Lander B : default, ascending
  {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19},
  // unit 4 - Lander C : back/front rows swapped 2026-08-25, so the back row
  //   (Ch10-19) is sampled first and the front row (Ch0-9) second. Within the
  //   back row Ch17 and Ch18 are also transposed - the two valves were fitted
  //   the other way round. Ch19 keeps its place at the end of the back row.
  {10,11,12,13,14,15,16,18,17,19, 0,1,2,3,4,5,6,7,8,9},
};

// ---- Per-lander calibration ------------------------------------------------
//  Open angles are trimmed per servo (visual centre, ~90 deg) so they differ per
//  unit. Row selected at runtime from the unit ID in NVS; one binary ships to
//  all four. A row with CAL_COMMITTED false REFUSES TO ARM.
//
//  Per unit:  setid <n> (calibrate build) -> calibrate 21 servos -> paste the
//  open angles into row (n-1) -> CAL_COMMITTED[n-1] = true -> commit -> flash.
//
//  Sample valves (Ch0-19) close at open + CLOSE_OFFSET_DEG, clamped to 180.
//  65 is the arithmetic ceiling: the highest committed open angle is 115
//  (Lander D, Ch13), and 115 + 65 = 180 exactly. The offset is currently 60.
//  _calRowCloseOk() fails the BUILD if any sample channel would clamp, so the
//  ceiling is enforced rather than remembered.
//  Ch20 (common) uses its own offset — see MAIN_CLOSE_OFFSET_DEG.
//
//  LABELING: the bench calls these Landers A/B/C/D; the firmware only knows
//  1-4. Nothing enforces the mapping — update this comment if a unit is
//  relabelled.
//    unit 1 = D    unit 2 = A    unit 3 = B    unit 4 = C
constexpr uint8_t CLOSE_OFFSET_DEG = 60;     // current 60 · hard max 65 (115+65=180)
constexpr uint8_t LANDER_COUNT     = 4;

//  ---- Common valve (Ch20) close angle ---------------------------------------
//  Closes at open + MAIN_CLOSE_OFFSET_DEG, capped at MAIN_CLOSE_MAX_DEG.
//  An offset, not a fixed angle: a pinch valve closes as a function of TRAVEL
//  from its calibrated open position, so a fixed angle gave the fleet
//  inconsistent travel (70-85 deg).
//
//  Currently 55. The original 45 was measured on Lander A (Ch20 open 105, best
//  close 150). Both the offset and the open angles have moved several times
//  since, so THIS COMMENT DELIBERATELY DOES NOT LIST PER-UNIT VALUES — they go
//  stale. `python tools/check_calibration.py` prints the live figures
//  ("common close per unit"), and the calibrate CLI's `table` prints the
//  stamped unit's. The invariant that matters is enforced, not remembered:
//  _calMainCloseFits() fails the BUILD if any unit's Ch20 open + this offset
//  exceeds MAIN_CLOSE_MAX_DEG.
//  INTERIM: "a good seal, not a perfect one". Still less travel than a sample
//  valve gets, which reverses an older assumption that the common valve
//  needed near-full travel.
//
//  Assumes 55 deg of travel is the same physical motion on every unit, which
//  holds only if "open" was calibrated to the same physical position on all
//  four. Confirm sealed + no creep at ~8 degC in oil.
constexpr uint8_t MAIN_CLOSE_OFFSET_DEG = 55;    // travel from open, per unit
constexpr uint8_t MAIN_CLOSE_MAX_DEG    = 175;   // hard ceiling: 5° off the stop

//  Rows are indexed [unitID - 1].  Unit IDs are 1..4  →  SSID LanderController1..4
constexpr uint8_t CH_OPEN_DEG_ALL[LANDER_COUNT][21] = {
  // ── unit 1 · Lander D ──  existing calibrated set (unchanged)
  { 100, 110, 100,  95,  95, 105,  95, 110,    // Ch0-7
     95, 105, 100, 105,  95, 115, 110,  95,    // Ch8-15
    100,  95,  95,  90,  90 },                 // Ch16-20 (Ch20 = main)
  // ── unit 2 · Lander A ──  bench-confirmed 2026-08-18
  {  95,  90,  90,  95, 95,  90,  90,  90,    // Ch0-7
    100, 90, 90,  90, 95,  90,  90, 105,    // Ch8-15
  95, 90,  90,  100, 100 },                  // Ch16-20 (Ch20 = main)     NNNBMV                      MMCX      
  {  95, 90, 90, 95, 90, 90,  95, 90,    // Ch0-720 95
    
   95,  95, 95,  90,  95, 95,  95, 95,     // Ch8-15
   95, 95, 95, 95,  90},                   // Ch16-20 (Ch20 = main)
  // ── unit 4 · Lander C ──  bench-confirmed 2026-08-18
  {  85, 80, 85, 85, 85, 85,  85, 85,    // Ch0-7
   85,  85, 80,  85,  85, 85,  90, 85,     // Ch8-15
   90, 90, 90, 85,  90 },                   // Ch16-20 (Ch20 = main)
};

//  Flip a row's flag to true ONLY after that physical unit's angles are
//  bench-confirmed and pasted above. An uncommitted row stays false and the
//  FLIGHT build refuses to arm that unit (calCommitted() -> g_idOk). The flag
//  gates ARMING ONLY: the angles are still loaded into CH_OPEN and are fully
//  usable, so an uncommitted unit behaves normally on the calibrate build --
//  setid, table, rest, jogging and hold-checks all work.
//
//  unit 3 - Lander B: UNCOMMITTED 2026-08-25. Its open angles were lowered
//  after a fuse was burnt closing too far, and an inner tube was fitted to ease
//  closing and stop the tubes being cut. Those new angles are NOT yet
//  bench-verified. Flip back to true once the water check confirms them.
//
//  (Lander C's Ch18-19 are placeholders, not real angles — see the
//  note on that row above; this does NOT block arming C, since the rest of
//  its board is real and those 2 dead ports are a known, accepted gap.)
constexpr bool CAL_COMMITTED[LANDER_COUNT] = { true, true, false, true };
//                                              D     A     B     C
//                                                          ^ pending water check

//  Bounds-checked row accessor. id is 1..LANDER_COUNT; returns nullptr when the
//  ID is unset (0) or out of range, so callers can fail safe.
inline const uint8_t* chOpenRow(uint8_t id) {
  return (id >= 1 && id <= LANDER_COUNT) ? CH_OPEN_DEG_ALL[id - 1] : nullptr;
}
inline bool calCommitted(uint8_t id) {
  return (id >= 1 && id <= LANDER_COUNT) && CAL_COMMITTED[id - 1];
}

//  This unit's mission sampling order, or nullptr when the ID is unset/out of
//  range — same fail-safe contract as chOpenRow().
inline const uint8_t* sampleOrderRow(uint8_t id) {
  return (id >= 1 && id <= LANDER_COUNT) ? SAMPLE_ORDER_ALL[id - 1] : nullptr;
}

// Closed-angle policy for a channel — THE single formula, shared by every
// build (deploy's closeServo()/buildPulseTables(), the calibrate CLI's
// `table` command). Sample valves (Ch0-19) close at open+CLOSE_OFFSET_DEG,
// clamped to 180. The main valve (Ch20/MAIN_SERVO_CH) closes to
// open + MAIN_CLOSE_OFFSET_DEG, capped at MAIN_CLOSE_MAX_DEG — its own offset
// and its own ceiling, independent of the sample valves'.
// Do not reimplement this inline anywhere else.
inline uint8_t closeDegFor(uint8_t ch, uint8_t openDeg) {
  if (ch == MAIN_SERVO_CH)
    return (uint8_t)min((int)openDeg + MAIN_CLOSE_OFFSET_DEG, (int)MAIN_CLOSE_MAX_DEG);
  return (uint8_t)min((int)openDeg + CLOSE_OFFSET_DEG, 180);
}

// Mechanical floor for any commanded angle. Valve linkages bind below this;
// the calibrate CLI refuses smaller values. (Bench "raw" mode should too.)
constexpr uint8_t SERVO_MIN_DEG = 70;

// ---- Concurrency rule ------------------------------------------------------
//  Design rule (Howard): only one or two servos MOVE at a time — the selected
//  valve plus common. Enforced at the drive primitive so no code path can
//  stampede the rail. See railUpAllCommanded() in the builds for the one
//  documented exception: all 21 are briefly HELD (not moved) at rail-up.
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

// Single clean rising edge + settle. No-op if already on.
//
// *** WARNING — read before calling this directly. ***
// Any servo powered by this call WITHOUT a valid pulse train already loaded
// drives to its ~90 deg neutral. Open angles are calibrated to "visual centre,
// ~90 deg", so that is functionally OPEN on all 21 channels: a bare railOn()
// from a cold rail throws every uncommanded valve open. Measured on HPS-2018
// 2026-08-19 — it reproduces with the signal wire physically unplugged, so it
// is the servo, not the PCA9685.
//
// Load pulses for ALL channels first. Both builds provide railUpAllCommanded()
// for exactly this; call that, not this, when the rail is cold.
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

// ---- Compile-time validation of the tables above -----------------------------
//  tools/check_calibration.py runs equivalent checks, but only in CI — and
//  firmware is routinely flashed from a locally-edited copy of this header
//  without pushing first, so CI cannot be the only gate. These fail the BUILD.
//
//  Written as recursive constexpr rather than loops so they work under C++11.
//  Do not rewrite with for-loops unless every env in platformio.ini is C++14+.

constexpr bool _calAngleOk(uint8_t deg) {
  return deg >= SERVO_MIN_DEG && deg <= 180;
}
// Every open angle in row r is inside the mechanical limits.
constexpr bool _calRowOpenOk(uint8_t r, uint8_t i = 0) {
  return (i >= 21) ? true
                   : (_calAngleOk(CH_OPEN_DEG_ALL[r][i]) && _calRowOpenOk(r, i + 1));
}
// Every SAMPLE channel (Ch0..19) reaches open+CLOSE_OFFSET_DEG without being
// clamped at 180. Ch20 is excluded — it has its own offset and cap, checked
// separately by _calMainCloseFits().
// This is the check that would have caught a pasted open angle above 115°.
constexpr bool _calRowCloseOk(uint8_t r, uint8_t i = 0) {
  return (i >= SAMPLE_SERVO_COUNT)
           ? true
           : ((CH_OPEN_DEG_ALL[r][i] + CLOSE_OFFSET_DEG <= 180) && _calRowCloseOk(r, i + 1));
}
constexpr bool _calAllRowsOk(uint8_t r = 0) {
  return (r >= LANDER_COUNT) ? true
                             : (_calRowOpenOk(r) && _calRowCloseOk(r) && _calAllRowsOk(r + 1));
}
// The main valve's full offset must fit under the ceiling on EVERY unit. If it
// clamps, that unit silently gets less travel than the others — which is
// exactly the inconsistency the offset was adopted to remove. The binding case
// is whichever unit has the highest Ch20 open angle — run the lint to see which
// that currently is, rather than trusting a number written here.
constexpr bool _calMainCloseFits(uint8_t r = 0) {
  return (r >= LANDER_COUNT)
           ? true
           : ((CH_OPEN_DEG_ALL[r][MAIN_SERVO_CH] + MAIN_CLOSE_OFFSET_DEG <= MAIN_CLOSE_MAX_DEG)
              && _calMainCloseFits(r + 1));
}

static_assert(_calAllRowsOk(),
              "calibration.h: an open angle is outside [SERVO_MIN_DEG,180], or a sample "
              "channel's open+CLOSE_OFFSET_DEG would clamp at 180. Fix CH_OPEN_DEG_ALL "
              "or lower CLOSE_OFFSET_DEG.");
static_assert(_calAngleOk(MAIN_CLOSE_MAX_DEG) && MAIN_CLOSE_MAX_DEG < 180,
              "calibration.h: MAIN_CLOSE_MAX_DEG must be inside [SERVO_MIN_DEG,180) — it is "
              "the cushion that keeps the common valve off its mechanical stop.");
static_assert(MAIN_CLOSE_OFFSET_DEG > 0,
              "calibration.h: MAIN_CLOSE_OFFSET_DEG must be positive — the common valve "
              "closes by moving AWAY from its open angle.");
static_assert(_calMainCloseFits(),
              "calibration.h: some unit's Ch20 open + MAIN_CLOSE_OFFSET_DEG exceeds "
              "MAIN_CLOSE_MAX_DEG, so that unit would clamp and get less travel than the "
              "rest of the fleet. Lower the offset, or lower that unit's Ch20 open angle.");
// Every SAMPLE_ORDER_ALL row must be a permutation of 0..SAMPLE_SERVO_COUNT-1:
// each sample valve used exactly once. A duplicate would sample one valve twice
// and leave another shut for the whole mission, and with open-loop servos
// nothing at runtime could tell. Recursive constexpr, C++11-safe.
constexpr uint8_t _calOrderCount(uint8_t r, uint8_t c, uint8_t i = 0) {
  return (i >= SAMPLE_SERVO_COUNT)
           ? 0
           : (uint8_t)((SAMPLE_ORDER_ALL[r][i] == c ? 1 : 0) + _calOrderCount(r, c, i + 1));
}
constexpr bool _calOrderRowOk(uint8_t r, uint8_t c = 0) {
  return (c >= SAMPLE_SERVO_COUNT)
           ? true
           : (_calOrderCount(r, c) == 1 && _calOrderRowOk(r, c + 1));
}
constexpr bool _calOrderAllOk(uint8_t r = 0) {
  return (r >= LANDER_COUNT) ? true : (_calOrderRowOk(r) && _calOrderAllOk(r + 1));
}
static_assert(_calOrderAllOk(),
              "calibration.h: a SAMPLE_ORDER_ALL row is not a permutation of 0..19 — some "
              "sample valve is used twice and another never opened. Fix the order table.");
static_assert(LANDER_COUNT == LANDER_COUNT_FWD,
              "calibration.h: LANDER_COUNT_FWD (the SAMPLE_ORDER_ALL bound) must match "
              "LANDER_COUNT.");
static_assert(MAIN_SERVO_CH == SAMPLE_SERVO_COUNT,
              "calibration.h: MAIN_SERVO_CH must sit immediately above the sample channels; "
              "buildPulseTables() and the park routines iterate 0..MAIN_SERVO_CH.");
static_assert(MAX_ACTIVE_SERVOS >= 1 && MAX_ACTIVE_SERVOS <= 2,
              "calibration.h: the certified power path is 1-2 concurrent servos.");
static_assert(PULSE_MIN_COUNT < PULSE_MAX_COUNT && PULSE_MAX_COUNT < 4096,
              "calibration.h: pulse counts must be ordered and inside the PCA9685 12-bit range "
              "(4096 is the full-OFF value used by releaseServo()).");