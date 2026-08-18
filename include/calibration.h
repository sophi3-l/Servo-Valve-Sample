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
//
//  CHANGES 2026-08-18 (calibration update — A/B/C/D relabel):
//    • The team now refers to the four physical units as Landers A/B/C/D on
//      the bench instead of numbers. This is a LABELING change only — the
//      firmware identity system underneath (NVS unitID, setid <n>, SSID
//      LanderController1..4) is UNCHANGED. See the mapping note above
//      CH_OPEN_DEG_ALL below.
//    • Bench angles for the three new units (A, B, C) pasted in; the
//      previously-existing committed row becomes Lander D (was "Lander 1").
//      All four rows are now real, non-placeholder data — CAL_COMMITTED is
//      all true.
//    • Lander C is missing/has burnt-out servos on Ch18-19. No structural
//      change to SAMPLE_SERVO_COUNT/SERVO_CHANNELS was made (team's call —
//      handle it operationally rather than teach the firmware a per-lander
//      valve count). C's row carries placeholder angles on Ch18-19 only —
//      see the note on that row.
//    • Added closeDegFor(ch, openDeg): the main/intake valve (Ch20) now
//      always closes to a hardcoded 180°, instead of open+CLOSE_OFFSET_DEG
//      like the sample valves. Both main.cpp builds must call this helper
//      instead of inlining the close-angle formula (deploy's closeServo() /
//      buildPulseTables(), the calibrate CLI's `table` command).
//    • CORRECTION (same day): the first A/B/C numbers had Lander C's Ch7-17
//      wrong (off-by-one from the real bench sheet). A and B were correct
//      and are unchanged. Fixed C below — it now only has 2 dead channels
//      (Ch18-19), not 4; Ch16-17 turned out to have real angles.
//
//  CHANGES 2026-08-18b (main-valve stop margin + compile-time table check):
//    • MAIN_CLOSE_DEG replaces the hardcoded 180 inside closeDegFor(). The
//      main valve was being commanded to the absolute end of the pulse range
//      (180° = 512 counts = 2500 us), i.e. straight into the mechanical stop,
//      on every close — twice per sample event plus every park. Backing it to
//      a named constant gives a defined cushion instead of none, and makes the
//      cushion a decision rather than a by-product of each servo's open trim.
//      *** MAIN_CLOSE_DEG IS NOT YET BENCH-CONFIRMED — see the note on it. ***
//    • static_assert block added at the bottom of this file. tools/
//      check_calibration.py only runs in CI, and firmware is routinely flashed
//      offline from a locally-edited copy of this header, so a bad table could
//      reach a board without ever passing the lint. These asserts make the
//      same class of mistake a compile error instead.
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
//  Closed = open + CLOSE_OFFSET_DEG, clamped to 180, for every SAMPLE valve
//  (Ch0-19). 65° is the max offset that keeps every sample channel at/under
//  180: the highest committed open angle is 115° (Lander D Ch13, Lander C
//  Ch10), so 115 + 65 = 180 exactly — that's the binding case setting this
//  ceiling. Ch20 (MAIN_SERVO_CH) does NOT use this formula: the main valve
//  closes to MAIN_CLOSE_DEG regardless of its open angle — mechanically it's
//  a different animal (near-full travel = fully closed) from the sample
//  valves.
//
//  WORKFLOW per unit:  `setid <n>` on the calibrate build (stamps NVS) →
//  calibrate the 21 servos → paste that unit's open angles into row (n-1)
//  below → flip CAL_COMMITTED[n-1] to true → commit → flash the assembly build.
//  A unit whose row is still a placeholder (CAL_COMMITTED false) REFUSES TO ARM.
//
//  LABELING: the team refers to the four physical units as Landers A/B/C/D on
//  the bench, but the firmware's identity system is unchanged underneath —
//  units are still numbered 1-4 in NVS/SSID/setid. The mapping is:
//    unit 1 = Lander D (existing, previously "Lander 1")
//    unit 2 = Lander A
//    unit 3 = Lander B
//    unit 4 = Lander C  (missing/burnt-out Ch18-19 — see note on that row)
//  If a physical unit gets relabeled, update this comment AND the row
//  headers below — nothing here enforces the letter mapping automatically.
constexpr uint8_t CLOSE_OFFSET_DEG = 65;     // !! MAX 65 — 115° hits 180 !!
constexpr uint8_t LANDER_COUNT     = 4;

//  Closed angle for the MAIN/common valve (Ch20) only. Sample valves are
//  unaffected by this and still use open + CLOSE_OFFSET_DEG.
//
//  *** NOT YET BENCH-CONFIRMED. Starting value only. ***
//
//  Why this exists: 180° is the absolute end of the commanded pulse range
//  (degToPulse(180) = PULSE_MAX_COUNT = 512 = 2500 us), so closing the main
//  valve to 180 drives it into the servo's mechanical stop every time. The
//  low end already has a guard (SERVO_MIN_DEG = 70; commanding below it is
//  what stalled Ch20/Ch14 on the bench) — the high end had none.
//
//  175 is a starting value giving a 5° cushion off the stop. It must be
//  confirmed per unit during the Section 14 cold/oil test at ~8 °C: drive the
//  main valve to MAIN_CLOSE_DEG, cut PWM, and verify it is FULLY SEALED and
//  does not creep. If a unit needs more travel to seal, raise this; if the
//  four units disagree, convert this to a per-lander row indexed [unitID-1]
//  exactly like CH_OPEN_DEG_ALL and add a closeDegFor() lookup.
//
//  Two things make this cushion matter more than it looks:
//    • The pack is 6 V nominal (2x PS-6100 in parallel) and the HPS-2018 is
//      rated 6.0-8.4 V, so the servos run at/below their minimum rating for
//      most of the discharge — slower and weaker than the 0.20 s/60° @ 7.4 V
//      datasheet figure.
//    • The servos are submerged in oil at ~8 °C, which the datasheet figure
//      does not account for at all.
//  A servo that cannot finish its stroke inside SERVO_SETTLE_MS ends up both
//  under-travelled (no seal) AND stalled for the whole settle window. Stall
//  current is 1.4-2.3 A per Hiwonder vs the ~0.227 A the 2-servo path is
//  certified for.
constexpr uint8_t MAIN_CLOSE_DEG = 175;

//  Rows are indexed [unitID - 1].  Unit IDs are 1..4  →  SSID LanderController1..4
constexpr uint8_t CH_OPEN_DEG_ALL[LANDER_COUNT][21] = {
  // ── unit 1 · Lander D ──  existing calibrated set (unchanged)
  { 100, 110, 100,  95,  95, 105,  95, 110,    // Ch0-7
     95, 105, 100, 105,  95, 115, 110,  95,    // Ch8-15
    100,  95,  95,  90,  90 },                 // Ch16-20 (Ch20 = main)
  // ── unit 2 · Lander A ──  bench-confirmed 2026-08-18
  {  95,  90,  90,  95, 100,  90,  85,  95,    // Ch0-7
    100, 100, 100,  90, 100,  95,  95, 105,    // Ch8-15
   105, 100,  95,  95, 105 },                  // Ch16-20 (Ch20 = main)
  // ── unit 3 · Lander B ──  bench-confirmed 2026-08-18
  {  95, 100, 100, 105, 100, 100,  95, 100,    // Ch0-7
   110,  95, 110,  90,  95, 105,  95, 100,     // Ch8-15
   100, 100, 110, 100,  90 },                  // Ch16-20 (Ch20 = main)
  // ── unit 4 · Lander C ──  bench-confirmed 2026-08-18 — Ch18-19 PLACEHOLDER
  //    Servos on Ch18-19 are burnt out / missing on this physical unit. The
  //    two values below (95, 90 — copied from Lander D) are NOT real bench
  //    angles; they only exist so degToPulse()/buildPulseTables() never
  //    operate on an uninitialized/garbage value. Per the team's call, the
  //    firmware was NOT changed to skip these channels for C — those
  //    physical ports must not be wired to real sample valves on this unit,
  //    and C's mission will spend 2 of its 20 sample wake cycles opening/
  //    closing dead or unconnected ports (harmless, just wasted cycles).
  //    Ch0-17 and Ch20 (main) below ARE real bench-confirmed angles.
  {  95, 100, 105, 100,  95, 100,  95, 100,    // Ch0-7 (real)
    95,  90, 115, 105, 105, 105, 100, 100,     // Ch8-15 (real)
   100, 105,  95,  90, 100 },                  // Ch16-17 real, Ch18-19 PLACEHOLDER, Ch20 real (main)
};

//  Flip a row's flag to true ONLY after that physical unit's angles are
//  bench-confirmed and pasted above. Placeholder rows stay false → the assembly
//  build refuses to arm them. All four rows are now bench-confirmed and
//  committed. (Lander C's Ch18-19 are placeholders, not real angles — see the
//  note on that row above; this does NOT block arming C, since the rest of
//  its board is real and those 2 dead ports are a known, accepted gap.)
constexpr bool CAL_COMMITTED[LANDER_COUNT] = { true, true, true, true };

//  Bounds-checked row accessor. id is 1..LANDER_COUNT; returns nullptr when the
//  ID is unset (0) or out of range, so callers can fail safe.
inline const uint8_t* chOpenRow(uint8_t id) {
  return (id >= 1 && id <= LANDER_COUNT) ? CH_OPEN_DEG_ALL[id - 1] : nullptr;
}
inline bool calCommitted(uint8_t id) {
  return (id >= 1 && id <= LANDER_COUNT) && CAL_COMMITTED[id - 1];
}

// Closed-angle policy for a channel — THE single formula, shared by every
// build (deploy's closeServo()/buildPulseTables(), the calibrate CLI's
// `table` command). Sample valves (Ch0-19) close at open+CLOSE_OFFSET_DEG,
// clamped to 180. The main valve (Ch20/MAIN_SERVO_CH) closes to
// MAIN_CLOSE_DEG instead — it does not use the offset formula at all.
// Do not reimplement this inline anywhere else.
inline uint8_t closeDegFor(uint8_t ch, uint8_t openDeg) {
  if (ch == MAIN_SERVO_CH) return MAIN_CLOSE_DEG;
  return (uint8_t)min((int)openDeg + CLOSE_OFFSET_DEG, 180);
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
// clamped at 180. Ch20 is excluded — it uses MAIN_CLOSE_DEG, not the offset.
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
// The main valve must CLOSE toward 180 from its open angle on every unit.
constexpr bool _calMainClosesUpward(uint8_t r = 0) {
  return (r >= LANDER_COUNT)
           ? true
           : ((MAIN_CLOSE_DEG > CH_OPEN_DEG_ALL[r][MAIN_SERVO_CH]) && _calMainClosesUpward(r + 1));
}

static_assert(_calAllRowsOk(),
              "calibration.h: an open angle is outside [SERVO_MIN_DEG,180], or a sample "
              "channel's open+CLOSE_OFFSET_DEG would clamp at 180. Fix CH_OPEN_DEG_ALL "
              "or lower CLOSE_OFFSET_DEG.");
static_assert(_calAngleOk(MAIN_CLOSE_DEG),
              "calibration.h: MAIN_CLOSE_DEG is outside [SERVO_MIN_DEG,180].");
static_assert(_calMainClosesUpward(),
              "calibration.h: MAIN_CLOSE_DEG is at or below some unit's Ch20 open angle — "
              "the main valve would 'close' by moving toward open.");
static_assert(MAIN_SERVO_CH == SAMPLE_SERVO_COUNT,
              "calibration.h: MAIN_SERVO_CH must sit immediately above the sample channels; "
              "buildPulseTables() and the park routines iterate 0..MAIN_SERVO_CH.");
static_assert(MAX_ACTIVE_SERVOS >= 1 && MAX_ACTIVE_SERVOS <= 2,
              "calibration.h: the certified power path is 1-2 concurrent servos.");
static_assert(PULSE_MIN_COUNT < PULSE_MAX_COUNT && PULSE_MAX_COUNT < 4096,
              "calibration.h: pulse counts must be ordered and inside the PCA9685 12-bit range "
              "(4096 is the full-OFF value used by releaseServo()).");