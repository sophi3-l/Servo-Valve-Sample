// ============================================================================
//  Karen Valve Controller — FLIGHT / DEPLOY BUILD       (src/deploy/main.cpp)
//  ESP32 DevKitC-32UE · 2x PCA9685 (0x40 / 0x41) · 21 HPS-2018 valves
//  common/main intake = Ch20 · sample valves = Ch0-19
//
//  calibration.h = electrical truth.  schedule.h = mission timing.
//  Do not redefine anything from either here.
//
//  MISSION MODEL
//  • Absolute schedule. Each sleep is next_event_time - elapsed_now, so time
//    spent awake comes out of the following sleep instead of pushing every
//    later event back. A fixed-delta scheme accumulated >1 h of lag.
//  • No operator arm step: a fresh power-on self-arms and starts the clock.
//    The web UI is a SERVICE interface (check / abort), not the arming path.
//  • LED1 blinks on every fresh boot, before anything moves — Rev K's proof
//    that the U10 latch caught.
//  • WiFi is windowed. 2.4 GHz does not reach 300 m, so a permanent AP burns
//    ~120 mA to talk to nobody. Windows on boot, on unexpected resets, and
//    after each event; never on a heartbeat wake.
//  • Any reset resumes the mission. Resuming only on deep-sleep/brownout meant
//    a watchdog or panic stopped the mission forever, unheard, at depth.
//  • Low battery logs and skips. It does not actuate — valves already hold.
//  • Mission end: no park. The last event leaves sample valves closed and
//    common open, which is the intended recovery state.
//
//  VALVE RULES
//  • Resting state (Ch0-19 closed, Ch20 open) is a PRECONDITION set on the
//    bench via the calibrate build's `rest`, not asserted at power-up. It used
//    to be, costing 21 actuations per boot — and esptool resets the board after
//    an upload, so every flash was a boot. NO PATH HERE MOVES A VALVE OUTSIDE A
//    SCHEDULED EVENT, except under operator control in Testing Mode.
//  • railUpAllCommanded() loads all 21 channels before energising. A servo
//    powered with no pulse train drives to ~90 deg = OPEN. See its comment.
//  • Valve positions read UNKNOWN after a boot, because they are.
//
//  NOT BENCH-CONFIRMED: MAIN_CLOSE_OFFSET_DEG, STEP_SETTLE_MS at temperature,
//  VBATT_CUTOFF_V (Rev K 15.2 needs the 8 degC discharge test), passive hold.
// ============================================================================

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include "calibration.h"
#include "schedule.h"

#define ARM_LED_PIN  LED_PIN
#define SERVO_FREQ   SERVO_FREQ_HZ

// Mission state (armed / done / evt / elapsed) lives in a PER-BUILD namespace,
// because NVS survives a reflash and MINI_DEPLOY changes only timings. Sharing
// one namespace meant a completed rehearsal made the flight build take the
// st.complete branch on the deck: 45 s LED blink indistinguishable from a
// healthy arm, then latchOff() — a unit dead on the seafloor. The unit ID stays
// in "karen": it is a property of the board, not of the mission, and must
// survive both builds. Use the calibrate CLI's `clearmission` to reset.
#ifdef MINI_DEPLOY
#define MISSION_NS "karen_m"
#else
#define MISSION_NS "karen_d"
#endif

// ── Network (service windows only — never up during a sleep) ─────────────────
const char* AP_PASSWORD = "lander1234";
char        apSsid[32]  = "LanderController-UNSET";

// Service window length. Costs ~120 mA while up. At 5 min x ~21 windows that
// is ~0.25 Ah of a 24 Ah pack (~1%). MINI is short on purpose: at flight
// length the windows would be longer than the entire rehearsal.
//  Two lengths, because they serve different purposes. The BOOT window is the
//  deck check and the operator's only chance to abort before valves move, so
//  it must be long enough to actually join the AP and load the page. The
//  per-EVENT windows are opportunistic (post-recovery access) and can be short.
#ifdef MINI_DEPLOY
const uint32_t AP_BOOT_WINDOW_MS = 60000;   // 60 s — enough to join WiFi
const uint32_t AP_WINDOW_MS      = 10000;   // 10 s between rehearsal events
const uint32_t LED_CONFIRM_MS    = 8000;    // 8 s
#else
const uint32_t AP_BOOT_WINDOW_MS = 300000;  // 5 min — the protocol's deck
                                            // allowance. It runs INSIDE the
                                            // mission clock; SAMPLE_TIME_S
                                            // already accounts for it. (Not to
                                            // be confused with the 3300 s
                                            // sheet-to-sheet offset = 55 min.)
const uint32_t AP_WINDOW_MS      = 300000;  // 5 min after each sample event
const uint32_t LED_CONFIRM_MS    = 45000;   // 45 s, per Rev K "30-60 s"
#endif
// If the operator aborts the mission, the AP stays up for bench work — but
// bounded, so a unit left disarmed on deck cannot drain the pack unnoticed.
const uint32_t BENCH_IDLE_MS  = 3600000UL;  // 1 h

// Wake this early so railUpAllCommanded() (rail settle + hold) completes before
// the event's t=0, keeping the protocol's 0/4/120/124 offsets exact.
constexpr uint32_t PRELOAD_MS = RAIL_SETTLE_MS + STEP_SETTLE_MS;
constexpr uint32_t PRELOAD_S  = (PRELOAD_MS + 999) / 1000;

// ── Per-unit identity ────────────────────────────────────────────────────────
uint8_t         g_unitId = 0;
bool            g_idOk   = false;   // valid row AND calibration committed -> may arm
bool            g_haveRow = false;  // valid row for this board (may be uncommitted)
const uint8_t*  CH_OPEN  = nullptr;   // this unit's open angles, indexed by CHANNEL
const uint8_t*  SAMPLE_ORDER = nullptr;   // this unit's mission order, indexed by EVENT

// ── Battery guard ────────────────────────────────────────────────────────────
//  Pack is 2x Power-Sonic PS-6100 in PARALLEL: 6 V nominal, 24 Ah.
//  NOTE the servos (HPS-2018) are rated 6.0-8.4 V, so on a 6 V SLA they sit at
//  or below their minimum rating for most of the discharge. That cannot be
//  fixed by moving this threshold — it is a pack/servo choice — but it is why
//  the settle times matter and why the common valve's close angle is capped
//  short of its mechanical stop.
//  5.60 V is INHERITED, NOT VALIDATED. Rev K Section 15 item 2 requires it to
//  come from an 8 degC discharge test with the real load. Read at wake with
//  the rail off, i.e. a rested reading — the right condition for SoC.
const float   VBATT_CUTOFF_V      = 5.60f;
const float   VBATT_PLAUSIBLE_MIN = 3.00f;   // below -> assume sense fault
const uint8_t VBATT_SAMPLES       = 16;

uint16_t chOpenPulse[21];
uint16_t chClosePulse[21];

// ── Persistent mission state ─────────────────────────────────────────────────
#define STATE_MAGIC 0xC0FFEE02
RTC_DATA_ATTR uint32_t rtcMagic     = 0;
RTC_DATA_ATTR uint32_t rtcElapsedS  = 0;
RTC_DATA_ATTR uint8_t  rtcEvtIdx    = 0;

struct MissionState {
  bool     armed;
  bool     complete;
  uint8_t  evtIdx;        // next event to run, 0..EVENT_COUNT
  uint32_t elapsedS;      // mission seconds at the last persisted point
};
MissionState st;
Preferences prefs;

Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(PCA0_ADDR);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(PCA1_ADDR);
WebServer server(80);

// POS_UNKNOWN is the honest startup state. This build no longer moves anything
// at power-up, so after a boot the firmware genuinely does not know where any
// valve is — and these servos have no position feedback, so it never will until
// something is commanded. Showing "CLOSED" for all 21 would be a guess dressed
// up as knowledge, which is exactly the sort of thing that gets trusted on a
// deck at 2am. The UI shows "--" until a channel is actually driven.
enum ServoPos { POS_UNKNOWN, POS_CLOSE, POS_MID, POS_OPEN };
ServoPos servoPositions[21];
bool     servoHasPWM[21];

bool     testMode     = false;
bool     g_benchMode  = false;   // aborted / unarmable: stay awake on the AP
bool     g_windowBreak    = false;  // an operator action ended the service window
bool     g_operatorAborted = false; // ...and it was an abort, so do NOT re-arm
uint32_t g_elapsedAtWake = 0;    // mission seconds at the instant we woke
uint32_t g_millisBase    = 0;    // millis() value that g_elapsedAtWake refers to
uint32_t g_benchStartMs  = 0;
float    g_lastVbatt     = 0.0f;
esp_reset_reason_t g_resetReason = ESP_RST_UNKNOWN;

// Mission seconds right now. millis() is this-boot-only; g_elapsedAtWake
// carries everything before it. g_millisBase lets the origin be re-anchored
// mid-boot (operator restart) without unsigned-underflow tricks.
uint32_t missionElapsedS() { return g_elapsedAtWake + ((millis() - g_millisBase) / 1000UL); }

// Re-anchor the mission clock so missionElapsedS() reads `elapsedS` right now.
void setMissionElapsed(uint32_t elapsedS) {
  g_elapsedAtWake = elapsedS;
  g_millisBase    = millis();
}

// ── Servo primitives (unchanged policy: everything routes through these) ─────
void setPWM(uint8_t ch, uint16_t pulse) {
  if (ch < 16) pwm0.setPWM(ch, 0, pulse);
  else         pwm1.setPWM(ch - 16, 0, pulse);
}

void railUpAllCommanded();   // defined below; no-op when the rail is already on

bool driveServo(uint8_t ch, uint16_t pulse, ServoPos pos, const char* tag, uint8_t degForLog) {
  if (!servoGuardAllows(ch)) {
    Serial.printf("[GUARD] Ch%2d refused — %u already active (", ch, servoActiveCount());
    printActiveServos(Serial);
    Serial.println(")");
    return false;
  }
  railUpAllCommanded();     // no-op if already on; never energise uncommanded
  setPWM(ch, pulse);
  servoMarkActive(ch, true);
  servoHasPWM[ch] = true;
  servoPositions[ch] = pos;
  railOn();                 // no-op after the line above
  Serial.printf("[%s] Ch%2d → %3d° (pulse %d)\n", tag, ch, degForLog, pulse);
  return true;
}

void releaseServo(uint8_t ch) {
  setPWM(ch, 4096);                       // full-OFF → output LOW → limp
  servoMarkActive(ch, false);
  servoHasPWM[ch] = false;
  Serial.printf("[REL]   Ch%2d → PWM off\n", ch);
}
void releaseAll() { for (uint8_t ch = 0; ch <= 20; ch++) releaseServo(ch); }

void openServo(uint8_t ch)  { driveServo(ch, chOpenPulse[ch],  POS_OPEN,  "OPEN ", CH_OPEN[ch]); }
void closeServo(uint8_t ch) { driveServo(ch, chClosePulse[ch], POS_CLOSE, "CLOSE", closeDegFor(ch, CH_OPEN[ch])); }
void midServo(uint8_t ch)   { driveServo(ch, degToPulse(90),   POS_MID,   "MID  ", 90); }

// Bench-only raw jog. Clamps the 70° mechanical floor (commanding below it is
// what stalled Ch20/Ch14 on the bench) — the offset is deliberately ignored.
void angleServo(uint8_t ch, uint8_t deg) {
  if (deg < SERVO_MIN_DEG) deg = SERVO_MIN_DEG;
  driveServo(ch, degToPulse(deg), POS_MID, "TEST ", deg);
}
void sweepServo(uint8_t ch) {
  if (!servoGuardAllows(ch)) { Serial.printf("[GUARD] sweep Ch%2d refused\n", ch); return; }
  railUpAllCommanded();
  servoMarkActive(ch, true); servoHasPWM[ch] = true; railOn();
  uint16_t lo = min(chOpenPulse[ch], chClosePulse[ch]);
  uint16_t hi = max(chOpenPulse[ch], chClosePulse[ch]);
  for (uint16_t p = lo; p <= hi; p += 6) { setPWM(ch, p); delay(15); }
  delay(150);
  for (int p = hi; p >= (int)lo; p -= 6) { setPWM(ch, (uint16_t)p); delay(15); }
  delay(150);
  releaseServo(ch);
}

// One servo under power at a time: drive → settle → release → breather.
// The trailing STEP_GAP_MS mirrors the protocol's release→next-command spacing
// and keeps consecutive inrush events off the C1 bank. A 21-channel march is
// where that matters most.
void moveRangeSequential(uint8_t first, uint8_t last, void (*op)(uint8_t)) {
  for (uint8_t ch = first; ch <= last; ch++) {
    op(ch);
    delay(STEP_SETTLE_MS);
    releaseServo(ch);
    delay(STEP_GAP_MS);
  }
}

// ── Hardware init ────────────────────────────────────────────────────────────
void buildPulseTables() {
  for (uint8_t i = 0; i <= 20; i++) {
    chOpenPulse[i]   = degToPulse(CH_OPEN[i]);
    chClosePulse[i]  = degToPulse(closeDegFor(i, CH_OPEN[i]));   // Ch20 → open + MAIN_CLOSE_OFFSET_DEG, capped
    servoPositions[i] = POS_UNKNOWN;   // nothing has been commanded yet
  }
}
// Resting state is a constant: sample valves closed, common open. A valve is
// only away from it during the 116 s of its own sample window, and that window
// lives entirely inside one wake — nothing to persist.
uint16_t restingPulse(uint8_t ch) {
  return (ch == MAIN_SERVO_CH) ? chOpenPulse[ch] : chClosePulse[ch];
}

// Energise the rail with EVERY channel already commanded.
//
// A servo that powers up with no pulse train drives to its ~90 deg neutral,
// which is functionally OPEN on all 21 channels (see the railOn() warning in
// calibration.h). Commanding only the target channel therefore threw the other
// twenty valves open at every rail-up. Loading all 21 first gives each servo
// something to follow the instant power arrives, so none of them defaults.
// Servos already in position just hold; a drifted one corrects itself.
//
// DOCUMENTED EXCEPTION to the <=2 rule (Howard, restated as "<=2 servos
// MOVING"): this holds all 21 for STEP_SETTLE_MS. Holding is not moving —
// bench measurement was ~0.32 A for 21 idle servos and ~+0.01 A per servo
// holding, so roughly 0.5 A for one second, against F3 at 10 A. There is no
// alternative: the jump hits every servo at the instant power arrives, so
// preventing it requires all of them commanded at that moment.
void railUpAllCommanded() {
  if (railIsOn()) return;
  // No valid row for this board => CH_OPEN is the unit-1 fallback, i.e. another
  // lander's angles. Preloading those risks driving a valve into its stop, and
  // a stalled servo is the whole burnout exposure. Same refusal the calibrate
  // build makes ("driving with another unit's angles risks over-travelling a
  // valve"); accept the uncommanded centring instead and say so loudly.
  // NOTE: keyed on g_haveRow, not g_idOk — a real-but-uncommitted row (a unit
  // awaiting bench confirmation) still holds ITS OWN correct angles.
  if (!g_haveRow) {
    Serial.println("[RAILUP] REFUSED to preload — no unit ID; uncommanded servos "
                   "will centre (~90 deg = OPEN). Stamp the board with `setid`.");
    railOn();
    return;
  }
  for (uint8_t ch = 0; ch <= MAIN_SERVO_CH; ch++) {
    setPWM(ch, restingPulse(ch));
    servoPositions[ch] = (ch == MAIN_SERVO_CH) ? POS_OPEN : POS_CLOSE;
  }
  Serial.println("[RAILUP] all 21 channels commanded, then power");
  railOn();
  delay(STEP_SETTLE_MS);
  releaseAll();             // limp, each holding its correct position
}

void initServoDrivers() {
  Wire.begin(I2C_SDA, I2C_SCL);
  pwm0.begin(); pwm0.reset(); delay(10);
  pwm0.setOscillatorFrequency(25000000); pwm0.setPWMFreq(SERVO_FREQ);
  pwm1.begin(); pwm1.reset(); delay(10);
  pwm1.setOscillatorFrequency(25000000); pwm1.setPWMFreq(SERVO_FREQ);
  delay(10);
}
void sleepServoDrivers() { pwm0.sleep(); pwm1.sleep(); }
void initBatteryAdc()    { analogSetPinAttenuation(VBATT_SENSE_PIN, ADC_11db); }
float readBatteryVoltage() {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < VBATT_SAMPLES; i++) { acc += analogReadMilliVolts(VBATT_SENSE_PIN); delay(2); }
  g_lastVbatt = ((acc / (float)VBATT_SAMPLES) / 1000.0f) / VBATT_DIVIDER_RATIO;
  return g_lastVbatt;
}
bool batteryTooLow(float vb) { return (vb >= VBATT_PLAUSIBLE_MIN && vb < VBATT_CUTOFF_V); }

// ── Mission-level valve actions ──────────────────────────────────────────────
// Resting state (20 sample valves CLOSED, common OPEN) is NOT established here.
// It is the operator's job, via the calibrate build's `rest` command, as the
// last step of bench setup. See the RESTING STATE note in the header block.
// The only remaining route to a mass move in this build is Testing Mode.
void restingStateSweep() {
  Serial.println("[VALVE] resting state: close 20 sample valves, then open common");
  moveRangeSequential(0, SAMPLE_SERVO_COUNT - 1, closeServo);
  openServo(MAIN_SERVO_CH);
  delay(STEP_SETTLE_MS);
  releaseServo(MAIN_SERVO_CH);
  railOff();
  Serial.println("[VALVE] VISUALLY CONFIRM: Ch0-19 closed, Ch20 (common) OPEN");
}

// The protocol's sample routine (see schedule.h). Commands are issued at fixed
// offsets from the start of the event — 0 / 4 / 120 / 124 s — and each servo is
// released STEP_SETTLE_MS later. The waits between are computed from those
// offsets, so changing the settle CANNOT shift a command time: the sample valve
// is open for 116 s and common is closed for 124 s at any settle value.
void runSampleEvent(uint8_t idx) {
  uint8_t ch = SAMPLE_ORDER[idx];        // per-unit order, NOT ascending channel
  Serial.printf("[EVENT] sample %u/%u  Ch%u  (t=%lu s, settle %u ms)\n",
                idx + 1, EVENT_COUNT, ch, (unsigned long)missionElapsedS(), STEP_SETTLE_MS);

  // t = 0 s — close common
  closeServo(MAIN_SERVO_CH); delay(STEP_SETTLE_MS); releaseServo(MAIN_SERVO_CH);
  delay(CMD_OPEN_SAMPLE_MS - CMD_CLOSE_COMMON_MS - STEP_SETTLE_MS);

  // t = 4 s — open sample n
  openServo(ch);             delay(STEP_SETTLE_MS); releaseServo(ch);
  delay(CMD_CLOSE_SAMPLE_MS - CMD_OPEN_SAMPLE_MS - STEP_SETTLE_MS);   // open, unpowered

  // t = 120 s — close sample n
  closeServo(ch);            delay(STEP_SETTLE_MS); releaseServo(ch);
  delay(CMD_OPEN_COMMON_MS - CMD_CLOSE_SAMPLE_MS - STEP_SETTLE_MS);

  // t = 124 s — open common, back to resting state
  openServo(MAIN_SERVO_CH);  delay(STEP_SETTLE_MS); releaseServo(MAIN_SERVO_CH);

  railOff();
  Serial.printf("[EVENT] sample %u done (t=%lu s)\n", idx + 1, (unsigned long)missionElapsedS());
}

// ── Persistence ──────────────────────────────────────────────────────────────
void saveState(uint32_t elapsedS) {
  st.elapsedS = elapsedS;
  prefs.begin(MISSION_NS, false);
  prefs.putBool ("armed",   st.armed);
  prefs.putBool ("done",    st.complete);
  prefs.putUChar("evt",     st.evtIdx);
  prefs.putUInt ("elapsed", st.elapsedS);
  prefs.end();
  rtcMagic = STATE_MAGIC; rtcElapsedS = st.elapsedS; rtcEvtIdx = st.evtIdx;
}
void loadState() {
  prefs.begin(MISSION_NS, true);
  st.armed    = prefs.getBool ("armed",   false);
  st.complete = prefs.getBool ("done",    false);
  st.evtIdx   = prefs.getUChar("evt",     0);
  st.elapsedS = prefs.getUInt ("elapsed", 0);
  prefs.end();
}

void loadUnitId() {
  prefs.begin("karen", true);
  g_unitId = prefs.getUChar("unitid", 0);
  prefs.end();
  const uint8_t* row = chOpenRow(g_unitId);
  g_haveRow = (row != nullptr);
  g_idOk    = g_haveRow && calCommitted(g_unitId);
  CH_OPEN = row ? row : CH_OPEN_DEG_ALL[0];   // row-0 fallback keeps math safe
  const uint8_t* ord = sampleOrderRow(g_unitId);
  SAMPLE_ORDER = ord ? ord : SAMPLE_ORDER_ALL[0];   // same fail-safe fallback
#ifdef MINI_DEPLOY
  const char* ssidTag = "-TEST";              // DO NOT SEAL
#else
  const char* ssidTag = "";
#endif
  if (row) snprintf(apSsid, sizeof(apSsid), "LanderController%u%s", g_unitId, ssidTag);
  else     snprintf(apSsid, sizeof(apSsid), "LanderController-UNSET%s", ssidTag);
  Serial.printf("[ID]    unit=%u  cal=%s  ssid=%s\n",
                g_unitId, g_idOk ? "committed" : "MISSING/placeholder", apSsid);
  // Print the mission order at every boot: it differs per unit, and the deck
  // log is the only place an operator can confirm which valve samples first.
  Serial.print("[ORDER] events 1-20 drive Ch:");
  for (uint8_t i = 0; i < SAMPLE_SERVO_COUNT; i++) Serial.printf(" %u", SAMPLE_ORDER[i]);
  Serial.println(ord ? "" : "   (NO UNIT ID — showing unit 1's order)");
}

// ── Sleep ────────────────────────────────────────────────────────────────────
void radiosOff() {
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
}

void enterDeepSleepS(uint32_t seconds) {
  // Persist BEFORE sleeping, including the sleep we are about to perform, so
  // the elapsed clock is correct the instant we wake.
  saveState(missionElapsedS() + seconds);
  digitalWrite(ARM_LED_PIN, LOW);
  releaseAll();
  railOff();
  sleepServoDrivers();
  radiosOff();
  Serial.printf("[SLEEP] %lu s  (elapsed will be %lu s, next event %u/%u)\n",
                (unsigned long)seconds, (unsigned long)st.elapsedS,
                st.evtIdx + 1, EVENT_COUNT);
  Serial.flush();
  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  esp_deep_sleep_start();
}

// Mission complete: no park — every sample valve was closed by its own event
// and common is open, which is the intended recovery state.
void endMission() {
  st.complete = true; st.armed = false;
  saveState(missionElapsedS());
  digitalWrite(ARM_LED_PIN, LOW);
  releaseAll();
  railOff();
  sleepServoDrivers();
  radiosOff();
  Serial.printf("[DONE]  all %u samples collected; common left OPEN; elapsed %lu s\n",
                EVENT_COUNT, (unsigned long)st.elapsedS);
  Serial.println("[DONE]  latching power off (U10)");
  Serial.flush();
  latchOff();                 // ← on battery, execution ends inside this delay
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  Serial.println("[DONE]  still alive (USB / latch not wired) — deep sleep until reset");
  Serial.flush();
  esp_deep_sleep_start();
}

// ── Web UI ───────────────────────────────────────────────────────────────────
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Lander — Service</title>
<style>
  :root{--bg:#0d1117;--card:#161b22;--border:#30363d;--text:#c9d1d9;--dim:#8b949e;
        --blue:#58a6ff;--green:#238636;--red:#da3633;--orange:#f0883e;--grey:#30363d;}
  *{box-sizing:border-box;}
  body{font-family:monospace;background:var(--bg);color:var(--text);padding:16px;max-width:620px;margin:0 auto;}
  h2{color:var(--blue);font-size:13px;text-transform:uppercase;letter-spacing:.08em;margin:0 0 8px;}
  .card{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:14px;margin-bottom:12px;}
  .status{text-align:center;} .state{font-size:18px;font-weight:bold;color:var(--blue);}
  .batt{font-size:34px;font-weight:bold;color:var(--orange);line-height:1.1;margin-top:4px;}
  .prog{font-size:13px;color:var(--dim);margin-top:3px;}
  .win{font-size:12px;color:var(--orange);margin-top:6px;}
  .row{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:10px;}
  button{flex:1;min-width:60px;padding:9px 6px;border:none;border-radius:5px;font-size:13px;font-weight:bold;cursor:pointer;}
  .g{background:var(--green);color:#fff;} .r{background:var(--red);color:#fff;}
  .b{background:#1f6feb;color:#fff;} .gr{background:var(--grey);color:#fff;}
  .servo-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:5px;margin-bottom:10px;}
  .sc{background:var(--card);border:1px solid var(--border);border-radius:5px;padding:6px;text-align:center;}
  .sc.main-ch{border-color:#58a6ff55;} .sc.open{border-color:var(--green);background:#0d1f0f;}
  .sc.closed{border-color:var(--red);background:#1f0d0d;}
  .sc-name{font-size:11px;font-weight:bold;margin-bottom:1px;} .sc-pos{font-size:10px;color:var(--dim);margin-bottom:4px;}
  .sc.open .sc-pos{color:#3fb950;} .sc.closed .sc-pos{color:#f85149;}
  .sc button{min-width:unset;padding:4px 2px;font-size:11px;flex:1;}
  label{font-size:11px;color:var(--dim);display:block;margin-bottom:3px;}
  select,input[type=number],input[type=range]{background:var(--card);color:var(--text);border:1px solid var(--border);border-radius:4px;padding:6px 8px;width:100%;font-size:13px;}
  .grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px;}
  .hms-row{display:flex;gap:4px;}
  .kv{display:flex;justify-content:space-between;font-size:12px;padding:3px 0;border-bottom:1px solid #21262d;}
  .kv span:first-child{color:var(--dim);}
  .note{font-size:11px;color:var(--dim);margin-bottom:8px;} .warn{font-size:11px;color:var(--orange);margin:6px 0 10px;}
  .sep{border:none;border-top:1px solid #21262d;margin:12px 0;}
  #act-status{font-size:11px;color:var(--dim);} .arm-btn{padding:14px;font-size:15px;}
  .banner{background:#3d1f00;border:1px solid var(--orange);color:var(--orange);border-radius:6px;
          padding:8px;text-align:center;font-size:12px;font-weight:bold;margin-bottom:12px;display:none;}
</style></head><body>

<div class="banner" id="mini">MINI-DEPLOY TEST BUILD — compressed schedule — DO NOT SEAL</div>

<div class="card status"><div class="state" id="st">--</div><div class="prog" id="unit"></div>
  <div class="batt" id="vb">-- V</div><div class="prog" id="pg"></div>
  <div class="win" id="win"></div></div>

<h2>Schedule</h2>
<div class="card">
  <div class="kv"><span>Mission elapsed</span><span id="k-elapsed">--</span></div>
  <div class="kv"><span>Next event</span><span id="k-next">--</span></div>
  <div class="kv"><span>Time to next event</span><span id="k-eta">--</span></div>
  <div class="kv"><span>Mission length</span><span id="k-end">--</span></div>
  <div class="note" style="margin:8px 0 0">Schedule is compiled in from the sampling protocol and cannot be edited here.</div>
</div>

<h2>Servos</h2><div class="note" id="offset-note"></div><div class="servo-grid" id="grid"></div>
<div class="note">Positions show <b>--</b> until a channel is commanded. This build does not move
valves on power-up, and these servos have no position feedback, so an uncommanded channel's
position is genuinely unknown. Sweeps are in Testing Mode.</div>
<div class="row"><button class="gr" onclick="cmd('cal_all_release')">Release All (cuts PWM, moves nothing)</button></div>

<hr class="sep"><h2>Testing Mode</h2>
<div class="note">Bench bring-up — one servo at a time. Aborts the mission while on; exit and restart to re-arm.</div>
<div class="row"><button class="b" id="tm-btn" onclick="toggleTest()">Enter Testing Mode</button></div>
<div id="tm-panel" style="display:none">
  <div class="grid2" style="margin-bottom:10px">
    <div><label>Channel under test</label><select id="tm-ch"></select></div>
    <div><label>Raw angle · clamped to 70° floor</label>
      <div class="hms-row"><input type="range" id="tm-deg" min="0" max="180" value="90" style="flex:2"
        oninput="document.getElementById('tm-degv').value=this.value">
        <input type="number" id="tm-degv" min="0" max="180" value="90" style="max-width:60px"
        oninput="document.getElementById('tm-deg').value=this.value">
        <button class="gr" style="flex:1" onclick="tmove()">Move</button></div></div>
  </div>
  <div class="row">
    <button class="g"  onclick="tsc('open')">Open</button><button class="r" onclick="tsc('close')">Close</button>
    <button class="gr" onclick="tsc('mid')">90°</button><button class="b" onclick="tsc('sweep')">Sweep</button>
    <button class="gr" onclick="tsc('release')">Release</button>
  </div>
  <div class="row">
    <button class="gr" onclick="thold('close')">Hold-check · closed</button>
    <button class="gr" onclick="thold('open')">Hold-check · open</button>
  </div>
  <div class="note" style="margin-top:6px">Sweeps — all 21 channels, one at a time. Each is 21 actuations;
  the authoritative resting-state command is <b>`rest`</b> on the calibrate build.</div>
  <div class="row">
    <button class="g"  onclick="cmd('cal_all_open')">All Open</button>
    <button class="r"  onclick="cmd('cal_all_close')">All Close</button>
    <button class="gr" onclick="cmd('cal_all_mid')">All 90°</button>
  </div>
  <div class="row"><button class="b" onclick="cmd('rest')">Resting state (Ch0–19 closed · common open)</button></div>
  <div class="warn">Hold-check drives to position, cuts PWM, leaves it limp — watch for creep. This is the passive-hold test the deep-sleep budget depends on. For COMMON (Ch20), hold-check &middot; closed is also the close-angle seal check.</div>
</div>

<hr class="sep"><h2>Mission</h2>
<div class="warn">The unit self-arms on power-up and runs on deep sleep. This page is only reachable during a service window; it closes automatically and the schedule continues.</div>
<div class="row"><button class="r arm-btn" onclick="abortMission()">Abort mission</button>
  <button class="g arm-btn" onclick="restartMission()">Restart from t=0</button></div>
<div id="act-status"></div>

<script>
const CHANNELS=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20];
const grid=document.getElementById('grid');
CHANNELS.forEach(ch=>{const m=ch===20;grid.innerHTML+=`
  <div class="sc${m?' main-ch':''}" id="sc${ch}"><div class="sc-name">${m?'COMMON':'Ch'+ch}</div>
  <div class="sc-pos" id="sp${ch}">--</div><div class="row" style="margin:0">
  <button class="g" onclick="sc(${ch},'open')">O</button>
  <button class="r" onclick="sc(${ch},'close')">C</button>
  <button class="gr" onclick="sc(${ch},'release')">R</button></div></div>`;});
const tsel=document.getElementById('tm-ch');
CHANNELS.forEach(ch=>{const o=document.createElement('option');o.value=ch;
  o.textContent=(ch===20?'COMMON (Ch20)':'Ch'+ch);tsel.appendChild(o);});
function cmd(a){fetch('/control?action='+a).catch(()=>{});}
function sc(ch,a){fetch('/servo?ch='+ch+'&action='+a).catch(()=>{});}
function tch(){return parseInt(document.getElementById('tm-ch').value);}
function tsc(a){fetch('/servo?ch='+tch()+'&action='+a).catch(()=>{});}
function tmove(){fetch('/servo?ch='+tch()+'&action=angle&deg='+document.getElementById('tm-degv').value).catch(()=>{});}
function thold(p){fetch('/servo?ch='+tch()+'&action=hold&pos='+p).catch(()=>{});}
function toggleTest(){const on=document.getElementById('tm-panel').style.display==='none';fetch('/testmode?on='+(on?1:0)).catch(()=>{});}
function say(t,c){const e=document.getElementById('act-status');e.textContent=t;e.style.color=c;}
function abortMission(){if(!confirm('Abort the mission? The schedule stops and the unit stays awake on WiFi for bench work.'))return;
  fetch('/clear?confirm=1').then(()=>say('Mission aborted — unit is in bench mode.','#f0883e')).catch(()=>say('send failed','#f85149'));}
function restartMission(){if(!confirm('Restart the mission from t=0? Sample index resets to 0 and the clock restarts now.'))return;
  fetch('/restart?confirm=1').then(()=>say('Mission restarted from t=0.','#3fb950')).catch(()=>say('send failed','#f85149'));}
function dur(s){if(s===null||s===undefined||s<0)return '--';
  const d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60),x=Math.floor(s%60);
  return (d?d+'d ':'')+String(h).padStart(2,'0')+'h'+String(m).padStart(2,'0')+'m'+String(x).padStart(2,'0')+'s';}
function poll(){fetch('/status').then(r=>r.json()).then(d=>{
  document.getElementById('mini').style.display=d.mini?'block':'none';
  document.getElementById('st').textContent=d.state;
  const idok=!!d.id_ok;const uEl=document.getElementById('unit');
  uEl.textContent=d.unit>0?('Lander '+d.unit+(idok?'':' · CAL NOT COMMITTED')):'NO UNIT ID SET — bench-set before deploy';
  uEl.style.color=(d.unit>0&&idok)?'#8b949e':'#f85149';
  document.getElementById('vb').textContent=(d.vbatt?d.vbatt.toFixed(2):'--')+' V';
  document.getElementById('vb').style.color=(d.vbatt&&d.vbatt<d.cutoff)?'#f85149':'#f0883e';
  document.getElementById('pg').textContent=d.complete?('Mission complete · '+d.total+' samples')
    :(d.armed?('Next sample '+(parseInt(d.evt)+1)+' of '+d.total):'Not armed · bench mode');
  document.getElementById('win').textContent=(d.window_ms>0)?('Service window closes in '+Math.ceil(d.window_ms/1000)+' s'):'';
  document.getElementById('k-elapsed').textContent=dur(d.elapsed_s);
  document.getElementById('k-next').textContent=(d.evt<d.total)?('#'+(d.evt+1)+' · Ch'+d.next_ch+' · t='+dur(d.next_s)):'—';
  document.getElementById('k-eta').textContent=(d.evt<d.total)?dur(d.next_s-d.elapsed_s):'—';
  document.getElementById('k-end').textContent=dur(d.end_s);
  const test=!!d.test;document.getElementById('tm-panel').style.display=test?'block':'none';
  const tb=document.getElementById('tm-btn');tb.textContent=test?'Exit Testing Mode':'Enter Testing Mode';tb.className=test?'r':'b';
  if(d.close_offset)document.getElementById('offset-note').textContent=
    'Sample valves: close = open +'+d.close_offset+'° · COMMON (Ch20): close = open +'+d.main_offset+'° = '+d.main_close+'° · common rests OPEN';
  CHANNELS.forEach(ch=>{const c=document.getElementById('sc'+ch),p=document.getElementById('sp'+ch),s=d.servoStates[ch];
    c.className='sc'+(ch===20?' main-ch':'')+(s==='open'?' open':s==='close'?' closed':'');
    p.textContent=s==='open'?'OPEN':s==='close'?'CLOSED':s==='mid'?'MID':'--';});
}).catch(()=>{});}
setInterval(poll,750); poll();
</script></body></html>
)rawliteral";

// ── Web handlers ─────────────────────────────────────────────────────────────
uint32_t g_windowEndMs = 0;      // 0 = not in a bounded window

void handleRoot() { server.send(200, "text/html", HTML_PAGE); }

void handleStatus() {
  const char* stateStr = st.complete ? "COMPLETE"
                       : testMode    ? "TESTING"
                       : !g_idOk     ? "NEEDS UNIT ID"
                       : st.armed    ? "RUNNING"
                       :               "BENCH";
  uint32_t elapsed = missionElapsedS();
  uint32_t winLeft = (g_windowEndMs > millis()) ? (g_windowEndMs - millis()) : 0;
  String json = "{";
  json += "\"state\":\""     + String(stateStr)               + "\",";
  json += "\"unit\":"        + String(g_unitId)               + ",";
  json += "\"id_ok\":"       + String(g_idOk ? 1 : 0)         + ",";
  json += "\"armed\":"       + String(st.armed ? 1 : 0)       + ",";
  json += "\"complete\":"    + String(st.complete ? 1 : 0)    + ",";
  json += "\"evt\":"         + String(st.evtIdx)              + ",";
  json += "\"total\":"       + String(EVENT_COUNT)            + ",";
  json += "\"elapsed_s\":"   + String(elapsed)                + ",";
  json += "\"next_s\":"      + String(st.evtIdx < EVENT_COUNT ? eventTimeS(st.evtIdx) : 0) + ",";
  json += "\"next_ch\":"     + String(st.evtIdx < EVENT_COUNT ? SAMPLE_ORDER[st.evtIdx] : 0) + ",";
  json += "\"end_s\":"       + String(missionEndS())          + ",";
  json += "\"window_ms\":"   + String(winLeft)                + ",";
  json += "\"close_offset\":"+ String(CLOSE_OFFSET_DEG)       + ",";
  json += "\"main_close\":"  + String(closeDegFor(MAIN_SERVO_CH, CH_OPEN[MAIN_SERVO_CH])) + ",";
  json += "\"main_offset\":" + String(MAIN_CLOSE_OFFSET_DEG)  + ",";
  json += "\"cutoff\":"      + String(VBATT_CUTOFF_V, 2)      + ",";
  json += "\"test\":"        + String(testMode ? 1 : 0)       + ",";
#ifdef MINI_DEPLOY
  json += "\"mini\":1,";
#else
  json += "\"mini\":0,";
#endif
  json += "\"vbatt\":"       + String(readBatteryVoltage(), 2) + ",";
  json += "\"servoStates\":[";
  for (int i = 0; i <= 20; i++) {
    json += (servoPositions[i]==POS_OPEN)  ? "\"open\""
          : (servoPositions[i]==POS_MID)   ? "\"mid\""
          : (servoPositions[i]==POS_CLOSE) ? "\"close\"" : "\"unknown\"";
    if (i < 20) json += ",";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

// Mass moves are gated behind Testing Mode. A stray click on a 21-valve sweep
// during a deck service window is 21 unnecessary actuations, and unnecessary
// actuation is the thing most likely to wear a valve out before it ever flies.
// Single-channel controls stay ungated: they are one deliberate actuation, and
// entering Testing Mode disarms the mission, which is too high a price for
// nudging one valve during a service window.
// Release-all is ungated — it only cuts PWM, it never drives anything.
void handleControl() {
  if (!server.hasArg("action")) { server.send(400,"text/plain","Missing action"); return; }
  String a = server.arg("action");
  if (a == "cal_all_release") { releaseAll(); railOff(); server.send(200,"text/plain","OK"); return; }

  if (!testMode) {
    server.send(409, "text/plain",
      "mass moves require testing mode (this build does not sweep valves outside it)");
    return;
  }
  if      (a == "cal_all_open")    { moveRangeSequential(0, 20, openServo);  railOff(); }
  else if (a == "cal_all_close")   { moveRangeSequential(0, 20, closeServo); railOff(); }
  else if (a == "cal_all_mid")     { moveRangeSequential(0, 20, midServo);   railOff(); }
  else if (a == "rest")            { restingStateSweep(); }
  server.send(200, "text/plain", "OK");
}

void handleServo() {
  if (!server.hasArg("ch") || !server.hasArg("action")) { server.send(400,"text/plain","Missing args"); return; }
  uint8_t ch = (uint8_t)server.arg("ch").toInt(); String a = server.arg("action");
  if (ch > 20) { server.send(400,"text/plain","Bad channel"); return; }
  if      (a == "open")    openServo(ch);
  else if (a == "close")   closeServo(ch);
  else if (a == "mid")     midServo(ch);
  else if (a == "release") releaseServo(ch);
  else if (a == "angle" || a == "sweep" || a == "hold") {
    if (!testMode) { server.send(409,"text/plain","testing mode is off"); return; }
    if      (a == "angle") { int d = server.hasArg("deg") ? server.arg("deg").toInt() : 90; angleServo(ch,(uint8_t)constrain(d,0,180)); }
    else if (a == "sweep") { sweepServo(ch); }
    else                   { if (server.arg("pos")=="open") openServo(ch); else closeServo(ch); delay(STEP_SETTLE_MS); releaseServo(ch); }
  }
  server.send(200,"text/plain","OK");
}

// Abort: stop the schedule and stay awake on the AP for bench work.
// Clears `complete` too, so a finished unit can be taken back to bench state
// without erasing NVS (which would also wipe the unit ID).
void handleClear() {
  if (server.arg("confirm") != "1") { server.send(400,"text/plain","confirm=1 required"); return; }
  st.armed = false; st.complete = false; st.evtIdx = 0;
  saveState(missionElapsedS());
  g_windowBreak = true; g_operatorAborted = true;
  Serial.println("[MISSION] aborted by operator — entering bench mode");
  server.send(200,"text/plain","ABORTED");
}

// Restart from t=0. Also the way to re-run a unit whose mission has completed,
// without a flash erase.
void handleRestart() {
  if (server.arg("confirm") != "1") { server.send(400,"text/plain","confirm=1 required"); return; }
  if (!g_idOk) {
    server.send(409, "text/plain",
      g_unitId == 0 ? "no unit ID set — run `setid <n>` on the calibrate build first"
                    : "calibration for this unit is not committed (placeholder row)");
    return;
  }
  st.armed = true; st.complete = false; st.evtIdx = 0;
  setMissionElapsed(0);                        // t = 0 is now
  saveState(0);
  g_benchMode = false; g_operatorAborted = false;
  g_windowBreak = true;
  Serial.println("[MISSION] restarted from t=0 by operator");
  server.send(200,"text/plain","RESTARTED");
}

void handleTestMode() {
  bool on = (server.arg("on") == "1");
  testMode = on;
  if (on) {
    // Testing mode and a running schedule must not share the servo rail.
    st.armed = false;
    saveState(missionElapsedS());
    g_windowBreak = true; g_operatorAborted = true;
    Serial.println("[TEST]  testing mode ON — mission disarmed, bench mode");
  } else {
    releaseAll(); railOff();
    Serial.println("[TEST]  testing mode OFF");
  }
  server.send(200,"text/plain", on ? "TEST_ON" : "TEST_OFF");
}

void handleNotFound() { server.send(404,"text/plain","Not found"); }

void startWebServer() {
  static bool registered = false;
  if (!registered) {
    server.on("/",         handleRoot);
    server.on("/status",   handleStatus);
    server.on("/control",  handleControl);
    server.on("/servo",    handleServo);
    server.on("/testmode", handleTestMode);
    server.on("/clear",    handleClear);
    server.on("/restart",  handleRestart);
    server.onNotFound(handleNotFound);
    registered = true;
  }
  server.begin();
}

// Bounded service window. Returns true if an operator action ended it early
// (abort, restart, or testing mode) — the caller then re-reads st to see which.
// ledConfirmMs > 0 blinks LED1 for that long at the start: the Rev K
// "the U10 latch caught, safe to deploy" confirmation.
bool runServiceWindow(uint32_t windowMs, uint32_t ledConfirmMs) {
  g_windowBreak = false;
  WiFi.softAP(apSsid, AP_PASSWORD);
  startWebServer();
  Serial.printf("[AP]    %s  http://%s  (window %lu s)\n",
                apSsid, WiFi.softAPIP().toString().c_str(), (unsigned long)(windowMs / 1000));
  uint32_t t0 = millis();
  g_windowEndMs = t0 + windowMs;
  while ((millis() - t0) < windowMs) {
    server.handleClient();
    if (ledConfirmMs && (millis() - t0) < ledConfirmMs) digitalWrite(ARM_LED_PIN, (millis() / 250) % 2);
    else                                                digitalWrite(ARM_LED_PIN, LOW);
    if (g_windowBreak) {
      g_windowEndMs = 0; g_windowBreak = false;
      digitalWrite(ARM_LED_PIN, LOW);
      Serial.println("[AP]    window ended early — operator action");
      return true;
    }
    delay(2);
  }
  g_windowEndMs = 0;
  digitalWrite(ARM_LED_PIN, LOW);
  radiosOff();
  Serial.println("[AP]    service window closed — radio off");
  return false;
}

// ── Mission wake ─────────────────────────────────────────────────────────────
void runMissionWake(bool freshBoot, bool unexpectedReset) {
  buildPulseTables();
  initBatteryAdc();
  initServoDrivers();
  // Rail stays OFF until the first driveServo() — the battery read below runs
  // at sleep-level draw, and a low battery never actuates.
  float vb = readBatteryVoltage();
  Serial.printf("[WAKE]  elapsed=%lu s  next=%u/%u  Vbatt=%.2f V%s\n",
                (unsigned long)missionElapsedS(), st.evtIdx + 1, EVENT_COUNT, vb,
                unexpectedReset ? "  (UNEXPECTED RESET — resuming)" : "");

  // Decide LOW *before* opening any service window. A window costs ~120 mA for
  // 5 min (~10 mAh) — more than the valve movement it is gating — and on the
  // unexpected-reset path there is nobody on deck to use it. The header's "low
  // battery logs and skips, it does not actuate" was previously undercut here.
  bool low = batteryTooLow(vb);
  if (low) Serial.printf("[WAKE]  LOW BATTERY %.2f V < %.2f V — no actuation this wake\n", vb, VBATT_CUTOFF_V);

  if (freshBoot) {
    // Confirm the latch caught, and let an operator intervene. NOTHING MOVES
    // HERE — the resting state is a precondition established on the bench via
    // the calibrate build's `rest`, not something this build asserts on every
    // power-up. See the RESTING STATE note in the header block.
    // Kept even on a low battery: this is the deck check and the operator's
    // only abort opportunity, and a low reading is exactly what they need to
    // see. The pack is also at its healthiest here, not 400 h into a mission.
    if (runServiceWindow(AP_BOOT_WINDOW_MS, LED_CONFIRM_MS) && !st.armed) { g_benchMode = true; return; }
  } else if (unexpectedReset && !low) {
    if (runServiceWindow(AP_BOOT_WINDOW_MS, 0) && !st.armed) { g_benchMode = true; return; }
  } else if (unexpectedReset) {
    Serial.println("[WAKE]  post-reset service window SKIPPED — low battery outranks access");
  }

  // Rail up with all channels commanded, then wait for the event's exact t=0.
  // Doing this before the routine (rather than inside its first drive) is what
  // stops the other twenty valves defaulting open when power arrives.
  if (!low && st.evtIdx < EVENT_COUNT) {
    uint32_t due = eventTimeS(st.evtIdx);
    if (missionElapsedS() + PRELOAD_S >= due) {
      railUpAllCommanded();
      while (missionElapsedS() < due) delay(50);
    }
  }

  // Run everything that is due. Normally 0 or 1; more only after an outage.
  bool ranEvent = false;
  while (st.evtIdx < EVENT_COUNT && missionElapsedS() >= eventTimeS(st.evtIdx)) {
    if (low) {
      Serial.printf("[WAKE]  SKIPPED sample %u/%u (Ch%u) — low battery\n",
                    st.evtIdx + 1, EVENT_COUNT, SAMPLE_ORDER[st.evtIdx]);
    } else {
      runSampleEvent(st.evtIdx);
      ranEvent = true;
    }
    st.evtIdx++;
    saveState(missionElapsedS());
  }

  if (st.evtIdx >= EVENT_COUNT) endMission();          // never returns on battery

  if (ranEvent) {
    if (runServiceWindow(AP_WINDOW_MS, 0) && !st.armed) { g_benchMode = true; return; }
  }

  // Sleep to the next event, capped by the heartbeat so battery gets logged
  // through the long gaps (up to 59h55m) instead of once every event.
  uint32_t now    = missionElapsedS();
  uint32_t target = eventTimeS(st.evtIdx);
  uint32_t wake   = (target > PRELOAD_S) ? (target - PRELOAD_S) : 0;   // preload room
  uint32_t sleepS = (wake > now) ? (wake - now) : 1;
  if (sleepS > HEARTBEAT_S) {
    sleepS = HEARTBEAT_S;
    Serial.printf("[WAKE]  heartbeat sleep (%lu s to event %u)\n",
                  (unsigned long)(target - now), st.evtIdx + 1);
  }
  enterDeepSleepS(sleepS);
}

// ── Setup / Loop ─────────────────────────────────────────────────────────────
void setup() {
  initPowerPins();            // FIRST: SHUTDOWN low (stay alive), servo rail OFF
  Serial.begin(115200); delay(300);
#ifdef MINI_DEPLOY
  Serial.println("###### MINI-DEPLOY TEST BUILD — compressed schedule, -TEST SSID, DO NOT SEAL ######");
#endif
  memset(servoHasPWM, false, sizeof(servoHasPWM));
  pinMode(ARM_LED_PIN, OUTPUT); digitalWrite(ARM_LED_PIN, LOW);

  loadState();
  loadUnitId();
  g_resetReason = esp_reset_reason();

  bool freshBoot       = (g_resetReason == ESP_RST_POWERON);
  bool wokeFromSleep   = (g_resetReason == ESP_RST_DEEPSLEEP);
  bool unexpectedReset = !freshBoot && !wokeFromSleep;

  // ---- Rebuild the mission clock ----
  // RTC memory survives deep sleep and software resets; it does NOT survive a
  // power-on. NVS survives everything. On a power-on we cannot know how long
  // we were dead, so we resume at the last persisted elapsed value: no event
  // is repeated or skipped, but the schedule shifts by the outage. Logged.
  if (freshBoot) {
    if (st.armed && st.evtIdx > 0) {
      setMissionElapsed(st.elapsedS);
      Serial.printf("[CLOCK] POWER-ON mid-mission — clock lost. Resuming at elapsed=%lu s, "
                    "event %u/%u. SCHEDULE HAS SLIPPED by the outage.\n",
                    (unsigned long)st.elapsedS, st.evtIdx + 1, EVENT_COUNT);
    } else {
      setMissionElapsed(0);                    // t = 0 is now
    }
  } else if (rtcMagic == STATE_MAGIC) {
    setMissionElapsed(rtcElapsedS);
    if (rtcEvtIdx != st.evtIdx)
      Serial.printf("[CLOCK] RTC evt %u != NVS evt %u — trusting NVS\n", rtcEvtIdx, st.evtIdx);
  } else {
    setMissionElapsed(st.elapsedS);            // RTC gone; fall back to NVS
    Serial.println("[CLOCK] RTC memory invalid — falling back to NVS elapsed");
  }

  Serial.printf("=== BOOT reset=%d elapsed=%lu s armed=%d done=%d evt=%u/%u ===\n",
                (int)g_resetReason, (unsigned long)g_elapsedAtWake,
                st.armed, st.complete, st.evtIdx, EVENT_COUNT);

  // A completed mission must still be reachable. NVS survives a reflash, so a
  // unit that finished a rehearsal would otherwise latch off instantly on every
  // subsequent boot — looking bricked, and recoverable only by erasing NVS,
  // which would also wipe the unit ID. Give the operator a window to Restart.
  if (st.complete) {
    Serial.println("[BOOT]  mission COMPLETE — service window before shutdown (Restart to re-run)");
    buildPulseTables(); initBatteryAdc(); initServoDrivers();
    runServiceWindow(AP_BOOT_WINDOW_MS, LED_CONFIRM_MS);
    if (st.complete) { endMission(); return; }        // nobody intervened
    Serial.println("[BOOT]  operator cleared the completed state");
  }

  // Self-arm on a fresh power-on: t=0 is now. No operator step in the flight
  // path. Skipped if the operator explicitly aborted during a window this boot.
  if (freshBoot && !st.armed && !g_operatorAborted) {
    if (g_idOk) {
      st.armed = true; st.evtIdx = 0;
      saveState(0);
      Serial.printf("[MISSION] self-armed at t=0 — %u samples, mission length %lu s\n",
                    EVENT_COUNT, (unsigned long)missionEndS());
    } else {
      Serial.println("[MISSION] NOT ARMED — unit ID unset or calibration not committed");
    }
  }

  if (st.armed) {
    runMissionWake(freshBoot, unexpectedReset);
    if (!g_benchMode) return;                  // runMissionWake deep-slept
  }

  // Bench mode: unarmable, or the operator aborted. AP stays up, but bounded —
  // a latched unit left on deck must not drain the pack unnoticed.
  g_benchMode = true;
  buildPulseTables(); initBatteryAdc(); initServoDrivers();
  WiFi.softAP(apSsid, AP_PASSWORD);
  startWebServer();
  g_benchStartMs = millis();
  Serial.printf("[BENCH] %s  http://%s  (idle limit %lu min)\n",
                apSsid, WiFi.softAPIP().toString().c_str(), (unsigned long)(BENCH_IDLE_MS / 60000));
}

void loop() {
  if (!g_benchMode) return;
  server.handleClient();
  if (!st.armed && (millis() - g_benchStartMs) > BENCH_IDLE_MS) {
    Serial.println("[BENCH] idle limit reached — sleeping to protect the pack (reset to wake)");
    releaseAll(); railOff(); sleepServoDrivers(); radiosOff();
    Serial.flush();
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    esp_deep_sleep_start();
  }
  // A /restart during bench mode re-arms; pick the schedule back up. If the
  // operator aborts again inside the new service window, runMissionWake sets
  // g_benchMode and we simply fall back into this loop.
  if (st.armed) {
    g_benchMode = false; g_operatorAborted = false;
    radiosOff();
    runMissionWake(true, false);               // deep-sleeps unless aborted again
    g_benchStartMs = millis();
  }
}
