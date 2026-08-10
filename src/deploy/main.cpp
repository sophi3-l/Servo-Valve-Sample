// ============================================================================
//  Karen Valve Controller — FLIGHT / DEPLOY BUILD       (src/deploy/main.cpp)
//  ESP32 DevKitC-32UE · 2× PCA9685 (0x40 / 0x41) · 21 Hiwonder HPS-2018 valves
//  (main intake = Ch20 · sample valves = Ch0–19)
//
//  Shared electrical truth (pins, PCA addresses, pulse mapping, calibration
//  angles, close offset, channel layout, rail control, ≤2-servo guard) lives
//  in include/calibration.h. Do not redefine any of it here.
//
//  CHANGES vs the "deploy" draft (pre-flash review, 2026-07-30):
//    • RAIL: the draft never drove GPIO25 — the mission would have sent PWM
//      into a dead SERVO_RAIL and no valve would ever have moved. All drives
//      now go through driveServo(), which calls railOn() (state-tracked, from
//      calibration.h); every sleep path calls railOff() first. The old
//      optional SERVO_RAIL_EN_PIN ifdef block is deleted — Q2 gating is not
//      optional on this board.
//    • LATCH: initPowerPins() is now the FIRST call in setup() (SHUTDOWN low =
//      stay alive, rail off), same as the calibrate build.
//    • GUARD: all drives route through the shared ≤2-active tracker. Every
//      multi-servo operation (park, brownout recovery, web All-Open/Close/90°)
//      is now strictly sequential: drive → settle → release → next. The draft
//      held all 21 under PWM at once — the exact Q2 collapse latch from the
//      bench sessions, and on the brownout-recovery path it would have looped.
//    • Mission complete: park, then latchOff() (U10 cuts our own supply — true
//      zero draw). Deep-sleep-forever remains only as the USB-powered fallback.
//    • Testing-mode raw angle now clamps to SERVO_MIN_DEG (70°) like the
//      calibrate CLI — commanding 0° into the mechanical stop is what stalled
//      Ch20/Ch14 on the bench.
//    • /settings rejects zero/absurd timing instead of arming a 0 ms interval.
//
//  Deep-sleep rail safety relies on the R15/R13 copper pulldowns holding the
//  Q1/Q2 gate chain off while GPIO25 floats (verified on Rev K). No gpio_hold
//  needed; railOff() is still called before every sleep for a clean edge.
//
//  NOTES (unchanged): passive hold must be bench-confirmed; main open during
//  each wait; 18 h interval; 5.60 V park cutoff; brownout resumes with WiFi
//  off while a fresh power-on enters ARM MODE; ~10 mA sleep floor is the
//  DevKitC LDO+USB, not firmware. Testing Mode locks out arming.
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
#include "calibration.h"          // shared: pins, mapping, angles, channels

// Build-specific spellings mapped onto the shared names in calibration.h
#define ARM_LED_PIN  LED_PIN
#define SERVO_FREQ   SERVO_FREQ_HZ

// ── Network (ARM MODE only — never started on a mission wake) ────────────────
// SSID is built from the per-unit ID at boot (loadUnitId): LanderController1..4,
// or LanderController-UNSET if no ID has been stamped into NVS yet.
const char* AP_PASSWORD = "lander1234";
char        apSsid[32]  = "LanderController-UNSET";

// ── Per-unit identity (set once on the bench via the calibrate build's setid) ─
uint8_t         g_unitId = 0;        // 1..LANDER_COUNT; 0 = unset
bool            g_idOk   = false;    // valid ID AND calibration committed
const uint8_t*  CH_OPEN  = nullptr;  // this unit's open-angle row (selected below)

// ── Battery guard (deploy-specific) ──────────────────────────────────────────
const float   VBATT_CUTOFF_V      = 5.60f;     // park below this
const float   VBATT_PLAUSIBLE_MIN = 3.00f;     // below → assume sense fault, ignore
const uint8_t VBATT_SAMPLES       = 16;

// ── ARM window + timing (ms) — defaults; overwritten from NVS at arm time ────
//  MINI_DEPLOY (env:minideploy) compresses these so a full 20-sample run takes
//  minutes, not weeks — SAME flight code path, short clock. Real deploy is
//  untouched. The web UI can still override any of these before arming.
#ifdef MINI_DEPLOY
const uint32_t ARM_BLINK_MS    = 8000;         //  8 s  (test only — not a real seal window)
uint32_t STARTUP_DELAY_MS      = 3000;         //  3 s
uint32_t SERVO_OPEN_TIME_MS    = 2000;         //  2 s
uint32_t INTER_SAMPLE_DELAY_MS = 15000UL;      // 15 s  → full run ≈ 5–6 min
#else
const uint32_t ARM_BLINK_MS    = 45000;        // 30–60 s per design
uint32_t STARTUP_DELAY_MS      = 10000;        // slept after arm, before 1st sample
uint32_t SERVO_OPEN_TIME_MS    = 5000;         // sample valve open duration
uint32_t INTER_SAMPLE_DELAY_MS = 64800000UL;   // 18 h
#endif

const uint16_t SERVO_SETTLE_MS = 500;          // drive, then cut PWM after this

uint16_t chOpenPulse[21];
uint16_t chClosePulse[21];

// ── Persistent mission state ──────────────────────────────────────────────────
#define STATE_MAGIC 0xC0FFEE01
RTC_DATA_ATTR uint32_t rtcMagic         = 0;
RTC_DATA_ATTR uint8_t  rtcNextSampleIdx = 0;
RTC_DATA_ATTR uint8_t  rtcPhase         = 0;
RTC_DATA_ATTR bool     rtcMissionDone   = false;

enum Phase { PHASE_WARMUP = 0, PHASE_SAMPLING = 1 };

struct MissionState {
  bool     armed;
  uint8_t  nextSampleIdx;
  uint8_t  phase;
  bool     missionComplete;
  uint32_t startupMs, openMs, interMs;
};
MissionState st;
Preferences prefs;

Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(PCA0_ADDR);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(PCA1_ADDR);
WebServer server(80);

enum ServoPos { POS_CLOSE, POS_MID, POS_OPEN };
ServoPos servoPositions[21];
bool     servoHasPWM[21];

bool     armPending = false;
uint32_t armStartMs = 0;
bool     testMode   = false;

// ── Servo primitives ──────────────────────────────────────────────────────────
// Raw PCA write. NO POLICY HERE — everything must route through driveServo() /
// releaseServo() so the ≤2-active guard and the Q2 rail stay coherent.
void setPWM(uint8_t ch, uint16_t pulse) {
  if (ch < 16) pwm0.setPWM(ch, 0, pulse);
  else         pwm1.setPWM(ch - 16, 0, pulse);
}

// Guarded drive: refuse if a 3rd concurrent servo is requested (structural
// ≤2 rule from calibration.h), otherwise signal first, then energize the rail.
// railOn() is state-tracked and idempotent — a no-op if already on.
bool driveServo(uint8_t ch, uint16_t pulse, ServoPos pos, const char* tag, uint8_t degForLog) {
  if (!servoGuardAllows(ch)) {
    Serial.printf("[GUARD] Ch%2d refused — %u already active (", ch, servoActiveCount());
    printActiveServos(Serial);
    Serial.println(")");
    return false;
  }
  setPWM(ch, pulse);
  servoMarkActive(ch, true);
  servoHasPWM[ch] = true;
  servoPositions[ch] = pos;
  railOn();
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

void openServo(uint8_t ch) {
  driveServo(ch, chOpenPulse[ch], POS_OPEN, "OPEN ", CH_OPEN[ch]);
}
void closeServo(uint8_t ch) {
  uint8_t closeDeg = min((int)CH_OPEN[ch] + CLOSE_OFFSET_DEG, 180);
  driveServo(ch, chClosePulse[ch], POS_CLOSE, "CLOSE", closeDeg);
}
void midServo(uint8_t ch) {
  driveServo(ch, degToPulse(90), POS_MID, "MID  ", 90);
}
// Bench-only raw jog (ignores the 65° close offset, NOT the 70° mechanical
// floor — commanding below SERVO_MIN_DEG is what stalled Ch20/Ch14 on bench).
void angleServo(uint8_t ch, uint8_t deg) {
  if (deg < SERVO_MIN_DEG) deg = SERVO_MIN_DEG;
  driveServo(ch, degToPulse(deg), POS_MID, "TEST ", deg);
}
void sweepServo(uint8_t ch) {
  if (!servoGuardAllows(ch)) { Serial.printf("[GUARD] sweep Ch%2d refused\n", ch); return; }
  servoMarkActive(ch, true); servoHasPWM[ch] = true; railOn();
  uint16_t lo = min(chOpenPulse[ch], chClosePulse[ch]);
  uint16_t hi = max(chOpenPulse[ch], chClosePulse[ch]);
  for (uint16_t p = lo; p <= hi; p += 6) { setPWM(ch, p); delay(15); }
  delay(150);
  for (int p = hi; p >= (int)lo; p -= 6) { setPWM(ch, (uint16_t)p); delay(15); }
  delay(150);
  releaseServo(ch);
}
uint8_t activePWMCount() { uint8_t n=0; for (uint8_t ch=0; ch<=20; ch++) if (servoHasPWM[ch]) n++; return n; }

// Sequential multi-servo move: ONE servo under power at a time — drive,
// settle, release, next. This is the only sanctioned way to touch many valves;
// the certified power path is 2 concurrent servos (0.227 A), never 21.
void moveAllSequential(void (*op)(uint8_t)) {
  for (uint8_t ch = 0; ch <= 20; ch++) {
    op(ch);
    delay(SERVO_SETTLE_MS);
    releaseServo(ch);
  }
}

// ── Hardware init helpers ─────────────────────────────────────────────────────
void buildPulseTables() {
  for (uint8_t i = 0; i <= 20; i++) {
    chOpenPulse[i]   = degToPulse(CH_OPEN[i]);
    uint8_t closeDeg = min((int)CH_OPEN[i] + CLOSE_OFFSET_DEG, 180);
    chClosePulse[i]  = degToPulse(closeDeg);
    servoPositions[i] = POS_CLOSE;
  }
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
void initBatteryAdc() { analogSetPinAttenuation(VBATT_SENSE_PIN, ADC_11db); }
float readBatteryVoltage() {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < VBATT_SAMPLES; i++) { acc += analogReadMilliVolts(VBATT_SENSE_PIN); delay(2); }
  return ((acc / (float)VBATT_SAMPLES) / 1000.0f) / VBATT_DIVIDER_RATIO;
}

// ── Mission-level valve actions ───────────────────────────────────────────────
void parkAllClosed() {
  Serial.println("[VALVE] park: sequential close (one servo powered at a time)");
  moveAllSequential(closeServo);
}
void openMainAndHold() { openServo(MAIN_SERVO_CH); delay(SERVO_SETTLE_MS); releaseServo(MAIN_SERVO_CH); }
void releaseAllSamples() { for (uint8_t i=0; i<SAMPLE_SERVO_COUNT; i++) releaseServo(SERVO_CHANNELS[i]); }
void takeSample(uint8_t idx) {
  uint8_t ch = SERVO_CHANNELS[idx];
  closeServo(MAIN_SERVO_CH); delay(SERVO_SETTLE_MS); releaseServo(MAIN_SERVO_CH);
  openServo(ch); delay(SERVO_SETTLE_MS); releaseServo(ch);
  uint32_t hold = (SERVO_OPEN_TIME_MS > SERVO_SETTLE_MS) ? SERVO_OPEN_TIME_MS - SERVO_SETTLE_MS : 0;
  delay(hold);
  closeServo(ch); delay(SERVO_SETTLE_MS); releaseServo(ch);
}

// ── Persistence ───────────────────────────────────────────────────────────────
void saveState() {
  prefs.begin("karen", false);
  prefs.putBool ("armed",   st.armed);
  prefs.putUChar("idx",     st.nextSampleIdx);
  prefs.putUChar("phase",   st.phase);
  prefs.putBool ("done",    st.missionComplete);
  prefs.putUInt ("startup", st.startupMs);
  prefs.putUInt ("open",    st.openMs);
  prefs.putUInt ("inter",   st.interMs);
  prefs.end();
  rtcMagic = STATE_MAGIC; rtcNextSampleIdx = st.nextSampleIdx;
  rtcPhase = st.phase;    rtcMissionDone   = st.missionComplete;
}
void loadState() {
  prefs.begin("karen", true);
  st.armed           = prefs.getBool ("armed",   false);
  st.nextSampleIdx   = prefs.getUChar("idx",     0);
  st.phase           = prefs.getUChar("phase",   PHASE_WARMUP);
  st.missionComplete = prefs.getBool ("done",    false);
  st.startupMs       = prefs.getUInt ("startup", STARTUP_DELAY_MS);
  st.openMs          = prefs.getUInt ("open",    SERVO_OPEN_TIME_MS);
  st.interMs         = prefs.getUInt ("inter",   INTER_SAMPLE_DELAY_MS);
  prefs.end();
  if (rtcMagic == STATE_MAGIC && rtcNextSampleIdx != st.nextSampleIdx)
    Serial.printf("[STATE] RTC idx %u != NVS idx %u — trusting NVS\n", rtcNextSampleIdx, st.nextSampleIdx);
  STARTUP_DELAY_MS      = st.startupMs;
  SERVO_OPEN_TIME_MS    = st.openMs;
  INTER_SAMPLE_DELAY_MS = st.interMs;
}

// Read the per-unit ID (stamped on the bench via the calibrate build's setid),
// select this unit's calibration row, and build its SSID. Fails safe: an unset
// or uncommitted unit falls back to row 0 for math (so nothing null-derefs) but
// g_idOk stays false, which blocks arming and shows UNSET on the web UI.
void loadUnitId() {
  prefs.begin("karen", true);
  g_unitId = prefs.getUChar("unitid", 0);
  prefs.end();

  const uint8_t* row = chOpenRow(g_unitId);
  g_idOk = (row != nullptr) && calCommitted(g_unitId);
  CH_OPEN = row ? row : CH_OPEN_DEG_ALL[0];   // row-0 fallback keeps math safe

#ifdef MINI_DEPLOY
  const char* ssidTag = "-TEST";   // mark test firmware on the deck — DO NOT SEAL
#else
  const char* ssidTag = "";
#endif
  if (row) snprintf(apSsid, sizeof(apSsid), "LanderController%u%s", g_unitId, ssidTag);
  else     snprintf(apSsid, sizeof(apSsid), "LanderController-UNSET%s", ssidTag);

  Serial.printf("[ID]    unit=%u  cal=%s  ssid=%s\n",
                g_unitId, g_idOk ? "committed" : "MISSING/placeholder", apSsid);
}

// ── Deep sleep ────────────────────────────────────────────────────────────────
void radiosOff() { WiFi.mode(WIFI_OFF); /* BT never started */ }

void enterDeepSleep(uint64_t us) {
  digitalWrite(ARM_LED_PIN, LOW);
  releaseAll();               // no PWM into a rail we're about to drop
  railOff();                  // clean edge; R15/R13 hold the gate off in sleep
  sleepServoDrivers();
  Serial.printf("[SLEEP] deep sleep %llu s\n", us / 1000000ULL);
  Serial.flush();
  esp_sleep_enable_timer_wakeup(us);
  esp_deep_sleep_start();
}

// Mission complete: cut our own supply via the U10 latch. On battery, power
// ends inside latchOff()'s delay and nothing below it runs. The deep-sleep
// tail is the USB-on-bench fallback only (USB back-feed keeps us alive).
void enterDeepSleepForever() {
  digitalWrite(ARM_LED_PIN, LOW);
  releaseAll();
  railOff();
  sleepServoDrivers();
  Serial.println("[SLEEP] mission complete — latching power off (U10)");
  Serial.flush();
  latchOff();                 // ← execution normally ends here on battery
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  Serial.println("[SLEEP] still alive (USB) — deep sleep until reset/recovery");
  Serial.flush();
  esp_deep_sleep_start();
}

// ── Mission wake ───────────────────────────────────────────────────────────────
void runMissionWake(esp_sleep_wakeup_cause_t cause) {
  radiosOff();
  // Rail stays OFF until the first driveServo() call — the battery check and
  // logging below run at sleep-level draw, and a low battery never actuates.
  buildPulseTables();
  initBatteryAdc();
  initServoDrivers();

  float vb = readBatteryVoltage();
  Serial.printf("[WAKE]  cause=%d idx=%u phase=%u Vbatt=%.2f V\n",
                cause, st.nextSampleIdx, st.phase, vb);

  if (vb >= VBATT_PLAUSIBLE_MIN && vb < VBATT_CUTOFF_V) {
    Serial.println("[WAKE]  LOW BATTERY → park, no actuation");
    parkAllClosed();
    enterDeepSleep((uint64_t)INTER_SAMPLE_DELAY_MS * 1000ULL);
  }

  if (cause != ESP_SLEEP_WAKEUP_TIMER) {
    // Sequential on purpose: this path runs right after a brownout — slamming
    // 20 servos at once here is how the Q2 collapse latch becomes a boot loop.
    Serial.println("[WAKE]  unexpected reset mid-mission → re-establish state (sequential)");
    for (uint8_t i = 0; i < SAMPLE_SERVO_COUNT; i++) {
      closeServo(SERVO_CHANNELS[i]);
      delay(SERVO_SETTLE_MS);
      releaseServo(SERVO_CHANNELS[i]);
    }
    if (st.phase == PHASE_WARMUP) enterDeepSleep((uint64_t)STARTUP_DELAY_MS * 1000ULL);
    else { openMainAndHold(); enterDeepSleep((uint64_t)INTER_SAMPLE_DELAY_MS * 1000ULL); }
  }

  if (st.phase == PHASE_WARMUP) {
    Serial.println("[WAKE]  warmup done → open main, begin flush");
    openMainAndHold();
    st.phase = PHASE_SAMPLING; saveState();
    enterDeepSleep((uint64_t)INTER_SAMPLE_DELAY_MS * 1000ULL);
  } else {
    Serial.printf("[WAKE]  sample %u/%u (Ch%u)\n",
                  st.nextSampleIdx + 1, SAMPLE_SERVO_COUNT, SERVO_CHANNELS[st.nextSampleIdx]);
    takeSample(st.nextSampleIdx);
    st.nextSampleIdx++;
    if (st.nextSampleIdx >= SAMPLE_SERVO_COUNT) {
      st.missionComplete = true; saveState();
      Serial.println("[WAKE]  ALL SAMPLES COLLECTED → park, sleep until recovery");
      parkAllClosed();
      enterDeepSleepForever();
    } else {
      openMainAndHold(); saveState();
      enterDeepSleep((uint64_t)INTER_SAMPLE_DELAY_MS * 1000ULL);
    }
  }

  enterDeepSleep((uint64_t)INTER_SAMPLE_DELAY_MS * 1000ULL);  // defensive catch-all
}

// ── Web UI (ARM MODE / bench only) ────────────────────────────────────────────
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Lander — Arm</title>
<style>
  :root{--bg:#0d1117;--card:#161b22;--border:#30363d;--text:#c9d1d9;--dim:#8b949e;
        --blue:#58a6ff;--green:#238636;--red:#da3633;--orange:#f0883e;--yellow:#9e6a03;--grey:#30363d;}
  *{box-sizing:border-box;}
  body{font-family:monospace;background:var(--bg);color:var(--text);padding:16px;max-width:620px;margin:0 auto;}
  h2{color:var(--blue);font-size:13px;text-transform:uppercase;letter-spacing:.08em;margin:0 0 8px;}
  .card{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:14px;margin-bottom:12px;}
  .status{text-align:center;} .state{font-size:18px;font-weight:bold;color:var(--blue);}
  .batt{font-size:34px;font-weight:bold;color:var(--orange);line-height:1.1;margin-top:4px;}
  .prog{font-size:13px;color:var(--dim);margin-top:3px;}
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
  input[type=number],select{background:var(--card);color:var(--text);border:1px solid var(--border);border-radius:4px;padding:6px 8px;width:100%;font-size:13px;}
  input.dirty{border-color:var(--orange);}
  .grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px;}
  .hms-row{display:flex;gap:4px;} .hms-row input{text-align:center;}
  .hms-label{display:flex;justify-content:space-between;align-items:baseline;} .hms-hint{font-size:10px;color:#555;}
  .note{font-size:11px;color:var(--dim);margin-bottom:8px;} .warn{font-size:11px;color:var(--orange);margin:6px 0 10px;}
  .sep{border:none;border-top:1px solid #21262d;margin:12px 0;}
  .apply-row{display:flex;gap:8px;align-items:center;} .apply-row button{flex:1;padding:10px;}
  #apply-status,#arm-status{font-size:11px;color:var(--dim);} .arm-btn{padding:14px;font-size:15px;}
</style></head><body>

<div class="card status"><div class="state" id="st">--</div><div class="prog" id="unit"></div><div class="batt" id="vb">-- V</div><div class="prog" id="pg"></div></div>

<h2>Servos</h2><div class="note" id="offset-note"></div><div class="servo-grid" id="grid"></div>
<div class="row">
  <button class="g"  onclick="cmd('cal_all_open')">All Open</button>
  <button class="r"  onclick="cmd('cal_all_close')">All Close</button>
  <button class="gr" onclick="cmd('cal_all_mid')">All 90°</button>
  <button class="gr" onclick="cmd('cal_all_release')">Release All</button>
</div>

<hr class="sep"><h2>Timing</h2>
<div class="grid2" style="margin-bottom:10px">
  <div><label>Startup delay (ms)</label><input type="number" id="t_start" step="500" min="0"></div>
  <div><label>Sample open time (ms)</label><input type="number" id="t_open" step="100" min="100"></div>
</div>
<div style="margin-bottom:10px"><div class="hms-label"><label>Inter-sample interval</label>
  <span class="hms-hint">h : mm : ss · 18 hr = 18 : 00 : 00</span></div>
  <div class="hms-row"><input type="number" id="t_h" placeholder="h" min="0" max="99">
  <input type="number" id="t_m" placeholder="mm" min="0" max="59">
  <input type="number" id="t_sec" placeholder="ss" min="0" max="59"></div></div>
<div class="apply-row"><button class="b" onclick="applySettings()">Apply Timing</button><span id="apply-status"></span></div>

<hr class="sep"><h2>Testing Mode</h2>
<div class="note">Bench bring-up — one servo at a time. Arming is locked while this is on; exit to arm.</div>
<div class="row"><button class="b" id="tm-btn" onclick="toggleTest()">Enter Testing Mode</button></div>
<div id="tm-panel" style="display:none">
  <div class="grid2" style="margin-bottom:10px">
    <div><label>Channel under test</label><select id="tm-ch"></select></div>
    <div><label>Raw angle · 0–180° · ignores 65° limit</label>
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
  <div class="warn">Hold-check drives to position, cuts PWM, leaves it limp — watch for creep. This is the passive-hold test the deep-sleep budget depends on.</div>
</div>

<hr class="sep"><h2>Arm / Deploy</h2>
<div class="warn">Arming closes all valves, blinks the ARM LED ~45 s (seal now), then drops WiFi and begins the deep-sleep mission. No remote control once armed.</div>
<div class="row"><button class="g arm-btn" onclick="arm()">ARM &amp; DEPLOY</button>
  <button class="gr arm-btn" onclick="clearMission()">Clear / Disarm</button></div>
<div id="arm-status"></div>

<script>
const CHANNELS=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20];
const grid=document.getElementById('grid');
CHANNELS.forEach(ch=>{const m=ch===20;grid.innerHTML+=`
  <div class="sc${m?' main-ch':''}" id="sc${ch}"><div class="sc-name">${m?'MAIN':'Ch'+ch}</div>
  <div class="sc-pos" id="sp${ch}">--</div><div class="row" style="margin:0">
  <button class="g" onclick="sc(${ch},'open')">O</button>
  <button class="r" onclick="sc(${ch},'close')">C</button>
  <button class="gr" onclick="sc(${ch},'release')">R</button></div></div>`;});
const tsel=document.getElementById('tm-ch');
CHANNELS.forEach(ch=>{const o=document.createElement('option');o.value=ch;
  o.textContent=(ch===20?'MAIN (Ch20)':'Ch'+ch);tsel.appendChild(o);});
function cmd(a){fetch('/control?action='+a).catch(()=>{});}
function sc(ch,a){fetch('/servo?ch='+ch+'&action='+a).catch(()=>{});}
function tch(){return parseInt(document.getElementById('tm-ch').value);}
function tsc(a){fetch('/servo?ch='+tch()+'&action='+a).catch(()=>{});}
function tmove(){fetch('/servo?ch='+tch()+'&action=angle&deg='+document.getElementById('tm-degv').value).catch(()=>{});}
function thold(p){fetch('/servo?ch='+tch()+'&action=hold&pos='+p).catch(()=>{});}
function toggleTest(){const on=document.getElementById('tm-panel').style.display==='none';fetch('/testmode?on='+(on?1:0)).catch(()=>{});}
let timingDirty=false;const timingIds=['t_start','t_open','t_h','t_m','t_sec'];
timingIds.forEach(id=>{document.getElementById(id).addEventListener('input',()=>{timingDirty=true;
  timingIds.forEach(i=>document.getElementById(i).classList.add('dirty'));
  document.getElementById('apply-status').textContent='unsaved';document.getElementById('apply-status').style.color='#f0883e';});});
function applySettings(){const h=parseInt(document.getElementById('t_h').value)||0,m=parseInt(document.getElementById('t_m').value)||0,s=parseInt(document.getElementById('t_sec').value)||0;
  const interMs=(h*3600+m*60+s)*1000;
  fetch(`/settings?startup=${document.getElementById('t_start').value}&open=${document.getElementById('t_open').value}&inter=${interMs}`).then(()=>{
    timingDirty=false;timingIds.forEach(i=>document.getElementById(i).classList.remove('dirty'));
    document.getElementById('apply-status').textContent='saved ✓';document.getElementById('apply-status').style.color='#3fb950';
    setTimeout(()=>{document.getElementById('apply-status').textContent='';},2000);
  }).catch(()=>{document.getElementById('apply-status').textContent='send failed';document.getElementById('apply-status').style.color='#f85149';});}
function arm(){if(!confirm('Arm and deploy? WiFi drops after the ~45 s LED blink and the mission runs on deep sleep with no remote control. Seal the enclosure during the blink.'))return;
  fetch('/arm?confirm=1').then(()=>{document.getElementById('arm-status').textContent='ARMED — LED blinking, seal now. WiFi will drop.';
  document.getElementById('arm-status').style.color='#f0883e';}).catch(()=>{document.getElementById('arm-status').textContent='send failed';});}
function clearMission(){if(!confirm('Clear mission state (disarm, reset sample index to 0)?'))return;
  fetch('/clear?confirm=1').then(()=>{document.getElementById('arm-status').textContent='Cleared / disarmed.';
  document.getElementById('arm-status').style.color='#3fb950';}).catch(()=>{});}
function msToHMS(ms){const t=Math.floor(ms/1000);return{h:Math.floor(t/3600),m:Math.floor((t%3600)/60),s:t%60};}
function poll(){fetch('/status').then(r=>r.json()).then(d=>{
  document.getElementById('st').textContent=d.state;
  const idok=!!d.id_ok;const uEl=document.getElementById('unit');
  uEl.textContent=d.unit>0?('Lander '+d.unit+(idok?'':' · CAL NOT COMMITTED')):'NO UNIT ID SET — bench-set before deploy';
  uEl.style.color=(d.unit>0&&idok)?'#8b949e':'#f85149';
  document.getElementById('vb').textContent=(d.vbatt?d.vbatt.toFixed(2):'--')+' V';
  document.getElementById('vb').style.color=(d.vbatt&&d.vbatt<5.6)?'#f85149':'#f0883e';
  document.getElementById('pg').textContent=d.armed?('Armed · next sample '+(parseInt(d.idx)+1)+' of '+d.total):(d.complete?'Mission complete · '+d.total+' samples':'Not armed');
  const test=!!d.test;document.getElementById('tm-panel').style.display=test?'block':'none';
  const tb=document.getElementById('tm-btn');tb.textContent=test?'Exit Testing Mode':'Enter Testing Mode';tb.className=test?'r':'b';
  const armBlock=test||!idok;
  const ab=document.querySelector('.arm-btn.g');if(ab){ab.disabled=armBlock;ab.style.opacity=armBlock?'0.4':'1';ab.style.cursor=armBlock?'not-allowed':'pointer';}
  if(!timingDirty){if(d.startup_ms!==undefined)document.getElementById('t_start').value=d.startup_ms;
    if(d.open_ms!==undefined)document.getElementById('t_open').value=d.open_ms;
    if(d.inter_ms!==undefined){const t=msToHMS(parseInt(d.inter_ms));document.getElementById('t_h').value=t.h;
      document.getElementById('t_m').value=t.m;document.getElementById('t_sec').value=t.s;}}
  if(d.close_offset)document.getElementById('offset-note').textContent='Open = calibrated · Close = open +'+d.close_offset+'° toward 180°';
  CHANNELS.forEach(ch=>{const c=document.getElementById('sc'+ch),p=document.getElementById('sp'+ch),s=d.servoStates[ch];
    c.className='sc'+(ch===20?' main-ch':'')+(s==='open'?' open':s==='close'?' closed':'');
    p.textContent=s==='open'?'OPEN':s==='close'?'CLOSED':'MID';});
}).catch(()=>{});}
setInterval(poll,750); poll();
</script></body></html>
)rawliteral";

// ── Web handlers ──────────────────────────────────────────────────────────────
void handleRoot() { server.send(200, "text/html", HTML_PAGE); }

void handleStatus() {
  const char* stateStr = st.missionComplete ? "COMPLETE"
                       : testMode            ? "TESTING"
                       : st.armed            ? "ARMED"
                       : !g_idOk             ? "NEEDS UNIT ID"
                       :                       "ARM MODE";
  String json = "{";
  json += "\"state\":\""     + String(stateStr)              + "\",";
  json += "\"unit\":"        + String(g_unitId)              + ",";
  json += "\"id_ok\":"       + String(g_idOk ? 1 : 0)        + ",";
  json += "\"armed\":"       + String(st.armed ? 1 : 0)      + ",";
  json += "\"complete\":"    + String(st.missionComplete?1:0) + ",";
  json += "\"idx\":"         + String(st.nextSampleIdx)      + ",";
  json += "\"total\":"       + String(SAMPLE_SERVO_COUNT)    + ",";
  json += "\"startup_ms\":"  + String(STARTUP_DELAY_MS)      + ",";
  json += "\"open_ms\":"     + String(SERVO_OPEN_TIME_MS)    + ",";
  json += "\"inter_ms\":"    + String(INTER_SAMPLE_DELAY_MS) + ",";
  json += "\"close_offset\":"+ String(CLOSE_OFFSET_DEG)      + ",";
  json += "\"test\":"        + String(testMode ? 1 : 0)      + ",";
  json += "\"vbatt\":"       + String(readBatteryVoltage(),2) + ",";
  json += "\"servoStates\":[";
  for (int i = 0; i <= 20; i++) {
    json += (servoPositions[i]==POS_OPEN) ? "\"open\"" : (servoPositions[i]==POS_MID) ? "\"mid\"" : "\"close\"";
    if (i < 20) json += ",";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleControl() {
  if (!server.hasArg("action")) { server.send(400,"text/plain","Missing action"); return; }
  String a = server.arg("action");
  // Sequential: one servo powered at a time (drive → settle → release).
  // Positions are held passively afterwards — that IS the flight behavior.
  if      (a == "cal_all_open")    { moveAllSequential(openServo); }
  else if (a == "cal_all_close")   { moveAllSequential(closeServo); }
  else if (a == "cal_all_mid")     { moveAllSequential(midServo); }
  else if (a == "cal_all_release") { releaseAll(); railOff(); }
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
    else                   { if (server.arg("pos")=="open") openServo(ch); else closeServo(ch); delay(SERVO_SETTLE_MS); releaseServo(ch); }
  }
  server.send(200,"text/plain","OK");
}

void handleSettings() {
  long startupMs = server.hasArg("startup") ? server.arg("startup").toInt() : (long)STARTUP_DELAY_MS;
  long openMs    = server.hasArg("open")    ? server.arg("open").toInt()    : (long)SERVO_OPEN_TIME_MS;
  long interMs   = server.hasArg("inter")   ? server.arg("inter").toInt()   : (long)INTER_SAMPLE_DELAY_MS;
  // Guard rails: an empty form field parses to 0, and a 0 ms interval arms a
  // mission that wakes continuously until the battery is flat.
  if (startupMs < 0 || openMs < 100 || interMs < 1000) {
    server.send(400, "text/plain", "rejected: startup>=0, open>=100ms, inter>=1s");
    return;
  }
  STARTUP_DELAY_MS      = (uint32_t)startupMs;
  SERVO_OPEN_TIME_MS    = (uint32_t)openMs;
  INTER_SAMPLE_DELAY_MS = (uint32_t)interMs;
  Serial.printf("[WEB]   timing startup:%lu open:%lu inter:%lu ms\n",
                STARTUP_DELAY_MS, SERVO_OPEN_TIME_MS, INTER_SAMPLE_DELAY_MS);
  server.send(200,"text/plain","OK");
}

void handleArm() {
  if (testMode) { server.send(409,"text/plain","exit testing mode before arming"); return; }
  if (!g_idOk) {
    server.send(409, "text/plain",
      g_unitId == 0 ? "no unit ID set — run `setid <n>` on the calibrate build first"
                    : "calibration for this unit is not committed (placeholder row)");
    return;
  }
  if (server.arg("confirm") != "1") { server.send(400,"text/plain","confirm=1 required"); return; }
  st.armed = true; st.nextSampleIdx = 0; st.phase = PHASE_WARMUP; st.missionComplete = false;
  st.startupMs = STARTUP_DELAY_MS; st.openMs = SERVO_OPEN_TIME_MS; st.interMs = INTER_SAMPLE_DELAY_MS;
  saveState();
  armPending = true; armStartMs = millis();
  Serial.println("[ARM]   armed; blinking LED then deep sleep");
  server.send(200,"text/plain","ARMED");
}

void handleClear() {
  if (server.arg("confirm") != "1") { server.send(400,"text/plain","confirm=1 required"); return; }
  st.armed = false; st.nextSampleIdx = 0; st.phase = PHASE_WARMUP; st.missionComplete = false;
  saveState();
  armPending = false; digitalWrite(ARM_LED_PIN, LOW);
  Serial.println("[ARM]   mission cleared / disarmed");
  server.send(200,"text/plain","CLEARED");
}

void handleTestMode() {
  if (armPending) { server.send(409,"text/plain","arming in progress"); return; }
  bool on = (server.arg("on") == "1");
  testMode = on;
  if (!on) { releaseAll(); railOff(); }
  Serial.printf("[TEST]  testing mode %s\n", on ? "ON" : "OFF");
  server.send(200,"text/plain", on ? "TEST_ON" : "TEST_OFF");
}

void handleNotFound() { server.send(404,"text/plain","Not found"); }

// ── ARM MODE ──────────────────────────────────────────────────────────────────
void startArmMode() {
  buildPulseTables();
  initBatteryAdc();
  initServoDrivers();
  parkAllClosed();
  railOff();                 // idle bench state: parked, passive hold, rail off
  WiFi.softAP(apSsid, AP_PASSWORD);
  Serial.printf("[AP]    %s  http://%s\n", apSsid, WiFi.softAPIP().toString().c_str());
  server.on("/",         handleRoot);
  server.on("/status",   handleStatus);
  server.on("/control",  handleControl);
  server.on("/servo",    handleServo);
  server.on("/testmode", handleTestMode);
  server.on("/settings", handleSettings);
  server.on("/arm",      handleArm);
  server.on("/clear",    handleClear);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("[AP]    ready — calibrate, set timing, then ARM");
}

// ── Setup / Loop ──────────────────────────────────────────────────────────────
void setup() {
  initPowerPins();            // FIRST: SHUTDOWN low (stay alive), servo rail OFF
  Serial.begin(115200); delay(300);
#ifdef MINI_DEPLOY
  Serial.println("###### MINI-DEPLOY TEST BUILD — short timings, -TEST SSID, DO NOT SEAL ######");
#endif
  memset(servoHasPWM, false, sizeof(servoHasPWM));
  pinMode(ARM_LED_PIN, OUTPUT); digitalWrite(ARM_LED_PIN, LOW);

  loadState();
  loadUnitId();               // select this unit's calibration row + SSID
  esp_reset_reason_t       rr = esp_reset_reason();
  esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
  Serial.printf("=== BOOT reset=%d wake=%d armed=%d done=%d idx=%u ===\n",
                rr, wc, st.armed, st.missionComplete, st.nextSampleIdx);

  bool selfWoke = (rr == ESP_RST_DEEPSLEEP);
  bool brownout = (rr == ESP_RST_BROWNOUT);

  if (st.armed && !st.missionComplete && (selfWoke || brownout)) {
    runMissionWake(wc);          // never returns
    return;                      // insurance: never fall through to WiFi mid-mission
  }
  if (st.missionComplete && (selfWoke || brownout)) {
    Serial.println("[BOOT]  mission complete — park + quiet sleep");
    buildPulseTables(); initServoDrivers(); parkAllClosed();
    enterDeepSleepForever();
  }

  startArmMode();                // fresh power-on / EN reset → bench + pre-seal arming
}

void loop() {
  server.handleClient();
  if (armPending) {
    digitalWrite(ARM_LED_PIN, (millis() / 250) % 2);
    if (millis() - armStartMs >= ARM_BLINK_MS) {
      armPending = false;
      Serial.println("[ARM]   seal window done → park + begin mission");
      parkAllClosed();
      enterDeepSleep((uint64_t)STARTUP_DELAY_MS * 1000ULL);
    }
  }
}