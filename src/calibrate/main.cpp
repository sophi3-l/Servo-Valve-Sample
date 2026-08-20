// ============================================================================
//  Karen Valve System — SERVO CALIBRATION            (src/calibrate/main.cpp)
//  Rev K · post short-fix revision (2026-07-27)
//
//  Find the OPEN and CLOSED horn angle for each valve, one channel at a time,
//  over the Serial Monitor. Stamp this board's lander ID with `setid <n>`, then
//  record the OPEN angles into row (n-1) of CH_OPEN_DEG_ALL in calibration.h and
//  flip CAL_COMMITTED[n-1]=true. This sketch only *finds* the numbers + the ID.
//
//  It pulls degToPulse(), the PCA addresses, the I2C pins, the servo frequency
//  AND the ≤2-active concurrency guard from include/calibration.h — the SAME
//  truth the flight builds use — so a "95°" here points the horn exactly where
//  "95°" does in deploy. (50 Hz · 25 MHz osc · 102≈500us / 307≈1500us /
//  512≈2500us.)
//
//  CHANNELS:  Ch0–15 → 0x40 ;  Ch16–31 → 0x41 (local = ch − 16).
//             Karen uses Ch0–19 = samples, Ch20 = main.
//
//  THIS IS ALSO THE Q2 CERTIFICATION BUILD (Howard's gate table):
//     fresh boot → GPIO25 LOW row  (rail off, zero PWM anywhere)
//     `rail`     → GPIO25 HIGH row (rail on, still zero PWM)
//     `off`      → back to the LOW row
//     `cycle`    → clean falling+rising rail edge for scope work
//
//  SERIAL @115200, newline ending — type `help` for this list:
//     <ch> <deg>    move (guarded, 70–180°)     e.g.  0 95   ·   16 110
//     r <ch>        release → limp              e.g.  r 0
//     off           release ALL + rail OFF
//     railup        rail ON with all 21 preloaded to resting (safe cold start)
//     rail          rail ON, no PWM (Q2 gate table HIGH row)
//     cycle [ms]    rail OFF, wait, rail ON (default 2000 ms)
//     active        list driven channels + count
//     v             battery/ADC voltage (validates GPIO34 divider)
//     table         print THIS unit's open/close map: angles + pulses
//     setid <n>     stamp this board's lander ID (1-4) into NVS
//     id            show the stamped lander ID
//     rest          set the mission RESTING STATE: Ch0-19 closed, Ch20 open
//                   ** LAST STEP OF BENCH SETUP, before flashing the mission **
//
//  Procedure: nudge to the OPEN angle (fully open, no strain) → record it.
//  Then try open+offset (≤65°) → confirm fully closed without grinding.
//  Keep one CLOSE_OFFSET_DEG that works for every channel (65° is the limit).
//
//  CHANGES 2026-08-18 (calibration update):
//    • The `table` command now calls the shared closeDegFor() from
//      calibration.h instead of inlining open+CLOSE_OFFSET_DEG, so it prints
//      the same close angle the deploy build will actually drive — including
//      Ch20 (main), which does not use the offset formula at all.
//
//  CHANGES 2026-08-18b (main-valve stop margin):
//    • Ch20's close angle is no longer a hardcoded 180 (calibration.h).
//      No change needed in this file — `table` already routes through
//      closeDegFor() — but note that `table` is now the fastest way to check
//      what the flight build will actually drive on the main valve.
//    • BENCH TASK: the common valve's close angle is a starting value, not a
//      confirmed one. Run `table` to see what this unit will actually drive on
//      Ch20, then `20 <that angle>` and `r 20` to drive it and cut PWM, and
//      confirm it seals fully and does not creep — at ~8 °C in oil, per unit.
//
//  CHANGES 2026-08-19 (resting state moved here — Howard):
//    • New `rest` command. The mission's resting state (Ch0-19 closed, Ch20
//      common OPEN) is now established HERE, deliberately, as the last step of
//      bench setup — not by the flight build on every power-up.
//      The flight build used to assert it at boot, which cost 21 actuations
//      per boot, and since esptool resets the board after an upload, every
//      FLASH was a boot. This build is the right place for it: you are already
//      here doing setid / jogs / hold-checks, and this build can never arm.
//    • This file now includes schedule.h so bench sweeps use the same
//      STEP_SETTLE_MS the mission uses. Side benefit: the schedule's
//      static_asserts get exercised in a third build environment.
// ============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include "calibration.h"   // degToPulse(), pins, guard, rail control — shared
#include "schedule.h"      // STEP_SETTLE_MS / STEP_GAP_MS — identical drive
                           // timing to the mission, so what you confirm on the
                           // bench is what the flight build actually does

Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(PCA0_ADDR);   // Ch 0–15
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(PCA1_ADDR);   // Ch 16–31
Preferences prefs;   // NVS — shared "karen" namespace with the flight build

const uint8_t VBATT_SAMPLES = 16;

// ── Raw PCA write (no policy — everything routes through the helpers below) ──
void applyPWM(uint8_t ch, uint16_t pulse) {
  if (ch <= 15) pwm0.setPWM(ch, 0, pulse);
  else          pwm1.setPWM(ch - 16, 0, pulse);
}

// ── Guarded drive / release ──────────────────────────────────────────────────
void railUpAllCommanded();   // defined below; no-op when the rail is already on

void moveChannel(uint8_t ch, uint8_t deg) {
  if (!servoGuardAllows(ch)) {                      // structural ≤2 rule
    Serial.printf("[GUARD] Ch%u refused — %u already active (", ch, servoActiveCount());
    printActiveServos(Serial);
    Serial.println("). Release one first:  r <ch>");
    return;
  }
  if (!railIsOn()) {
    Serial.println("[WARN] cold rail-up: uncommanded servos will centre (~90 deg = OPEN).");
    Serial.println("       Run `railup` first if you want all 21 held at resting.");
  }
  uint16_t pulse = degToPulse(deg);
  applyPWM(ch, pulse);                              // signal first...
  servoMarkActive(ch, true);
  railOn();                                         // no-op after the line above
  if (ch <= 15) Serial.printf("Board 0x40 Ch%u -> %u deg (pulse:%u)\n", ch, deg, pulse);
  else          Serial.printf("Board 0x41 Ch%u (local %u) -> %u deg (pulse:%u)\n",
                              ch, ch - 16, deg, pulse);
}

void releaseChannel(uint8_t ch) {
  applyPWM(ch, 4096);                               // full-OFF → output LOW → limp
  servoMarkActive(ch, false);
}

void releaseEverything() {
  for (uint8_t c = 0; c <= 31; c++) releaseChannel(c);
}

// This unit's open-angle row, or nullptr if no valid lander ID is stamped.
// Callers that DRIVE valves must refuse on nullptr — driving with another
// unit's angles is how a valve gets over-travelled into its stop.
const uint8_t* thisUnitRow() {
  prefs.begin("karen", true);
  uint8_t id = prefs.getUChar("unitid", 0);
  prefs.end();
  return chOpenRow(id);
}

// Energise the rail with every channel already commanded. A servo powered with
// no pulse train drives to its ~90 deg neutral, which is OPEN on all 21
// channels (see the railOn() warning in calibration.h) — so a bare railOn()
// from cold throws every valve open before the commanded one moves.
// No-op once the rail is up, so this costs one preload per bench session.
void railUpAllCommanded() {
  if (railIsOn()) return;
  const uint8_t* row = thisUnitRow();
  if (!row) {
    Serial.println("[RAILUP] no lander ID — uncommanded channels will centre (~90 deg)");
    railOn();
    return;
  }
  for (uint8_t ch = 0; ch <= MAIN_SERVO_CH; ch++) {
    uint8_t deg = (ch == MAIN_SERVO_CH) ? row[ch] : closeDegFor(ch, row[ch]);
    applyPWM(ch, degToPulse(deg));
  }
  Serial.println("[RAILUP] all 21 channels commanded to resting, then power");
  railOn();
  delay(STEP_SETTLE_MS);
  for (uint8_t ch = 0; ch <= MAIN_SERVO_CH; ch++) releaseChannel(ch);
}

// ── Battery ADC (same chain as the flight builds — validates the divider) ────
float readBatteryVoltage() {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < VBATT_SAMPLES; i++) {
    acc += analogReadMilliVolts(VBATT_SENSE_PIN);
    delay(2);
  }
  return ((acc / (float)VBATT_SAMPLES) / 1000.0f) / VBATT_DIVIDER_RATIO;
}

// ── Help ─────────────────────────────────────────────────────────────────────
void printHelp() {
  Serial.println();
  Serial.println("Karen calibrate/cert CLI — commands:");
  Serial.println("  <ch> <deg>   move (70-180 deg, max 2 active)   e.g. 0 95");
  Serial.println("  r <ch>       release channel (limp)");
  Serial.println("  off          release ALL + servo rail OFF");
  Serial.println("  railup       rail ON with all 21 held at resting (safe cold start)");
  Serial.println("  rail         rail ON, no PWM  (Q2 gate-table HIGH row)");
  Serial.println("  cycle [ms]   rail OFF, wait, rail ON  (default 2000)");
  Serial.println("  active       list driven channels");
  Serial.println("  v            battery voltage via GPIO34 divider");
  Serial.println("  table        print this unit's open/close map");
  Serial.println("  setid <n>    stamp this board's lander ID 1-4 into NVS");
  Serial.println("  id           show the stamped lander ID");
  Serial.println("  rest         RESTING STATE: Ch0-19 closed, Ch20 open");
  Serial.println("               (last step of bench setup, before flashing the mission)");
  Serial.println("  help         this list");
  Serial.println("Ch 0-15 -> 0x40 ;  Ch 16-31 -> 0x41 (local = ch-16). Ch20 = MAIN.");
  Serial.println();
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  initPowerPins();          // FIRST: SHUTDOWN low (stay alive), servo rail off
  Serial.begin(115200);
  delay(1000);
  Wire.begin(I2C_SDA, I2C_SCL);

  pwm0.begin(); pwm0.reset(); delay(100);
  pwm0.setOscillatorFrequency(25000000);
  pwm0.setPWMFreq(SERVO_FREQ_HZ);

  pwm1.begin(); pwm1.reset(); delay(100);
  pwm1.setOscillatorFrequency(25000000);
  pwm1.setPWMFreq(SERVO_FREQ_HZ);
  delay(100);

  releaseEverything();      // defined state: every output OFF, nothing tracked
  analogSetPinAttenuation(VBATT_SENSE_PIN, ADC_11db);

  while (Serial.available()) Serial.read();
  Serial.println("Ready. Rail is OFF (GPIO25 LOW row). Rail powers ON on any move.");
  printHelp();
}

// ── Command loop ─────────────────────────────────────────────────────────────
void loop() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;                  // ignore blank lines

  if (input == "help" || input == "?") { printHelp(); return; }

  // Release everything + rail off (de-energize when done for the day)
  if (input == "off") {
    releaseEverything();
    railOff();
    Serial.println("All channels released, servo rail OFF (GPIO25 LOW row)");
    return;
  }

  // Rail on with zero PWM — the GPIO25 HIGH row of Howard's gate table
  // Rail up with all 21 held at resting — the safe way to energise from cold.
  if (input == "railup") {
    if (railIsOn()) { Serial.println("Rail already ON."); return; }
    railUpAllCommanded();
    Serial.println("Rail ON, all 21 preloaded to resting then released.");
    return;
  }

  if (input == "rail") {
    Serial.println("[WARN] zero-PWM rail-up: uncommanded servos will centre (~90 deg = OPEN).");
    Serial.println("       This is the Q2 gate-table test. For normal work use a move command,");
    Serial.println("       which preloads all channels first.");
    railOn();
    Serial.println("Rail ON, no PWM (GPIO25 HIGH row)");
    return;
  }

  // Deliberate rail power-cycle: clean falling + rising edge for scope work.
  // NOTE: any active channels will snap back to position when power returns.
  if (input.startsWith("cycle")) {
    long ms = 2000;
    int sp = input.indexOf(' ');
    if (sp > 0) ms = input.substring(sp + 1).toInt();
    if (ms < 100) ms = 100;
    if (servoActiveCount() > 0) {
      Serial.print("[WARN] active channels will snap on power-up: ");
      printActiveServos(Serial); Serial.println();
    }
    Serial.printf("Rail OFF %ld ms...\n", ms);
    railCycle((uint32_t)ms);
    Serial.println("Rail ON (single edge)");
    return;
  }

  // Establish the mission's resting state: Ch0-19 CLOSED, Ch20 (common) OPEN.
  // One servo powered at a time — drive, settle, release, breather — using the
  // same STEP_SETTLE_MS the mission uses, so what you confirm here is what the
  // flight build does. This is the LAST thing you run before flashing the
  // mission build; that build will not move a valve until its first sample.
  if (input == "rest") {
    const uint8_t* row = thisUnitRow();
    if (!row) {
      Serial.println("Refused: no lander ID stamped. Run `setid <1-4>` first —");
      Serial.println("driving with another unit's angles risks over-travelling a valve.");
      return;
    }
    Serial.printf("Resting state: closing Ch0-%u, then opening Ch%u (common). "
                  "~%lu s, one servo at a time.\n",
                  SAMPLE_SERVO_COUNT - 1, MAIN_SERVO_CH,
                  (unsigned long)(((uint32_t)SAMPLE_SERVO_COUNT *
                                   (STEP_SETTLE_MS + STEP_GAP_MS) + STEP_SETTLE_MS) / 1000));
    for (uint8_t i = 0; i < SAMPLE_SERVO_COUNT; i++) {
      uint8_t ch = SERVO_CHANNELS[i];
      moveChannel(ch, closeDegFor(ch, row[ch]));
      delay(STEP_SETTLE_MS);
      releaseChannel(ch);
      delay(STEP_GAP_MS);
    }
    moveChannel(MAIN_SERVO_CH, row[MAIN_SERVO_CH]);   // common OPEN = its open angle
    delay(STEP_SETTLE_MS);
    releaseChannel(MAIN_SERVO_CH);
    railOff();
    Serial.println();
    Serial.println("Resting state commanded. These servos have NO position feedback —");
    Serial.printf("VISUALLY CONFIRM: Ch0-%u closed, Ch%u (common) OPEN, before flashing.\n",
                  SAMPLE_SERVO_COUNT - 1, MAIN_SERVO_CH);
    return;
  }

  if (input == "active") {
    Serial.printf("Active (%u/%u): ", servoActiveCount(), MAX_ACTIVE_SERVOS);
    printActiveServos(Serial); Serial.println();
    return;
  }

  if (input == "v") {
    Serial.printf("Vbatt = %.2f V (rail %s)\n", readBatteryVoltage(),
                  railIsOn() ? "ON" : "OFF");
    return;
  }

  if (input == "table") {
    prefs.begin("karen", true);
    uint8_t id = prefs.getUChar("unitid", 0);
    prefs.end();
    const uint8_t* row = chOpenRow(id);
    const uint8_t* show = row ? row : CH_OPEN_DEG_ALL[0];
    if (row) Serial.printf("Open/close map for Lander %u (cal %s):\n",
                           id, calCommitted(id) ? "committed" : "PLACEHOLDER");
    else     Serial.println("No unit ID stamped — showing Lander 1 row for reference:");
    Serial.println("ch | open deg/pulse | close deg/pulse");
    for (uint8_t i = 0; i <= 20; i++) {
      uint8_t o = show[i];
      uint8_t c = closeDegFor(i, o);   // Ch20 (main) uses open + MAIN_CLOSE_OFFSET_DEG
      Serial.printf("%2u |   %3u / %4u   |   %3u / %4u%s\n",
                    i, o, degToPulse(o), c, degToPulse(c),
                    i == MAIN_SERVO_CH ? "   (MAIN)" : "");
    }
    return;
  }

  // Show the stamped lander ID
  if (input == "id") {
    prefs.begin("karen", true);
    uint8_t id = prefs.getUChar("unitid", 0);
    prefs.end();
    if (id >= 1 && id <= LANDER_COUNT)
      Serial.printf("Lander ID = %u  (flight SSID: LanderController%u, cal %s)\n",
                    id, id, calCommitted(id) ? "committed" : "PLACEHOLDER — arm blocked");
    else
      Serial.println("Lander ID = UNSET. Stamp it with:  setid <1-4>");
    return;
  }

  // Stamp this board's lander ID into NVS (persists across reflashes; the flight
  // build reads it to pick this unit's calibration row and SSID).
  if (input.startsWith("setid")) {
    int sp = input.indexOf(' ');
    int n  = (sp > 0) ? input.substring(sp + 1).toInt() : -1;
    if (n < 1 || n > LANDER_COUNT) {
      Serial.printf("Usage: setid <1-%u>\n", LANDER_COUNT);
      return;
    }
    prefs.begin("karen", false);
    prefs.putUChar("unitid", (uint8_t)n);
    prefs.end();
    Serial.printf("Lander ID stamped = %d. Flight build will be LanderController%d.\n", n, n);
    if (!calCommitted((uint8_t)n))
      Serial.printf("NOTE: row %d in calibration.h is still a placeholder — "
                    "paste this unit's angles and set CAL_COMMITTED[%d]=true before flashing flight.\n",
                    n, n - 1);
    return;
  }

  // Release: "r <ch>" / "release <ch>"
  if (input.startsWith("r ") || input.startsWith("release ")) {
    int sp = input.indexOf(' ');
    int ch = input.substring(sp + 1).toInt();
    if (ch >= 0 && ch <= 31) {
      releaseChannel((uint8_t)ch);
      Serial.printf("Ch%d released (PWM off — limp)\n", ch);
    } else Serial.println("Invalid — channel 0-31");
    return;
  }

  // Move: "<channel> <degrees>"
  int spaceIdx = input.indexOf(' ');
  if (spaceIdx <= 0) {
    Serial.println("Format: <channel> <degrees>  e.g. 0 95   (type `help` for commands)");
    return;
  }
  int ch  = input.substring(0, spaceIdx).toInt();
  int deg = input.substring(spaceIdx + 1).toInt();
  if (ch < 0 || ch > 31) { Serial.println("Invalid — channel 0-31"); return; }
  if (deg < SERVO_MIN_DEG || deg > 180) {
    Serial.printf("Refused: %u-180 deg only (mechanical limit)\n", SERVO_MIN_DEG);
    return;
  }
  moveChannel((uint8_t)ch, (uint8_t)deg);
}