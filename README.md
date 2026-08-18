# Karen Valve System — Firmware

ESP32 firmware for an autonomous underwater sequential water sampler. Twenty-one
servo pinch valves (Ch0–19 = sample valves, Ch20 = common/main intake) are driven
by two PCA9685 PWM boards over I²C and sequenced by an ESP32 DevKitC. The unit
self-starts when the deployment plug latches it on and runs the whole mission on
deep sleep, waking only to actuate one sample per scheduled event.

One firmware image ships to all four landers. Each unit carries a stable ID in
NVS that selects its own servo calibration and names its WiFi AP, so a single
binary covers the fleet.

> **Status:** the firmware implements the Zach & Howard sampling protocol on an
> absolute schedule. Three numbers are **not yet bench-confirmed** — see
> [Before flight](#before-flight) at the bottom. Do not seal a unit until they are.

---

## Repository layout

```
platformio.ini                     three build environments (below)
include/calibration.h              ELECTRICAL truth — pins, angles, rail, latch, guard
include/schedule.h                 MISSION TIMING — the sampling protocol
src/deploy/main.cpp                flight build — self-arming deep-sleep mission + service UI
src/calibrate/main.cpp             bench/calibration CLI (serial) — also stamps the unit ID
tools/check_calibration.py         validates both headers (run locally or in CI)
docs/protocol.md                   the sampling protocol as implemented, and why
docs/calibration.md                per-unit calibration procedure and recorded angles
.github/workflows/calibration-check.yml   CI: lint both headers + build every environment
```

Two shared headers, deliberately separate: **`calibration.h` owns electrical
truth, `schedule.h` owns mission timing.** An angle change and a schedule change
are different reviews by different people. If a value lives in either, do not
redefine it in a `main.cpp`.

## Build environments

Build/upload from the PlatformIO sidebar (Project Tasks → *env*) or the CLI:

| Env | Source | Purpose |
|---|---|---|
| `calibrate` | `src/calibrate/` | Serial CLI to find each servo's open/close angles, drive one channel at a time, and **`setid`** the board's lander ID into NVS. |
| `deploy` | `src/deploy/` | The real flight build. Self-arms on power-up, 18-day mission. |
| `minideploy` | `src/deploy/` | **Same source**, compiled with `-DMINI_DEPLOY`: the schedule's *waits* are compressed but the valve routine runs at full flight timing. ~55 min end to end, `-TEST` SSID. **Not for sealing.** |

```
pio run -e calibrate  -t upload
pio run -e deploy     -t upload
pio run -e minideploy -t upload
pio run -e calibrate -e deploy -e minideploy   # compile all three (what CI does)
```

`default_envs = deploy`, so a bare `pio run` or an unqualified Upload button
builds the **flight** firmware. Always name the environment explicitly.

## Per-lander calibration

Open angles are trimmed to each individual servo, so they differ per physical
lander. The four rows live in `CH_OPEN_DEG_ALL[4][21]`; the unit picks its row at
boot from the ID in NVS.

**The team calls the units A/B/C/D; the firmware only knows numbers.** `setid`,
NVS, and the SSID all use 1–4. Nothing enforces the letter mapping — it is a
comment in `calibration.h` and this table:

| Bench name | Unit ID | SSID | Notes |
|---|---|---|---|
| Lander D | 1 | `LanderController1` | the original calibrated set |
| Lander A | 2 | `LanderController2` | bench-confirmed 2026-08-18 |
| Lander B | 3 | `LanderController3` | bench-confirmed 2026-08-18 |
| Lander C | 4 | `LanderController4` | Ch18–19 pending servo transplant from D |

Closed angles come from one shared helper, `closeDegFor(ch, openDeg)`:

- **Sample valves (Ch0–19)** close at `open + CLOSE_OFFSET_DEG` (65°). 65 is the
  ceiling: the highest committed open angle is 115° (D Ch13, C Ch10), and
  115 + 65 = 180 exactly.
- **Common valve (Ch20)** closes at `MAIN_CLOSE_DEG` (175°) and does not use the
  offset at all. It previously used a hardcoded 180 — the absolute end of the
  pulse range, i.e. straight into the servo's mechanical stop, twice per sample
  event. 175 gives a 5° cushion and is **not yet bench-confirmed.**

`SERVO_MIN_DEG` (70°) is the mechanical floor; the calibrate CLI refuses below it.

## Mission model

**There is no operator ARM step.** A fresh power-on (deployment plug shorts
SubConn 3–4 → ARM one-shot → U10 latches → ESP32 boots) *is* the arming action,
and that instant is **t = 0**. The web UI is a *service* interface — check,
abort, restart — not the way you arm.

A boot proceeds:

1. **LED1 blinks 45 s.** This is the Rev K arm confirmation: the ESP32 is powered
   *through* U10, so a blinking LED proves the latch caught. No blink → do not deploy.
2. **Service window, 5 min.** WiFi AP up. Verify unit, battery, state. Abort here
   if you need to — it is the last chance before anything moves.
3. **Resting state established:** 20 sample valves closed, then common opened.
4. **Schedule runs on deep sleep.** First sample at t = 32 h 55 m.

Each sleep is computed as `next_event_time − elapsed_now`, so time spent awake
comes out of the following sleep instead of pushing every later event back. Over
the mission that is the difference between staying on schedule and finishing well
over an hour late.

Other behaviour worth knowing:

- **Service windows** also open after each sample event (5 min) and on any
  unexpected reset — never on a heartbeat wake. 2.4 GHz does not propagate at
  300 m, so these matter on deck and after recovery. Cost is ~0.21 Ah of a 24 Ah
  pack.
- **Heartbeat wakes** every 6 h log battery and elapsed time with no actuation,
  so the long gaps (up to 59 h 55 m) are not blind. ~1.2 mAh total.
- **Any reset resumes the mission** — watchdog, panic, brownout, deep-sleep
  timer. A power-on mid-mission resumes at the right event but cannot know how
  long the outage was, so the schedule slips; it logs `[CLOCK]` loudly when this
  happens.
- **Low battery skips, it does not park.** Below cutoff the wake logs, skips the
  event, and sleeps without touching the rail. Valves already hold passively.
- **Mission end: no final park.** Each event closes its own sample valve and the
  last event leaves common open, which is the intended recovery state. Then
  `latchOff()` cuts the unit's own supply.

See [`docs/protocol.md`](docs/protocol.md) for the event table and the reasoning.

## Power

| Item | Value |
|---|---|
| Pack | 2 × Power-Sonic PS-6100 **in parallel** — 6 V nominal, 24 Ah |
| Servos | 21 × Hiwonder HPS-2018 — rated **6.0–8.4 V**, 0.20 s/60° @ 7.4 V no-load, stall 1.4–2.3 A |
| Concurrency | `MAX_ACTIVE_SERVOS = 2`, enforced at the drive primitive; the sample routine only ever drives one |
| Low-battery cutoff | 5.60 V — **inherited, not validated** |

Note the servos are at or below their minimum rated voltage for most of a 6 V
SLA's discharge, and they run submerged in oil at ~8 °C. The datasheet speed
figure is optimistic on several axes at once, which is why the settle times are
2 s and why the common valve backs off its mechanical stop.

**The servo rail is not powered by USB.** `SERVO_RAIL` comes from `+6V_FUSED` via
F3 and Q2; USB only powers the ESP32 and (through it) the PCA9685 VCC pins. So
flashing over USB with the battery disconnected runs the full firmware, serves
the UI, and moves nothing. That is the safe bench configuration.

---

## Working on more than one machine

The repo is portable by design: **no tracked file references a specific machine.**
The three that do — `.vscode/launch.json`, `.vscode/c_cpp_properties.json`, and
`.pio/` — are gitignored and must stay that way. `launch.json` hardcodes the
builder's home directory and toolchain path; `.pio/build/*/idedata.json` embeds
absolute compiler paths. Committing either breaks the other machine.

**Do not put a working clone inside OneDrive (or Dropbox).** Two sync mechanisms
fighting over one working tree is the fastest way to a corrupted `.git` or a
half-written build. `.pio/` alone is ~100 MB of churn per build, and gitignore
does not stop OneDrive from syncing it. Clone to a plain local path:

```
git clone <repo-url> C:\dev\karen-valve
cd C:\dev\karen-valve
```

Let GitHub be the sync mechanism, not the file-sync client.

### Setting up a fresh machine

```
pip install -U platformio            # or install the PlatformIO IDE extension
git clone <repo-url> C:\dev\karen-valve
cd C:\dev\karen-valve
pio run -e calibrate -e deploy -e minideploy      # downloads toolchain, builds all three
python tools/check_calibration.py
```

That is the whole setup. PlatformIO fetches the platform, toolchain, and
libraries into `~/.platformio` (outside the repo) on the first build, and
`upload_port` is auto-detected so the COM number differing per laptop does not
matter.

### Pin the platform before flight

`platformio.ini` currently tracks the pioarduino `stable` release, which is a
**moving target** — two machines cloning on different days can get different
ESP32 Arduino cores. Pin it once, from whichever machine has a known-good build:

```
pio pkg list -e deploy      # note the espressif32 version
```

then edit the `platform =` line in `platformio.ini` per the comment there, and
commit. Unpinned is fine for bench iteration; it is not fine for the build that
gets sealed into a lander.

Library versions are already pinned exactly (`@3.0.3`, not `@^3.0.3`).

## Running a mini-deploy (lab)

A rehearsal runs the **real** valve routine — 2 s settles, 116 s sample-open,
126 s per event — with only the waits compressed. It therefore cannot be shorter
than about 42 minutes, and lands around **55 minutes**.

### Bench power

With the U10 latch not yet installed, the working lab setup is **USB for logic,
PSU for the servo rail**:

- USB → ESP32 (and PCA9685 VCC)
- PSU → PARALLEL_BATT+ (battery terminals) → F1 → Q3 → `+6V_FUSED` → F3 → Q2 → `SERVO_RAIL`
- PSU ground tied to the common ground bus

Set **~6.4 V** (a charged 6 V SLA at rest) and a current limit of **at least 3 A**
per board — one servo moves at a time, but stall is 1.4–2.3 A and there is inrush
into the ~600 µF C1 bank on every rail-on.

> **Mind the dead band.** The battery guard reads `+6V_FUSED` through the
> 220k/100k divider. A PSU set between **3.00 V and 5.60 V** makes every event log
> `LOW BATTERY` and skip — the run "completes" in 55 minutes having actuated
> nothing. Below 3.00 V (or with no PSU on the battery bus at all) the reading is
> treated as a sense fault and ignored, and events run normally.

### Procedure

```
pio run -e calibrate -e deploy -e minideploy     # compile everything first

pio run -e calibrate -t upload
#   serial @ 115200, newline ending:
setid 2          # NUMBER, not letter:  D=1  A=2  B=3  C=4
id               # confirm "Lander ID = 2 ... cal committed"
table            # confirm Ch20 close shows 175, not 180

pio run -e minideploy -t upload
```

Confirm on boot: the `###### MINI-DEPLOY TEST BUILD` banner and
`ssid=LanderController2-TEST`. Then it runs itself — 8 s LED blink, 60 s boot
window, resting state, 20 events about 2½ minutes apart, ~55 min total. WiFi is
optional; serial has everything.

**Watch for:** `[EVENT] sample n/20` twenty times, then `[DONE] all 20 samples
collected`. A `[GUARD]` line means something tried to drive a third servo and was
refused — that should be unreachable. `[WAKE] LOW BATTERY` means the PSU is in
the dead band above. `[CLOCK]` means an unexpected reset.

Check that each `[EVENT]`'s reported elapsed time is close to its scheduled time.
Flight overshoot should be 0 s; a rehearsal may overshoot up to ~80 s because the
compressed gaps are shorter than the routine plus service window, which
deliberately exercises the overdue-event path.

### Re-running

With no latch wired, `latchOff()` does nothing and the unit deep-sleeps, logging
`still alive (USB / latch not wired)`. Tap EN/reset — the boot window opens with
state `COMPLETE` and a **Restart from t=0** button. No flash erase needed.

## Deploying a lander

1. Calibrate and `setid` the unit (see [`docs/calibration.md`](docs/calibration.md)).
2. `python tools/check_calibration.py` → commit → push (CI validates both headers).
3. `pio run -e deploy -t upload`. **Confirm the SSID has no `-TEST` suffix.**
4. Bench check on the deploy build: power up battery-disconnected, abort during
   the boot window, connect the PSU/battery, use Testing Mode for hold-checks
   (for Ch20, "Hold-check · closed" is the `MAIN_CLOSE_DEG` seal test), then
   "Set resting state".
5. On deck: charge, remove the charge plug, record pack voltage, install the
   deployment plug, **watch for the 45 s LED blink**, then use the 5 min service
   window to verify unit number, battery, and `RUNNING` with next event #1.
6. Oil-fill and pressure-test. The first sample is 32 h 55 m out — that margin is
   deliberate.

## Continuous integration

`.github/workflows/calibration-check.yml` runs on every push/PR:

- **calibration-lint** — `tools/check_calibration.py` checks row shape and count,
  angle ranges, that no sample channel's close angle clamps, that `MAIN_CLOSE_DEG`
  is in range and above every unit's Ch20 open angle, that no two *committed* rows
  are identical, and that the sample schedule is strictly increasing with no two
  events closer than the routine takes to run.
- **build** — `pio run -e calibrate -e deploy -e minideploy`. This also exercises
  the `static_assert`s in both headers, which are the stronger gate: they fail the
  *build*, not just the lint.

Run the lint locally before pushing: `python tools/check_calibration.py`.

## Gotchas / must-knows

- **A flash starts a mission clock.** esptool resets the board at the end of an
  upload, which the firmware sees as a power-on and self-arms. Flash with the
  battery disconnected, or abort during the boot window, or come back to a unit
  hours into a mission.
- **NVS survives a reflash, not an erase.** `pio run -t upload` preserves the unit
  ID and mission state. `pio run -t erase` / `esptool erase_flash` wipes them and
  you re-`setid` and re-verify. Never erase a calibrated board.
- **A completed mission is recoverable.** `done` persists in NVS, so a finished
  board opens a service window on every boot before shutting down — use *Restart
  from t=0*. (Without that window it would latch off instantly and look bricked.)
- **USB back-feeds the logic rail and bypasses the latch.** Bench freely on USB,
  but `latchOff()` and true sleep current only prove out on battery.
- **Testing Mode disarms the mission.** It and a running schedule must not share
  the servo rail. Exiting does *not* re-arm; use *Restart from t=0*.
- **Passive hold must be bench-confirmed.** Valves are driven then released (PWM
  cut) to save power. The deep-sleep budget depends on them not creeping.
- **The flashed binary depends on your local headers, not the last push.** Commit
  and push so CI sees it.

## Before flight

Three numbers in this repo are placeholders that look like decisions:

| Item | Where | Needs |
|---|---|---|
| `MAIN_CLOSE_DEG = 175` | `calibration.h` | Cold/oil seal check at ~8 °C, per unit — does the common valve fully seal, and does it creep? |
| `VBATT_CUTOFF_V = 5.60` | `src/deploy/main.cpp` | 8 °C discharge test with the real load (Rev K Section 15 item 2). Must leave enough charge to finish. |
| Passive hold | all channels | Hold-check per channel, open and closed |

Also open: whether the protocol's Tfinal times are hard requirements (the ESP32's
internal RC oscillator will drift roughly 1% over 18 days — see
[`docs/protocol.md`](docs/protocol.md)), and Lander C's Ch18–19 servo transplant
from Lander D.
