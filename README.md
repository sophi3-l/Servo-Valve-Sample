# Karen Valve System — Firmware

ESP32 firmware for an autonomous underwater sequential water sampler. Twenty-one
servo pinch valves (Ch0–19 = sample valves, Ch20 = main intake) are driven by two
PCA9685 PWM boards over I²C and sequenced by an ESP32 DevKitC. The unit powers on
into a WiFi arming interface, then runs the whole mission on deep-sleep, waking
only to actuate one sample per interval.

One firmware image ships to all four landers. Each unit carries a stable ID in NVS
that selects its own servo calibration and names its WiFi AP, so a single binary
covers the fleet.

---

## Repository layout

```
platformio.ini                     three build environments (below)
include/calibration.h              single source of electrical truth (shared by every build)
src/deploy/main.cpp                flight build — arm + deep-sleep mission + web UI
src/calibrate/main.cpp             bench/calibration CLI (serial) — also stamps the unit ID
tools/check_calibration.py         validates the calibration table (run locally or in CI)
.github/workflows/calibration-check.yml   CI: lint the table + build every environment
```

## Build environments

Build/upload from the PlatformIO sidebar (Project Tasks → *env*) or the CLI:

| Env          | Source          | Purpose |
|--------------|-----------------|---------|
| `calibrate`  | `src/calibrate/`| Serial CLI to find each servo's open/close angles, drive one channel at a time, and **`setid`** the board's lander ID into NVS. Doubles as the interactive bench tool. |
| `deploy`     | `src/deploy/`   | The real flight build. Arm over WiFi, then deep-sleep mission. |
| `minideploy` | `src/deploy/`   | **Same source as `deploy`**, compiled with `-DMINI_DEPLOY`: short default timings (full 20-sample run in ~5–6 min) and a `-TEST` SSID tag. For rehearsing the mission quickly. **Not for sealing.** |

```
pio run -e calibrate  -t upload        # bench / calibration
pio run -e deploy     -t upload         # flight firmware
pio run -e minideploy -t upload         # short-timescale rehearsal
pio run -e calibrate -e deploy -e minideploy   # compile all three (verify)
```

`default_envs = deploy`, so a bare `pio run` builds only the flight firmware.

## Shared truth: `calibration.h`

Pin numbers, PCA addresses, the `degToPulse()` mapping, the servo channel layout,
the close offset, the ≤2-servo concurrency guard, and the rail/latch primitives
all live in `include/calibration.h`. **If a value lives there, do not redefine it
in any `main.cpp`** — change it once and rebuild all environments. This is what
guarantees "95°" means the same horn position in every build.

Key constants: `degToPulse()` = `map(deg, 0, 180, 102, 512)` at 50 Hz; sample
count 20 (Ch0–19), main = Ch20; mechanical floor `SERVO_MIN_DEG = 70`; close =
open + `CLOSE_OFFSET_DEG` (65) clamped to 180; `MAX_ACTIVE_SERVOS = 2`.

## Per-lander calibration

Open angles are trimmed to each individual servo, so they differ per physical
lander. The four sets live in `CH_OPEN_DEG_ALL[4][21]`; the unit picks its row at
boot from the ID stored in NVS. `CLOSE_OFFSET_DEG` (65) is shared — Ch13 opens at
115°, so 115 + 65 = 180 is the binding case that sets the 65° ceiling.

- **`CH_OPEN_DEG_ALL[LANDER_COUNT][21]`** — one open-angle row per lander, indexed by `unitID − 1`.
- **`CAL_COMMITTED[LANDER_COUNT]`** — a per-row flag. A row still on placeholder values (flag `false`) will not deploy: the flight build **refuses to arm** it.
- **`chOpenRow(id)` / `calCommitted(id)`** — bounds-checked accessors; `chOpenRow` returns `nullptr` for an unset/invalid ID so callers fail safe.

The ID also names the AP: `LanderController1`…`4` (or `LanderController-UNSET`
before an ID is stamped), so all four are distinguishable on deck instead of
sharing one SSID.

### Deploying a lander

For lander **N**:

1. `pio run -e calibrate -t upload`
2. Over serial: `setid N`, then `id` to confirm it stuck.
3. Calibrate all 21 servos; record each open angle (nudge to visual centre, confirm open + 65 closes without grinding).
4. Paste this unit's 21 angles into row **N** of `CH_OPEN_DEG_ALL`, set `CAL_COMMITTED[N-1] = true`.
5. `python tools/check_calibration.py` → commit → push (CI validates the table).
6. `pio run -e deploy -t upload`. The board reads ID N from NVS, selects row N, and comes up as `LanderControllerN`.
7. Verify on the web UI (status reads "Lander N", ARM enabled), test with the per-channel controls, then arm **on battery** and seal during the ~45 s LED blink.

Do the calibrate→deploy reflash **without** a full flash erase in between — see
gotchas.

### Rehearsal (`minideploy`)

`pio run -e minideploy -t upload` runs the identical mission logic on a short
clock (~5–6 min end to end) with a `-TEST` SSID. Requires a set ID + committed
calibration, same as flight, so it's a true dress rehearsal. To re-run: tap reset
(a fresh power-on brings the web UI back even after "mission complete"), then
Clear/Disarm and arm again. **A `-TEST` unit must never be sealed.**

## Power / arm / mission model (deploy)

- `initPowerPins()` is the first call in `setup()` — SHUTDOWN (GPIO26) low to stay latched on, servo rail (GPIO25) off — before anything else.
- Fresh power-on → **ARM MODE**: WiFi AP + web UI, all valves parked closed, rail off.
- Operator calibrates/checks, then taps **ARM** → ~45 s LED blink (seal window) → park closed → deep-sleep mission begins.
- Each wake actuates one step (warmup opens main; each sample closes main, opens its valve for the hold, recloses, reopens main) then deep-sleeps the inter-sample interval. WiFi never comes up on a mission wake.
- After the last sample: park closed → `latchOff()` cuts the unit's own supply (true zero draw).
- Deep-sleep rail safety relies on the Rev K copper pulldowns **R15** (holds the rail off while GPIO25 floats) and **R13** (keeps the latch alive while GPIO26 floats) — no `gpio_hold` needed. Verify both with a meter.
- Battery guard: below `VBATT_CUTOFF_V` (5.60 V) a wake parks and skips actuation; a reading below 3.00 V is treated as a sense fault and ignored.

## Continuous integration

`.github/workflows/calibration-check.yml` runs on every push/PR:

- **calibration-lint** — `tools/check_calibration.py` asserts each row has 21 values, every open angle is in `[70, 180]`, the row/flag counts match `LANDER_COUNT`, and no two *committed* rows are identical (catches a committed row left as a placeholder copy).
- **build** — `pio run -e calibrate -e deploy -e minideploy`, i.e. every environment compiles.

Run the lint locally before pushing: `python tools/check_calibration.py`.

## Gotchas / must-knows

- **NVS survives a reflash, not a full erase.** A normal `pio run -t upload` leaves the `nvs` partition (and the unit ID) intact. `pio run -t erase` / `esptool erase_flash` wipes it — you'd need to `setid` again. All three envs use the same default partition table, so the ID written under `calibrate` is where `deploy` reads it; keep it that way if you ever set `board_build.partitions`.
- **USB back-feeds the logic rail and bypasses the latch.** Bench and calibrate freely on USB, but the real arm / `latchOff` / deep-sleep behavior only proves out **on battery**. An "armed" board on USB will not actually cut power at mission end.
- **Passive hold must be bench-confirmed.** Valves are driven to position then released (PWM cut) to save power; use the web UI hold-check to confirm they don't creep. The deep-sleep power budget depends on this.
- **The flashed binary depends on your local `calibration.h`, not the last push.** You can flash offline; commit/push the calibration when you're back online so it's in version control and CI-checked.