# Calibration

How to find each valve's open/closed angles, stamp a unit ID, and where to record
the result.

The angles live in [`include/calibration.h`](../include/calibration.h) →
`CH_OPEN_DEG_ALL[unitID − 1]`. That header is the only copy the flight builds
read; the tables below are the human record of how those numbers were obtained.

## Unit numbering

The team calls the units **A/B/C/D**; the firmware only knows **1–4**. `setid`,
NVS, and the SSID all use numbers. Nothing enforces the mapping automatically —
if a unit is relabelled, update this table, the comment in `calibration.h`, and
the README together.

| Bench name | Unit ID | SSID | Calibration |
|---|---|---|---|
| Lander D | 1 | `LanderController1` | committed (original set) |
| Lander A | 2 | `LanderController2` | committed 2026-08-18 |
| Lander B | 3 | `LanderController3` | committed 2026-08-18 |
| Lander C | 4 | `LanderController4` | committed 2026-08-18; **Ch18–19 pending** servo transplant from D |

## Procedure

1. `pio run -e calibrate -t upload`, then open the serial monitor at **115200**
   (newline line ending).
2. **Stamp the unit ID:** `setid <n>` — the *number*, not the letter. Confirm with
   `id`. This persists in NVS across reflashes; only a flash **erase** loses it.
3. For each channel, type `<ch> <deg>` (e.g. `0 95`). Nudge until the valve is
   **fully open with no strain** — that angle is the channel's open angle. The
   CLI refuses anything below `SERVO_MIN_DEG` (70°); the valve linkages bind
   below that, and commanding into the stop is what stalled Ch20/Ch14 on the bench.
4. Confirm the closed position:
   - **Sample valves (Ch0–19):** open + `CLOSE_OFFSET_DEG` (65°) must be fully
     closed without grinding. 65 is shared by every channel and is already at its
     ceiling — the highest committed open angle is 115°, and 115 + 65 = 180.
   - **Common valve (Ch20):** closes to `open + MAIN_CLOSE_OFFSET_DEG` (70°),
     capped at 175° — a larger offset than the sample valves, not open + 65.
     See the note below.
5. `r <ch>` releases a channel (cuts PWM) so you can re-clock a horn by hand or
   watch for creep. `off` releases everything and drops the rail.
6. `table` prints this unit's full open/close map — the same angles the flight
   build will actually drive, including Ch20. Use it as the final check.
7. Paste the 21 open angles into row `n − 1` of `CH_OPEN_DEG_ALL`, set
   `CAL_COMMITTED[n − 1] = true`, run `python tools/check_calibration.py`, commit.
8. **`rest`** — the last thing you run here. It closes Ch0–19 and opens Ch20,
   one servo at a time, which is the state the mission both starts and ends in.
   The flight build will not move a valve until its first sample, so this is the
   only thing that establishes it. Confirm visually: these servos have no
   position feedback, so your eyes are the only verification that exists.

A unit whose row is still a placeholder (`CAL_COMMITTED` false) **refuses to
arm**. Bad angles now also fail the *build* via `static_assert`, not just the lint.

### Other calibrate-build commands

| Command | Effect |
|---|---|
| `rail` / `off` | Rail ON with zero PWM (Q2 gate-table HIGH row) / release all + rail OFF |
| `cycle [ms]` | Deliberate rail power-cycle for scope work |
| `active` | List driven channels and the ≤2 count |
| `v` | Battery voltage via the GPIO34 divider |
| `table` | This unit's open/close map |
| `id` / `setid <n>` | Show / stamp the lander ID |
| **`rest`** | **Set the mission resting state: Ch0–19 closed, Ch20 open (~61 s). Run this LAST, before flashing the mission build.** Refuses without a stamped unit ID, since driving with another unit's angles risks over-travelling a valve. |

## The common valve is different

Ch20 uses its **own, larger offset** — `open + MAIN_CLOSE_OFFSET_DEG` (70°)
capped at `MAIN_CLOSE_MAX_DEG` (175°) — because the common valve needs more
travel to seal than a sample valve does.

It briefly used a fixed 175° instead. That was wrong in a subtle way: a pinch
valve closes as a function of how far the horn *travels* from its calibrated open
position, and the open angle already absorbs each servo's horn offset. A fixed
angle therefore gave inconsistent travel across the fleet (D and B 85°, C 75°,
A 70°). The offset holds travel constant at 70° everywhere and lets the cushion
vary; the cap is what protects the stop.

**70 is the largest offset available.** The fleet ceiling is set by the highest
Ch20 open angle — Lander A at 105° — and at offset 75 A would land exactly on
180°, the mechanical stop. If a unit turns out to need more than 70° of travel,
its Ch20 *open* angle has to come down first; that is what frees up headroom.

It previously used a hardcoded **180°** — the absolute end of the commanded pulse
range (`degToPulse(180)` = 512 counts = 2500 µs), i.e. driven into the servo's
mechanical stop on every close, twice per sample event plus every park. The low
end had a guard (`SERVO_MIN_DEG`); the high end had none.

**`MAIN_CLOSE_OFFSET_DEG = 70` is a starting value, not a bench-confirmed one.**
Verify per unit during the Rev K Section 14 cold/oil test at ~8 °C: run `table` to
see the angle this unit will actually drive on Ch20, drive it, cut PWM, and
confirm it is *fully sealed* and does not creep. If the four units disagree,
convert it to a per-lander row like `CH_OPEN_DEG_ALL`.

Two things make this cushion matter more than it looks:

- The pack is 6 V nominal and the HPS-2018 is rated 6.0–8.4 V, so the servos run
  at or below their minimum rating for most of the discharge.
- They are submerged in oil at ~8 °C, which the 0.20 s/60° datasheet figure (7.4 V,
  no load, in air) does not account for at all.

A servo that cannot finish its stroke inside the 2 s settle ends up both
under-travelled (no seal) *and* stalled for the whole window, at 1.4–2.3 A against
the ~0.227 A the two-servo path is certified for.

## Recorded angles

Mapping: 50 Hz, 25 MHz oscillator, `map(deg, 0,180, 102,512)` → 102 ≈ 500 µs,
512 ≈ 2500 µs. `CLOSE_OFFSET_DEG` = 65 (sample valves).
`MAIN_CLOSE_OFFSET_DEG` = 70 capped at 175 (Ch20 common).

Open angles as committed in `calibration.h`:

| Ch | Board | D (1) | A (2) | B (3) | C (4) | Notes |
|----|-------|-------|-------|-------|-------|-------|
| 0  | 0x40 | 100 | 95 | 95 | 95 | |
| 1  | 0x40 | 110 | 90 | 100 | 100 | |
| 2  | 0x40 | 100 | 90 | 100 | 105 | |
| 3  | 0x40 | 95 | 95 | 105 | 100 | |
| 4  | 0x40 | 95 | 100 | 100 | 95 | |
| 5  | 0x40 | 105 | 90 | 100 | 100 | |
| 6  | 0x40 | 95 | 85 | 95 | 95 | |
| 7  | 0x40 | 110 | 95 | 100 | 100 | |
| 8  | 0x40 | 95 | 100 | 110 | 95 | |
| 9  | 0x40 | 105 | 100 | 95 | 90 | |
| 10 | 0x40 | 100 | 100 | 110 | **115** | C at the 65° offset ceiling |
| 11 | 0x40 | 105 | 90 | 90 | 105 | |
| 12 | 0x40 | 95 | 100 | 95 | 105 | |
| 13 | 0x40 | **115** | 95 | 105 | 105 | D at the 65° offset ceiling |
| 14 | 0x40 | 110 | 95 | 95 | 100 | |
| 15 | 0x40 | 95 | 105 | 100 | 100 | |
| 16 | 0x41 | 100 | 105 | 100 | 100 | local 0 |
| 17 | 0x41 | 95 | 100 | 100 | 105 | local 1 |
| 18 | 0x41 | 95 | 95 | 110 | *95* | local 2 · **C placeholder** |
| 19 | 0x41 | 90 | 95 | 100 | *90* | local 3 · **C placeholder** |
| 20 | 0x41 | 90 | 105 | 90 | 100 | local 4 · **COMMON** — closes to open+70 (160/175/160/170) |

Lander C's Ch18–19 values are copies of Lander D's and are **not real bench
angles** — those ports are unpopulated pending the servo transplant. They exist
only so the pulse tables never operate on uninitialised values. C will run all 20
sample events regardless; the firmware is open-loop and cannot tell that two of
them actuate nothing.

After the transplant, re-verify those two channels on the bench. The angle carries
the servo's trim and horn position but **not** the valve body's — you are keeping
two of three variables and changing the third.

## Passive-hold check (critical)

Using the deploy build's **Testing Mode → Hold-check**: drive a valve open (and
separately closed), cut PWM, and confirm it does **not** creep over a span far
longer than the field interval, under representative load. The deep-sleep power
budget assumes the gear train self-holds; if a channel creeps, it is a mechanical
fix, not a firmware one.

For Ch20, "Hold-check · closed" doubles as the common-valve seal test.

Record pass/fail per channel per unit:

| Ch | D (1) | A (2) | B (3) | C (4) |
|----|-------|-------|-------|-------|
| 0–19 | | | | |
| 20 (common) | | | | |
