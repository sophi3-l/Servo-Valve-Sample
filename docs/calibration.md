# Calibration

How to find each valve's open/closed angles and where to record them.

## Procedure

1. Flash the calibration build: `pio run -e calibrate -t upload`, then open the
   serial monitor at 115200 (newline line ending).
2. For each channel, type `<ch> <deg>` (e.g. `0 90`). Nudge until the valve is
   **fully open with no strain** — that angle is the channel's `CH_OPEN_DEG`.
3. Then type `open + offset` and confirm the valve is **fully closed but not
   grinding** against its stop. The offset that works must be ≤ 65° for *every*
   channel, because `CLOSE_OFFSET_DEG` is one shared value.
4. `r <ch>` releases a channel (cuts PWM) so you can re-clock a horn by hand or
   watch for creep.
5. Record the open angles below **and** in
   [`include/calibration.h`](../include/calibration.h) → `CH_OPEN_DEG[]`.
   That header is the only copy the flight builds read.

## Recorded angles

Mapping in use: 50 Hz, 25 MHz osc, `map(deg, 0,180, 102,512)` → 102 ≈ 500 µs,
512 ≈ 2500 µs. `CLOSE_OFFSET_DEG` = ___° (≤ 65).

| Ch | Board | Open ° | Close ° (open+offset) | Notes |
|----|-------|--------|-----------------------|-------|
| 0  | 0x40  |        |                       | |
| 1  | 0x40  |        |                       | |
| 2  | 0x40  |        |                       | |
| 3  | 0x40  |        |                       | |
| 4  | 0x40  |        |                       | |
| 5  | 0x40  |        |                       | |
| 6  | 0x40  |        |                       | |
| 7  | 0x40  |        |                       | |
| 8  | 0x40  |        |                       | |
| 9  | 0x40  |        |                       | |
| 10 | 0x40  |        |                       | |
| 11 | 0x40  |        |                       | |
| 12 | 0x40  |        |                       | |
| 13 | 0x40  |        |                       | |
| 14 | 0x40  |        |                       | |
| 15 | 0x40  |        |                       | |
| 16 | 0x41  |        |                       | local 0 |
| 17 | 0x41  |        |                       | local 1 |
| 18 | 0x41  |        |                       | local 2 |
| 19 | 0x41  |        |                       | local 3 |
| 20 | 0x41  |        |                       | local 4 · MAIN intake |

## Passive-hold check (critical)

Using the bench build's Testing Mode → Hold-check: drive a valve open (and
separately closed), cut PWM, and confirm it does **not** creep over a span far
longer than the 18 h field interval, ideally under representative load. The
deep-sleep power budget assumes the gear train self-holds; if a channel creeps,
it's a mechanical fix, not a firmware one. Record pass/fail per channel here.
