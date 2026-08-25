# Sampling protocol — as implemented

Source: Zach & Howard's sampling protocol workbook, transcribed 2026-08-18.
Implemented in [`include/schedule.h`](../include/schedule.h).

This document records **what the firmware does and why**, including the two
readings of the workbook that were rejected. If the protocol changes, change
`schedule.h` and this file together.

---

## Clock origin

**t = 0 is power-on** — the moment the deployment plug shorts SubConn pins 3–4,
the ARM one-shot pulses U10 ON, and the ESP32 boots. There is no operator arm
step in the flight path.

> **Corrected 2026-08-25.** This section previously claimed Sheet 1's own origin
> *was* power-on, so `SAMPLE_TIME_S` held raw Sheet 1 times and every event fired
> **5 minutes early**. Sheet 1's clock starts at the *end* of the 5 min deck
> window. The firmware's clock starts at power-on and keeps running through that
> window, so the table now stores **Sheet 1 + 300 s**. First sample: 33h00m00s.
> See `claude/review-2026-08-25-schedule-5min-offset.md`.

## Which sheet is authoritative

The workbook has two sheets whose clocks differ by **exactly 3300 s** on every
row — that is 1 h − 5 min, matching the helper cells at E25/E26. Those are two
separate things added together, which is what made this easy to misread:

- **1 h** — the real offset between a lander event and its sample.
- **− 5 min** — a difference in *clock origin*, not in event timing. It is the
  deck WiFi/service window (`AP_BOOT_WINDOW_MS`).

So:

- **Sheet 1, "Sampling Protocol"** — the authority for sample times. Its origin
  is power-on **+ 5 min**. Add 300 s to put a Sheet 1 row on the firmware clock.
- **Sheet 2, "Lander+Samples"** — the whole-lander view. Every sample row equals
  Sheet 1 minus 3300 s.

On the power-on clock the firmware uses, each sample lands exactly **1 h** after
its lander event, and each Tfinal exactly **288 s** before the lander's next
"turn to ambient" — the ordering requirement under "Timer accuracy" below.

**The lander's own program must share this convention.** Both were built from the
same workbook, so its six "turn to ambient" events should fall on exact hours
after power-on: 93h, 161h, 229h, 297h, 365h, 433h. If they are at 92h55m and so
on instead, the two devices are 5 min apart and the 288 s margin inverts.

Sheet 2 also lists twelve `valve opens to ambient` / `valve closes to inc N`
events. **Those are not ours** — they belong to the lander's own programming, per
the "Lander Programming (Ignore)" annotation on that sheet. They are deliberately
absent from `SAMPLE_TIME_S`.

## The common valve rests OPEN

Ch20 (common/main intake) sits **open** and is closed only for the 124 s inside a
sample event. Nothing needs to remember a long-term common position, so the
scheduler carries no common-valve state.

Two other readings were considered and rejected:

- *"All 32 events are ours, and step 7 restores common to its scheduled state."*
  Physically sensible for an incubation, but it would make steps 1 and 7 no-ops
  at all twenty sample events.
- *"All 32 events are ours, routine taken literally."* This is what a naive
  implementation produces, and it silently ruins the science: common would sit
  open to ambient for the whole of incubations 3–6. For example, opening common
  at the end of T0 inc3 (t = 608,524 s) with no scheduled close until
  t = 824,400 s leaves it open for 60 hours.

## Per-event routine

The protocol fixes **when each command is issued**. It does not fix how long the
servo stays powered afterwards. The firmware keeps those separate, so the release
time can be tuned for valve wear without shifting the sampling schedule by a
single second.

| Command offset (fixed by protocol) | Action | Released at |
|---|---|---|
| 0 s | close common | 0 s + settle |
| 4 s | open sample *n* | 4 s + settle |
| 120 s | close sample *n* | 120 s + settle |
| 124 s | open common | 124 s + settle |

The two windows the science depends on are constant at any settle value:
**sample valve open 116 s**, **common closed 124 s**. Both are `static_assert`ed
and checked by the lint, so tuning the settle cannot silently move them.

### Release time (`STEP_SETTLE_MS`) — the valve-burnout budget

**1000 ms**, set 2026-08-19 (Howard). Basis: a pinch-close timed at ~0.75 s on
the bench, so about 33% headroom.

The distinction that matters: a servo that has *reached* its commanded position
is not stalling — it holds at near-idle. A servo that *cannot* reach position
(mechanical bind, or commanded into a stop) draws stall current — 1.4–2.3 A per
Hiwonder, against the ~0.227 A the two-servo path is certified for — for exactly
this long. So this constant only matters in the fault case, where it is the
entire exposure.

> **The 0.75 s measurement was at room temperature, in air, on a bench supply.**
> Flight is ~8 °C, submerged in oil, on a 6 V pack that sits at or below the
> HPS-2018's 6.0 V minimum rating for most of its discharge. All three make the
> stroke slower, and 33% is not much to give away. A stroke that overruns the
> window leaves the valve under-travelled (no seal) **and** stalled for the whole
> window — both failure modes at once.
>
> **Re-measure in oil at temperature before flight.** Watching the bench supply's
> current readout during a single command is enough: current rises through the
> stroke and drops when the servo arrives. Raising it costs one constant.

Routine length is `124 s + settle` = **125 s** at the current value.
Total servo drive across the mission: 20 events × 4 commands × 1 s = **80 s**.

## Resting state is a precondition, not a startup action

Sample valves rest **closed**, common rests **open**. The mission both starts and
ends in that state — the last commanded action of the final event is "open
common," so no cleanup pass is needed.

The flight build does **not** establish it. It used to, at every power-up, which
cost 21 actuations per boot — and because esptool resets the board when an upload
finishes, every *flash* was a boot. A bench day with ten flashes silently spent
210 valve movements before anyone ran a test.

The operator now sets it deliberately, once, with the calibrate build's `rest`
command as the last step of bench setup (~61 s). What that buys:

> **No code path in the flight build moves a valve outside a scheduled sample
> event, except under operator control in Testing Mode.**

Mass sweeps in the deploy web UI are gated behind Testing Mode for the same
reason. Single-channel controls are not, because entering Testing Mode disarms
the mission — too high a price for nudging one valve during a service window.

Because nothing moves at power-up, the firmware genuinely does not know where any
valve is after a boot, and these open-loop servos give it no way to find out. The
UI shows `--` rather than asserting "CLOSED" for all 21.

## Event table

Twenty events, one per sample valve, so the event index *is* the sample index.

**Which valve event *i* drives is per-unit.** `SAMPLE_ORDER_ALL[unitID-1][i]` in
`calibration.h` maps events to channels; the times below are identical on every
lander, only the mapping differs. Units 1–3 (D, A, B) use the default ascending
order. **Unit 4 (Lander C)** was re-plumbed on 2026-08-25 — its back and front
rows were swapped, so it samples Ch10–19 first and Ch0–9 second, with Ch17 and
Ch18 transposed within the back row:

```
10, 11, 12, 13, 14, 15, 16, 18, 17, 19,  0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

Open angles are *not* reordered with it — `CH_OPEN_DEG_ALL` stays indexed by
physical channel, so each channel keeps its own calibration wherever it falls in
the order. Every row must be a permutation of 0–19; a `static_assert` and the
lint both reject a duplicate, which would otherwise sample one valve twice and
leave another shut for the whole mission.

| # | Ch | t (s) | t | gap | Label |
|---|---|---|---|---|---|
| 1 | 0 | 118,800 | 33h00m00s | 33h00m00s | T0 inc1 |
| 2 | 1 | 334,512 | 92h55m12s | 59h55m12s | Tfinal inc1 |
| 3 | 2 | 363,600 | 101h00m00s | 8h04m48s | T0 inc2 |
| 4 | 3 | 579,312 | 160h55m12s | 59h55m12s | Tfinal inc2 |
| 5 | 4 | 608,400 | 169h00m00s | 8h04m48s | T0 inc3 |
| 6 | 5 | 680,400 | 189h00m00s | 20h00m00s | T1 inc3 |
| 7 | 6 | 752,400 | 209h00m00s | 20h00m00s | T2 inc3 |
| 8 | 7 | 824,112 | 228h55m12s | 19h55m12s | Tfinal inc3 |
| 9 | 8 | 853,200 | 237h00m00s | 8h04m48s | T0 inc4 |
| 10 | 9 | 925,200 | 257h00m00s | 20h00m00s | T1 inc4 |
| 11 | 10 | 997,200 | 277h00m00s | 20h00m00s | T2 inc4 |
| 12 | 11 | 1,068,912 | 296h55m12s | 19h55m12s | Tfinal inc4 |
| 13 | 12 | 1,098,000 | 305h00m00s | 8h04m48s | T0 inc5 |
| 14 | 13 | 1,170,000 | 325h00m00s | 20h00m00s | T1 inc5 |
| 15 | 14 | 1,242,000 | 345h00m00s | 20h00m00s | T2 inc5 |
| 16 | 15 | 1,313,712 | 364h55m12s | 19h55m12s | Tfinal inc5 |
| 17 | 16 | 1,342,800 | 373h00m00s | 8h04m48s | T0 inc6 |
| 18 | 17 | 1,414,800 | 393h00m00s | 20h00m00s | T1 inc6 |
| 19 | 18 | 1,486,800 | 413h00m00s | 20h00m00s | T2 inc6 |
| 20 | 19 | 1,558,512 | 432h55m12s | 19h55m12s | Tfinal inc6 |

Six incubations of 2/2/4/4/4/4 samples = 20, exactly the number of sample valves.
Mission ends 125 s after the last event: **1,558,637 s = 432.95 h = 18.04 days**,
inside the Rev K 21-day planning basis.

Smallest gap is 8 h 04 m 48 s against a 125 s routine — a 230× margin, so events
can never collide.

## Scheduling

The scheduler walks the table by index and sleeps
`next_event_time − elapsed_now`. Because the target is absolute, the 125 s
routine and any service window come out of the *following* sleep rather than
pushing every later event back. Simulated across the full mission this gives
**zero overshoot**; a fixed-delta scheduler would have accumulated well over an
hour of lag.

Elapsed time is an explicit accumulator persisted to NVS and mirrored in RTC
memory before each sleep, rather than a system-time API, so it is deterministic
across deep sleep, watchdog resets, and panics. A **power-on** mid-mission cannot
know how long the outage lasted: the unit resumes at the correct event — never
repeating or skipping one — but the absolute schedule slips by the outage, and
the `[CLOCK]` log line is the only record. Read the logs at recovery.

### MINI_DEPLOY compression

Rehearsals compress the **waits only**; the 125 s routine runs at full flight
timing, because the servo behaviour is the untested part (8 °C, in oil, on a 6 V
pack). Gaps are scaled proportionally (`MINI_DIV = 2000`, floor 10 s) rather than
flattened, so a rehearsal still walks the real irregular table.

Consequence: a rehearsal cannot be shorter than 20 × 125 s = 42 min, and lands at
**~55 min**. This is deliberate.

## Timer accuracy

The Tfinal times are specified to 0.08 h (~5 min). That figure is not a precision
requirement — each Tfinal sits exactly 288 s before the lander's following
"turn to ambient" event, and the routine takes 125 s, so the `.92` spacing exists
to fit the last sample of an incubation in before the switch. It is an **ordering**
requirement, and ordering survives clock drift because every event comes off the
same clock.

What drift does cost: incubation *durations* stretch or compress by the same
percentage (1% of a 62 h incubation is ~37 min), the mission ends somewhat before
or after 18 days, and sample times stop agreeing with the other instruments on
the lander.

The ESP32's deep-sleep timer runs off RTC_SLOW_CLK — on a stock DevKitC, the
internal ~150 kHz RC oscillator. Roughly 1% over 432 h is ~4.3 h.

**Open question for Zach & Howard: is ±1% on a 62 h incubation acceptable?**
Options if not, in increasing cost:

1. **Log actual elapsed time at every event** (cheap, worth doing regardless) — if
   a sample lands at 92.4 h instead of 91.92 h you *know*, and the data is still
   good.
2. **Lean on ESP-IDF's automatic RC calibration.** Its weakness is temperature,
   and the seafloor is a dead-stable 8 °C — close to its best case. A long bench
   soak at temperature would give the real number instead of the worst case.
3. **Add a 32.768 kHz crystal** (~20 ppm, ~30 s over the whole mission). Hardware
   change; DevKitC-32UE boards generally do not populate it and it consumes
   GPIO32/33, which are currently free.

## Other open items

- **Lander C, Ch18–19.** Servos are being transplanted from Lander D. Until then C
  runs all 20 events but two actuate nothing — the firmware is open-loop and
  cannot tell. Re-verify those two channels' angles after the swap: the angle
  carries the servo's trim but not the valve body's.
- **Lander D** is short two servos afterwards and cannot run this protocol; it is
  getting a separate program. Decide whether that shares these headers.
