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

This matches both the workbook's own column header ("Seconds since powered on")
and the Rev K deployment procedure (install plug → confirm → it runs).

## Which sheet is authoritative

The workbook has two sheets whose clocks differ by **exactly 3300 s** on every
row — that is 1 h − 5 min, matching the helper cells at E25/E26.

- **Sheet 1, "Sampling Protocol"** — origin is power-on. **This is the authority
  for this controller.** The 5 min is the deck WiFi/service window.
- **Sheet 2, "Lander+Samples"** — the whole-lander view on a different origin.
  Every sample row equals Sheet 1 minus 3300 s.

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
  at the end of T0 inc3 (t = 608,224 s) with no scheduled close until
  t = 824,100 s leaves it open for 60 hours.

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
Event *i* drives `SERVO_CHANNELS[i]`.

| # | Ch | t (s) | t | gap | Label |
|---|---|---|---|---|---|
| 1 | 0 | 118,500 | 32h55m | 32h55m | T0 inc1 |
| 2 | 1 | 334,212 | 92h50m | 59h55m | Tfinal inc1 |
| 3 | 2 | 363,300 | 100h55m | 8h04m | T0 inc2 |
| 4 | 3 | 579,012 | 160h50m | 59h55m | Tfinal inc2 |
| 5 | 4 | 608,100 | 168h55m | 8h04m | T0 inc3 |
| 6 | 5 | 680,100 | 188h55m | 20h00m | T1 inc3 |
| 7 | 6 | 752,100 | 208h55m | 20h00m | T2 inc3 |
| 8 | 7 | 823,812 | 228h50m | 19h55m | Tfinal inc3 |
| 9 | 8 | 852,900 | 236h55m | 8h04m | T0 inc4 |
| 10 | 9 | 924,900 | 256h55m | 20h00m | T1 inc4 |
| 11 | 10 | 996,900 | 276h55m | 20h00m | T2 inc4 |
| 12 | 11 | 1,068,612 | 296h50m | 19h55m | Tfinal inc4 |
| 13 | 12 | 1,097,700 | 304h55m | 8h04m | T0 inc5 |
| 14 | 13 | 1,169,700 | 324h55m | 20h00m | T1 inc5 |
| 15 | 14 | 1,241,700 | 344h55m | 20h00m | T2 inc5 |
| 16 | 15 | 1,313,412 | 364h50m | 19h55m | Tfinal inc5 |
| 17 | 16 | 1,342,500 | 372h55m | 8h04m | T0 inc6 |
| 18 | 17 | 1,414,500 | 392h55m | 20h00m | T1 inc6 |
| 19 | 18 | 1,486,500 | 412h55m | 20h00m | T2 inc6 |
| 20 | 19 | 1,558,212 | 432h50m | 19h55m | Tfinal inc6 |

Six incubations of 2/2/4/4/4/4 samples = 20, exactly the number of sample valves.
Mission ends 126 s after the last event: **1,558,338 s = 432.87 h = 18.04 days**,
inside the Rev K 21-day planning basis.

Smallest gap is 8 h 04 m against a 126 s routine — a 230× margin, so events can
never collide.

## Scheduling

The scheduler walks the table by index and sleeps
`next_event_time − elapsed_now`. Because the target is absolute, the 126 s
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

Rehearsals compress the **waits only**; the 126 s routine runs at full flight
timing, because the servo behaviour is the untested part (8 °C, in oil, on a 6 V
pack). Gaps are scaled proportionally (`MINI_DIV = 2000`, floor 10 s) rather than
flattened, so a rehearsal still walks the real irregular table.

Consequence: a rehearsal cannot be shorter than 20 × 126 s = 42 min, and lands at
**~55 min**. This is deliberate.

## Timer accuracy

The Tfinal times are specified to 0.08 h (~5 min). That figure is not a precision
requirement — each Tfinal sits exactly 288 s before the lander's following
"turn to ambient" event, and the routine takes 126 s, so the `.92` spacing exists
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
