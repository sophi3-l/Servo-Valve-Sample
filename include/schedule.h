#pragma once
#include <Arduino.h>
#include "calibration.h"

// =============================================================================
//  schedule.h — SAMPLING PROTOCOL AUTHORITY for the Karen valve system
//  Zach & Howard sampling protocol, transcribed 2026-08-18
//
//  calibration.h owns ELECTRICAL truth (pins, angles, rail, guard).
//  This file owns MISSION TIMING. Keep them separate: an angle change and a
//  schedule change are different reviews by different people.
//
//  ---------------------------------------------------------------------------
//  CLOCK ORIGIN
//  ---------------------------------------------------------------------------
//  t = 0 is POWER-ON — the moment the deployment plug shorts SubConn pins 3-4,
//  the ARM one-shot pulses U10 ON, and the ESP32 boots. There is no operator
//  "arm" step in the flight path; the unit self-starts. This matches the
//  protocol sheet's own column header ("Seconds since powered on") and the
//  Rev K deployment procedure (install plug -> confirm -> it runs).
//
//  ---------------------------------------------------------------------------
//  WHICH SHEET THIS CAME FROM
//  ---------------------------------------------------------------------------
//  The protocol workbook has two sheets whose clocks differ by exactly 3300 s
//  (= 1 h - 5 min). Sheet 1 "Sampling Protocol" is the authority for THIS
//  controller: its origin is power-on, and the 5 min is the deck WiFi/service
//  window. Sheet 2 "Lander+Samples" is the whole-lander view on a different
//  origin; every one of its sample rows equals Sheet 1 minus 3300 s.
//
//  Sheet 2 also lists "valve opens to ambient" / "valve closes to inc N"
//  events. THOSE ARE NOT OURS — they belong to the lander's own programming
//  (see the "Lander Programming (Ignore)" annotation on that sheet). Our
//  common valve (Ch20) rests OPEN and is closed only for the duration of a
//  sample event. Do not add those 12 events to the table below.
//
//  ---------------------------------------------------------------------------
//  PER-EVENT ROUTINE
//  ---------------------------------------------------------------------------
//  The protocol fixes WHEN each command is issued. It does NOT fix how long the
//  servo stays powered afterwards — that is a separate, independently tunable
//  quantity. Keeping the two apart means the release time can be changed for
//  valve-wear reasons WITHOUT shifting the sampling schedule by a single second.
//
//    COMMAND OFFSET (fixed by protocol)    RELEASE (= command + STEP_SETTLE_MS)
//        0 s   close common                    0 s + settle
//        4 s   open sample n                   4 s + settle
//      120 s   close sample n                120 s + settle
//      124 s   open common                   124 s + settle
//
//  The sample valve is open from 4 s to 120 s = 116 s, and common is closed
//  from 0 s to 124 s, whatever the settle is. Those are the numbers the science
//  depends on, and they are structurally protected from the wear tuning.
// =============================================================================

// ---- Command offsets within an event (protocol — do not change) -------------
constexpr uint32_t CMD_CLOSE_COMMON_MS = 0;
constexpr uint32_t CMD_OPEN_SAMPLE_MS  = 4000;
constexpr uint32_t CMD_CLOSE_SAMPLE_MS = 120000;
constexpr uint32_t CMD_OPEN_COMMON_MS  = 124000;

// ---- How long a servo stays powered after a command -------------------------
//  THIS IS THE VALVE-BURNOUT BUDGET, and it is the only thing that controls it.
//
//  A servo that has REACHED its commanded position is not stalling — it holds
//  at near-idle. A servo that CANNOT reach position (mechanical bind, or
//  commanded into a stop) draws stall current — 1.4-2.3 A per Hiwonder, versus
//  the ~0.227 A the 2-servo path is certified for — for exactly this long.
//  Shorter = less damage in a fault. Longer = more certainty the valve seated.
//
//  1000 ms, set 2026-08-19 (Howard). Basis: a pinch-close was timed at ~0.75 s
//  on the bench, giving ~33% headroom.
//
//  *** THAT MEASUREMENT WAS AT ROOM TEMPERATURE, IN AIR, ON A BENCH SUPPLY. ***
//  Flight is ~8 degC, submerged in oil, on a 6 V pack that sits at or below the
//  HPS-2018's 6.0 V minimum rating for most of its discharge. All three make
//  the stroke SLOWER, and 33% is not a lot of margin to give away. A stroke
//  that overruns this window leaves the valve under-travelled (no seal) AND
//  stalled for the whole window — both failure modes at once.
//
//  RE-MEASURE IN OIL AT TEMPERATURE BEFORE FLIGHT. Watching the bench supply's
//  current readout during a single command is enough: current rises through the
//  stroke and drops when the servo arrives. Raising this costs one constant and
//  shifts no schedule.
constexpr uint16_t STEP_SETTLE_MS = 1000;

// Breather between servos in a SEQUENTIAL SWEEP (the calibrate build's `rest`,
// the deploy build's manual all-open / all-close). NOT part of the protocol —
// it exists to keep consecutive inrush events off the ~600 uF C1 bank.
constexpr uint16_t STEP_GAP_MS = 2000;

// ---- Derived ----------------------------------------------------------------
constexpr uint32_t SAMPLE_OPEN_MS  = CMD_CLOSE_SAMPLE_MS - CMD_OPEN_SAMPLE_MS;  // 116 s
constexpr uint32_t ROUTINE_MS      = CMD_OPEN_COMMON_MS + STEP_SETTLE_MS;       // last release
constexpr uint32_t ROUTINE_S       = ROUTINE_MS / 1000;

// Unpowered dwell between "release sample" and "close sample".
constexpr uint32_t SAMPLE_DWELL_MS = SAMPLE_OPEN_MS - STEP_SETTLE_MS;

// ---- Absolute sample schedule (Sheet 1, seconds since power-on) -------------
//  Event i drives SERVO_CHANNELS[i]; there are exactly SAMPLE_SERVO_COUNT of
//  them, so the sample index IS the event index. No action column is needed —
//  every event in this mission is a sample.
//
//  #   Ch   t (s)        t          gap        protocol label
constexpr uint8_t  EVENT_COUNT = SAMPLE_SERVO_COUNT;      // 20
constexpr uint32_t SAMPLE_TIME_S[EVENT_COUNT] = {
    118500UL,   //  1  Ch0    32h55m   32h55m   T0    inc1
    334212UL,   //  2  Ch1    92h50m   59h55m   Tfin  inc1
    363300UL,   //  3  Ch2   100h55m    8h04m   T0    inc2
    579012UL,   //  4  Ch3   160h50m   59h55m   Tfin  inc2
    608100UL,   //  5  Ch4   168h55m    8h04m   T0    inc3
    680100UL,   //  6  Ch5   188h55m   20h00m   T1    inc3
    752100UL,   //  7  Ch6   208h55m   20h00m   T2    inc3
    823812UL,   //  8  Ch7   228h50m   19h55m   Tfin  inc3
    852900UL,   //  9  Ch8   236h55m    8h04m   T0    inc4
    924900UL,   // 10  Ch9   256h55m   20h00m   T1    inc4
    996900UL,   // 11  Ch10  276h55m   20h00m   T2    inc4
   1068612UL,   // 12  Ch11  296h50m   19h55m   Tfin  inc4
   1097700UL,   // 13  Ch12  304h55m    8h04m   T0    inc5
   1169700UL,   // 14  Ch13  324h55m   20h00m   T1    inc5
   1241700UL,   // 15  Ch14  344h55m   20h00m   T2    inc5
   1313412UL,   // 16  Ch15  364h50m   19h55m   Tfin  inc5
   1342500UL,   // 17  Ch16  372h55m    8h04m   T0    inc6
   1414500UL,   // 18  Ch17  392h55m   20h00m   T1    inc6
   1486500UL,   // 19  Ch18  412h55m   20h00m   T2    inc6
   1558212UL,   // 20  Ch19  432h50m   19h55m   Tfin  inc6
};
//  Mission ends 126 s after the last event: 1,558,338 s = 432.87 h = 18.04 d.

// ---- Heartbeat --------------------------------------------------------------
//  The schedule has gaps up to 59h55m. Twenty wakes across eighteen days is
//  very few battery readings, and nothing at all for the first 32h55m, so we
//  insert no-actuation wakes that log battery + elapsed and go straight back
//  to sleep. Cost is ~1 s of ESP32 active current each: ~1.2 mAh across the
//  whole mission against a ~8,700 mAh sleep budget. Effectively free.
#ifdef MINI_DEPLOY
constexpr uint32_t HEARTBEAT_S = 60;         // exercise the path in rehearsal
#else
constexpr uint32_t HEARTBEAT_S = 6UL * 3600UL;
#endif

// ---- MINI_DEPLOY compression ------------------------------------------------
//  Rehearsal rule (decided 2026-08-18): compress the WAITS ONLY. The 126 s
//  routine runs at full flight timing — 2 s settles, 116 s sample-open — so a
//  rehearsal exercises the real servo behaviour, which is the part that is
//  actually untested at 8 degC in oil on a 6 V pack.
//
//  Consequence: a full 20-sample rehearsal cannot be shorter than
//  20 x 126 s = 42 min. With the divisor below it lands around 55 min.
//  That is a lunch break, not a coffee break. This is deliberate.
//
//  Gaps are scaled proportionally rather than flattened, so the rehearsal
//  still walks the real irregular table (8h / 20h / 60h gaps stay in
//  proportion) instead of a uniform stride that would not exercise it.
constexpr uint32_t MINI_DIV          = 2000;   // divide the SLEEP portion only
constexpr uint32_t MINI_MIN_SLEEP_S  = 10;     // floor, so events never collide

// Absolute time of event i on this build's clock.
inline uint32_t eventTimeS(uint8_t i) {
#ifndef MINI_DEPLOY
  return SAMPLE_TIME_S[i];
#else
  uint32_t first = SAMPLE_TIME_S[0] / MINI_DIV;
  if (first < MINI_MIN_SLEEP_S) first = MINI_MIN_SLEEP_S;
  uint32_t t = first;
  for (uint8_t k = 1; k <= i; k++) {
    uint32_t realGap = SAMPLE_TIME_S[k] - SAMPLE_TIME_S[k - 1];
    uint32_t sleepS  = (realGap > ROUTINE_S) ? (realGap - ROUTINE_S) / MINI_DIV : 0;
    if (sleepS < MINI_MIN_SLEEP_S) sleepS = MINI_MIN_SLEEP_S;
    t += ROUTINE_S + sleepS;
  }
  return t;
#endif
}

// Wall-clock length of the whole mission on this build, for logging.
inline uint32_t missionEndS() { return eventTimeS(EVENT_COUNT - 1) + ROUTINE_S; }

// ---- Compile-time checks ----------------------------------------------------
//  The real table is validated here; the MINI table is derived from it by a
//  monotonic transform, so if the real one is sane the mini one is too.
constexpr bool _schedIncreasing(uint8_t i = 1) {
  return (i >= EVENT_COUNT)
           ? true
           : ((SAMPLE_TIME_S[i] > SAMPLE_TIME_S[i - 1]) && _schedIncreasing(i + 1));
}
// No event may start before the previous event's routine has finished. The
// real minimum gap is 8h04m against a 126 s routine, so this has enormous
// margin — it exists to catch a mistyped timestamp, not a design error.
constexpr bool _schedNoOverlap(uint8_t i = 1) {
  return (i >= EVENT_COUNT)
           ? true
           : ((SAMPLE_TIME_S[i] - SAMPLE_TIME_S[i - 1] > ROUTINE_S) && _schedNoOverlap(i + 1));
}

static_assert(_schedIncreasing(),
              "schedule.h: SAMPLE_TIME_S must be strictly increasing.");
static_assert(_schedNoOverlap(),
              "schedule.h: two events are closer together than the 126 s sample routine.");
static_assert(EVENT_COUNT == SAMPLE_SERVO_COUNT,
              "schedule.h: one event per sample valve — event index IS the sample index.");
static_assert(SAMPLE_OPEN_MS > STEP_SETTLE_MS,
              "schedule.h: sample-open must exceed the settle, or the dwell underflows.");
// Command offsets must be strictly increasing.
static_assert(CMD_CLOSE_COMMON_MS < CMD_OPEN_SAMPLE_MS &&
              CMD_OPEN_SAMPLE_MS  < CMD_CLOSE_SAMPLE_MS &&
              CMD_CLOSE_SAMPLE_MS < CMD_OPEN_COMMON_MS,
              "schedule.h: per-event command offsets are out of order.");
// The settle must fit inside every inter-command interval, or one command's
// powered window would still be open when the next command is issued — which
// would put two servos under power and trip the <=2 guard in odd ways.
static_assert(STEP_SETTLE_MS < (CMD_OPEN_SAMPLE_MS  - CMD_CLOSE_COMMON_MS) &&
              STEP_SETTLE_MS < (CMD_CLOSE_SAMPLE_MS - CMD_OPEN_SAMPLE_MS)  &&
              STEP_SETTLE_MS < (CMD_OPEN_COMMON_MS  - CMD_CLOSE_SAMPLE_MS),
              "schedule.h: STEP_SETTLE_MS is longer than the gap to the next command.");
// The science-critical durations must not move when the settle is tuned.
static_assert(SAMPLE_OPEN_MS == 116000UL,
              "schedule.h: sample-open duration is no longer 116 s.");
static_assert(CMD_OPEN_COMMON_MS - CMD_CLOSE_COMMON_MS == 124000UL,
              "schedule.h: common-closed duration is no longer 124 s.");
static_assert(ROUTINE_S * 1000UL == ROUTINE_MS,
              "schedule.h: ROUTINE_MS must be a whole number of seconds.");
