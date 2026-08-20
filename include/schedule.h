#pragma once
#include <Arduino.h>
#include "calibration.h"

// =============================================================================
//  schedule.h — mission timing. Zach & Howard sampling protocol, 2026-08-18.
//  calibration.h owns electrical truth; this owns WHEN. Keep them separate.
//
//  t = 0 is POWER-ON (deployment plug latches U10, ESP32 boots). No operator
//  arm step. Matches the protocol sheet's "seconds since powered on".
//
//  SHEET 1 "Sampling Protocol" is the authority. Sheet 2 "Lander+Samples" is
//  the whole-lander view on a clock 3300 s (1 h - 5 min) earlier; its 12
//  "valve opens to ambient / closes to inc N" rows belong to the LANDER, not
//  this controller (see its "Lander Programming (Ignore)" note). Common rests
//  OPEN and closes only inside a sample event. Do not add those 12 events.
//
//  PER-EVENT ROUTINE. The protocol fixes WHEN commands are issued, not how long
//  the servo stays powered. Keeping them separate means the release time can be
//  tuned for valve wear without shifting the schedule.
//
//      0 s  close common        released at cmd + STEP_SETTLE_MS
//      4 s  open sample n
//    120 s  close sample n
//    124 s  open common
//
//  Sample valve open 4->120 = 116 s; common closed 0->124 = 124 s. Both hold at
//  any settle value, and both are static_asserted below.
// =============================================================================

// ---- Command offsets within an event (protocol — do not change) -------------
constexpr uint32_t CMD_CLOSE_COMMON_MS = 0;
constexpr uint32_t CMD_OPEN_SAMPLE_MS  = 4000;
constexpr uint32_t CMD_CLOSE_SAMPLE_MS = 120000;
constexpr uint32_t CMD_OPEN_COMMON_MS  = 124000;

// ---- Release time: the valve-burnout budget --------------------------------
//  A servo that REACHED its position holds at near-idle. One that CANNOT reach
//  it (bind, or commanded into a stop) draws stall current — 1.4-2.3 A vs the
//  ~0.227 A the 2-servo path is certified for — for exactly this long. So this
//  only matters in the fault case, where it is the whole exposure.
//
//  1000 ms (Howard, 2026-08-19), from a ~0.75 s bench pinch-close: ~33% margin.
//  THAT WAS ROOM TEMPERATURE, IN AIR, ON A BENCH SUPPLY. Flight is ~8 degC, in
//  oil, on a 6 V pack at or below the HPS-2018's 6.0 V rating — all slower. A
//  stroke that overruns leaves the valve under-travelled AND stalled for the
//  whole window. RE-MEASURE IN OIL AT TEMPERATURE BEFORE FLIGHT: watch the
//  supply current during one command, it drops when the servo arrives.
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

// ---- Heartbeat -------------------------------------------------------------
//  Gaps run to 59h55m and nothing happens for the first 32h55m. These wakes log
//  battery + elapsed and sleep again, no actuation: ~1.2 mAh across the mission
//  against a ~8,700 mAh sleep budget.
#ifdef MINI_DEPLOY
constexpr uint32_t HEARTBEAT_S = 60;         // exercise the path in rehearsal
#else
constexpr uint32_t HEARTBEAT_S = 6UL * 3600UL;
#endif

// ---- MINI_DEPLOY compression -----------------------------------------------
//  Compress the WAITS only; the routine runs at full flight timing, because the
//  servo behaviour is the untested part (8 degC, oil, 6 V pack). Floor: 20 x
//  125 s = 42 min; lands ~55 min. Gaps scale proportionally rather than
//  flattening, so a rehearsal still walks the real irregular table.
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
