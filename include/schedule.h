#pragma once
#include <Arduino.h>
#include "calibration.h"

// =============================================================================
//  schedule.h — mission timing. Zach & Howard sampling protocol, 2026-08-18.
//  calibration.h owns electrical truth; this owns WHEN. Keep them separate.
//
//  t = 0 is POWER-ON (deployment plug latches U10, ESP32 boots). No operator
//  arm step.
//
//  *** THE 5 MIN DECK WINDOW IS ALREADY IN THESE NUMBERS. DO NOT SUBTRACT IT. ***
//  Sheet 1's clock starts 5 min AFTER power-on - that 5 min is the deck
//  service window (AP_BOOT_WINDOW_MS). This firmware's clock starts AT
//  power-on and keeps running THROUGH that window (runServiceWindow() never
//  touches the elapsed accumulator), so every entry below is a Sheet 1 time
//  + 300 s. First sample: 33h00m00s after power-on.
//  Corrected 2026-08-25: the table previously held raw Sheet 1 times, so every
//  event fired 5 min early. See claude/review-2026-08-25-schedule-5min-offset.md.
//
//  SHEET 1 "Sampling Protocol" is the authority for sample times. Sheet 2
//  "Lander+Samples" is the whole-lander view, and a Sheet 2 row = Sheet 1
//  - 3300 s. That 3300 is two things at once: 1 h of real offset between a
//  lander event and its sample, minus the 5 min of clock-origin difference.
//  On the power-on clock used below, each sample therefore sits exactly 1 h
//  after its lander event, and each Tfin exactly 288 s before the lander's
//  next "turn to ambient". Sheet 2's 12 "valve opens to ambient / closes to
//  inc N" rows belong to the LANDER, not this controller (see its "Lander
//  Programming (Ignore)" note) - do not add them. Common rests OPEN and
//  closes only inside a sample event.
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
//        t (s)      #   Ch          t           gap       label  increment
constexpr uint8_t  EVENT_COUNT = SAMPLE_SERVO_COUNT;      // 20
constexpr uint32_t SAMPLE_TIME_S[EVENT_COUNT] = {
     118800UL,   //  1  Ch0      33h00m00s   33h00m00s  T0    inc1
     334512UL,   //  2  Ch1      92h55m12s   59h55m12s  Tfin  inc1
     363600UL,   //  3  Ch2     101h00m00s    8h04m48s  T0    inc2
     579312UL,   //  4  Ch3     160h55m12s   59h55m12s  Tfin  inc2
     608400UL,   //  5  Ch4     169h00m00s    8h04m48s  T0    inc3
     680400UL,   //  6  Ch5     189h00m00s   20h00m00s  T1    inc3
     752400UL,   //  7  Ch6     209h00m00s   20h00m00s  T2    inc3
     824112UL,   //  8  Ch7     228h55m12s   19h55m12s  Tfin  inc3
     853200UL,   //  9  Ch8     237h00m00s    8h04m48s  T0    inc4
     925200UL,   // 10  Ch9     257h00m00s   20h00m00s  T1    inc4
     997200UL,   // 11  Ch10    277h00m00s   20h00m00s  T2    inc4
    1068912UL,   // 12  Ch11    296h55m12s   19h55m12s  Tfin  inc4
    1098000UL,   // 13  Ch12    305h00m00s    8h04m48s  T0    inc5
    1170000UL,   // 14  Ch13    325h00m00s   20h00m00s  T1    inc5
    1242000UL,   // 15  Ch14    345h00m00s   20h00m00s  T2    inc5
    1313712UL,   // 16  Ch15    364h55m12s   19h55m12s  Tfin  inc5
    1342800UL,   // 17  Ch16    373h00m00s    8h04m48s  T0    inc6
    1414800UL,   // 18  Ch17    393h00m00s   20h00m00s  T1    inc6
    1486800UL,   // 19  Ch18    413h00m00s   20h00m00s  T2    inc6
    1558512UL,   // 20  Ch19    432h55m12s   19h55m12s  Tfin  inc6
};
//  Mission ends 125 s after the last event: 1,558,637 s = 432.95 h = 18.04 d.

// ---- Heartbeat -------------------------------------------------------------
//  Gaps run to 59h55m12s and nothing happens for the first 33h00m. These wakes log
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
// real minimum gap is 8h04m48s against a 125 s routine, so this has enormous
// margin — it exists to catch a mistyped timestamp, not a design error.
constexpr bool _schedNoOverlap(uint8_t i = 1) {
  return (i >= EVENT_COUNT)
           ? true
           : ((SAMPLE_TIME_S[i] - SAMPLE_TIME_S[i - 1] > ROUTINE_S) && _schedNoOverlap(i + 1));
}

static_assert(_schedIncreasing(),
              "schedule.h: SAMPLE_TIME_S must be strictly increasing.");
static_assert(_schedNoOverlap(),
              "schedule.h: two events are closer together than the 125 s sample routine.");
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
