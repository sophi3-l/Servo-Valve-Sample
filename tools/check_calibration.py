#!/usr/bin/env python3
"""
check_calibration.py — validate the shared headers that every build reads.

include/calibration.h and include/schedule.h carry static_asserts that catch the
same mistakes at compile time. This script exists because the two gates fail at
different moments: the asserts stop a bad build on the bench laptop, and this
stops a bad table reaching main. Keep both.

calibration.h
  1. every row has exactly 21 values,
  2. every open angle is in [SERVO_MIN_DEG, 180],
  3. the row / flag counts match LANDER_COUNT,
  4. no two COMMITTED rows are identical (a committed row that is still a
     leftover placeholder copy — the failure that ships one unit with another
     unit's angles),
  5. every SAMPLE channel (Ch0..19) reaches open + CLOSE_OFFSET_DEG without
     being clamped at 180,
  6. the common valve's open + MAIN_CLOSE_OFFSET_DEG fits under
     MAIN_CLOSE_MAX_DEG on every unit (a unit that clamps silently gets less
     travel than the rest of the fleet).

schedule.h
  7. SAMPLE_TIME_S has exactly one entry per sample valve,
  8. it is strictly increasing,
  9. no two events are closer together than the sample routine takes to run.

Usage:  python tools/check_calibration.py [include/calibration.h] [include/schedule.h]
Exit 0 = OK, exit 1 = one or more checks failed.
"""

import re
import sys


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)          # /* ... */
    out = []
    for line in text.split("\n"):
        i = line.find("//")
        out.append(line[:i] if i >= 0 else line)               # // ...
    return "\n".join(out)


def find_int(src: str, name: str, default=None):
    m = re.search(rf"\b{name}\s*=\s*(\d+)", src)
    return int(m.group(1)) if m else default


def find_array(src: str, name: str, dims: str):
    """Return the flat list of ints inside `name[dims...] = { ... };`."""
    m = re.search(rf"{name}\s*\[{dims}\]\s*=\s*\{{(.*?)\}}\s*;", src, flags=re.S)
    return m.group(1) if m else None


# ---------------------------------------------------------------- calibration.h
def parse_calibration(path: str):
    src = strip_comments(open(path, "r", encoding="utf-8").read())

    cfg = {
        "lander_count": find_int(src, "LANDER_COUNT", 4),
        "servo_min": find_int(src, "SERVO_MIN_DEG", 70),
        "close_offset": find_int(src, "CLOSE_OFFSET_DEG", 65),
        "main_offset": find_int(src, "MAIN_CLOSE_OFFSET_DEG"),
        "main_max": find_int(src, "MAIN_CLOSE_MAX_DEG"),
        "main_ch": find_int(src, "MAIN_SERVO_CH", 20),
        "sample_count": find_int(src, "SAMPLE_SERVO_COUNT", 20),
    }

    body = find_array(src, "CH_OPEN_DEG_ALL", r"[^\]]*\]\s*\[\s*21\s*")
    if body is None:
        raise SystemExit("FAIL: could not find CH_OPEN_DEG_ALL[...][21] = { ... };")
    rows = [[int(x) for x in re.findall(r"\d+", grp)]
            for grp in re.findall(r"\{([^{}]*)\}", body)]

    c = re.search(r"CAL_COMMITTED\s*\[[^\]]*\]\s*=\s*\{([^}]*)\}", src, flags=re.S)
    if not c:
        raise SystemExit("FAIL: could not find CAL_COMMITTED[...] = { ... };")
    committed = [tok.strip() == "true" for tok in c.group(1).split(",") if tok.strip()]

    return cfg, rows, committed


def check_calibration(path, errors, notes):
    cfg, rows, committed = parse_calibration(path)
    n, floor = cfg["lander_count"], cfg["servo_min"]

    if len(rows) != n:
        errors.append(f"CH_OPEN_DEG_ALL has {len(rows)} rows, expected LANDER_COUNT={n}")
    if len(committed) != n:
        errors.append(f"CAL_COMMITTED has {len(committed)} flags, expected LANDER_COUNT={n}")

    for i, row in enumerate(rows, start=1):
        if len(row) != 21:
            errors.append(f"unit {i}: {len(row)} values, expected 21")
            continue
        for ch, v in enumerate(row):
            if not (floor <= v <= 180):
                errors.append(f"unit {i} Ch{ch}: open angle {v} out of range [{floor}, 180]")
            # Sample channels must not clamp; Ch20 uses its own offset/cap.
            if ch < cfg["sample_count"] and v + cfg["close_offset"] > 180:
                errors.append(
                    f"unit {i} Ch{ch}: open {v} + CLOSE_OFFSET_DEG {cfg['close_offset']} "
                    f"= {v + cfg['close_offset']} would clamp at 180")

    off, cap, mch = cfg["main_offset"], cfg["main_max"], cfg["main_ch"]
    mains = []
    if off is None or cap is None:
        errors.append("MAIN_CLOSE_OFFSET_DEG / MAIN_CLOSE_MAX_DEG not found in calibration.h")
    else:
        if not (floor <= cap < 180):
            errors.append(f"MAIN_CLOSE_MAX_DEG {cap} must be in [{floor}, 180) — it is the "
                          f"cushion that keeps the common valve off its mechanical stop")
        if off <= 0:
            errors.append("MAIN_CLOSE_OFFSET_DEG must be positive — the common valve closes "
                          "by moving away from its open angle")
        for i, row in enumerate(rows, start=1):
            if len(row) <= mch:
                continue
            o = row[mch]
            if o + off > cap:
                errors.append(
                    f"unit {i}: Ch{mch} open {o} + offset {off} = {o + off} exceeds "
                    f"MAIN_CLOSE_MAX_DEG {cap} — that unit would clamp and get less travel "
                    f"than the rest of the fleet. Lower the offset, or lower its open angle.")
            mains.append((i, o, min(o + off, cap)))

    committed_rows = [(i + 1, rows[i]) for i in range(min(len(rows), len(committed))) if committed[i]]
    for a in range(len(committed_rows)):
        for b in range(a + 1, len(committed_rows)):
            ia, ra = committed_rows[a]
            ib, rb = committed_rows[b]
            if ra == rb:
                errors.append(f"units {ia} and {ib} are both COMMITTED but have identical "
                              f"angles — one is still a placeholder copy")

    for i in range(min(len(rows), len(committed))):
        state = "committed" if committed[i] else "placeholder (not deployable)"
        notes.append(f"  unit {i+1}: {state}")
    notes.append(f"  sample close = open + {cfg['close_offset']}°   ·   "
                 f"common (Ch{mch}) close = open + {off}°, capped at {cap}°")
    if mains:
        notes.append("  common close per unit: " +
                     ", ".join(f"u{i} {o}->{c}" for i, o, c in mains))
    return cfg


# ------------------------------------------------------------------- schedule.h
def check_schedule(path, cfg, errors, notes):
    try:
        src = strip_comments(open(path, "r", encoding="utf-8").read())
    except FileNotFoundError:
        errors.append(f"{path} not found")
        return

    # ROUTINE_S is derived in the header (CMD_OPEN_COMMON_MS + STEP_SETTLE_MS),
    # so it is no longer a literal we can grep. Recompute it from the parts.
    settle = find_int(src, "STEP_SETTLE_MS")
    offs = [find_int(src, n) for n in ("CMD_CLOSE_COMMON_MS", "CMD_OPEN_SAMPLE_MS",
                                       "CMD_CLOSE_SAMPLE_MS", "CMD_OPEN_COMMON_MS")]
    if settle is not None and offs[3] is not None:
        routine = (offs[3] + settle) // 1000
    else:
        routine = find_int(src, "ROUTINE_S", 126)

    # The settle must fit inside every gap between protocol commands, or one
    # servo would still be powered when the next command is issued. And the two
    # science-critical windows must not have moved while the settle was tuned.
    if settle is not None and all(o is not None for o in offs):
        for a, b in zip(offs, offs[1:]):
            if b <= a:
                errors.append(f"per-event command offsets out of order ({a} -> {b} ms)")
            elif settle >= b - a:
                errors.append(f"STEP_SETTLE_MS {settle} ms does not fit the {b - a} ms "
                              f"gap between the commands at {a} and {b} ms")
        if offs[2] - offs[1] != 116000:
            errors.append(f"sample-open window is {(offs[2]-offs[1])/1000:g} s, expected 116 s")
        if offs[3] - offs[0] != 124000:
            errors.append(f"common-closed window is {(offs[3]-offs[0])/1000:g} s, expected 124 s")
        notes.append(f"  routine {routine} s · settle {settle} ms · "
                     f"sample open {(offs[2]-offs[1])//1000} s · "
                     f"common closed {(offs[3]-offs[0])//1000} s")

    body = find_array(src, "SAMPLE_TIME_S", r"[^\]]*")
    if body is None:
        errors.append("could not find SAMPLE_TIME_S[...] = { ... };")
        return
    t = [int(x) for x in re.findall(r"\d+", body)]

    expected = cfg.get("sample_count", 20)
    if len(t) != expected:
        errors.append(f"SAMPLE_TIME_S has {len(t)} entries, expected one per sample "
                      f"valve (SAMPLE_SERVO_COUNT={expected})")

    gaps = []
    for i in range(1, len(t)):
        gap = t[i] - t[i - 1]
        gaps.append(gap)
        if gap <= 0:
            errors.append(f"SAMPLE_TIME_S is not strictly increasing at index {i} "
                          f"({t[i-1]} -> {t[i]})")
        elif gap <= routine:
            errors.append(f"events {i} and {i+1} are {gap} s apart, which is not more "
                          f"than the {routine} s sample routine")

    if t and gaps:
        notes.append(f"  {len(t)} events · first at {t[0]} s ({t[0]/3600:.2f} h) · "
                     f"last at {t[-1]} s ({t[-1]/3600:.2f} h)")
        notes.append(f"  mission length {t[-1] + routine} s "
                     f"({(t[-1]+routine)/86400:.2f} d) · min gap {min(gaps)} s "
                     f"({min(gaps)/3600:.2f} h) vs {routine} s routine")


def main(cal_path: str, sched_path: str) -> int:
    errors, notes = [], []
    print(f"Checking {cal_path}")
    cfg = check_calibration(cal_path, errors, notes)
    print("\n".join(notes)); notes = []
    print(f"\nChecking {sched_path}")
    check_schedule(sched_path, cfg, errors, notes)
    print("\n".join(notes))

    if errors:
        print("\nFAILED:")
        for e in errors:
            print(f"  ✗ {e}")
        return 1
    print("\nOK — calibration table and sample schedule are consistent.")
    return 0


if __name__ == "__main__":
    cal = sys.argv[1] if len(sys.argv) > 1 else "include/calibration.h"
    sch = sys.argv[2] if len(sys.argv) > 2 else "include/schedule.h"
    sys.exit(main(cal, sch))
