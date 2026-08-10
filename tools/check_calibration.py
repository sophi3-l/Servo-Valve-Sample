#!/usr/bin/env python3
"""
check_calibration.py — validate the per-lander calibration table in calibration.h

Catches the mechanical mistakes a human editing CH_OPEN_DEG_ALL is most likely to
make, so a mis-pasted or forgotten calibration can't reach a sealed unit:

  1. every row has exactly 21 values,
  2. every open angle is in [SERVO_MIN_DEG, 180],
  3. the row / flag counts match LANDER_COUNT,
  4. no two COMMITTED rows are identical  (i.e. a committed row is still a
     leftover copy of the placeholder — the failure mode that ships one unit
     with another unit's angles).

Usage:  python tools/check_calibration.py [path/to/calibration.h]
Exit 0 = OK, exit 1 = one or more checks failed.
"""

import re
import sys


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)          # /* ... */
    out = []
    for line in text.split("\n"):
        i = line.find("//")
        out.append(line[:i] if i >= 0 else line)              # // ...
    return "\n".join(out)


def find_int(src: str, name: str, default=None):
    m = re.search(rf"\b{name}\s*=\s*(\d+)", src)
    return int(m.group(1)) if m else default


def parse(path: str):
    raw = open(path, "r", encoding="utf-8").read()
    src = strip_comments(raw)

    lander_count = find_int(src, "LANDER_COUNT", default=4)
    servo_min = find_int(src, "SERVO_MIN_DEG", default=70)

    # CH_OPEN_DEG_ALL[...][21] = { {...}, {...}, ... };
    m = re.search(
        r"CH_OPEN_DEG_ALL\s*\[[^\]]*\]\s*\[\s*21\s*\]\s*=\s*\{(.*?)\}\s*;",
        src, flags=re.S)
    if not m:
        raise SystemExit("FAIL: could not find CH_OPEN_DEG_ALL[...][21] = { ... };")
    rows = [[int(x) for x in re.findall(r"\d+", grp)]
            for grp in re.findall(r"\{([^{}]*)\}", m.group(1))]

    # CAL_COMMITTED[...] = { true, false, ... };
    c = re.search(r"CAL_COMMITTED\s*\[[^\]]*\]\s*=\s*\{([^}]*)\}", src, flags=re.S)
    if not c:
        raise SystemExit("FAIL: could not find CAL_COMMITTED[...] = { ... };")
    committed = [tok.strip() == "true"
                 for tok in c.group(1).split(",") if tok.strip()]

    return lander_count, servo_min, rows, committed


def main(path: str) -> int:
    lander_count, servo_min, rows, committed = parse(path)
    errors, notes = [], []

    # (3) counts line up
    if len(rows) != lander_count:
        errors.append(f"CH_OPEN_DEG_ALL has {len(rows)} rows, expected LANDER_COUNT={lander_count}")
    if len(committed) != lander_count:
        errors.append(f"CAL_COMMITTED has {len(committed)} flags, expected LANDER_COUNT={lander_count}")

    # (1) + (2) shape and range, per row
    for i, row in enumerate(rows, start=1):
        if len(row) != 21:
            errors.append(f"Lander {i}: {len(row)} values, expected 21")
            continue
        bad = [(ch, v) for ch, v in enumerate(row) if not (servo_min <= v <= 180)]
        for ch, v in bad:
            errors.append(f"Lander {i} Ch{ch}: open angle {v} out of range [{servo_min}, 180]")

    # (4) committed rows must be pairwise distinct
    committed_rows = [(i + 1, rows[i]) for i in range(min(len(rows), len(committed)))
                      if committed[i]]
    for a in range(len(committed_rows)):
        for b in range(a + 1, len(committed_rows)):
            ia, ra = committed_rows[a]
            ib, rb = committed_rows[b]
            if ra == rb:
                errors.append(
                    f"Landers {ia} and {ib} are both COMMITTED but have identical angles "
                    f"— one is still a placeholder copy")

    # informational: which units are ready
    for i in range(min(len(rows), len(committed))):
        state = "committed" if committed[i] else "placeholder (not deployable)"
        notes.append(f"  Lander {i+1}: {state}")

    print(f"Checked {path}  (LANDER_COUNT={lander_count}, floor={servo_min}°)")
    print("\n".join(notes))
    if errors:
        print("\nFAILED:")
        for e in errors:
            print(f"  ✗ {e}")
        return 1
    print("\nOK — calibration table is consistent.")
    return 0


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "include/calibration.h"
    sys.exit(main(path))
