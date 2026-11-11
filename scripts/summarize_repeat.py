#!/usr/bin/env python3
"""Summarise repeated runs produced by scripts/repeat.sh.

    python3 scripts/summarize_repeat.py docs/metrics/repeat/static_*

Reads each eval_*.txt (a JSON blob followed by one markdown row) and prints the
pass counts the acceptance criteria ask for. No number is invented here: every
value is copied from the per-run eval.
"""

from __future__ import annotations

import glob
import json
import sys

LIMITS = {
    # acceptance: 10/10 reach the goal, no crash/timeout, no contact
    "min_clearance_contact_m": 0.0,
    # T5.4 acceptance: fewer than 2 yaw-rate reversals per approach
    "max_ang_zero_cross": 2,
    # T4.4 acceptance (static only): the driven line is compared with the A*
    # polyline for the same start and goal, so allow the plan's own stair-step
    # plus the robot's corridor offset. On a dynamic run the robot deliberately
    # leaves that line to go round the box, so the same number is reported but
    # not gated: T5.4 asks for no contact and < 2 crossings per approach.
    "max_lateral_m": 0.30,
}


def load(path: str) -> dict:
    text = open(path).read()
    head, _, _ = text.partition("\n| ")
    return json.loads(head)


def main() -> None:
    pattern = sys.argv[1] if len(sys.argv) > 1 else "docs/metrics/repeat/*.txt"
    files = sorted(glob.glob(pattern))
    if not files:
        raise SystemExit(f"no eval files matched {pattern}")

    runs = [(f.rsplit("/", 1)[-1][:-4], load(f)) for f in files]
    name = runs[0][1]["run"].rsplit("_", 1)[0]

    print(f"| Run | success | t (s) | travelled / optimal | clearance | decel | RMSE raw/EKF | lateral | approaches / max zero-cross |")
    print("|---|---|---|---|---|---|---|---|---|")
    for tag, r in runs:
        print(
            f"| {tag} | {'yes' if r['success'] else 'NO'} | {r['time_to_goal_s']} | "
            f"{r['travelled_m']} / {r['astar_optimal_m']} | {r['min_clearance_m']} | "
            f"{r['max_decel_mps2']} | {r['rmse_raw_m']} / {r['rmse_ekf_m']} | "
            f"{r['max_lateral_m']} | {r['approach_episodes']} / {r['max_ang_zero_cross']} |"
        )

    n = len(runs)
    ok = sum(1 for _, r in runs if r["success"])
    contact = [t for t, r in runs if r["min_clearance_m"] <= LIMITS["min_clearance_contact_m"]]
    osc = [t for t, r in runs if (r["max_ang_zero_cross"] or 0) >= LIMITS["max_ang_zero_cross"]]
    static = name == "static"
    lat = (
        [t for t, r in runs if (r["max_lateral_m"] or 0) > LIMITS["max_lateral_m"]]
        if static
        else []
    )
    slipped = [t for t, r in runs if r["rmse_raw_m"] > 0.5]
    holds = sum(1 for _, r in runs if r["approach_episodes"] > 0)

    print(f"\n{name}: success {ok}/{n}")
    print(f"  runs with an obstacle approach (obs < 0.30 m): {holds}/{n}")
    print(f"  contact (clearance <= 0 m): {len(contact)}/{n}" + (f" {contact}" if contact else ""))
    print(f"  yaw oscillation >= {LIMITS['max_ang_zero_cross']} per approach: {len(osc)}/{n}"
          + (f" {osc}" if osc else ""))
    if static:
        print(f"  lateral > {LIMITS['max_lateral_m']} m: {len(lat)}/{n}"
              + (f" {lat}" if lat else ""))
    else:
        worst = max(runs, key=lambda tr: tr[1]["max_lateral_m"] or 0)
        print(f"  lateral (not gated on a dynamic run, the path leaves the static "
              f"line on purpose): worst {worst[1]['max_lateral_m']} m ({worst[0]})")
    if slipped:
        print(f"  raw odometry drifted > 0.5 m (wheel slip while blocked): {slipped}")
    verdict = ok == n and not contact and not osc and not lat
    print(f"  verdict: {'PASS' if verdict else 'NOT PASS'} "
          f"({n}/{n} required for the acceptance criterion)")


if __name__ == "__main__":
    main()