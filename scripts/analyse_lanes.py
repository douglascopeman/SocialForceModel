#!/usr/bin/env python3
"""Compare transverse direction segregation in balanced bidirectional runs.

Uses the scenario's even-ID/right, odd-ID/left assignment. The diagnostic bins
the entire channel into horizontal strips; it is an exploratory lane indicator.
"""

import argparse
import csv
from functools import lru_cache
import json
import math
from pathlib import Path


@lru_cache(maxsize=None)
def expected_imbalance(population, occupancy):
    """Exact imbalance under balanced random relabelling, at fixed occupancy."""
    half = population // 2
    return sum(abs(2 * k - occupancy) * math.comb(half, k) * math.comb(half, occupancy - k)
               for k in range(max(0, occupancy - half), min(half, occupancy) + 1)) / math.comb(population, occupancy)


def analyse(run_dir, height, bins, late_window):
    frames = {}
    with (run_dir / "trajectory.csv").open(newline="") as stream:
        for row in csv.DictReader(stream):
            frames.setdefault(float(row["time_s"]), []).append(row)
    population = len(next(iter(frames.values())))
    expected_ids = {int(row["agent_id"]) for row in next(iter(frames.values()))}
    if population % 2 or sum(agent % 2 == 0 for agent in expected_ids) != population // 2:
        raise ValueError("analysis expects balanced populations of even size")
    rows = []
    for time, frame in sorted(frames.items()):
        if len(frame) != population or {int(row["agent_id"]) for row in frame} != expected_ids:
            raise ValueError("population or identities change between frames")
        counts = [[0, 0] for _ in range(bins)]
        progress = 0.0
        outside = 0
        for agent in frame:
            side = int(agent["agent_id"]) % 2
            y = float(agent["y_m"])
            outside += not 0 <= y <= height
            counts[max(0, min(bins - 1, int(y / height * bins)))][side] += 1
            progress += (1 if side == 0 else -1) * float(agent["vx_m_per_s"])
        purity = sum(abs(a - b) for a, b in counts) / population
        null = sum(expected_imbalance(population, a + b) for a, b in counts) / population
        corrected = (purity - null) / (1 - null) if null < 1 else 0.0
        rows.append({"time_s": time, "outside_y_walls": outside, "strip_purity": purity, "random_label_baseline": null,
                     "corrected_segregation": corrected, "mean_forward_speed_m_per_s": progress / population})
    with (run_dir / "lane_metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    late = [row for row in rows if row["time_s"] > rows[-1]["time_s"] - late_window]
    report = {"run": run_dir.name, "agents": population, "height_m": height,
              "strip_width_m": height / bins, "late_window_s": late_window,
              "initial_corrected_segregation": rows[0]["corrected_segregation"],
              "late_mean_corrected_segregation": sum(r["corrected_segregation"] for r in late) / len(late),
              "late_mean_forward_speed_m_per_s": sum(r["mean_forward_speed_m_per_s"] for r in late) / len(late),
              "samples_outside_y_walls": sum(r["outside_y_walls"] for r in rows),
              "note": "0 matches random mixing in expectation, 1 is pure horizontal strips; negative fluctuations possible. Depends on strip width; not a proof of stable lanes. Outside-wall samples are clamped into edge strips and flagged."}
    (run_dir / "lane_summary.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directories", type=Path, nargs="+")
    parser.add_argument("--height", type=float, default=6.0)
    parser.add_argument("--bins", type=int, default=12)
    parser.add_argument("--late-window", type=float, default=60.0)
    args = parser.parse_args()
    if args.height <= 0 or args.bins <= 0 or args.late_window <= 0:
        parser.error("height, bins and window must be positive")
    for run_dir in args.run_directories:
        print(json.dumps(analyse(run_dir, args.height, args.bins, args.late_window), indent=2))


if __name__ == "__main__":
    main()
