#!/usr/bin/env python3
"""Check a periodic trajectory and save a compact validation report beside it."""

import argparse
import csv
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--length", type=float, required=True, help="Periodic length in metres")
    parser.add_argument("--min-x", type=float, default=0.0)
    parser.add_argument("--height", type=float, help="Also validate periodic y with this length in metres")
    parser.add_argument("--min-y", type=float, default=0.0)
    parser.add_argument("--agents", type=int, required=True)
    args = parser.parse_args()
    if args.length <= 0.0 or args.agents <= 0:
        parser.error("length and population must be positive")
    if args.height is not None and args.height <= 0.0:
        parser.error("periodic height must be positive")

    frames = {}
    previous_x = {}
    previous_y = {}
    wraps_right = wraps_left = 0
    wraps_up = wraps_down = 0
    xmin = ymin = math.inf
    xmax = ymax = -math.inf
    with (args.run_directory / "trajectory.csv").open(newline="") as stream:
        for row in csv.DictReader(stream):
            values = {key: float(value) for key, value in row.items()}
            if not all(math.isfinite(value) for value in values.values()):
                raise ValueError("trajectory contains a nonfinite value")
            time = values["time_s"]
            agent = int(row["agent_id"])
            x, y = values["x_m"], values["y_m"]
            if not args.min_x <= x < args.min_x + args.length:
                raise ValueError(f"agent {agent} at {time} s has unwrapped x={x}")
            if args.height is not None and not args.min_y <= y < args.min_y + args.height:
                raise ValueError(f"agent {agent} at {time} s has unwrapped y={y}")
            frame = frames.setdefault(time, {})
            if agent in frame:
                raise ValueError("duplicate agent in a trajectory frame")
            frame[agent] = math.hypot(values["vx_m_per_s"], values["vy_m_per_s"])
            if agent in previous_x:
                jump = x - previous_x[agent]
                wraps_left += jump > args.length / 2
                wraps_right += jump < -args.length / 2
            previous_x[agent] = x
            if args.height is not None and agent in previous_y:
                jump = y - previous_y[agent]
                wraps_down += jump > args.height / 2
                wraps_up += jump < -args.height / 2
            previous_y[agent] = y
            xmin, xmax = min(xmin, x), max(xmax, x)
            ymin, ymax = min(ymin, y), max(ymax, y)
    if not frames:
        raise ValueError("trajectory is empty")
    expected_ids = set(next(iter(frames.values())))
    if len(expected_ids) != args.agents or any(set(frame) != expected_ids for frame in frames.values()):
        raise ValueError("population or identities change across samples")
    final = frames[max(frames)]
    report = {
        "status": "passed",
        "sampled_frames": len(frames),
        "agents_per_frame": args.agents,
        "final_time_s": max(frames),
        "all_state_values_finite": True,
        "all_x_positions_wrapped": True,
        "rightward_wraps_detected": wraps_right,
        "leftward_wraps_detected": wraps_left,
        "x_range_m": [xmin, xmax],
        "y_range_m": [ymin, ymax],
        "final_mean_speed_m_per_s": sum(final.values()) / len(final),
        "note": "Wrap counts inferred from sampled jumps; sampling must resolve travel below half each periodic length per interval.",
    }
    if args.height is not None:
        report.update(all_y_positions_wrapped=True, upward_wraps_detected=wraps_up,
                      downward_wraps_detected=wraps_down)
    output = json.dumps(report, indent=2) + "\n"
    (args.run_directory / "validation.json").write_text(output)
    print(output, end="")


if __name__ == "__main__":
    main()
