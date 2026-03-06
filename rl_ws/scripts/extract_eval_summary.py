"""Extract summary metrics from random eval CSV for report tables.

Complements analyze_eval_csv.py by providing metrics not covered there:
- Pitch std (deg)
- Contact ratio L/R

Usage:
    uv run python scripts/extract_eval_summary.py {N} [N-1 ...] --prefix droid-walking-omni-v
"""

import argparse
import csv
import math
import os
import sys


def load_csv(path):
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        return list(reader)


def compute_stats(values):
    n = len(values)
    if n == 0:
        return 0.0, 0.0
    mean = sum(values) / n
    variance = sum((v - mean) ** 2 for v in values) / n
    return mean, math.sqrt(variance)


def analyze_version(version, prefix, epoch):
    log_dir = os.path.join("logs", f"{prefix}{version}")
    csv_path = os.path.join(log_dir, f"eval_{epoch}.csv")
    if not os.path.exists(csv_path):
        print(f"  CSV not found: {csv_path}")
        return

    rows = load_csv(csv_path)
    dt = 0.02
    steady_start = 2.0
    steady_rows = [r for r in rows if float(r["timestamp"]) > steady_start]

    # Pitch std
    pitch_values = [float(r["pitch_deg"]) for r in steady_rows]
    pitch_mean, pitch_std = compute_stats(pitch_values)

    # Contact ratio L/R
    l_contact = sum(1 for r in steady_rows if float(r["contact_left"]) > 0.5)
    r_contact = sum(1 for r in steady_rows if float(r["contact_right"]) > 0.5)
    contact_ratio = l_contact / r_contact if r_contact > 0 else float("inf")

    print(f"  Pitch: mean={pitch_mean:.2f} deg, std={pitch_std:.2f} deg")
    print(f"  Contact: L={l_contact}, R={r_contact}, ratio(L/R)={contact_ratio:.3f}")


def main():
    parser = argparse.ArgumentParser(description="Extract eval summary metrics")
    parser.add_argument("versions", nargs="+", type=int, help="Version numbers")
    parser.add_argument(
        "--prefix", default="droid-walking-omni-v", help="Experiment name prefix"
    )
    parser.add_argument("--epoch", type=int, default=3999, help="Eval epoch")
    args = parser.parse_args()

    for v in args.versions:
        print(f"===== V{v} Summary =====")
        analyze_version(v, args.prefix, args.epoch)


if __name__ == "__main__":
    main()
