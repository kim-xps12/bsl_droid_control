"""Batch evaluation - run multiple biped_eval.py instances in parallel.

Parallelizes the data collection phase by running multiple biped_eval.py
instances concurrently using subprocesses.

Usage:
    cd rl_ws

    # Full evaluation suite (random + 4-direction × 3-seed)
    uv run python scripts/batch_eval.py \
        -e droid-walking-omni-v14 --vx-max 0.3 --vy-max 0.15

    # Multiple experiments at once
    uv run python scripts/batch_eval.py \
        -e droid-walking-omni-v14 droid-walking-omni-v13 \
        --vx-max 0.3 --vy-max 0.15 --workers 4

    # Directional only (skip random eval)
    uv run python scripts/batch_eval.py \
        -e droid-walking-omni-v14 --vx-max 0.3 --vy-max 0.15 --no-random

    # Random only (skip directional eval)
    uv run python scripts/batch_eval.py -e droid-walking-omni-v14 --no-directional

    # Specific fixed command(s) only
    uv run python scripts/batch_eval.py \
        -e droid-walking-omni-v13 --no-random --no-directional \
        --extra-cmd "0.3 0.0 0.0"

    # Dry run to see planned jobs
    uv run python scripts/batch_eval.py \
        -e droid-walking-omni-v14 --vx-max 0.3 --vy-max 0.15 --dry-run
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path


@dataclass
class EvalJob:
    """A single biped_eval.py invocation."""

    exp_name: str
    duration: int
    command: tuple[float, float, float] | None  # (vx, vy, vyaw) or None for random
    seed: int
    csv_path: str  # Expected output path (for skip check)
    label: str  # Human-readable description

    def build_args(self) -> list[str]:
        """Build command-line arguments for biped_eval.py."""
        args = [
            sys.executable,
            "biped_walking/biped_eval.py",
            "-e",
            self.exp_name,
            "--no-viewer",
            "--duration",
            str(self.duration),
            "--csv",
            "--seed",
            str(self.seed),
        ]
        if self.command is not None:
            vx, vy, vyaw = self.command
            args.extend(["--command", str(vx), str(vy), str(vyaw)])
        return args


def get_latest_ckpt(exp_name: str) -> int:
    """Get the latest checkpoint number for an experiment."""
    log_dir = Path("logs") / exp_name
    if not log_dir.exists():
        raise FileNotFoundError(f"Log directory not found: {log_dir}")

    ckpts = [
        f for f in log_dir.iterdir() if f.name.startswith("model_") and f.suffix == ".pt"
    ]
    if not ckpts:
        raise FileNotFoundError(f"No checkpoints found in {log_dir}")

    return max(int(f.stem.split("_")[1]) for f in ckpts)


def generate_jobs(
    exp_name: str,
    ckpt_num: int,
    vx_max: float | None,
    vy_max: float | None,
    random_duration: int,
    dir_duration: int,
    seeds: list[int],
    include_random: bool,
    include_directional: bool,
    extra_cmds: list[tuple[float, float, float]],
) -> list[EvalJob]:
    """Generate evaluation jobs for a single experiment."""
    log_dir = f"logs/{exp_name}"
    jobs: list[EvalJob] = []

    # Random command eval
    if include_random:
        csv_path = f"{log_dir}/eval_{ckpt_num}.csv"
        jobs.append(
            EvalJob(
                exp_name=exp_name,
                duration=random_duration,
                command=None,
                seed=1,
                csv_path=csv_path,
                label=f"{exp_name} random",
            )
        )

    # Directional eval (4 directions × N seeds)
    if include_directional:
        if vx_max is None or vy_max is None:
            raise ValueError("--vx-max and --vy-max are required for directional eval")

        directions = [
            ("FWD", (vx_max, 0.0, 0.0)),
            ("BWD", (-vx_max, 0.0, 0.0)),
            ("LFT", (0.0, vy_max, 0.0)),
            ("RGT", (0.0, -vy_max, 0.0)),
        ]

        for dir_name, (vx, vy, vyaw) in directions:
            for seed in seeds:
                dir_tag = f"_cmd_{vx:.2f}_{vy:.2f}_{vyaw:.2f}_s{seed}"
                csv_path = f"{log_dir}/eval_{ckpt_num}{dir_tag}.csv"
                jobs.append(
                    EvalJob(
                        exp_name=exp_name,
                        duration=dir_duration,
                        command=(vx, vy, vyaw),
                        seed=seed,
                        csv_path=csv_path,
                        label=f"{exp_name} {dir_name} s{seed}",
                    )
                )

    # Extra fixed commands (e.g., FWD-only for V{N-1} yaw eval)
    for vx, vy, vyaw in extra_cmds:
        dir_tag = f"_cmd_{vx:.2f}_{vy:.2f}_{vyaw:.2f}_s1"
        csv_path = f"{log_dir}/eval_{ckpt_num}{dir_tag}.csv"
        # Derive label from velocity
        if vx > 0:
            d = "FWD"
        elif vx < 0:
            d = "BWD"
        elif vy > 0:
            d = "LFT"
        elif vy < 0:
            d = "RGT"
        else:
            d = "ZERO"
        jobs.append(
            EvalJob(
                exp_name=exp_name,
                duration=dir_duration,
                command=(vx, vy, vyaw),
                seed=1,
                csv_path=csv_path,
                label=f"{exp_name} extra-{d}",
            )
        )

    return jobs


def run_job(job: EvalJob) -> tuple[str, bool, float, str]:
    """Run a single eval job. Returns (label, success, elapsed_sec, message)."""
    t0 = time.monotonic()
    args = job.build_args()
    try:
        result = subprocess.run(
            args,
            capture_output=True,
            text=True,
            timeout=300,  # 5 min timeout per job
        )
        elapsed = time.monotonic() - t0
        if result.returncode != 0:
            err = result.stderr.strip().splitlines()
            err_msg = err[-1] if err else "Unknown error"
            return (job.label, False, elapsed, err_msg)
        return (job.label, True, elapsed, job.csv_path)
    except subprocess.TimeoutExpired:
        elapsed = time.monotonic() - t0
        return (job.label, False, elapsed, "Timeout (300s)")
    except Exception as e:
        elapsed = time.monotonic() - t0
        return (job.label, False, elapsed, str(e))


def parse_extra_cmd(s: str) -> tuple[float, float, float]:
    """Parse 'VX VY VYAW' string into a tuple."""
    parts = s.strip().split()
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f"Expected 'VX VY VYAW', got: {s!r}")
    return (float(parts[0]), float(parts[1]), float(parts[2]))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Batch evaluation - run multiple biped_eval.py instances in parallel",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "-e", "--exp-names", nargs="+", required=True, metavar="EXP", help="Experiment name(s)"
    )
    parser.add_argument(
        "--workers", type=int, default=3, help="Max concurrent processes (default: 3)"
    )
    parser.add_argument("--vx-max", type=float, help="Max forward velocity for directional eval")
    parser.add_argument("--vy-max", type=float, help="Max lateral velocity for directional eval")
    parser.add_argument(
        "--random-duration", type=int, default=10, help="Duration for random eval in seconds (default: 10)"
    )
    parser.add_argument(
        "--dir-duration", type=int, default=20, help="Duration for directional eval in seconds (default: 20)"
    )
    parser.add_argument(
        "--seeds", type=int, nargs="+", default=[1, 2, 3], help="Seeds for directional eval (default: 1 2 3)"
    )
    parser.add_argument("--no-random", action="store_true", help="Skip random command eval")
    parser.add_argument("--no-directional", action="store_true", help="Skip directional eval")
    parser.add_argument(
        "--extra-cmd",
        type=parse_extra_cmd,
        action="append",
        default=[],
        metavar='"VX VY VYAW"',
        help='Additional fixed-command eval (e.g., --extra-cmd "0.3 0.0 0.0"). Repeatable.',
    )
    parser.add_argument("--force", action="store_true", help="Re-run even if CSV already exists")
    parser.add_argument("--dry-run", action="store_true", help="Show planned jobs without executing")
    args = parser.parse_args()

    if args.no_random and args.no_directional and not args.extra_cmd:
        parser.error("Nothing to do: --no-random, --no-directional, and no --extra-cmd")

    # Generate all jobs across experiments
    all_jobs: list[EvalJob] = []
    for exp_name in args.exp_names:
        ckpt_num = get_latest_ckpt(exp_name)
        print(f"[{exp_name}] Latest checkpoint: model_{ckpt_num}.pt")
        jobs = generate_jobs(
            exp_name=exp_name,
            ckpt_num=ckpt_num,
            vx_max=args.vx_max,
            vy_max=args.vy_max,
            random_duration=args.random_duration,
            dir_duration=args.dir_duration,
            seeds=args.seeds,
            include_random=not args.no_random,
            include_directional=not args.no_directional,
            extra_cmds=args.extra_cmd,
        )
        all_jobs.extend(jobs)

    # Filter out existing CSVs (unless --force)
    if not args.force:
        pending: list[EvalJob] = []
        skipped = 0
        for job in all_jobs:
            if os.path.exists(job.csv_path):
                skipped += 1
            else:
                pending.append(job)
        if skipped:
            print(f"\nSkipped {skipped} job(s) (CSV already exists). Use --force to re-run.")
        all_jobs = pending

    if not all_jobs:
        print("\nNo jobs to run. All CSVs already exist.")
        return

    # Show job plan
    print(f"\n{'=' * 60}")
    print(f"Jobs to run: {len(all_jobs)} | Workers: {args.workers}")
    print(f"{'=' * 60}")
    for i, job in enumerate(all_jobs, 1):
        if job.command:
            cmd_str = f"cmd=({job.command[0]:.2f}, {job.command[1]:.2f}, {job.command[2]:.2f}) s{job.seed}"
        else:
            cmd_str = "random"
        print(f"  [{i:2d}] {job.label:40s} dur={job.duration}s  {cmd_str}")

    if args.dry_run:
        print("\n[dry-run] No jobs executed.")
        return

    # Execute jobs in parallel using ThreadPoolExecutor
    # (each worker just blocks on subprocess.run, so threads are ideal)
    print(f"\nStarting evaluation (max {args.workers} concurrent)...\n")
    t_start = time.monotonic()
    completed = 0
    failed = 0

    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        future_to_job = {executor.submit(run_job, job): job for job in all_jobs}
        for future in as_completed(future_to_job):
            label, success, elapsed, msg = future.result()
            completed += 1
            status = f"[{completed}/{len(all_jobs)}]"
            if success:
                print(f"  OK {status} {label:40s} ({elapsed:.1f}s)")
            else:
                failed += 1
                print(f"  NG {status} {label:40s} ({elapsed:.1f}s) ERROR: {msg}")

    total_time = time.monotonic() - t_start
    seq_estimate = sum(j.duration + 5 for j in all_jobs)

    print(f"\n{'=' * 60}")
    print(f"Done: {completed - failed}/{len(all_jobs)} succeeded, {failed} failed")
    print(f"Total time:  {total_time:.1f}s")
    print(f"Sequential estimate: ~{seq_estimate}s (speedup: {seq_estimate / max(total_time, 0.1):.1f}x)")
    print(f"{'=' * 60}")

    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
