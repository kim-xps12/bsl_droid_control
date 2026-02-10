#!/usr/bin/env python3
"""
学習ログから個別報酬項目の最終値を表示するスクリプト。

Usage:
    uv run python scripts/show_reward_components.py <exp-name> [<exp-name2> ...]

Examples:
    uv run python scripts/show_reward_components.py droid-walking-v18
    uv run python scripts/show_reward_components.py droid-walking-v18 droid-walking-v19
"""

from __future__ import annotations

import argparse
import os
import sys

from tensorboard.backend.event_processing import event_accumulator


def show_reward_components(exp_name: str, log_base: str = "logs") -> bool:
    """指定した実験の個別報酬項目を表示する。

    Args:
        exp_name: 実験名（例: droid-walking-v18）
        log_base: ログディレクトリのベースパス

    Returns:
        成功した場合True、失敗した場合False
    """
    log_dir = os.path.join(log_base, exp_name)

    if not os.path.exists(log_dir):
        print(f"Error: Log directory not found: {log_dir}", file=sys.stderr)
        return False

    event_files = [f for f in os.listdir(log_dir) if f.startswith("events.")]
    if not event_files:
        print(f"Error: No event files found in {log_dir}", file=sys.stderr)
        return False

    ea = event_accumulator.EventAccumulator(os.path.join(log_dir, event_files[0]))
    ea.Reload()

    print(f"\n=== {exp_name} Individual Reward Components (Final Step) ===")

    reward_tags = [tag for tag in sorted(ea.Tags()["scalars"]) if tag.startswith("Episode/rew_")]

    if not reward_tags:
        print("  No reward components found in log.")
        return True

    for tag in reward_tags:
        events = ea.Scalars(tag)
        if events:
            reward_name = tag.replace("Episode/rew_", "")
            final_value = events[-1].value
            # 負の値は警告色で表示（設計意図と逆の可能性）
            marker = " ⚠️" if final_value < 0 else ""
            print(f"  {reward_name:25s}: {final_value:10.4f}{marker}")

    return True


def main() -> None:
    parser = argparse.ArgumentParser(
        description="学習ログから個別報酬項目の最終値を表示する",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  uv run python scripts/show_reward_components.py droid-walking-v18
  uv run python scripts/show_reward_components.py droid-walking-v18 droid-walking-v19

Notes:
  - 負の値には ⚠️ マークが付きます（設計意図と逆の学習の可能性）
  - 主報酬（tracking_lin_vel）がペナルティ合計より小さい場合、
    「動かない」が最適解になりやすいので注意
        """,
    )
    parser.add_argument(
        "exp_names",
        nargs="+",
        metavar="EXP_NAME",
        help="実験名（例: droid-walking-v18）",
    )
    parser.add_argument(
        "--log-dir",
        default="logs",
        help="ログディレクトリのベースパス（デフォルト: logs）",
    )

    args = parser.parse_args()

    success = True
    for exp_name in args.exp_names:
        if not show_reward_components(exp_name, args.log_dir):
            success = False

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
