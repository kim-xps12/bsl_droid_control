#!/usr/bin/env python3
"""学習ログの分析スクリプト"""

from __future__ import annotations

import os
import sys

from tensorboard.backend.event_processing import event_accumulator


def analyze_log(exp_name: str) -> None:
    log_dir = f"logs/{exp_name}"
    event_files = [f for f in os.listdir(log_dir) if f.startswith("events.")]
    if not event_files:
        print(f"No event files found in {log_dir}")
        return

    ea = event_accumulator.EventAccumulator(os.path.join(log_dir, event_files[0]))
    ea.Reload()

    print(f"=== {exp_name} Training Log Analysis ===\n")
    print("Available Scalar Tags:")
    for tag in ea.Tags()["scalars"]:
        print(f"  - {tag}")

    print("\n=== Key Metrics Over Training ===\n")

    # 主要指標を詳細に取得
    key_tags = ["Train/mean_reward", "Train/mean_episode_length", "Loss/value_function", "Loss/surrogate"]

    for tag in key_tags:
        try:
            events = ea.Scalars(tag)
            if events:
                print(f"{tag}:")
                # 10区間でサンプリング
                n = len(events)
                indices = [
                    0,
                    n // 10,
                    n // 5,
                    3 * n // 10,
                    2 * n // 5,
                    n // 2,
                    3 * n // 5,
                    7 * n // 10,
                    4 * n // 5,
                    9 * n // 10,
                    n - 1,
                ]
                indices = sorted({min(i, n - 1) for i in indices})

                for i in indices:
                    e = events[i]
                    print(f"  Step {e.step:4d}: {e.value:12.6f}")

                # 傾向分析
                first_10 = [events[i].value for i in range(min(10, n))]
                last_10 = [events[i].value for i in range(max(0, n - 10), n)]
                first_avg = sum(first_10) / len(first_10)
                last_avg = sum(last_10) / len(last_10)

                if "reward" in tag.lower():
                    trend = "↑ 上昇" if last_avg > first_avg else "↓ 下降" if last_avg < first_avg else "→ 横ばい"
                    print(f"  Trend: {trend} (first 10 avg: {first_avg:.4f}, last 10 avg: {last_avg:.4f})")
                print()
        except Exception as e:
            print(f"{tag}: Error - {e}\n")

    # 報酬の詳細分析
    print("=== Reward Trend Analysis ===\n")
    try:
        reward_events = ea.Scalars("Train/mean_reward")
        if reward_events:
            rewards = [e.value for e in reward_events]
            steps = [e.step for e in reward_events]

            # 区間ごとの平均
            n = len(rewards)
            quarters = [
                ("Steps 0-125", rewards[: n // 4]),
                ("Steps 125-250", rewards[n // 4 : n // 2]),
                ("Steps 250-375", rewards[n // 2 : 3 * n // 4]),
                ("Steps 375-500", rewards[3 * n // 4 :]),
            ]

            print("Quarter-wise reward average:")
            prev_avg = None
            for name, vals in quarters:
                avg = sum(vals) / len(vals) if vals else 0
                change = ""
                if prev_avg is not None:
                    diff = avg - prev_avg
                    change = f" ({diff:+.4f})"
                print(f"  {name}: {avg:.4f}{change}")
                prev_avg = avg

            # 最大・最小
            max_reward = max(rewards)
            min_reward = min(rewards)
            max_step = steps[rewards.index(max_reward)]
            min_step = steps[rewards.index(min_reward)]

            print(f"\n  Max reward: {max_reward:.4f} at step {max_step}")
            print(f"  Min reward: {min_reward:.4f} at step {min_step}")
            print(f"  Final reward: {rewards[-1]:.4f}")

            # 収束判定
            last_50 = rewards[-50:] if len(rewards) >= 50 else rewards
            variance = sum((r - sum(last_50) / len(last_50)) ** 2 for r in last_50) / len(last_50)
            std = variance**0.5
            print(f"\n  Last 50 steps std: {std:.6f}")
            if std < 0.01:
                print("  → 収束している（std < 0.01）")
            elif std < 0.1:
                print("  → ほぼ収束（0.01 < std < 0.1）")
            else:
                print("  → まだ変動中（std > 0.1）、追加学習の余地あり")
    except Exception as e:
        print(f"Error analyzing rewards: {e}")


if __name__ == "__main__":
    exp_name = sys.argv[1] if len(sys.argv) > 1 else "droid-walking-v19"
    analyze_log(exp_name)
