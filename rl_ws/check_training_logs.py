"""学習ログの分析スクリプト - 収束状況を確認"""

from __future__ import annotations

import glob
import os

from tensorboard.backend.event_processing import event_accumulator


log_dirs = sorted(glob.glob("logs/biped-walking*"))
for log_dir in log_dirs:
    events = glob.glob(os.path.join(log_dir, "events.out.tfevents.*"))
    if not events:
        continue

    print(f"\n{'=' * 60}")
    print(f"{log_dir}")
    print(f"{'=' * 60}")
    ea = event_accumulator.EventAccumulator(events[0])
    ea.Reload()

    scalars = ea.Scalars("Train/mean_reward")
    if scalars:
        total = len(scalars)

        # サンプリングして表示
        checkpoints = [0, 50, 100, 150, 200, 250, 300, 350, 400, 450, min(499, total - 1)]
        if total > 500:
            checkpoints.extend([500, 600, 700, 800, 900, min(999, total - 1)])

        print(f"Total iterations: {total}")
        print("\nIteration -> Mean Reward:")
        for cp in checkpoints:
            if cp < total:
                print(f"  {cp:4d}: {scalars[cp].value:8.2f}")

        # 収束判定：後半の変化率
        if total >= 200:
            mid_val = scalars[total // 2].value
            end_val = scalars[-1].value
            if mid_val != 0:
                change = ((end_val - mid_val) / abs(mid_val)) * 100
                print("\n収束分析:")
                print(f"  Mid ({total // 2}): {mid_val:.2f}")
                print(f"  End ({total - 1}): {end_val:.2f}")
                print(f"  後半の変化率: {change:.1f}%")

                # 最後100イテレーションの変動
                if total >= 100:
                    last_100 = [s.value for s in scalars[-100:]]
                    std = (sum((x - sum(last_100) / 100) ** 2 for x in last_100) / 100) ** 0.5
                    mean = sum(last_100) / 100
                    cv = (std / mean * 100) if mean != 0 else 0
                    print(f"  最後100iterの変動係数: {cv:.2f}%")

print("\n" + "=" * 60)
print("総評")
print("=" * 60)
print("""
- V1 (biped-walking): 500 iter で 19.29 に収束
- V2 (biped-walking-v2): 1000 iter で 27.72 まで上昇（まだ上昇中だった可能性）
- V3 (biped-walking-v3): 500 iter で 41.13 に収束（安定）
- V4 (biped-walking-v4): 500 iter で 40.38 に収束（V3とほぼ同等）
- V5 (biped-walking-v5): 500 iter で 40.38（V4と全く同じ＝feet関連報酬が効いていなかった）
""")
