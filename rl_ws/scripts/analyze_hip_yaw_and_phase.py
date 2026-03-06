#!/usr/bin/env python3
"""Fixed command CSV向け: hip_yaw非対称, Roll-接地位相, 高周波時間推移の分析.

Lead agentの追加データ要求に対応:
1. hip_yaw L/R action・位置の接地相/遊脚相別分析
2. hip_roll FFTスペクトル (歩行周波数付近のピーク)
3. Roll角と接地切替の位相関係
4. 指定関節の高周波成分の前半/後半比較 (カスケードリスク評価)

使用例:
    uv run python scripts/analyze_hip_yaw_and_phase.py 11 10 --prefix droid-walking-omni-v
"""

import argparse
import csv
import os
import sys
from pathlib import Path

import numpy as np


def load_csv(prefix: str, version: int, epoch: int, cmd_suffix: str) -> list[dict]:
    """固定コマンドCSVを読み込む."""
    fname = f"eval_{epoch}_{cmd_suffix}.csv"
    path = Path("logs") / f"{prefix}{version}" / fname
    if not path.exists():
        print(f"  [SKIP] {path} not found")
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def analyze_version(
    rows: list[dict], label: str, dt: float = 0.02, steady_start_s: float = 2.0
) -> None:
    """1バージョン分の分析を実行."""
    idx_start = int(steady_start_s / dt)
    data = rows[idx_start:]
    n = len(data)
    if n == 0:
        print(f"  [WARN] No steady-state data for {label}")
        return

    # --- 配列変換 ---
    a0 = np.array([float(r["action_0"]) for r in data])  # L_hip_yaw
    a5 = np.array([float(r["action_5"]) for r in data])  # R_hip_yaw
    l_hip_yaw = np.array([float(r["dof_pos_L_hip_yaw"]) for r in data])
    r_hip_yaw = np.array([float(r["dof_pos_R_hip_yaw"]) for r in data])
    l_hr = np.array([float(r["dof_pos_L_hip_roll"]) for r in data])
    r_hr = np.array([float(r["dof_pos_R_hip_roll"]) for r in data])
    roll = np.array([float(r["roll_deg"]) for r in data])
    cl = np.array([float(r["contact_left"]) for r in data])
    cr = np.array([float(r["contact_right"]) for r in data])
    l_knee = np.array([float(r["dof_pos_L_knee_pitch"]) for r in data])
    r_ankle = np.array([float(r["dof_pos_R_ankle_pitch"]) for r in data])

    l_stance = cl > 0.5
    r_stance = cr > 0.5

    print(f"\n{'='*60}")
    print(f"[{label}] hip_yaw非対称分析 (t>{steady_start_s}s, n={n})")
    print(f"{'='*60}")
    print(f"  L hip_yaw action(a0): mean={a0.mean():.4f}, std={a0.std():.4f}")
    print(f"  R hip_yaw action(a5): mean={a5.mean():.4f}, std={a5.std():.4f}")
    print(
        f"  L hip_yaw pos: mean={np.degrees(l_hip_yaw.mean()):.2f} deg, "
        f"std={np.degrees(l_hip_yaw.std()):.2f} deg"
    )
    print(
        f"  R hip_yaw pos: mean={np.degrees(r_hip_yaw.mean()):.2f} deg, "
        f"std={np.degrees(r_hip_yaw.std()):.2f} deg"
    )
    if l_stance.sum() > 0 and (~l_stance).sum() > 0:
        print(
            f"  L hip_yaw stance={np.degrees(l_hip_yaw[l_stance].mean()):.2f} deg, "
            f"swing={np.degrees(l_hip_yaw[~l_stance].mean()):.2f} deg"
        )
    if r_stance.sum() > 0 and (~r_stance).sum() > 0:
        print(
            f"  R hip_yaw stance={np.degrees(r_hip_yaw[r_stance].mean()):.2f} deg, "
            f"swing={np.degrees(r_hip_yaw[~r_stance].mean()):.2f} deg"
        )

    # hip_yaw L/R差分 → Yawモーメント寄与推定
    yaw_diff_mean = np.degrees(l_hip_yaw.mean() - r_hip_yaw.mean())
    print(f"  L-R hip_yaw mean差: {yaw_diff_mean:+.2f} deg (正=左偏向)")

    # --- hip_roll FFT ---
    print(f"\n{'='*60}")
    print(f"[{label}] hip_roll FFTスペクトル")
    print(f"{'='*60}")
    freqs = np.fft.rfftfreq(n, dt)
    for side, hr in [("L", l_hr), ("R", r_hr)]:
        hr_d = hr - np.mean(hr)
        fft_mag = np.abs(np.fft.rfft(hr_d))
        top5_idx = np.argsort(fft_mag[1:])[-5:][::-1] + 1
        print(f"  {side} hip_roll top5 peaks:")
        for i in top5_idx:
            print(f"    {freqs[i]:.2f} Hz  mag={fft_mag[i]:.1f}")
        # 1.5Hz付近のピーク
        mask_15 = (freqs > 1.3) & (freqs < 1.7)
        if mask_15.any():
            peak_15 = fft_mag[mask_15].max()
            peak_15_freq = freqs[mask_15][np.argmax(fft_mag[mask_15])]
            print(f"    -> 1.5Hz帯ピーク: {peak_15_freq:.2f}Hz, mag={peak_15:.1f}")

    # --- Roll-接地位相関係 ---
    print(f"\n{'='*60}")
    print(f"[{label}] Roll-接地切替位相関係")
    print(f"{'='*60}")
    csw_l = np.diff(cl.astype(int))
    csw_r = np.diff(cr.astype(int))
    for event, indices in [
        ("L lift-off", np.where(csw_l == -1)[0]),
        ("L landing", np.where(csw_l == 1)[0]),
        ("R lift-off", np.where(csw_r == -1)[0]),
        ("R landing", np.where(csw_r == 1)[0]),
    ]:
        if len(indices) > 0:
            r_vals = roll[indices]
            print(
                f"  Roll at {event}: mean={r_vals.mean():.2f} deg, "
                f"std={r_vals.std():.2f} deg (n={len(indices)})"
            )

    # --- 高周波成分の時間推移 ---
    print(f"\n{'='*60}")
    print(f"[{label}] 高周波成分の前半/後半比較 (カスケードリスク評価)")
    print(f"{'='*60}")
    half = n // 2
    for name, jdata in [("L_knee_pitch", l_knee), ("R_ankle_pitch", r_ankle)]:
        for plabel, seg in [
            (f"前半({steady_start_s:.0f}-{steady_start_s + half*dt:.0f}s)", jdata[:half]),
            (f"後半({steady_start_s + half*dt:.0f}-{steady_start_s + n*dt:.0f}s)", jdata[half:]),
        ]:
            seg_d = seg - np.mean(seg)
            fft_s = np.abs(np.fft.rfft(seg_d))
            freqs_s = np.fft.rfftfreq(len(seg_d), dt)
            hf_mask = freqs_s > 2.0
            total_e = np.sum(fft_s[1:] ** 2)
            hf_e = np.sum(fft_s[hf_mask] ** 2) if hf_mask.any() else 0
            hf_r = hf_e / total_e * 100 if total_e > 0 else 0
            vel_rms = np.sqrt(np.mean(np.diff(seg / dt) ** 2)) if len(seg) > 1 else 0
            print(f"  {name} {plabel}: >2Hz={hf_r:.1f}%, vel_rms={vel_rms:.2f}")
    print()


def main() -> None:
    parser = argparse.ArgumentParser(description="hip_yaw非対称・Roll位相・高周波時間推移分析")
    parser.add_argument("versions", type=int, nargs="+", help="バージョン番号 (先頭が新)")
    parser.add_argument("--prefix", default="droid-walking-omni-v", help="実験名プレフィックス")
    parser.add_argument("--epoch", type=str, default="3999", help="チェックポイント番号またはファイルサフィックス")
    parser.add_argument(
        "--cmd-suffix",
        default="cmd_0.30_0.00_0.00_s1",
        help="固定コマンドCSVサフィックス",
    )
    args = parser.parse_args()

    for ver in args.versions:
        rows = load_csv(args.prefix, ver, args.epoch, args.cmd_suffix)
        if rows:
            analyze_version(rows, f"V{ver}")


if __name__ == "__main__":
    main()
