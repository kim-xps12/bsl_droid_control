"""
方向別評価データの集計分析スクリプト

固定コマンドCSV（4方向 × 3 seed）から方向別の評価指標を集計し、
追従率・直交速度・Yawドリフト・姿勢安定性・歩容品質などをテーブル形式で出力する。

Usage:
    uv run python scripts/analyze_directional_eval.py 14 13 --prefix droid-walking-omni-v
    uv run python scripts/analyze_directional_eval.py 14 --prefix droid-walking-omni-v --vx-max 0.30 --vy-max 0.20
    uv run python scripts/analyze_directional_eval.py 14 13 --prefix droid-walking-omni-v --epoch 499
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from numpy.typing import NDArray


# 方向定義: (name, vx, vy, vyaw, cmd_axis, ortho_axis)
DIRECTIONS = [
    ("FWD", 1, 0, "x", "y"),
    ("BWD", -1, 0, "x", "y"),
    ("LFT", 0, 1, "y", "x"),
    ("RGT", 0, -1, "y", "x"),
]

SEEDS = [1, 2, 3]


def load_csv(path: str) -> tuple[list[str], NDArray[np.floating]]:
    """CSVをロードしてヘッダとデータを返す"""
    with open(path) as f:
        header = f.readline().strip().split(",")
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    return header, data


def col(header: list[str], data: NDArray[np.floating], name: str) -> NDArray[np.floating]:
    """カラム名からデータ列を取得"""
    idx = header.index(name)
    return data[:, idx]


def analyze_direction(
    header: list[str],
    data: NDArray[np.floating],
    cmd_axis: str,
    ortho_axis: str,
    cmd_speed: float,
    sign: int,
) -> dict[str, float]:
    """1試行分の方向別指標を算出する（定常状態 t>2s）"""
    dt = float(data[1, 0] - data[0, 0])
    start_idx = int(2.0 / dt)
    ss = data[start_idx:]
    ss_header = header  # ヘッダは共通

    # コマンド方向速度
    cmd_vel_raw = float(np.mean(col(ss_header, ss, f"base_vel_{cmd_axis}")))
    cmd_vel = cmd_vel_raw * sign  # BWD/RGTは符号反転して正値にする

    # 直交方向速度
    ortho_vel = float(np.mean(col(ss_header, ss, f"base_vel_{ortho_axis}")))

    # 最終Yaw角
    yaw_final = float(col(header, data, "yaw_deg")[-1])

    # Roll std
    roll_ss = col(ss_header, ss, "roll_deg")
    roll_std = float(np.std(roll_ss))

    # hip_pitch相関
    l_hp = col(ss_header, ss, "dof_pos_L_hip_pitch")
    r_hp = col(ss_header, ss, "dof_pos_R_hip_pitch")
    hip_corr = float(np.corrcoef(l_hp, r_hp)[0, 1])

    # 追従率
    tracking = cmd_vel / cmd_speed * 100.0

    # L/R接地比率
    l_contact = float(np.mean(col(ss_header, ss, "contact_left")))
    r_contact = float(np.mean(col(ss_header, ss, "contact_right")))
    contact_ratio = l_contact / r_contact if r_contact > 0 else float("inf")

    # hip_pitch range asymmetry
    l_range = float(np.max(l_hp) - np.min(l_hp))
    r_range = float(np.max(r_hp) - np.min(r_hp))
    mean_range = (l_range + r_range) / 2.0
    hip_asym = (l_range - r_range) / mean_range * 100.0 if mean_range > 0 else 0.0

    return {
        "cmd_vel": cmd_vel,
        "ortho_vel": ortho_vel,
        "yaw_final": yaw_final,
        "roll_std": roll_std,
        "hip_corr": hip_corr,
        "tracking": tracking,
        "contact_ratio": contact_ratio,
        "hip_asym": hip_asym,
    }


def print_direction_results(dname: str, results: list[dict[str, float]], cmd_speed: float) -> dict[str, float]:
    """1方向の3 seed平均を表示し、追従率meanを返す"""
    keys = ["cmd_vel", "ortho_vel", "yaw_final", "roll_std", "hip_corr", "tracking", "contact_ratio", "hip_asym"]
    arrays = {k: np.array([r[k] for r in results]) for k in keys}

    print(f"  {dname}:")
    print(f"    cmd_vel: {arrays['cmd_vel'].mean():.4f} ± {arrays['cmd_vel'].std():.4f} m/s")
    print(f"    ortho_vel: {arrays['ortho_vel'].mean():.4f} ± {arrays['ortho_vel'].std():.4f} m/s")
    print(f"    yaw_final: {arrays['yaw_final'].mean():.1f} ± {arrays['yaw_final'].std():.1f} deg")
    print(f"    roll_std: {arrays['roll_std'].mean():.2f} ± {arrays['roll_std'].std():.2f} deg")
    print(f"    hip_pitch_corr: {arrays['hip_corr'].mean():.3f} ± {arrays['hip_corr'].std():.3f}")
    print(f"    tracking_rate: {arrays['tracking'].mean():.1f} ± {arrays['tracking'].std():.1f} %")
    print(f"    contact_ratio L/R: {arrays['contact_ratio'].mean():.2f}")
    print(f"    hip_pitch_asym: {arrays['hip_asym'].mean():+.1f}%")

    return {
        "tracking_mean": float(arrays["tracking"].mean()),
        "tracking_std": float(arrays["tracking"].std()),
    }


def analyze_version(
    version: int,
    prefix: str,
    epoch: str,
    vx_max: float,
    vy_max: float,
) -> dict[str, dict[str, float]] | None:
    """1バージョンの全方向分析を実行"""
    log_dir = Path(f"logs/{prefix}{version}")
    if not log_dir.exists():
        print(f"\n[WARNING] ディレクトリが見つかりません: {log_dir}")
        return None

    print(f"\n===== V{version} 方向別評価結果 =====")

    dir_tracking: dict[str, dict[str, float]] = {}

    for dname, vx_sign, vy_sign, cmd_axis, ortho_axis in DIRECTIONS:
        # 速度値の決定
        if cmd_axis == "x":
            cmd_speed = vx_max
            vx = vx_max * vx_sign
            vy = 0.0
        else:
            cmd_speed = vy_max
            vx = 0.0
            vy = vy_max * vy_sign

        sign = 1 if (vx_sign > 0 or vy_sign > 0) else -1

        results: list[dict[str, float]] = []
        for seed in SEEDS:
            fname = log_dir / f"eval_{epoch}_cmd_{vx:.2f}_{vy:.2f}_0.00_s{seed}.csv"
            if not fname.exists():
                print(f"  {dname} seed {seed}: FILE NOT FOUND ({fname})")
                continue
            header, data = load_csv(str(fname))
            r = analyze_direction(header, data, cmd_axis, ortho_axis, cmd_speed, sign)
            results.append(r)

        if results:
            summary = print_direction_results(dname, results, cmd_speed)
            dir_tracking[dname] = summary
        else:
            print(f"  {dname}: データなし")

    return dir_tracking


def print_balance_summary(tracking: dict[str, dict[str, float]]) -> None:
    """方向間バランスサマリを出力"""
    if len(tracking) < 2:
        return

    means = [v["tracking_mean"] for v in tracking.values()]
    arr = np.array(means)

    print("\n  方向間バランス:")
    print(f"    追従率 mean（全方向平均）: {arr.mean():.1f}%")
    print(f"    追従率 std（方向間ばらつき）: {arr.std():.1f}%")

    if "FWD" in tracking and "BWD" in tracking:
        diff_fb = tracking["FWD"]["tracking_mean"] - tracking["BWD"]["tracking_mean"]
        print(f"    FWD/BWD追従率差: {diff_fb:+.1f}%")
    if "LFT" in tracking and "RGT" in tracking:
        diff_lr = tracking["LFT"]["tracking_mean"] - tracking["RGT"]["tracking_mean"]
        print(f"    LFT/RGT追従率差: {diff_lr:+.1f}%")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="方向別評価データの集計分析（複数バージョン比較対応）",
    )
    parser.add_argument(
        "versions",
        type=int,
        nargs="+",
        help="分析するバージョン番号（1つ以上）。例: 14 13",
    )
    parser.add_argument(
        "--epoch",
        type=str,
        default="3999",
        help="評価エポック番号（デフォルト: 3999）",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="droid-walking-omni-v",
        help="Experiment name prefix (default: droid-walking-omni-v)",
    )
    parser.add_argument(
        "--vx-max",
        type=float,
        default=0.30,
        help="FWD/BWD方向のコマンド速度上限（デフォルト: 0.30）",
    )
    parser.add_argument(
        "--vy-max",
        type=float,
        default=0.30,
        help="LFT/RGT方向のコマンド速度上限（デフォルト: 0.30）",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    for v in args.versions:
        tracking = analyze_version(v, args.prefix, args.epoch, args.vx_max, args.vy_max)
        if tracking:
            print_balance_summary(tracking)
        print()


if __name__ == "__main__":
    main()
