"""
横方向並進揺れの詳細分析スクリプト

Roll振動の周期性、横方向並進変位、Roll-横方向相関、接地相/遊脚相別hip_roll分析など、
横方向バランスに特化した深い定量分析を行い、複数バージョン間で比較する。

analyze_eval_csv.py の Section 7 (Roll/Lateral Sway) を補完する位置づけ。

Usage:
    uv run python scripts/analyze_lateral_sway.py 14 13 --prefix droid-walking-narrow-v
    uv run python scripts/analyze_lateral_sway.py 14 13 4 6 --prefix droid-walking-narrow-v
    uv run python scripts/analyze_lateral_sway.py 14 13 --epoch 300 --prefix droid-walking-narrow-v
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from numpy.typing import NDArray


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


def analyze_roll_periodicity(roll: NDArray[np.floating], dt: float, label: str) -> dict[str, float]:
    """Roll振動の周期性分析（自己相関）"""
    print(f"\n{'=' * 60}")
    print(f"[{label}] Roll振動の周期性分析")
    print(f"{'=' * 60}")

    roll_centered = roll - np.mean(roll)
    n = len(roll_centered)

    # 自己相関（正規化）
    autocorr_full = np.correlate(roll_centered, roll_centered, mode="full")
    autocorr = autocorr_full[n - 1 :] / autocorr_full[n - 1]  # 正規化

    # 最初のピーク検出（lag > 0.3秒）
    min_lag = int(0.3 / dt)
    max_lag = min(int(3.0 / dt), len(autocorr) - 1)
    search_region = autocorr[min_lag:max_lag]

    if len(search_region) > 0:
        peak_idx_local = np.argmax(search_region)
        peak_lag = min_lag + peak_idx_local
        peak_value = autocorr[peak_lag]
        period = peak_lag * dt
        frequency = 1.0 / period if period > 0 else 0.0
    else:
        peak_lag = 0
        peak_value = 0.0
        period = 0.0
        frequency = 0.0

    # Roll角速度 RMS（deg/s）
    roll_vel = np.diff(roll) / dt
    roll_vel_rms = float(np.sqrt(np.mean(roll_vel**2)))

    print("  自己相関 第1ピーク:")
    print(f"    lag = {peak_lag} steps ({period:.3f} s)")
    print(f"    値 = {peak_value:.3f} (1.0=完全周期, 0=非周期)")
    print(f"    推定周波数 = {frequency:.2f} Hz")
    print(f"  Roll std: {np.std(roll):.2f} deg")
    print(f"  Roll peak-to-peak: {np.max(roll) - np.min(roll):.2f} deg")
    print(f"  Roll角速度 RMS: {roll_vel_rms:.1f} deg/s")

    periodicity = "高周期性" if peak_value > 0.7 else ("中周期性" if peak_value > 0.4 else "低周期性")
    print(f"  判定: {periodicity}")

    return {
        "roll_std": float(np.std(roll)),
        "roll_p2p": float(np.max(roll) - np.min(roll)),
        "roll_vel_rms": roll_vel_rms,
        "autocorr_peak": float(peak_value),
        "autocorr_period": period,
        "autocorr_freq": frequency,
    }


def analyze_detrended_lateral(
    pos_y: NDArray[np.floating], vel_y: NDArray[np.floating], timestamps: NDArray[np.floating], label: str
) -> dict[str, float]:
    """横方向並進変位の分析（線形ドリフト除去）"""
    print(f"\n{'=' * 60}")
    print(f"[{label}] 横方向並進変位分析（detrended）")
    print(f"{'=' * 60}")

    # 線形トレンド除去
    coeffs = np.polyfit(timestamps, pos_y, 1)
    trend = np.polyval(coeffs, timestamps)
    detrended = pos_y - trend
    drift_rate = coeffs[0]

    # mm単位に変換
    detrended_mm = detrended * 1000
    std_mm = float(np.std(detrended_mm))
    p2p_mm = float(np.max(detrended_mm) - np.min(detrended_mm))
    vel_y_std = float(np.std(vel_y))

    print(f"  横方向ドリフト速度: {drift_rate * 1000:.1f} mm/s")
    print("  detrended 横方向位置:")
    print(f"    std = {std_mm:.1f} mm")
    print(f"    peak-to-peak = {p2p_mm:.1f} mm")
    print("  Y速度:")
    print(f"    mean = {np.mean(vel_y):.4f} m/s")
    print(f"    std = {vel_y_std:.4f} m/s")

    return {
        "lateral_drift_rate": float(drift_rate * 1000),
        "lateral_detrended_std": std_mm,
        "lateral_detrended_p2p": p2p_mm,
        "vel_y_std": vel_y_std,
    }


def analyze_roll_lateral_correlation(
    roll: NDArray[np.floating],
    pos_y: NDArray[np.floating],
    timestamps: NDArray[np.floating],
    base_height: float,
    dt: float,
    label: str,
) -> dict[str, float]:
    """Roll角と横方向位置の相関分析"""
    print(f"\n{'=' * 60}")
    print(f"[{label}] Roll-横方向相関分析")
    print(f"{'=' * 60}")

    # detrend pos_y
    coeffs_y = np.polyfit(timestamps, pos_y, 1)
    pos_y_detrended = pos_y - np.polyval(coeffs_y, timestamps)
    pos_y_mm = pos_y_detrended * 1000

    # R² (決定係数)
    min_len = min(len(roll), len(pos_y_mm))
    roll_trimmed = roll[:min_len]
    pos_y_trimmed = pos_y_mm[:min_len]

    corr = np.corrcoef(roll_trimmed, pos_y_trimmed)[0, 1]
    r_squared = corr**2

    # 回帰係数（slope: mm/deg）
    slope, intercept = np.polyfit(roll_trimmed, pos_y_trimmed, 1)

    # 幾何学的予測slope（CoM高さに基づく剛体近似）
    # 胴体がφ[rad]傾くと、CoMはh*sin(φ)≈h*φ[rad]だけ横方向に移動
    # 1 deg = π/180 rad → h * (π/180) * 1000 mm/deg
    geometric_slope = -base_height * (np.pi / 180) * 1000

    # 残差分析
    predicted = slope * roll_trimmed + intercept
    residual = pos_y_trimmed - predicted
    residual_std = float(np.std(residual))

    # Roll成分のstd
    roll_component_std = float(np.std(predicted))

    # 相互相関（位相遅れの推定）
    roll_norm = roll_trimmed - np.mean(roll_trimmed)
    pos_norm = pos_y_trimmed - np.mean(pos_y_trimmed)
    xcorr = np.correlate(roll_norm, pos_norm, mode="full")
    lags = np.arange(-min_len + 1, min_len)
    peak_lag_idx = np.argmax(np.abs(xcorr))
    peak_lag = lags[peak_lag_idx]
    phase_delay = peak_lag * dt

    print(f"  R² (決定係数): {r_squared:.3f}")
    print(f"    → Rollが横方向運動の {r_squared * 100:.1f}% を説明")
    print(f"  回帰slope: {slope:.2f} mm/deg (実測)")
    print(f"  幾何学的slope: {geometric_slope:.2f} mm/deg (理論値, h={base_height:.3f}m)")
    print(f"  増幅率: {abs(slope / geometric_slope):.2f}x")
    print(f"  Roll成分 std: {roll_component_std:.1f} mm")
    print(f"  残差 std: {residual_std:.1f} mm")
    print(f"  相互相関ピーク: lag={peak_lag} steps ({phase_delay:.3f}s)")

    amplification = abs(slope / geometric_slope) if geometric_slope != 0 else 0
    if amplification > 1.3:
        print(f"    → 横方向変位がRollから{amplification:.1f}倍に増幅（能動バランス不足の兆候）")
    elif amplification < 0.8:
        print(f"    → 横方向変位がRollから{amplification:.1f}倍に減衰（能動バランスが機能）")
    else:
        print("    → 幾何学的予測値とほぼ一致")

    return {
        "r_squared": float(r_squared),
        "slope": float(slope),
        "geometric_slope": float(geometric_slope),
        "amplification": float(amplification),
        "roll_component_std": roll_component_std,
        "residual_std": residual_std,
        "cross_corr_lag": float(phase_delay),
    }


def analyze_stance_swing_hip_roll(
    header: list[str],
    data: NDArray[np.floating],
    start_idx: int,
    dt: float,
    label: str,
) -> dict[str, float]:
    """接地相/遊脚相別hip_roll分析"""
    print(f"\n{'=' * 60}")
    print(f"[{label}] 接地相/遊脚相別hip_roll分析")
    print(f"{'=' * 60}")

    contact_l = col(header, data, "contact_left")[start_idx:]
    contact_r = col(header, data, "contact_right")[start_idx:]
    l_hip_roll = col(header, data, "dof_pos_L_hip_roll")[start_idx:]
    r_hip_roll = col(header, data, "dof_pos_R_hip_roll")[start_idx:]

    deg = 180.0 / np.pi

    results: dict[str, float] = {}

    for side, contact, hip_roll, sign_name in [
        ("L", contact_l, l_hip_roll, "内向き=負"),
        ("R", contact_r, r_hip_roll, "内向き=正"),
    ]:
        stance_mask = contact == 1
        swing_mask = contact == 0

        stance_vals = hip_roll[stance_mask] * deg if np.any(stance_mask) else np.array([0.0])
        swing_vals = hip_roll[swing_mask] * deg if np.any(swing_mask) else np.array([0.0])

        stance_mean = float(np.mean(stance_vals))
        swing_mean = float(np.mean(swing_vals))
        delta = abs(stance_mean - swing_mean)

        print(f"\n  {side} hip_roll ({sign_name}):")
        print(f"    接地相 mean: {stance_mean:+.1f} deg (n={np.sum(stance_mask)})")
        print(f"    遊脚相 mean: {swing_mean:+.1f} deg (n={np.sum(swing_mask)})")
        print(f"    接地-遊脚 デルタ: {delta:.1f} deg (「バランスストローク」)")
        print(f"    全体 range: [{np.min(hip_roll) * deg:.1f}, {np.max(hip_roll) * deg:.1f}] deg")

        # 外向き方向の利用状況
        if side == "L":
            outward_max = float(np.max(hip_roll) * deg)  # L: 正=外向き
            print(f"    外向き最大値: {outward_max:+.1f} deg", end="")
            if outward_max < 1.0:
                print(" ← 外向き修正を放棄")
            else:
                print(" (外向き修正活用中)")
            results["l_outward_max"] = outward_max
        else:
            outward_min = float(np.min(hip_roll) * deg)  # R: 負=外向き
            print(f"    外向き最大値: {outward_min:+.1f} deg", end="")
            if outward_min > -1.0:
                print(" ← 外向き修正を放棄")
            else:
                print(" (外向き修正活用中)")
            results["r_outward_min"] = outward_min

        results[f"{side.lower()}_stance_mean"] = stance_mean
        results[f"{side.lower()}_swing_mean"] = swing_mean
        results[f"{side.lower()}_delta"] = delta

    return results


def analyze_single_version(header: list[str], data: NDArray[np.floating], label: str) -> dict[str, float]:
    """1バージョンの横方向揺れ分析を実行"""
    dt = float(data[1, 0] - data[0, 0])
    start_idx = int(2.0 / dt)

    roll = col(header, data, "roll_deg")[start_idx:]
    pos_y = col(header, data, "base_pos_y")[start_idx:]
    vel_y = col(header, data, "base_vel_y")[start_idx:]
    timestamps = col(header, data, "timestamp")[start_idx:]
    pos_z = col(header, data, "base_pos_z")[start_idx:]
    base_height = float(np.mean(pos_z))

    results: dict[str, float] = {}

    # 1. Roll周期性分析
    roll_results = analyze_roll_periodicity(roll, dt, label)
    results.update(roll_results)

    # 2. 横方向並進変位分析
    lateral_results = analyze_detrended_lateral(pos_y, vel_y, timestamps, label)
    results.update(lateral_results)

    # 3. Roll-横方向相関分析
    corr_results = analyze_roll_lateral_correlation(roll, pos_y, timestamps, base_height, dt, label)
    results.update(corr_results)

    # 4. 接地相/遊脚相別hip_roll分析
    hip_roll_results = analyze_stance_swing_hip_roll(header, data, start_idx, dt, label)
    results.update(hip_roll_results)

    results["base_height"] = base_height

    return results


def print_comparison_table(all_results: list[tuple[str, dict[str, float]]]) -> None:
    """全バージョンの比較テーブルを出力"""
    print(f"\n{'#' * 70}")
    print("# 横方向揺れ 比較サマリ")
    print(f"{'#' * 70}")

    labels = [label for label, _ in all_results]
    results_list = [r for _, r in all_results]

    # テーブル行定義: (表示名, キー, フォーマット, 単位)
    rows: list[tuple[str, str, str, str]] = [
        ("Roll std", "roll_std", ".2f", "deg"),
        ("Roll p2p", "roll_p2p", ".2f", "deg"),
        ("Roll角速度 RMS", "roll_vel_rms", ".1f", "deg/s"),
        ("Roll自己相関ピーク", "autocorr_peak", ".3f", ""),
        ("Roll周波数", "autocorr_freq", ".2f", "Hz"),
        ("横方向 detrended std", "lateral_detrended_std", ".1f", "mm"),
        ("横方向 detrended p2p", "lateral_detrended_p2p", ".1f", "mm"),
        ("横方向ドリフト速度", "lateral_drift_rate", ".1f", "mm/s"),
        ("Y速度 std", "vel_y_std", ".4f", "m/s"),
        ("Roll-横方向 R²", "r_squared", ".3f", ""),
        ("Roll-横方向 slope", "slope", ".2f", "mm/deg"),
        ("幾何学的slope", "geometric_slope", ".2f", "mm/deg"),
        ("増幅率", "amplification", ".2f", "x"),
        ("Roll成分 std", "roll_component_std", ".1f", "mm"),
        ("残差 std", "residual_std", ".1f", "mm"),
        ("L stance hip_roll", "l_stance_mean", "+.1f", "deg"),
        ("R stance hip_roll", "r_stance_mean", "+.1f", "deg"),
        ("Lバランスストローク", "l_delta", ".1f", "deg"),
        ("Rバランスストローク", "r_delta", ".1f", "deg"),
        ("base_height", "base_height", ".3f", "m"),
    ]

    # ヘッダ行
    col_width = 12
    header_str = f"{'指標':<24s}"
    for label in labels:
        header_str += f" | {label:>{col_width}s}"
    print(f"\n{header_str}")
    print("-" * len(header_str))

    # データ行
    for display_name, key, fmt, unit in rows:
        row_str = f"{display_name:<24s}"
        for r in results_list:
            val = r.get(key)
            if val is not None:
                formatted = f"{val:{fmt}}"
                if unit:
                    formatted += f" {unit}"
                row_str += f" | {formatted:>{col_width}s}"
            else:
                row_str += f" | {'N/A':>{col_width}s}"
        print(row_str)

    # 変化率（先頭2バージョン）
    if len(all_results) >= 2:
        new_label, new_r = all_results[0]
        old_label, old_r = all_results[1]
        print(f"\n{'=' * 60}")
        print(f"{new_label} vs {old_label} 変化率")
        print(f"{'=' * 60}")

        change_rows: list[tuple[str, str, str]] = [
            ("Roll std", "roll_std", "lower_better"),
            ("Roll角速度 RMS", "roll_vel_rms", "lower_better"),
            ("横方向 detrended std", "lateral_detrended_std", "lower_better"),
            ("Y速度 std", "vel_y_std", "lower_better"),
            ("Roll-横方向 R²", "r_squared", "neutral"),
            ("増幅率", "amplification", "lower_better"),
            ("Lバランスストローク", "l_delta", "higher_better"),
            ("Rバランスストローク", "r_delta", "higher_better"),
        ]

        for display_name, key, direction in change_rows:
            new_val = new_r.get(key)
            old_val = old_r.get(key)
            if new_val is not None and old_val is not None and old_val != 0:
                pct = (new_val - old_val) / abs(old_val) * 100
                if direction == "lower_better":
                    indicator = "改善" if pct < 0 else "悪化"
                elif direction == "higher_better":
                    indicator = "改善" if pct > 0 else "悪化"
                else:
                    indicator = ""
                print(f"  {display_name}: {pct:+.1f}% {indicator}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="横方向並進揺れの詳細分析（複数バージョン比較）",
    )
    parser.add_argument(
        "versions",
        type=int,
        nargs="+",
        help="比較するバージョン番号（2つ以上）。先頭が新バージョン。例: 14 13",
    )
    parser.add_argument(
        "--epoch",
        type=int,
        default=499,
        help="評価エポック番号（デフォルト: 499）",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="droid-walking-unitree-v",
        help="Experiment name prefix (default: droid-walking-unitree-v)",
    )
    args = parser.parse_args()
    if len(args.versions) < 2:
        parser.error("バージョン番号は2つ以上指定してください")
    return args


def main() -> None:
    args = parse_args()

    all_results: list[tuple[str, dict[str, float]]] = []

    for v in args.versions:
        label = f"V{v}"
        path = f"logs/{args.prefix}{v}/eval_{args.epoch}.csv"

        # ファイル存在チェック
        if not Path(path).exists():
            print(f"\n[WARNING] {path} が見つかりません。V{v} をスキップします。")
            continue

        print(f"\n{'#' * 70}")
        print(f"# {label} 横方向揺れ詳細分析")
        print(f"{'#' * 70}")

        header, data = load_csv(path)
        print(f"  データ: {len(data)} rows, dt={data[1, 0] - data[0, 0]:.4f}s")

        results = analyze_single_version(header, data, label)
        all_results.append((label, results))

    if len(all_results) >= 2:
        print_comparison_table(all_results)


if __name__ == "__main__":
    main()
