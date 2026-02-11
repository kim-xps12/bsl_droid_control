"""
クロスバージョン トレンド分析スクリプト

複数バージョンのeval CSVから標準化指標を一括計算し、Markdownトレンドテーブルを出力する。
各バージョンの歩行品質・安定性・報酬を横断的に比較し、改善/悪化トレンドを可視化する。

Usage:
    uv run python scripts/analyze_cross_version_trend.py 5 6 7 8 --prefix droid-walking-omni-v
    uv run python scripts/analyze_cross_version_trend.py 4 5 6 7 8 --prefix droid-walking-omni-v --include-rewards
    uv run python scripts/analyze_cross_version_trend.py 5 6 7 8 --prefix droid-walking-omni-v --epoch 3999
"""

from __future__ import annotations

import argparse
import os
import sys
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


def resolve_csv_path(prefix: str, version: int, epoch: int) -> str:
    """eval CSVのパスを自動解決する。指定epochを優先し、見つからない場合はフォールバック。"""
    candidates = [epoch, 3999, 1999, 499]
    # 指定epochを先頭に移動（重複回避）
    seen: set[int] = set()
    ordered: list[int] = []
    for e in candidates:
        if e not in seen:
            ordered.append(e)
            seen.add(e)

    for e in ordered:
        path = f"logs/{prefix}{version}/eval_{e}.csv"
        if os.path.exists(path):
            return path

    # 見つからない場合は元のパスを返す（エラーメッセージ用）
    return f"logs/{prefix}{version}/eval_{epoch}.csv"


def compute_metrics(header: list[str], data: NDArray[np.floating]) -> dict[str, float]:
    """1バージョンのeval CSVから全標準化指標を計算する。

    定常状態（t > 2s）のみを使用。
    """
    dt = float(data[1, 0] - data[0, 0])
    start_idx = int(2.0 / dt)

    # 定常状態のデータ
    ss = data[start_idx:]
    ss_header = header  # ヘッダは同じ

    results: dict[str, float] = {}

    # --- 1. hip_pitch相関 ---
    l_hip_pitch = col(ss_header, ss, "dof_pos_L_hip_pitch")
    r_hip_pitch = col(ss_header, ss, "dof_pos_R_hip_pitch")
    results["hip_pitch_corr"] = float(np.corrcoef(l_hip_pitch, r_hip_pitch)[0, 1])

    # --- 2. Roll std ---
    roll_deg = col(ss_header, ss, "roll_deg")
    results["roll_std_deg"] = float(np.std(roll_deg))

    # --- 3. 横方向 detrended std ---
    pos_y = col(ss_header, ss, "base_pos_y")
    timestamps = col(ss_header, ss, "timestamp")
    # 線形ドリフト除去
    coeffs = np.polyfit(timestamps, pos_y, 1)
    trend = np.polyval(coeffs, timestamps)
    detrended = pos_y - trend
    results["lateral_detrended_std_mm"] = float(np.std(detrended) * 1000)

    # --- 4. 増幅率 ---
    base_height = float(np.mean(col(ss_header, ss, "base_pos_z")))
    roll_rad = roll_deg * np.pi / 180.0
    geometric_slope = base_height * 1000 / (180.0 / np.pi)  # mm/deg
    # 回帰
    detrended_mm = detrended * 1000
    roll_centered = roll_deg - np.mean(roll_deg)
    if np.std(roll_centered) > 1e-6:
        slope = float(np.polyfit(roll_centered, detrended_mm, 1)[0])
        amplification = abs(slope / geometric_slope) if geometric_slope != 0 else 0.0
    else:
        amplification = 0.0
    results["amplification"] = amplification

    # --- 5. 遷移レート ---
    contact_l = col(ss_header, ss, "contact_left")
    contact_r = col(ss_header, ss, "contact_right")
    transitions_l = int(np.sum(np.abs(np.diff(contact_l)) > 0.5))
    transitions_r = int(np.sum(np.abs(np.diff(contact_r)) > 0.5))
    duration = float(timestamps[-1] - timestamps[0])
    results["transition_rate"] = (transitions_l + transitions_r) / duration if duration > 0 else 0.0

    # --- 6. L/R歩行周波数 ---
    for side in ["L", "R"]:
        hip_pitch = col(ss_header, ss, f"dof_pos_{side}_hip_pitch")
        sig = hip_pitch - np.mean(hip_pitch)
        freqs = np.fft.rfftfreq(len(sig), d=dt)
        fft_mag = np.abs(np.fft.rfft(sig))
        fft_mag[0] = 0  # DC成分除外
        peak_idx = int(np.argmax(fft_mag))
        results[f"{side.lower()}_walk_freq_hz"] = float(freqs[peak_idx])

    # --- 7. stance hip_roll ---
    l_hip_roll = col(ss_header, ss, "dof_pos_L_hip_roll")
    r_hip_roll = col(ss_header, ss, "dof_pos_R_hip_roll")
    deg_conv = 180.0 / np.pi

    l_stance_mask = contact_l == 1
    r_stance_mask = contact_r == 1

    if np.any(l_stance_mask):
        results["l_stance_hip_roll_deg"] = float(np.mean(l_hip_roll[l_stance_mask]) * deg_conv)
    else:
        results["l_stance_hip_roll_deg"] = float("nan")

    if np.any(r_stance_mask):
        results["r_stance_hip_roll_deg"] = float(np.mean(r_hip_roll[r_stance_mask]) * deg_conv)
    else:
        results["r_stance_hip_roll_deg"] = float("nan")

    # --- 8. X速度 mean ---
    results["x_vel_mean"] = float(np.mean(col(ss_header, ss, "base_vel_x")))

    # --- 9. Y速度 std ---
    results["y_vel_std"] = float(np.std(col(ss_header, ss, "base_vel_y")))

    # --- 10. Yawドリフト ---
    yaw_deg = col(ss_header, ss, "yaw_deg")
    results["yaw_drift_deg"] = float(yaw_deg[-1] - yaw_deg[0])

    # --- 11. 全関節 >2Hz比率 ---
    joint_names = [
        "L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee_pitch", "L_ankle_pitch",
        "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee_pitch", "R_ankle_pitch",
    ]
    high_freq_ratios: list[float] = []
    for joint in joint_names:
        sig = col(ss_header, ss, f"dof_pos_{joint}")
        sig_detrend = sig - np.mean(sig)
        freqs = np.fft.rfftfreq(len(sig_detrend), d=dt)
        fft_mag = np.abs(np.fft.rfft(sig_detrend))
        fft_mag[0] = 0
        total_energy = float(np.sum(fft_mag))
        if total_energy > 1e-10:
            high_freq_mask = freqs > 2.0
            high_energy = float(np.sum(fft_mag[high_freq_mask]))
            high_freq_ratios.append(high_energy / total_energy * 100)
        else:
            high_freq_ratios.append(0.0)
    results["high_freq_ratio_mean_pct"] = float(np.mean(high_freq_ratios))

    # --- 12. アクション変化率RMS平均 ---
    action_rms_list: list[float] = []
    for i in range(10):
        action = col(ss_header, ss, f"action_{i}")
        action_diff = np.diff(action) / dt
        action_rms_list.append(float(np.sqrt(np.mean(action_diff**2))))
    results["action_rate_rms_mean"] = float(np.mean(action_rms_list))

    # --- 13. base_height ---
    results["base_height_m"] = base_height

    # --- 14. 横方向ドリフト速度 ---
    results["lateral_drift_mm_s"] = float(coeffs[0] * 1000)  # mm/s

    return results


def load_reward_components(exp_name: str, log_base: str = "logs") -> dict[str, float] | None:
    """TensorBoardイベントファイルから報酬コンポーネントを読み込む。"""
    try:
        from tensorboard.backend.event_processing import event_accumulator
    except ImportError:
        print("Warning: tensorboard not available, skipping reward components", file=sys.stderr)
        return None

    log_dir = os.path.join(log_base, exp_name)
    if not os.path.exists(log_dir):
        return None

    event_files = [f for f in os.listdir(log_dir) if f.startswith("events.")]
    if not event_files:
        return None

    ea = event_accumulator.EventAccumulator(os.path.join(log_dir, event_files[0]))
    ea.Reload()

    reward_tags = [tag for tag in ea.Tags()["scalars"] if tag.startswith("Episode/rew_")]
    rewards: dict[str, float] = {}
    positive_total = 0.0
    penalty_total = 0.0

    for tag in reward_tags:
        events = ea.Scalars(tag)
        if events:
            name = tag.replace("Episode/rew_", "")
            val = events[-1].value
            rewards[name] = val
            if val > 0:
                positive_total += val
            else:
                penalty_total += val

    rewards["_positive_total"] = positive_total
    rewards["_penalty_total"] = penalty_total
    rewards["_ratio"] = abs(penalty_total / positive_total) if positive_total > 0 else 0.0

    return rewards


def qualify_trend(values: list[float], metric_name: str) -> str:
    """値の列からトレンドの定性的判定を返す。"""
    if len(values) < 2:
        return "-"

    first = values[0]
    last = values[-1]

    # 特殊パターン検出
    # V字回復: 途中で悪化→最後に回復
    if len(values) >= 3:
        mid_worst = min(values[1:-1]) if "corr" in metric_name else max(values[1:-1])
        if "corr" in metric_name:
            # hip_pitch相関: 負が良い（交互歩行）
            if last < first * 0.5 and mid_worst > 0:
                return "V字回復"
        if "std" in metric_name or "amplification" in metric_name or "rate" in metric_name:
            # 値が小さい方が良い指標
            if len(values) >= 3 and values[-1] < max(values[1:-1]):
                pass  # 最後に改善しているが完全回復ではない

    # 単調トレンド検出
    diffs = [values[i + 1] - values[i] for i in range(len(values) - 1)]
    all_increasing = all(d > 0 for d in diffs)
    all_decreasing = all(d < 0 for d in diffs)

    # 指標別の良し悪し判定
    lower_better = any(
        kw in metric_name
        for kw in ["std", "amplification", "rate", "drift", "high_freq", "action_rate"]
    )
    higher_better = any(kw in metric_name for kw in ["x_vel", "positive"])

    if "corr" in metric_name:
        # hip_pitch相関は特殊: 負が良い（交互歩行）
        if last < -0.15 and any(v > 0 for v in values[1:]):
            return "V字回復"
        if all_increasing:
            return "一貫悪化"
        if all_decreasing:
            return "一貫改善"

    if "freq" in metric_name and "high" not in metric_name:
        # L/R周波数一致度
        return "-"

    if lower_better:
        if all_increasing:
            return "一貫悪化"
        if all_decreasing:
            return "一貫改善"
        if last == max(values):
            return "最悪"
        if last == min(values):
            return "最良"
    elif higher_better:
        if all_increasing:
            return "一貫改善"
        if all_decreasing:
            return "一貫悪化"
        if last == max(values):
            return "最良"
        if last == min(values):
            return "最悪"

    # パターン判定
    if len(values) >= 4:
        first_half = values[: len(values) // 2]
        second_half = values[len(values) // 2 :]
        if np.mean(first_half) < np.mean(second_half) and np.mean(second_half) > np.mean(values):
            return "悪化傾向"
        if np.mean(first_half) > np.mean(second_half):
            return "改善傾向"

    pct_change = (last - first) / abs(first) * 100 if abs(first) > 1e-6 else 0
    if abs(pct_change) < 10:
        return "安定"
    return f"{pct_change:+.0f}%"


def format_trend_table(
    versions: list[int],
    all_metrics: list[dict[str, float]],
    all_rewards: list[dict[str, float] | None] | None,
) -> str:
    """Markdownトレンドテーブルを生成する。"""
    lines: list[str] = []

    # メトリクス定義: (表示名, キー, フォーマット, 単位)
    metric_defs: list[tuple[str, str, str, str]] = [
        ("hip_pitch相関", "hip_pitch_corr", ".3f", ""),
        ("Roll std", "roll_std_deg", ".2f", "deg"),
        ("横方向 detrended std", "lateral_detrended_std_mm", ".1f", "mm"),
        ("増幅率", "amplification", ".2f", "x"),
        ("遷移レート", "transition_rate", ".2f", "/s"),
        ("L歩行周波数", "l_walk_freq_hz", ".2f", "Hz"),
        ("R歩行周波数", "r_walk_freq_hz", ".2f", "Hz"),
        ("L stance hip_roll", "l_stance_hip_roll_deg", "+.1f", "deg"),
        ("R stance hip_roll", "r_stance_hip_roll_deg", "+.1f", "deg"),
        ("X速度 mean", "x_vel_mean", ".3f", "m/s"),
        ("Y速度 std", "y_vel_std", ".3f", "m/s"),
        ("Yawドリフト", "yaw_drift_deg", "+.1f", "deg"),
        ("全関節>2Hz比率", "high_freq_ratio_mean_pct", ".1f", "%"),
        ("アクション変化率RMS", "action_rate_rms_mean", ".1f", "/s"),
        ("横方向ドリフト速度", "lateral_drift_mm_s", ".1f", "mm/s"),
        ("base_height", "base_height_m", ".3f", "m"),
    ]

    # 報酬メトリクスを追加
    if all_rewards and any(r is not None for r in all_rewards):
        metric_defs.extend([
            ("正報酬合計", "_positive_total", ".4f", ""),
            ("ペナルティ合計", "_penalty_total", ".4f", ""),
            ("ratio (pen/pos)", "_ratio", ".3f", ""),
        ])

    # ヘッダ行
    v_headers = " | ".join(f"V{v}" for v in versions)
    lines.append(f"| 指標 | {v_headers} | トレンド |")
    lines.append(f"|{'------|' * (len(versions) + 2)}")

    for display_name, key, fmt, unit in metric_defs:
        values: list[str] = []
        float_vals: list[float] = []

        for i, metrics in enumerate(all_metrics):
            # eval CSV由来のメトリクス
            if key in metrics:
                val = metrics[key]
                float_vals.append(val)
                values.append(f"{val:{fmt}}")
            elif all_rewards and all_rewards[i] is not None and key in all_rewards[i]:
                val = all_rewards[i][key]
                float_vals.append(val)
                values.append(f"{val:{fmt}}")
            else:
                values.append("-")

        # トレンド判定
        trend = qualify_trend(float_vals, key) if len(float_vals) >= 2 else "-"

        v_cells = " | ".join(values)
        unit_str = f" {unit}" if unit else ""
        lines.append(f"| {display_name}{unit_str} | {v_cells} | {trend} |")

    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="クロスバージョン トレンド分析（複数バージョンの標準化指標を一括比較）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  uv run python scripts/analyze_cross_version_trend.py 5 6 7 8 --prefix droid-walking-omni-v
  uv run python scripts/analyze_cross_version_trend.py 4 5 6 7 8 --prefix droid-walking-omni-v --include-rewards
  uv run python scripts/analyze_cross_version_trend.py 5 6 7 8 --epoch 1999
        """,
    )
    parser.add_argument(
        "versions",
        type=int,
        nargs="+",
        help="分析するバージョン番号（2つ以上）。例: 5 6 7 8",
    )
    parser.add_argument(
        "--epoch",
        type=int,
        default=3999,
        help="評価エポック番号（デフォルト: 3999）。見つからない場合は自動フォールバック",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="droid-walking-omni-v",
        help="Experiment name prefix (default: droid-walking-omni-v)",
    )
    parser.add_argument(
        "--include-rewards",
        action="store_true",
        default=False,
        help="TensorBoardイベントから報酬コンポーネントも読み込む",
    )
    args = parser.parse_args()
    if len(args.versions) < 2:
        parser.error("バージョン番号は2つ以上指定してください")
    return args


def main() -> None:
    args = parse_args()

    all_metrics: list[dict[str, float]] = []
    all_rewards: list[dict[str, float] | None] = []
    loaded_versions: list[int] = []

    for v in args.versions:
        csv_path = resolve_csv_path(args.prefix, v, args.epoch)
        if not os.path.exists(csv_path):
            print(f"Warning: CSV not found for V{v}: {csv_path}", file=sys.stderr)
            continue

        print(f"Loading V{v}: {csv_path}", file=sys.stderr)
        header, data = load_csv(csv_path)
        metrics = compute_metrics(header, data)
        all_metrics.append(metrics)
        loaded_versions.append(v)

        # 報酬コンポーネント
        if args.include_rewards:
            exp_name = f"{args.prefix}{v}"
            rewards = load_reward_components(exp_name)
            all_rewards.append(rewards)
        else:
            all_rewards.append(None)

    if len(loaded_versions) < 2:
        print("Error: 2つ以上のバージョンが必要です", file=sys.stderr)
        sys.exit(1)

    # トレンドテーブル出力
    print(f"\n## V{'–V'.join(str(v) for v in loaded_versions)} クロスバージョン トレンド分析\n")
    table = format_trend_table(
        loaded_versions,
        all_metrics,
        all_rewards if args.include_rewards else None,
    )
    print(table)

    # 個別メトリクス詳細（標準出力）
    print(f"\n\n### 個別バージョン詳細\n")
    for v, metrics in zip(loaded_versions, all_metrics):
        print(f"#### V{v}")
        for key, val in sorted(metrics.items()):
            print(f"  {key}: {val}")
        print()


if __name__ == "__main__":
    main()
