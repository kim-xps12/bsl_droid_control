"""
高周波振動・ジタバタ挙動の定量分析スクリプト

関節角FFT（高周波成分比率）、アクション変化率、DOF速度ホットスポット、
接触切替頻度を分析し、複数バージョン間で比較する。

analyze_eval_csv.py の関節ダイナミクス分析を補完する深い分析に特化。
目視で「ジタバタ」「高周波振動」が指摘された場合に特に有用。

Usage:
    uv run python scripts/analyze_vibration.py 4 3 --prefix droid-walking-omni-v
    uv run python scripts/analyze_vibration.py 4 3 --prefix droid-walking-omni-v --epoch 1999
    uv run python scripts/analyze_vibration.py 4 3 --prefix droid-walking-narrow-v --freq-threshold 3.0
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from numpy.typing import NDArray


JOINTS = ["hip_yaw", "hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch"]
SIDES = ["L", "R"]


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


def analyze_joint_fft(
    header: list[str],
    data: NDArray[np.floating],
    label: str,
    freq_threshold: float,
) -> dict[str, dict[str, float]]:
    """関節角FFT分析: 各関節の主周波数と高周波成分比率を算出"""
    print(f"\n{'=' * 60}")
    print(f"[{label}] 関節角FFT分析 (高周波閾値: {freq_threshold:.1f} Hz)")
    print(f"{'=' * 60}")

    dt = float(data[1, 0] - data[0, 0])
    start_idx = int(2.0 / dt)
    results: dict[str, dict[str, float]] = {}

    print(f"  {'関節':<18s} | {'主周波数':>8s} | {'>' + str(freq_threshold) + 'Hz比率':>10s} | {'振幅std':>8s} | {'range':>8s}")
    print("  " + "-" * 65)

    for side in SIDES:
        for joint in JOINTS:
            col_name = f"dof_pos_{side}_{joint}"
            sig = col(header, data, col_name)[start_idx:]
            sig_detrend = sig - np.mean(sig)

            # FFT
            freqs = np.fft.rfftfreq(len(sig_detrend), d=dt)
            fft_mag = np.abs(np.fft.rfft(sig_detrend))
            fft_mag[0] = 0  # DC除外

            # 主周波数
            peak_idx = np.argmax(fft_mag)
            peak_freq = float(freqs[peak_idx])

            # 高周波成分比率
            total_energy = float(np.sum(fft_mag**2))
            high_freq_mask = freqs > freq_threshold
            high_freq_energy = float(np.sum(fft_mag[high_freq_mask] ** 2))
            high_freq_ratio = high_freq_energy / total_energy if total_energy > 0 else 0.0

            key = f"{side}_{joint}"
            results[key] = {
                "peak_freq": peak_freq,
                "high_freq_ratio": high_freq_ratio,
                "std": float(np.std(sig)),
                "range": float(np.max(sig) - np.min(sig)),
            }

            print(
                f"  {key:<18s} | {peak_freq:>7.2f}Hz | {high_freq_ratio * 100:>9.1f}% | "
                f"{np.std(sig):>7.4f} | {np.max(sig) - np.min(sig):>7.4f}"
            )

    # 全関節平均
    avg_ratio = np.mean([r["high_freq_ratio"] for r in results.values()])
    print(f"\n  全関節平均 >{freq_threshold}Hz比率: {avg_ratio * 100:.1f}%")

    return results


def analyze_action_rate(
    header: list[str],
    data: NDArray[np.floating],
    label: str,
) -> dict[str, dict[str, float]]:
    """アクション変化率分析: 各アクションの時間微分RMS"""
    print(f"\n{'=' * 60}")
    print(f"[{label}] アクション変化率分析")
    print(f"{'=' * 60}")

    dt = float(data[1, 0] - data[0, 0])
    start_idx = int(2.0 / dt)
    results: dict[str, dict[str, float]] = {}

    joint_map = [f"{s}_{j}" for s in SIDES for j in JOINTS]

    print(f"  {'アクション':<12s} | {'対応関節':<18s} | {'変化率RMS':>10s} | {'アクションRMS':>12s} | {'mean':>8s}")
    print("  " + "-" * 75)

    for i in range(10):
        act = col(header, data, f"action_{i}")[start_idx:]
        act_diff = np.diff(act) / dt
        diff_rms = float(np.sqrt(np.mean(act_diff**2)))
        act_rms = float(np.sqrt(np.mean(act**2)))
        act_mean = float(np.mean(act))

        joint_name = joint_map[i] if i < len(joint_map) else f"unknown_{i}"
        key = f"action_{i}"
        results[key] = {
            "joint": float(i),
            "diff_rms": diff_rms,
            "rms": act_rms,
            "mean": act_mean,
        }

        print(f"  action_{i:<4d} | {joint_name:<18s} | {diff_rms:>9.1f} /s | {act_rms:>11.4f} | {act_mean:>+7.4f}")

    avg_diff_rms = np.mean([r["diff_rms"] for r in results.values()])
    print(f"\n  全体平均 変化率RMS: {avg_diff_rms:.1f} /s")

    return results


def analyze_dof_velocity_hotspot(
    header: list[str],
    data: NDArray[np.floating],
    label: str,
) -> dict[str, dict[str, float]]:
    """DOF速度ホットスポット分析: 各関節の角速度RMSとピーク"""
    print(f"\n{'=' * 60}")
    print(f"[{label}] DOF速度ホットスポット分析")
    print(f"{'=' * 60}")

    dt = float(data[1, 0] - data[0, 0])
    start_idx = int(2.0 / dt)
    results: dict[str, dict[str, float]] = {}

    print(f"  {'関節':<18s} | {'vel RMS':>10s} | {'vel peak':>10s} | {'L/R比':>8s}")
    print("  " + "-" * 55)

    for joint in JOINTS:
        l_vel = col(header, data, f"dof_vel_L_{joint}")[start_idx:]
        r_vel = col(header, data, f"dof_vel_R_{joint}")[start_idx:]
        l_rms = float(np.sqrt(np.mean(l_vel**2)))
        r_rms = float(np.sqrt(np.mean(r_vel**2)))
        l_peak = float(np.max(np.abs(l_vel)))
        r_peak = float(np.max(np.abs(r_vel)))

        results[f"L_{joint}"] = {"vel_rms": l_rms, "vel_peak": l_peak}
        results[f"R_{joint}"] = {"vel_rms": r_rms, "vel_peak": r_peak}

        ratio = l_rms / r_rms if r_rms > 0 else float("inf")
        print(f"  L_{joint:<14s} | {l_rms:>9.3f} | {l_peak:>9.3f} | {ratio:>7.2f}")
        print(f"  R_{joint:<14s} | {r_rms:>9.3f} | {r_peak:>9.3f} |")

    return results


def analyze_contact_switching(
    header: list[str],
    data: NDArray[np.floating],
    label: str,
    micro_threshold: float = 0.1,
) -> dict[str, float]:
    """接触切替頻度分析: 遷移回数、マイクロイベント検出"""
    print(f"\n{'=' * 60}")
    print(f"[{label}] 接触切替頻度分析 (マイクロ閾値: {micro_threshold:.2f}s)")
    print(f"{'=' * 60}")

    dt = float(data[1, 0] - data[0, 0])
    start_idx = int(2.0 / dt)
    timestamps = col(header, data, "timestamp")[start_idx:]
    duration = float(timestamps[-1] - timestamps[0])
    results: dict[str, float] = {}

    total_transitions = 0

    for side, contact_name in [("L", "contact_left"), ("R", "contact_right")]:
        contact = col(header, data, contact_name)[start_idx:]
        contact_int = contact.astype(int)
        diff = np.diff(contact_int)
        transitions = int(np.sum(np.abs(diff)))
        total_transitions += transitions
        rate = transitions / duration if duration > 0 else 0.0

        # マイクロイベント検出（stance/swing < threshold）
        # 連続区間の長さを計算
        micro_stance = 0
        micro_swing = 0
        total_stance_events = 0
        total_swing_events = 0

        # stance区間（contact == 1）
        in_stance = False
        stance_start = 0
        for i in range(len(contact)):
            if contact[i] == 1 and not in_stance:
                in_stance = True
                stance_start = i
            elif contact[i] == 0 and in_stance:
                in_stance = False
                dur_s = (i - stance_start) * dt
                total_stance_events += 1
                if dur_s < micro_threshold:
                    micro_stance += 1

        # swing区間（contact == 0）
        in_swing = False
        swing_start = 0
        for i in range(len(contact)):
            if contact[i] == 0 and not in_swing:
                in_swing = True
                swing_start = i
            elif contact[i] == 1 and in_swing:
                in_swing = False
                dur_s = (i - swing_start) * dt
                total_swing_events += 1
                if dur_s < micro_threshold:
                    micro_swing += 1

        micro_stance_pct = micro_stance / total_stance_events * 100 if total_stance_events > 0 else 0
        micro_swing_pct = micro_swing / total_swing_events * 100 if total_swing_events > 0 else 0

        print(f"\n  {side}足:")
        print(f"    接触状態遷移数: {transitions}")
        print(f"    遷移レート: {rate:.2f} /s")
        print(
            f"    マイクロスタンス (<{micro_threshold}s): {micro_stance}/{total_stance_events} ({micro_stance_pct:.1f}%)"
        )
        print(
            f"    マイクロスイング (<{micro_threshold}s): {micro_swing}/{total_swing_events} ({micro_swing_pct:.1f}%)"
        )

        results[f"{side}_transitions"] = float(transitions)
        results[f"{side}_rate"] = rate
        results[f"{side}_micro_stance_pct"] = micro_stance_pct
        results[f"{side}_micro_swing_pct"] = micro_swing_pct

    results["total_transitions"] = float(total_transitions)
    results["total_rate"] = total_transitions / duration if duration > 0 else 0.0
    print(f"\n  合計遷移: {total_transitions}, レート: {results['total_rate']:.2f} /s")

    return results


def print_comparison(
    label_new: str,
    label_old: str,
    fft_new: dict[str, dict[str, float]],
    fft_old: dict[str, dict[str, float]],
    action_new: dict[str, dict[str, float]],
    action_old: dict[str, dict[str, float]],
    vel_new: dict[str, dict[str, float]],
    vel_old: dict[str, dict[str, float]],
    contact_new: dict[str, float],
    contact_old: dict[str, float],
) -> None:
    """2バージョン比較サマリ"""
    print(f"\n{'#' * 70}")
    print(f"# {label_new} vs {label_old} 振動分析比較サマリ")
    print(f"{'#' * 70}")

    # FFT比較
    print("\n  [関節角FFT: 高周波成分比率 V/V比]")
    print(
        f"  {'関節':<18s} | {label_new + ' 比率':>10s} | {label_old + ' 比率':>10s} | {'V/V比':>8s} | {label_new + ' 主freq':>10s} | {label_old + ' 主freq':>10s}"
    )
    print("  " + "-" * 80)
    for key in fft_new:
        if key in fft_old:
            new_r = fft_new[key]["high_freq_ratio"]
            old_r = fft_old[key]["high_freq_ratio"]
            ratio = new_r / old_r if old_r > 0 else float("inf")
            new_f = fft_new[key]["peak_freq"]
            old_f = fft_old[key]["peak_freq"]
            marker = " **" if ratio > 1.3 or ratio < 0.7 else ""
            print(
                f"  {key:<18s} | {new_r * 100:>9.1f}% | {old_r * 100:>9.1f}% | {ratio:>7.2f}x | "
                f"{new_f:>9.2f}Hz | {old_f:>9.2f}Hz{marker}"
            )

    # アクション変化率比較
    print("\n  [アクション変化率 V/V比]")
    joint_map = [f"{s}_{j}" for s in SIDES for j in JOINTS]
    print(f"  {'アクション':<12s} | {'関節':<18s} | {label_new:>10s} | {label_old:>10s} | {'V/V比':>8s}")
    print("  " + "-" * 65)
    for key in action_new:
        if key in action_old:
            new_d = action_new[key]["diff_rms"]
            old_d = action_old[key]["diff_rms"]
            ratio = new_d / old_d if old_d > 0 else float("inf")
            i = int(key.split("_")[1])
            jname = joint_map[i] if i < len(joint_map) else "?"
            marker = " **" if ratio > 1.2 else ""
            print(f"  {key:<12s} | {jname:<18s} | {new_d:>9.1f} | {old_d:>9.1f} | {ratio:>7.2f}x{marker}")

    # DOF速度比較
    print("\n  [DOF速度ホットスポット V/V比]")
    print(
        f"  {'関節':<18s} | {label_new + ' RMS':>10s} | {label_old + ' RMS':>10s} | {'V/V比':>8s} | {label_new + ' peak':>10s} | {label_old + ' peak':>10s}"
    )
    print("  " + "-" * 80)
    for key in vel_new:
        if key in vel_old:
            new_rms = vel_new[key]["vel_rms"]
            old_rms = vel_old[key]["vel_rms"]
            ratio = new_rms / old_rms if old_rms > 0 else float("inf")
            new_pk = vel_new[key]["vel_peak"]
            old_pk = vel_old[key]["vel_peak"]
            marker = " **" if ratio > 1.3 else ""
            print(
                f"  {key:<18s} | {new_rms:>9.3f} | {old_rms:>9.3f} | {ratio:>7.2f}x | "
                f"{new_pk:>9.3f} | {old_pk:>9.3f}{marker}"
            )

    # 接触切替比較
    print("\n  [接触切替頻度 V/V比]")
    for metric, display in [
        ("total_transitions", "遷移合計"),
        ("total_rate", "遷移レート (/s)"),
        ("L_micro_stance_pct", "Lマイクロスタンス (%)"),
        ("R_micro_stance_pct", "Rマイクロスタンス (%)"),
        ("L_micro_swing_pct", "Lマイクロスイング (%)"),
        ("R_micro_swing_pct", "Rマイクロスイング (%)"),
    ]:
        new_v = contact_new.get(metric, 0)
        old_v = contact_old.get(metric, 0)
        ratio = new_v / old_v if old_v > 0 else float("inf")
        print(f"  {display:<24s}: {new_v:>8.1f} / {old_v:>8.1f} = {ratio:.2f}x")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="高周波振動・ジタバタ挙動の定量分析（複数バージョン比較）",
    )
    parser.add_argument(
        "versions",
        type=int,
        nargs="+",
        help="比較するバージョン番号（2つ以上）。先頭が新バージョン。例: 4 3",
    )
    parser.add_argument(
        "--epoch",
        type=int,
        default=3999,
        help="評価エポック番号（デフォルト: 3999）",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="droid-walking-unitree-v",
        help="Experiment name prefix (default: droid-walking-unitree-v)",
    )
    parser.add_argument(
        "--freq-threshold",
        type=float,
        default=2.0,
        help="高周波成分の閾値周波数 (Hz, デフォルト: 2.0)",
    )
    args = parser.parse_args()
    if len(args.versions) < 2:
        parser.error("バージョン番号は2つ以上指定してください")
    return args


def main() -> None:
    args = parse_args()

    all_fft: list[tuple[str, dict[str, dict[str, float]]]] = []
    all_action: list[tuple[str, dict[str, dict[str, float]]]] = []
    all_vel: list[tuple[str, dict[str, dict[str, float]]]] = []
    all_contact: list[tuple[str, dict[str, float]]] = []

    for v in args.versions:
        label = f"V{v}"
        path = f"logs/{args.prefix}{v}/eval_{args.epoch}.csv"

        if not Path(path).exists():
            print(f"\n[WARNING] {path} が見つかりません。V{v} をスキップします。")
            continue

        print(f"\n{'#' * 70}")
        print(f"# {label} 振動分析")
        print(f"{'#' * 70}")

        header, data = load_csv(path)
        print(f"  データ: {len(data)} rows, dt={data[1, 0] - data[0, 0]:.4f}s")

        fft_results = analyze_joint_fft(header, data, label, args.freq_threshold)
        action_results = analyze_action_rate(header, data, label)
        vel_results = analyze_dof_velocity_hotspot(header, data, label)
        contact_results = analyze_contact_switching(header, data, label)

        all_fft.append((label, fft_results))
        all_action.append((label, action_results))
        all_vel.append((label, vel_results))
        all_contact.append((label, contact_results))

    # 先頭2バージョンの比較
    if len(all_fft) >= 2:
        print_comparison(
            all_fft[0][0],
            all_fft[1][0],
            all_fft[0][1],
            all_fft[1][1],
            all_action[0][1],
            all_action[1][1],
            all_vel[0][1],
            all_vel[1][1],
            all_contact[0][1],
            all_contact[1][1],
        )


if __name__ == "__main__":
    main()
