"""
膝トルク非対称度・高周波・体高Yaw相関の詳細分析スクリプト

Q1: Swing期間のL/R膝actionRMS非対称度
Q2: 前進/後退でknee非対称度の符号反転有無
Q3: コマンド方向別R_knee高周波(>2Hz)比率
Q4: 体高時系列とYawドリフト速度の時間相関

Usage:
    uv run python scripts/analyze_knee_asymmetry_detail.py
"""

from __future__ import annotations

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


def q1_swing_knee_rms_asymmetry(
    header: list[str], data: NDArray[np.floating], label: str
) -> tuple[float, float, float]:
    """Q1: Swing期間中のL/R knee action RMS非対称度"""
    dt = data[1, 0] - data[0, 0]
    start_idx = int(2.0 / dt)

    contact_l = col(header, data, "contact_left")[start_idx:]
    contact_r = col(header, data, "contact_right")[start_idx:]
    action_l_knee = col(header, data, "action_3")[start_idx:]  # L_knee_pitch
    action_r_knee = col(header, data, "action_8")[start_idx:]  # R_knee_pitch

    # Swing = contact==0 for that leg
    l_swing = contact_l == 0
    r_swing = contact_r == 0

    l_rms = np.sqrt(np.mean(action_l_knee[l_swing] ** 2)) if np.any(l_swing) else 0.0
    r_rms = np.sqrt(np.mean(action_r_knee[r_swing] ** 2)) if np.any(r_swing) else 0.0
    mean_rms = (l_rms + r_rms) / 2.0
    asym_pct = (r_rms - l_rms) / mean_rms * 100 if mean_rms > 0 else 0.0

    l_count = int(np.sum(l_swing))
    r_count = int(np.sum(r_swing))

    print(f"  {label}: L_swing={l_count}steps R_swing={r_count}steps")
    print(f"    L_knee_action_RMS={l_rms:.4f}, R_knee_action_RMS={r_rms:.4f}")
    print(f"    非対称度(R-L)/mean = {asym_pct:+.1f}%")

    return l_rms, r_rms, asym_pct


def q2_knee_range_mean_by_direction(
    header: list[str], data: NDArray[np.floating], label: str
) -> tuple[float, float]:
    """Q2: 前進/後退でのknee position range/mean"""
    dt = data[1, 0] - data[0, 0]
    start_idx = int(2.0 / dt)

    lk = col(header, data, "dof_pos_L_knee_pitch")[start_idx:]
    rk = col(header, data, "dof_pos_R_knee_pitch")[start_idx:]

    l_range = float(np.max(lk) - np.min(lk))
    r_range = float(np.max(rk) - np.min(rk))
    range_asym = (r_range - l_range) / ((r_range + l_range) / 2) * 100

    l_mean = float(np.mean(lk))
    r_mean = float(np.mean(rk))
    mean_diff = r_mean - l_mean

    print(f"  {label}:")
    print(f"    L_knee: mean={l_mean:.4f} range={l_range:.4f}")
    print(f"    R_knee: mean={r_mean:.4f} range={r_range:.4f}")
    print(f"    range非対称度(R-L)/mean = {range_asym:+.1f}%")
    print(f"    mean差(R-L) = {mean_diff:+.4f} rad")

    return range_asym, mean_diff


def q3_r_knee_hf_ratio(
    header: list[str], data: NDArray[np.floating], label: str
) -> float:
    """Q3: R_knee_pitchの>2Hz高周波成分比率"""
    dt = data[1, 0] - data[0, 0]
    start_idx = int(2.0 / dt)

    rk = col(header, data, "dof_pos_R_knee_pitch")[start_idx:]
    sig = rk - np.mean(rk)
    N = len(sig)

    yf = np.abs(np.fft.rfft(sig))
    freqs = np.fft.rfftfreq(N, d=dt)

    total_power = float(np.sum(yf**2))
    hf_mask = freqs > 2.0
    hf_power = float(np.sum(yf[hf_mask] ** 2))
    ratio = hf_power / total_power * 100 if total_power > 0 else 0.0

    # Also compute for L_knee for comparison
    lk = col(header, data, "dof_pos_L_knee_pitch")[start_idx:]
    sig_l = lk - np.mean(lk)
    yf_l = np.abs(np.fft.rfft(sig_l))
    total_l = float(np.sum(yf_l**2))
    hf_l = float(np.sum(yf_l[freqs > 2.0] ** 2))
    ratio_l = hf_l / total_l * 100 if total_l > 0 else 0.0

    print(f"  {label}: R_knee HF(>2Hz)={ratio:.2f}%, L_knee HF(>2Hz)={ratio_l:.2f}%")
    return ratio


def q4_height_yaw_correlation(
    header: list[str], data: NDArray[np.floating], label: str
) -> float:
    """Q4: 体高(base_pos_z)移動平均とYawドリフト速度(yaw_deg微分)の相関"""
    dt = data[1, 0] - data[0, 0]
    start_idx = int(2.0 / dt)

    pos_z = col(header, data, "base_pos_z")[start_idx:]
    yaw = col(header, data, "yaw_deg")[start_idx:]

    # 移動平均 (window=25=0.5s)
    window = 25
    z_ma = np.convolve(pos_z, np.ones(window) / window, mode="valid")

    # Yaw rate (deg/s)
    yaw_rate = np.diff(yaw) / dt
    # Align lengths
    min_len = min(len(z_ma), len(yaw_rate))
    # Offset for convolution valid mode
    offset = (window - 1) // 2
    yaw_rate_aligned = yaw_rate[offset : offset + min_len]
    z_ma_aligned = z_ma[:min_len]

    corr = float(np.corrcoef(z_ma_aligned, yaw_rate_aligned)[0, 1])

    # Also compute total yaw drift
    yaw_full = col(header, data, "yaw_deg")
    yaw_drift = float(yaw_full[-1] - yaw_full[0])

    # Yaw drift rate (linear fit)
    t_ss = col(header, data, "timestamp")[start_idx:]
    coeffs = np.polyfit(t_ss, col(header, data, "yaw_deg")[start_idx:], 1)
    drift_rate = coeffs[0]

    print(f"  {label}: corr(z_ma, yaw_rate)={corr:+.4f}")
    print(f"    yaw_drift_total={yaw_drift:+.2f} deg, drift_rate={drift_rate:+.3f} deg/s")

    return corr


def main() -> None:
    files = {
        "V9-fwd": "logs/droid-walking-omni-v9/eval_3999_cmd_0.30_0.00_0.00_s1.csv",
        "V9-bwd": "logs/droid-walking-omni-v9/eval_3999_cmd_-0.30_0.00_0.00_s1.csv",
        "V8-fwd": "logs/droid-walking-omni-v8/eval_3999_cmd_0.30_0.00_0.00_s1.csv",
        "V8-bwd": "logs/droid-walking-omni-v8/eval_3999_cmd_-0.30_0.00_0.00_s1.csv",
    }

    datasets: dict[str, tuple[list[str], NDArray[np.floating]]] = {}
    for label, path in files.items():
        datasets[label] = load_csv(path)

    # === Q1 ===
    print("=" * 60)
    print("Q1: Swing期間 L/R膝action RMS非対称度")
    print("=" * 60)
    q1_results: dict[str, float] = {}
    for label, (h, d) in datasets.items():
        _, _, asym = q1_swing_knee_rms_asymmetry(h, d, label)
        q1_results[label] = asym
    print()

    # === Q2 ===
    print("=" * 60)
    print("Q2: 前進/後退でのknee非対称度（符号反転の有無）")
    print("=" * 60)
    q2_results: dict[str, tuple[float, float]] = {}
    for label, (h, d) in datasets.items():
        range_asym, mean_diff = q2_knee_range_mean_by_direction(h, d, label)
        q2_results[label] = (range_asym, mean_diff)

    print("\n  [符号反転チェック]")
    for ver in ["V9", "V8"]:
        fwd_asym = q2_results[f"{ver}-fwd"][0]
        bwd_asym = q2_results[f"{ver}-bwd"][0]
        sign_flip = (fwd_asym > 0) != (bwd_asym > 0)
        print(f"  {ver}: fwd range非対称={fwd_asym:+.1f}%, bwd range非対称={bwd_asym:+.1f}%")
        print(f"    -> 符号反転: {'YES' if sign_flip else 'NO'}")
    print()

    # === Q3 ===
    print("=" * 60)
    print("Q3: コマンド方向別 R_knee高周波(>2Hz)比率")
    print("=" * 60)
    for label, (h, d) in datasets.items():
        q3_r_knee_hf_ratio(h, d, label)
    print()

    # === Q4 ===
    print("=" * 60)
    print("Q4: 体高(base_pos_z) vs Yawドリフト速度の時間相関")
    print("=" * 60)
    for label, (h, d) in datasets.items():
        q4_height_yaw_correlation(h, d, label)


if __name__ == "__main__":
    main()
