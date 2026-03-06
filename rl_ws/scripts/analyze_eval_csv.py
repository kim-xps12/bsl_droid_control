"""
CSV時系列データの詳細分析スクリプト
歩行周期、接地パターン、L/R位相関係、Roll/横スウェイなどを複数バージョン間で比較分析する。

Usage:
    uv run python scripts/analyze_eval_csv.py 37 36              # eval_3999.csv を比較
    uv run python scripts/analyze_eval_csv.py 37 36 --epoch 300  # eval_300.csv を比較
    uv run python scripts/analyze_eval_csv.py 37 36 --epoch 3999_cmd_0.30_0.00_0.00_s1  # 固定コマンドCSVを比較
"""

from __future__ import annotations

import argparse

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


def analyze_gait_cycle(header: list[str], data: NDArray[np.floating], label: str) -> None:
    """FFTでhip_pitchの歩行周波数を分析"""
    print(f"\n{'=' * 60}")
    print(f"[{label}] 歩行周期分析 (FFT)")
    print(f"{'=' * 60}")

    dt = data[1, 0] - data[0, 0]
    1.0 / dt
    len(data)

    for side in ["L", "R"]:
        hip_pitch = col(header, data, f"dof_pos_{side}_hip_pitch")
        # 定常状態のみ使用（最初の2秒を除外）
        start_idx = int(2.0 / dt)
        sig = hip_pitch[start_idx:]
        sig_detrend = sig - np.mean(sig)

        # FFT
        freqs = np.fft.rfftfreq(len(sig_detrend), d=dt)
        fft_mag = np.abs(np.fft.rfft(sig_detrend))

        # DC成分を除外してピーク検出
        fft_mag[0] = 0
        peak_idx = np.argmax(fft_mag)
        peak_freq = freqs[peak_idx]
        peak_period = 1.0 / peak_freq if peak_freq > 0 else float("inf")

        # 上位3ピーク
        top3_idx = np.argsort(fft_mag)[-3:][::-1]
        print(f"  {side}_hip_pitch:")
        print(f"    主周波数: {peak_freq:.2f} Hz (周期: {peak_period:.3f} s)")
        print(f"    振幅(mean±std): {np.mean(sig):.4f} ± {np.std(sig):.4f} rad")
        print(f"    range: [{np.min(sig):.4f}, {np.max(sig):.4f}] rad")
        print("    上位3ピーク周波数: ", end="")
        for i in top3_idx:
            if freqs[i] > 0:
                print(f"{freqs[i]:.2f}Hz(mag={fft_mag[i]:.2f}), ", end="")
        print()

    # knee_pitchも分析
    for side in ["L", "R"]:
        knee_pitch = col(header, data, f"dof_pos_{side}_knee_pitch")
        start_idx = int(2.0 / dt)
        sig = knee_pitch[start_idx:]
        sig_detrend = sig - np.mean(sig)
        freqs = np.fft.rfftfreq(len(sig_detrend), d=dt)
        fft_mag = np.abs(np.fft.rfft(sig_detrend))
        fft_mag[0] = 0
        peak_idx = np.argmax(fft_mag)
        peak_freq = freqs[peak_idx]
        peak_period = 1.0 / peak_freq if peak_freq > 0 else float("inf")
        print(f"  {side}_knee_pitch:")
        print(f"    主周波数: {peak_freq:.2f} Hz (周期: {peak_period:.3f} s)")
        print(f"    振幅(mean±std): {np.mean(sig):.4f} ± {np.std(sig):.4f} rad")


def analyze_contact_patterns(header: list[str], data: NDArray[np.floating], label: str) -> None:
    """接地パターンの時系列分析"""
    print(f"\n{'=' * 60}")
    print(f"[{label}] 接地パターン分析")
    print(f"{'=' * 60}")

    dt = data[1, 0] - data[0, 0]
    timestamps = col(header, data, "timestamp")
    contact_l = col(header, data, "contact_left")
    contact_r = col(header, data, "contact_right")

    # 接地状態の分類
    both_ground = (contact_l == 1) & (contact_r == 1)
    left_only = (contact_l == 1) & (contact_r == 0)
    right_only = (contact_l == 0) & (contact_r == 1)
    both_air = (contact_l == 0) & (contact_r == 0)

    total = len(data)
    print(f"  両足接地: {np.sum(both_ground):>4d} steps ({100 * np.sum(both_ground) / total:.1f}%)")
    print(f"  左足のみ: {np.sum(left_only):>4d} steps ({100 * np.sum(left_only) / total:.1f}%)")
    print(f"  右足のみ: {np.sum(right_only):>4d} steps ({100 * np.sum(right_only) / total:.1f}%)")
    print(f"  両足空中: {np.sum(both_air):>4d} steps ({100 * np.sum(both_air) / total:.1f}%)")

    # 定常状態（2s以降）での分析
    start_idx = int(2.0 / dt)
    print("\n  [定常状態 (t>2s)]")
    ss_total = total - start_idx
    print(
        f"  両足接地: {np.sum(both_ground[start_idx:]):>4d} steps ({100 * np.sum(both_ground[start_idx:]) / ss_total:.1f}%)"
    )
    print(
        f"  左足のみ: {np.sum(left_only[start_idx:]):>4d} steps ({100 * np.sum(left_only[start_idx:]) / ss_total:.1f}%)"
    )
    print(
        f"  右足のみ: {np.sum(right_only[start_idx:]):>4d} steps ({100 * np.sum(right_only[start_idx:]) / ss_total:.1f}%)"
    )
    print(
        f"  両足空中: {np.sum(both_air[start_idx:]):>4d} steps ({100 * np.sum(both_air[start_idx:]) / ss_total:.1f}%)"
    )

    # タップダンスイベントの検出（両足接地 → 片足のみ の遷移）
    print("\n  [タップダンスイベント（両足接地の区間）]")
    # 両足接地の連続区間を検出
    both_ground_int = both_ground.astype(int)
    diff = np.diff(both_ground_int)
    starts = np.where(diff == 1)[0] + 1  # 両足接地開始
    ends = np.where(diff == -1)[0] + 1  # 両足接地終了

    # 先頭が両足接地の場合
    if both_ground_int[0] == 1:
        starts = np.insert(starts, 0, 0)
    if both_ground_int[-1] == 1:
        ends = np.append(ends, len(both_ground_int))

    n_events = min(len(starts), len(ends))
    if n_events > 0:
        print(f"  検出された両足接地区間: {n_events} 回")
        for i in range(min(n_events, 20)):  # 最大20個表示
            duration = (ends[i] - starts[i]) * dt
            t_start = timestamps[starts[i]]
            t_end = timestamps[min(ends[i], len(timestamps) - 1)]
            print(
                f"    区間{i + 1}: t={t_start:.2f}s ~ {t_end:.2f}s (持続: {duration:.3f}s = {ends[i] - starts[i]} steps)"
            )
        if n_events > 20:
            print(f"    ... 残り {n_events - 20} 区間省略")

        durations = [(ends[i] - starts[i]) * dt for i in range(n_events)]
        print(
            f"  両足接地持続時間: mean={np.mean(durations):.3f}s, max={np.max(durations):.3f}s, min={np.min(durations):.3f}s"
        )
    else:
        print("  両足接地区間なし")

    # 片足接地の連続区間分析
    print("\n  [片足スイング区間]")
    for side_name, contact in [("左足接地(右足スイング)", left_only), ("右足接地(左足スイング)", right_only)]:
        contact_int = contact.astype(int)
        diff = np.diff(contact_int)
        s = np.where(diff == 1)[0] + 1
        e = np.where(diff == -1)[0] + 1
        if contact_int[0] == 1:
            s = np.insert(s, 0, 0)
        if contact_int[-1] == 1:
            e = np.append(e, len(contact_int))
        n = min(len(s), len(e))
        if n > 0:
            durs = [(e[i] - s[i]) * dt for i in range(n)]
            print(f"  {side_name}: {n}回, mean={np.mean(durs):.3f}s, max={np.max(durs):.3f}s, min={np.min(durs):.3f}s")
        else:
            print(f"  {side_name}: 0回")


def analyze_phase_relationship(header: list[str], data: NDArray[np.floating], label: str) -> None:
    """L/R hip_pitchの位相関係分析"""
    print(f"\n{'=' * 60}")
    print(f"[{label}] 左右位相関係分析")
    print(f"{'=' * 60}")

    dt = data[1, 0] - data[0, 0]
    start_idx = int(2.0 / dt)

    l_hip = col(header, data, "dof_pos_L_hip_pitch")[start_idx:]
    r_hip = col(header, data, "dof_pos_R_hip_pitch")[start_idx:]

    # 全体相関
    corr = np.corrcoef(l_hip, r_hip)[0, 1]
    print(f"  L/R hip_pitch 相関係数（全体）: {corr:.4f}")

    # ローリング相関 (1秒窓)
    window = int(1.0 / dt)  # 50 steps
    rolling_corr = []
    for i in range(len(l_hip) - window):
        c = np.corrcoef(l_hip[i : i + window], r_hip[i : i + window])[0, 1]
        rolling_corr.append(c)
    rolling_corr_arr = np.array(rolling_corr)
    print(f"  ローリング相関 (1s窓): mean={np.mean(rolling_corr_arr):.4f}, std={np.std(rolling_corr_arr):.4f}")
    print(f"    min={np.min(rolling_corr_arr):.4f}, max={np.max(rolling_corr_arr):.4f}")

    # 位相差の推定（相互相関）
    l_norm = l_hip - np.mean(l_hip)
    r_norm = r_hip - np.mean(r_hip)
    xcorr = np.correlate(l_norm, r_norm, mode="full")
    lags = np.arange(-len(l_norm) + 1, len(l_norm))
    peak_lag = lags[np.argmax(xcorr)]
    phase_delay = peak_lag * dt
    print(f"  相互相関ピーク時のラグ: {peak_lag} steps ({phase_delay:.3f} s)")

    # L/R対称性の指標
    l_range = np.max(l_hip) - np.min(l_hip)
    r_range = np.max(r_hip) - np.min(r_hip)
    asymmetry = (l_range - r_range) / (l_range + r_range) * 100
    print(f"  L hip_pitch range: {l_range:.4f} rad")
    print(f"  R hip_pitch range: {r_range:.4f} rad")
    print(f"  非対称度: {asymmetry:+.1f}% (正=L>R)")

    # knee_pitchも
    l_knee = col(header, data, "dof_pos_L_knee_pitch")[start_idx:]
    r_knee = col(header, data, "dof_pos_R_knee_pitch")[start_idx:]
    corr_knee = np.corrcoef(l_knee, r_knee)[0, 1]
    l_kr = np.max(l_knee) - np.min(l_knee)
    r_kr = np.max(r_knee) - np.min(r_knee)
    asym_knee = (l_kr - r_kr) / (l_kr + r_kr) * 100
    print(f"\n  L/R knee_pitch 相関係数: {corr_knee:.4f}")
    print(f"  L knee_pitch range: {l_kr:.4f} rad, R: {r_kr:.4f} rad")
    print(f"  非対称度: {asym_knee:+.1f}% (正=L>R)")


def _detect_command_switches(
    header: list[str], data: NDArray[np.floating]
) -> list[int]:
    """コマンドの切替点（インデックス）を検出する。

    cmd_vel_x/y/yaw のいずれかが 0.01 以上変化した時点を切替と判定する。
    CSVにコマンド列がなければ空リストを返す。
    """
    if "cmd_vel_x" not in header:
        return []
    cmd_x = col(header, data, "cmd_vel_x")
    cmd_y = col(header, data, "cmd_vel_y")
    cmd_yaw = col(header, data, "cmd_vel_yaw")
    switches: list[int] = []
    for i in range(1, len(cmd_x)):
        if (
            abs(cmd_x[i] - cmd_x[i - 1]) > 0.01
            or abs(cmd_y[i] - cmd_y[i - 1]) > 0.01
            or abs(cmd_yaw[i] - cmd_yaw[i - 1]) > 0.01
        ):
            switches.append(i)
    return switches


def analyze_yaw_drift(header: list[str], data: NDArray[np.floating], label: str) -> None:
    """Yawドリフトの時系列分析

    コマンド切替を検出し、ランダムコマンド評価の場合は累積ドリフトが
    隠蔽される旨を警告する。信頼性の高いYaw評価には方向別固定コマンド
    評価（--command）を使用すること。
    """
    print(f"\n{'=' * 60}")
    print(f"[{label}] Yawドリフト分析")
    print(f"{'=' * 60}")

    dt = data[1, 0] - data[0, 0]
    timestamps = col(header, data, "timestamp")
    yaw = col(header, data, "yaw_deg")
    max_t = int(timestamps[-1])

    # コマンド切替の検出
    cmd_switches = _detect_command_switches(header, data)
    if cmd_switches:
        switch_times = ", ".join(f"t={timestamps[s]:.1f}s" for s in cmd_switches[:5])
        if len(cmd_switches) > 5:
            switch_times += f" ... 他{len(cmd_switches) - 5}回"
        print(f"  !! コマンド切替検出: {len(cmd_switches)}回 ({switch_times})")
        print(f"     【警告】ランダムコマンド評価ではコマンド方向変化がYawドリフトを")
        print(f"     隠蔽する。Yaw改善率は固定コマンド評価（--command）から算出すべき")

    # 1秒ごとのYaw値
    print(f"\n  Yaw推移（1秒ごと）:")
    for t in range(0, max_t + 1):
        idx = int(t / dt)
        if idx < len(yaw):
            print(f"    t={t:>2d}s: Yaw={yaw[idx]:>+7.2f}°")

    # 定常状態ドリフト速度（全体）
    start_idx = int(2.0 / dt)
    yaw_ss = yaw[start_idx:]
    t_ss = timestamps[start_idx:]
    coeffs = np.polyfit(t_ss, yaw_ss, 1)
    qualifier = "（コマンド切替あり・参考値）" if cmd_switches else ""
    print(f"\n  定常状態ドリフト速度: {coeffs[0]:.3f} °/s{qualifier}")
    print(f"  最終Yaw: {yaw[-1]:+.2f}°")

    # コマンドセグメント別のドリフト分析
    if cmd_switches:
        cmd_x = col(header, data, "cmd_vel_x")
        cmd_y = col(header, data, "cmd_vel_y")
        cmd_yaw_col = col(header, data, "cmd_vel_yaw")
        print(f"\n  コマンドセグメント別ドリフト:")
        seg_bounds = [0] + cmd_switches + [len(data)]
        for i in range(len(seg_bounds) - 1):
            s, e = seg_bounds[i], seg_bounds[i + 1]
            if e - s < 10:
                continue
            t_seg = timestamps[s:e]
            yaw_seg = yaw[s:e]
            seg_coeffs = np.polyfit(t_seg, yaw_seg, 1)
            yaw_delta = yaw_seg[-1] - yaw_seg[0]
            print(
                f"    [{t_seg[0]:5.1f}s-{t_seg[-1]:5.1f}s] "
                f"cmd=({cmd_x[s]:+.2f},{cmd_y[s]:+.2f},{cmd_yaw_col[s]:+.2f}) "
                f"rate={seg_coeffs[0]:+.3f} deg/s  dYaw={yaw_delta:+.2f} deg"
            )


def analyze_velocity_profile(header: list[str], data: NDArray[np.floating], label: str) -> None:
    """速度プロファイル分析"""
    print(f"\n{'=' * 60}")
    print(f"[{label}] 速度プロファイル分析")
    print(f"{'=' * 60}")

    dt = data[1, 0] - data[0, 0]
    col(header, data, "timestamp")
    vx = col(header, data, "base_vel_x")
    vy = col(header, data, "base_vel_y")
    pos_x = col(header, data, "base_pos_x")
    pos_z = col(header, data, "base_pos_z")

    # 1秒ごとの速度と位置
    print("  X方向速度・位置推移（1秒ごと）:")
    for t in range(0, 11):
        idx = int(t / dt)
        if idx < len(vx):
            print(f"    t={t:>2d}s: vx={vx[idx]:>+.4f} m/s, x={pos_x[idx]:>+.4f} m, z={pos_z[idx]:.4f} m")

    # 定常状態の速度統計
    start_idx = int(2.0 / dt)
    print("\n  定常状態 (t>2s):")
    print(f"    vx: mean={np.mean(vx[start_idx:]):.4f}, std={np.std(vx[start_idx:]):.4f}")
    print(f"    vy: mean={np.mean(vy[start_idx:]):.4f}, std={np.std(vy[start_idx:]):.4f}")
    print(f"    z(高さ): mean={np.mean(pos_z[start_idx:]):.4f}, std={np.std(pos_z[start_idx:]):.4f}")


def analyze_joint_dynamics(header: list[str], data: NDArray[np.floating], label: str) -> None:
    """関節角速度・トルク指標の分析"""
    print(f"\n{'=' * 60}")
    print(f"[{label}] 関節ダイナミクス分析")
    print(f"{'=' * 60}")

    dt = data[1, 0] - data[0, 0]
    start_idx = int(2.0 / dt)

    joints = ["hip_yaw", "hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch"]
    print("  定常状態の関節角速度 RMS (rad/s):")
    for j in joints:
        l_vel = col(header, data, f"dof_vel_L_{j}")[start_idx:]
        r_vel = col(header, data, f"dof_vel_R_{j}")[start_idx:]
        l_rms = np.sqrt(np.mean(l_vel**2))
        r_rms = np.sqrt(np.mean(r_vel**2))
        print(
            f"    {j:>14s}: L={l_rms:.3f}, R={r_rms:.3f}, ratio(L/R)={l_rms / r_rms:.3f}"
            if r_rms > 0
            else f"    {j:>14s}: L={l_rms:.3f}, R={r_rms:.3f}"
        )

    print("\n  定常状態のアクション RMS:")
    for i in range(10):
        act = col(header, data, f"action_{i}")[start_idx:]
        print(f"    action_{i}: rms={np.sqrt(np.mean(act**2)):.4f}, mean={np.mean(act):+.4f}")


def analyze_roll_lateral_sway(header: list[str], data: NDArray[np.floating], label: str) -> None:
    """Roll振動と横方向スウェイの分析"""
    print(f"\n{'=' * 60}")
    print(f"[{label}] Roll振動・横方向スウェイ分析")
    print(f"{'=' * 60}")

    dt = data[1, 0] - data[0, 0]
    start_idx = int(2.0 / dt)

    # 1. Roll角時系列統計
    roll = col(header, data, "roll_deg")[start_idx:]
    print("  Roll (定常状態):")
    print(f"    mean={np.mean(roll):.2f} deg, std={np.std(roll):.2f} deg")
    print(f"    range: [{np.min(roll):.2f}, {np.max(roll):.2f}] deg")
    print(f"    peak-to-peak: {np.max(roll) - np.min(roll):.2f} deg")

    # 2. Roll FFT分析（振動の主周波数）
    roll_detrend = roll - np.mean(roll)
    freqs = np.fft.rfftfreq(len(roll_detrend), d=dt)
    fft_mag = np.abs(np.fft.rfft(roll_detrend))
    fft_mag[0] = 0  # DC成分除外
    peak_idx = np.argmax(fft_mag)
    peak_freq = freqs[peak_idx]
    peak_period = 1.0 / peak_freq if peak_freq > 0 else float("inf")
    print(f"    Roll主振動周波数: {peak_freq:.2f} Hz (周期: {peak_period:.3f} s)")

    # 上位3ピーク
    top3_idx = np.argsort(fft_mag)[-3:][::-1]
    print("    上位3ピーク: ", end="")
    for i in top3_idx:
        if freqs[i] > 0:
            print(f"{freqs[i]:.2f}Hz(mag={fft_mag[i]:.1f}), ", end="")
    print()

    # 3. base_pos_y（横方向位置）統計
    pos_y = col(header, data, "base_pos_y")[start_idx:]
    print("\n  base_pos_y (横方向位置, 定常状態):")
    print(f"    mean={np.mean(pos_y):.4f} m, std={np.std(pos_y):.4f} m")
    print(f"    range: [{np.min(pos_y):.4f}, {np.max(pos_y):.4f}] m")

    # 4. base_vel_y（横方向速度）統計
    vel_y = col(header, data, "base_vel_y")[start_idx:]
    print("\n  base_vel_y (横方向速度, 定常状態):")
    print(f"    mean={np.mean(vel_y):.4f} m/s, std={np.std(vel_y):.4f} m/s")
    print(f"    range: [{np.min(vel_y):.4f}, {np.max(vel_y):.4f}] m/s")

    # 5. base_vel_y FFT
    vel_y_detrend = vel_y - np.mean(vel_y)
    freqs_vy = np.fft.rfftfreq(len(vel_y_detrend), d=dt)
    fft_mag_vy = np.abs(np.fft.rfft(vel_y_detrend))
    fft_mag_vy[0] = 0
    peak_idx_vy = np.argmax(fft_mag_vy)
    peak_freq_vy = freqs_vy[peak_idx_vy]
    print(f"    vel_y主振動周波数: {peak_freq_vy:.2f} Hz")

    # 6. Roll-vel_y相関（カップリングの確認）
    min_len = min(len(roll), len(vel_y))
    corr_roll_vy = np.corrcoef(roll[:min_len], vel_y[:min_len])[0, 1]
    print(f"\n  Roll - vel_y 相関係数: {corr_roll_vy:.4f}")

    # 7. hip_roll分析（L/R）
    l_hip_roll = col(header, data, "dof_pos_L_hip_roll")[start_idx:]
    r_hip_roll = col(header, data, "dof_pos_R_hip_roll")[start_idx:]
    print("\n  hip_roll (定常状態):")
    print(f"    L: mean={np.mean(l_hip_roll):.4f}, std={np.std(l_hip_roll):.4f} rad")
    print(f"    R: mean={np.mean(r_hip_roll):.4f}, std={np.std(r_hip_roll):.4f} rad")
    print(f"    L range: {np.max(l_hip_roll) - np.min(l_hip_roll):.4f} rad")
    print(f"    R range: {np.max(r_hip_roll) - np.min(r_hip_roll):.4f} rad")

    # hip_roll角速度RMS
    l_hip_roll_vel = col(header, data, "dof_vel_L_hip_roll")[start_idx:]
    r_hip_roll_vel = col(header, data, "dof_vel_R_hip_roll")[start_idx:]
    l_rms = np.sqrt(np.mean(l_hip_roll_vel**2))
    r_rms = np.sqrt(np.mean(r_hip_roll_vel**2))
    print(f"    L vel RMS: {l_rms:.3f} rad/s, R vel RMS: {r_rms:.3f} rad/s")

    # 8. Roll振動周波数 vs 歩行周波数の比較
    l_hip_pitch = col(header, data, "dof_pos_L_hip_pitch")[start_idx:]
    l_hp_detrend = l_hip_pitch - np.mean(l_hip_pitch)
    freqs_hp = np.fft.rfftfreq(len(l_hp_detrend), d=dt)
    fft_hp = np.abs(np.fft.rfft(l_hp_detrend))
    fft_hp[0] = 0
    gait_freq = freqs_hp[np.argmax(fft_hp)]
    freq_ratio = peak_freq / gait_freq if gait_freq > 0 else float("inf")
    print("\n  周波数比較:")
    print(f"    Roll振動周波数: {peak_freq:.2f} Hz")
    print(f"    歩行周波数: {gait_freq:.2f} Hz")
    print(f"    比率(Roll/gait): {freq_ratio:.2f}")
    if abs(freq_ratio - 1.0) < 0.15:
        print("    -> Roll振動は歩行周波数と同期（1:1）")
    elif abs(freq_ratio - 2.0) < 0.15:
        print("    -> Roll振動は歩行周波数の2倍（倍周波）")
    elif abs(freq_ratio - 0.5) < 0.15:
        print("    -> Roll振動は歩行周波数の半分（副周波）")
    else:
        print("    -> Roll振動と歩行周波数は非同期")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="CSV時系列データの詳細分析（複数バージョン比較）",
    )
    parser.add_argument(
        "versions",
        type=int,
        nargs="+",
        help="比較するバージョン番号（2つ以上）。先頭が新バージョン。例: 37 36",
    )
    parser.add_argument(
        "--epoch",
        type=str,
        default="3999",
        help="評価エポック番号またはファイルサフィックス（デフォルト: 3999）。"
        "固定コマンドCSVの場合は '3999_cmd_0.30_0.00_0.00_s1' のように指定",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="droid-walking-omni-v",
        help="Experiment name prefix (default: droid-walking-omni-v)",
    )
    args = parser.parse_args()
    if len(args.versions) < 2:
        parser.error("バージョン番号は2つ以上指定してください")
    return args


def main() -> None:
    args = parse_args()

    versions: list[tuple[str, str]] = []
    for v in args.versions:
        label = f"V{v}"
        path = f"logs/{args.prefix}{v}/eval_{args.epoch}.csv"
        versions.append((label, path))

    for label, path in versions:
        print(f"\n{'#' * 70}")
        print(f"# {label} 時系列分析")
        print(f"{'#' * 70}")

        header, data = load_csv(path)
        print(f"  データ: {len(data)} rows, {len(header)} columns")

        analyze_gait_cycle(header, data, label)
        analyze_contact_patterns(header, data, label)
        analyze_phase_relationship(header, data, label)
        analyze_yaw_drift(header, data, label)
        analyze_velocity_profile(header, data, label)
        analyze_joint_dynamics(header, data, label)
        analyze_roll_lateral_sway(header, data, label)

    # 比較サマリ（先頭2バージョン）
    label_new, path_new = versions[0]
    label_old, path_old = versions[1]

    print(f"\n{'#' * 70}")
    print(f"# {label_new} vs {label_old} 比較サマリ")
    print(f"{'#' * 70}")

    h_new, d_new = load_csv(path_new)
    h_old, d_old = load_csv(path_old)

    dt = d_new[1, 0] - d_new[0, 0]
    start = int(2.0 / dt)

    # 歩行対称性比較
    for label, h, d in [(label_new, h_new, d_new), (label_old, h_old, d_old)]:
        l_hip = col(h, d, "dof_pos_L_hip_pitch")[start:]
        r_hip = col(h, d, "dof_pos_R_hip_pitch")[start:]
        corr = np.corrcoef(l_hip, r_hip)[0, 1]
        l_range = np.max(l_hip) - np.min(l_hip)
        r_range = np.max(r_hip) - np.min(r_hip)
        asym = (l_range - r_range) / (l_range + r_range) * 100

        cl = col(h, d, "contact_left")[start:]
        cr = col(h, d, "contact_right")[start:]
        both = np.sum((cl == 1) & (cr == 1)) / len(cl) * 100
        l_only = np.sum((cl == 1) & (cr == 0)) / len(cl) * 100
        r_only = np.sum((cl == 0) & (cr == 1)) / len(cl) * 100

        print(f"\n  [{label}]")
        print(f"    hip_pitch相関: {corr:.4f}, 非対称度: {asym:+.1f}%")
        print(f"    接地(定常): 両足={both:.1f}%, L={l_only:.1f}%, R={r_only:.1f}%")


if __name__ == "__main__":
    main()
