#!/usr/bin/env python3
"""
Comprehensive analysis of V24 vs V23 contact dynamics, asymmetry, and Yaw drift.
Focuses on understanding why heel tapping is NOT improved despite ankle_pitch velocity std reduction.
"""

import pandas as pd
import numpy as np
from pathlib import Path

# ============================================================
# Load Data
# ============================================================
base = Path("/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs")
v24 = pd.read_csv(base / "droid-walking-narrow-v24" / "eval_499.csv")
v23 = pd.read_csv(base / "droid-walking-narrow-v23" / "eval_499.csv")

dt = 0.02  # 50 Hz
# Skip first 2 seconds of startup transient
skip_steps = int(2.0 / dt)

print(f"V24: {len(v24)} steps, V23: {len(v23)} steps")
print(f"Skipping first {skip_steps} steps (2.0s startup transient)")
print()

# ============================================================
# Helper Functions
# ============================================================

def detect_contact_onsets(contact_signal, min_gap=3):
    """Detect swing→stance transitions (0→1). Returns indices of first contact step."""
    onsets = []
    in_swing = True
    for i in range(1, len(contact_signal)):
        if in_swing and contact_signal.iloc[i] == 1.0:
            onsets.append(i)
            in_swing = False
        elif contact_signal.iloc[i] == 0.0:
            in_swing = True
    return np.array(onsets)

def detect_contact_offsets(contact_signal):
    """Detect stance→swing transitions (1→0). Returns indices of first swing step."""
    offsets = []
    in_stance = False
    for i in range(1, len(contact_signal)):
        if in_stance and contact_signal.iloc[i] == 0.0:
            offsets.append(i)
            in_stance = False
        elif contact_signal.iloc[i] == 1.0:
            in_stance = True
    return np.array(offsets)

def extract_window(series, center_idx, half_window=5):
    """Extract a window of values centered on center_idx."""
    start = max(0, center_idx - half_window)
    end = min(len(series), center_idx + half_window + 1)
    return series.iloc[start:end].values

def count_zero_crossings(signal):
    """Count zero crossings in a signal."""
    if len(signal) < 2:
        return 0
    signs = np.sign(signal)
    # Remove zeros (treat as same sign as previous)
    for i in range(len(signs)):
        if signs[i] == 0:
            signs[i] = signs[i-1] if i > 0 else 1
    crossings = np.sum(np.abs(np.diff(signs)) > 0)
    return crossings

# ============================================================
# TASK 1: Contact-moment ankle dynamics
# ============================================================
print("=" * 80)
print("TASK 1: CONTACT-MOMENT ANKLE DYNAMICS")
print("=" * 80)

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n{'='*60}")
    print(f"  {version_name}")
    print(f"{'='*60}")

    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    for side in ["L", "R"]:
        contact_col = f"contact_{'left' if side == 'L' else 'right'}"
        ankle_pos_col = f"dof_pos_{side}_ankle_pitch"
        ankle_vel_col = f"dof_vel_{side}_ankle_pitch"

        onsets = detect_contact_onsets(df_trimmed[contact_col])
        print(f"\n  --- {side} foot: {len(onsets)} contact onsets ---")

        if len(onsets) == 0:
            print("    No contact onsets detected!")
            continue

        # 1. Contact-moment ankle velocity (at exact onset step)
        contact_velocities = []
        contact_positions = []
        for idx in onsets:
            if idx < len(df_trimmed):
                contact_velocities.append(df_trimmed[ankle_vel_col].iloc[idx])
                contact_positions.append(df_trimmed[ankle_pos_col].iloc[idx])

        contact_velocities = np.array(contact_velocities)
        contact_positions = np.array(contact_positions)

        print(f"    Contact-moment ankle_pitch velocity:")
        print(f"      Mean: {np.mean(contact_velocities):.4f} rad/s")
        print(f"      Std:  {np.std(contact_velocities):.4f} rad/s")
        print(f"      |Mean|: {np.mean(np.abs(contact_velocities)):.4f} rad/s")
        print(f"      Max |vel|: {np.max(np.abs(contact_velocities)):.4f} rad/s")
        print(f"      Median |vel|: {np.median(np.abs(contact_velocities)):.4f} rad/s")

        print(f"    Contact-moment ankle_pitch position:")
        print(f"      Mean: {np.mean(contact_positions):.4f} rad ({np.degrees(np.mean(contact_positions)):.2f}°)")
        print(f"      Std:  {np.std(contact_positions):.4f} rad ({np.degrees(np.std(contact_positions)):.2f}°)")

        # 2. Pre-contact velocity (1 step before contact = last swing step)
        pre_contact_velocities = []
        for idx in onsets:
            if idx - 1 >= 0:
                pre_contact_velocities.append(df_trimmed[ankle_vel_col].iloc[idx - 1])
        pre_contact_velocities = np.array(pre_contact_velocities)
        print(f"    Pre-contact (1 step before) ankle_pitch velocity:")
        print(f"      Mean: {np.mean(pre_contact_velocities):.4f} rad/s")
        print(f"      |Mean|: {np.mean(np.abs(pre_contact_velocities)):.4f} rad/s")

        # 3. Post-contact oscillation (zero-crossings in first 10 steps = 0.2s)
        zc_counts = []
        for idx in onsets:
            end_idx = min(idx + 10, len(df_trimmed))
            if end_idx - idx >= 3:
                window_vel = df_trimmed[ankle_vel_col].iloc[idx:end_idx].values
                zc = count_zero_crossings(window_vel)
                zc_counts.append(zc)

        zc_counts = np.array(zc_counts)
        print(f"    Post-contact oscillation (zero-crossings in 0.2s):")
        print(f"      Mean: {np.mean(zc_counts):.2f}")
        print(f"      Std:  {np.std(zc_counts):.2f}")
        print(f"      Max:  {np.max(zc_counts)}")
        print(f"      Distribution: {dict(zip(*np.unique(zc_counts, return_counts=True)))}")

        # 4. Post-contact velocity envelope (peak in first 5 steps = 0.1s)
        peak_velocities = []
        for idx in onsets:
            end_idx = min(idx + 5, len(df_trimmed))
            if end_idx - idx >= 2:
                window_vel = df_trimmed[ankle_vel_col].iloc[idx:end_idx].values
                peak_velocities.append(np.max(np.abs(window_vel)))

        peak_velocities = np.array(peak_velocities)
        print(f"    Post-contact peak |velocity| (first 0.1s):")
        print(f"      Mean: {np.mean(peak_velocities):.4f} rad/s")
        print(f"      Std:  {np.std(peak_velocities):.4f} rad/s")
        print(f"      Median: {np.median(peak_velocities):.4f} rad/s")
        print(f"      90th percentile: {np.percentile(peak_velocities, 90):.4f} rad/s")

# ============================================================
# TASK 1.5: Foot XYZ velocity at contact
# ============================================================
print("\n" + "=" * 80)
print("TASK 1.5: JOINT VELOCITY AT CONTACT (X-Y-Z proxy via hip/knee/ankle)")
print("=" * 80)
print("(No direct foot velocity in CSV. Using joint velocities as proxy.)")

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n  --- {version_name} ---")
    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    for side in ["L", "R"]:
        contact_col = f"contact_{'left' if side == 'L' else 'right'}"
        onsets = detect_contact_onsets(df_trimmed[contact_col])

        joints = ["hip_yaw", "hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch"]
        print(f"  {side} foot ({len(onsets)} onsets):")
        for joint in joints:
            vel_col = f"dof_vel_{side}_{joint}"
            vals = [df_trimmed[vel_col].iloc[idx] for idx in onsets if idx < len(df_trimmed)]
            vals = np.array(vals)
            print(f"    {joint:15s}: mean={np.mean(vals):+.3f}, |mean|={np.mean(np.abs(vals)):.3f}, std={np.std(vals):.3f}")

# ============================================================
# TASK 1.6: Full-stance vs contact-moment ankle velocity comparison
# ============================================================
print("\n" + "=" * 80)
print("TASK 1.6: FULL-STANCE vs CONTACT-MOMENT ANKLE VELOCITY")
print("(This explains why 'std decreased' but heel tapping persists)")
print("=" * 80)

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n  --- {version_name} ---")
    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    for side in ["L", "R"]:
        contact_col = f"contact_{'left' if side == 'L' else 'right'}"
        ankle_vel_col = f"dof_vel_{side}_ankle_pitch"

        # Full stance average
        stance_mask = df_trimmed[contact_col] == 1.0
        full_stance_vel_std = df_trimmed.loc[stance_mask, ankle_vel_col].std()
        full_stance_vel_abs_mean = df_trimmed.loc[stance_mask, ankle_vel_col].abs().mean()

        # Contact-moment only (first 3 steps after contact onset)
        onsets = detect_contact_onsets(df_trimmed[contact_col])
        contact_moment_vels = []
        for idx in onsets:
            for offset in range(3):
                if idx + offset < len(df_trimmed):
                    contact_moment_vels.append(df_trimmed[ankle_vel_col].iloc[idx + offset])
        contact_moment_vels = np.array(contact_moment_vels)

        # Mid-stance average (steps 5-15 after contact onset)
        mid_stance_vels = []
        for idx in onsets:
            for offset in range(5, 15):
                if idx + offset < len(df_trimmed) and df_trimmed[contact_col].iloc[idx + offset] == 1.0:
                    mid_stance_vels.append(df_trimmed[ankle_vel_col].iloc[idx + offset])
        mid_stance_vels = np.array(mid_stance_vels)

        print(f"  {side} ankle_pitch velocity:")
        print(f"    Full-stance std:           {full_stance_vel_std:.4f} rad/s")
        print(f"    Full-stance |mean|:        {full_stance_vel_abs_mean:.4f} rad/s")
        print(f"    Contact-moment (0-3 steps) std:  {np.std(contact_moment_vels):.4f} rad/s")
        print(f"    Contact-moment (0-3 steps) |mean|: {np.mean(np.abs(contact_moment_vels)):.4f} rad/s")
        if len(mid_stance_vels) > 0:
            print(f"    Mid-stance (5-15 steps) std:    {np.std(mid_stance_vels):.4f} rad/s")
            print(f"    Mid-stance (5-15 steps) |mean|: {np.mean(np.abs(mid_stance_vels)):.4f} rad/s")
        print(f"    Ratio contact/mid-stance |mean|: {np.mean(np.abs(contact_moment_vels)) / (np.mean(np.abs(mid_stance_vels)) + 1e-10):.2f}x")

# ============================================================
# TASK 1.7: Ankle_pitch POSITION trajectory around contact
# ============================================================
print("\n" + "=" * 80)
print("TASK 1.7: ANKLE_PITCH POSITION TRAJECTORY AROUND CONTACT")
print("(Average trajectory ±5 steps around contact onset)")
print("=" * 80)

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n  --- {version_name} ---")
    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    for side in ["L", "R"]:
        contact_col = f"contact_{'left' if side == 'L' else 'right'}"
        ankle_pos_col = f"dof_pos_{side}_ankle_pitch"
        ankle_vel_col = f"dof_vel_{side}_ankle_pitch"

        onsets = detect_contact_onsets(df_trimmed[contact_col])

        # Collect windows
        half_w = 8
        pos_windows = []
        vel_windows = []
        for idx in onsets:
            if idx - half_w >= 0 and idx + half_w < len(df_trimmed):
                pos_win = df_trimmed[ankle_pos_col].iloc[idx-half_w:idx+half_w+1].values
                vel_win = df_trimmed[ankle_vel_col].iloc[idx-half_w:idx+half_w+1].values
                pos_windows.append(pos_win)
                vel_windows.append(vel_win)

        pos_windows = np.array(pos_windows)
        vel_windows = np.array(vel_windows)

        print(f"\n  {side} ankle_pitch around contact ({len(pos_windows)} events):")
        print(f"    Step | Pos(mean±std) rad | Vel(mean±std) rad/s | |Vel| mean")
        for step in range(2 * half_w + 1):
            t_rel = step - half_w
            label = "CONTACT" if t_rel == 0 else f"  t{t_rel:+d}  "
            pos_m = np.mean(pos_windows[:, step])
            pos_s = np.std(pos_windows[:, step])
            vel_m = np.mean(vel_windows[:, step])
            vel_s = np.std(vel_windows[:, step])
            vel_abs = np.mean(np.abs(vel_windows[:, step]))
            print(f"    {label}: {pos_m:+.4f}±{pos_s:.4f} | {vel_m:+.4f}±{vel_s:.4f} | {vel_abs:.4f}")


# ============================================================
# TASK 2: ASYMMETRY ANALYSIS
# ============================================================
print("\n\n" + "=" * 80)
print("TASK 2: ASYMMETRY ANALYSIS")
print("=" * 80)

def detect_gait_cycles(df, skip=100):
    """Detect gait cycles based on left foot contact onsets."""
    df_trimmed = df.iloc[skip:].reset_index(drop=True)
    l_onsets = detect_contact_onsets(df_trimmed["contact_left"])
    return df_trimmed, l_onsets

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n{'='*60}")
    print(f"  {version_name} - ASYMMETRY")
    print(f"{'='*60}")

    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    # 2.1: ankle_pitch range per gait cycle
    print(f"\n  --- 2.1: Ankle_pitch range per gait cycle ---")

    l_onsets = detect_contact_onsets(df_trimmed["contact_left"])
    r_onsets = detect_contact_onsets(df_trimmed["contact_right"])

    # Use L onsets to define gait cycles
    l_ranges = []
    r_ranges = []
    for i in range(len(l_onsets) - 1):
        start = l_onsets[i]
        end = l_onsets[i + 1]
        if end - start > 5:  # Minimum cycle length
            l_range = df_trimmed["dof_pos_L_ankle_pitch"].iloc[start:end].max() - df_trimmed["dof_pos_L_ankle_pitch"].iloc[start:end].min()
            r_range = df_trimmed["dof_pos_R_ankle_pitch"].iloc[start:end].max() - df_trimmed["dof_pos_R_ankle_pitch"].iloc[start:end].min()
            l_ranges.append(l_range)
            r_ranges.append(r_range)

    l_ranges = np.array(l_ranges)
    r_ranges = np.array(r_ranges)

    print(f"    L ankle_pitch range: mean={np.mean(l_ranges):.4f} rad ({np.degrees(np.mean(l_ranges)):.2f}°), std={np.std(l_ranges):.4f}")
    print(f"    R ankle_pitch range: mean={np.mean(r_ranges):.4f} rad ({np.degrees(np.mean(r_ranges)):.2f}°), std={np.std(r_ranges):.4f}")
    print(f"    L-R difference: {np.mean(l_ranges) - np.mean(r_ranges):.4f} rad ({np.degrees(np.mean(l_ranges) - np.mean(r_ranges)):.2f}°)")
    print(f"    Asymmetry (L-R)/mean: {(np.mean(l_ranges) - np.mean(r_ranges)) / (0.5*(np.mean(l_ranges) + np.mean(r_ranges))) * 100:.1f}%")

    # 2.2: knee_pitch L/R cross-correlation
    print(f"\n  --- 2.2: Knee_pitch L/R relationship ---")

    l_knee = df_trimmed["dof_pos_L_knee_pitch"].values
    r_knee = df_trimmed["dof_pos_R_knee_pitch"].values

    # Range per gait cycle
    l_knee_ranges = []
    r_knee_ranges = []
    for i in range(len(l_onsets) - 1):
        start = l_onsets[i]
        end = l_onsets[i + 1]
        if end - start > 5:
            l_kr = df_trimmed["dof_pos_L_knee_pitch"].iloc[start:end].max() - df_trimmed["dof_pos_L_knee_pitch"].iloc[start:end].min()
            r_kr = df_trimmed["dof_pos_R_knee_pitch"].iloc[start:end].max() - df_trimmed["dof_pos_R_knee_pitch"].iloc[start:end].min()
            l_knee_ranges.append(l_kr)
            r_knee_ranges.append(r_kr)

    l_knee_ranges = np.array(l_knee_ranges)
    r_knee_ranges = np.array(r_knee_ranges)

    print(f"    L knee_pitch range: mean={np.mean(l_knee_ranges):.4f} rad ({np.degrees(np.mean(l_knee_ranges)):.2f}°)")
    print(f"    R knee_pitch range: mean={np.mean(r_knee_ranges):.4f} rad ({np.degrees(np.mean(r_knee_ranges)):.2f}°)")
    asym = (np.mean(l_knee_ranges) - np.mean(r_knee_ranges)) / (0.5*(np.mean(l_knee_ranges) + np.mean(r_knee_ranges))) * 100
    print(f"    Asymmetry (L-R)/mean: {asym:.1f}%")

    # Cross-correlation at zero lag
    corr = np.corrcoef(l_knee[:min(len(l_knee), len(r_knee))], r_knee[:min(len(l_knee), len(r_knee))])[0, 1]
    print(f"    L/R cross-correlation (zero lag): {corr:.4f}")

    # 2.3: Contact timing L vs R
    print(f"\n  --- 2.3: Contact timing L vs R ---")

    for side, side_name in [("left", "L"), ("right", "R")]:
        contact_col = f"contact_{side}"
        onsets_s = detect_contact_onsets(df_trimmed[contact_col])
        offsets_s = detect_contact_offsets(df_trimmed[contact_col])

        # Stance durations
        stance_durations = []
        for on in onsets_s:
            # Find next offset after this onset
            next_offs = offsets_s[offsets_s > on]
            if len(next_offs) > 0:
                dur = (next_offs[0] - on) * dt
                stance_durations.append(dur)

        # Swing durations
        swing_durations = []
        for off in offsets_s:
            next_ons = onsets_s[onsets_s > off]
            if len(next_ons) > 0:
                dur = (next_ons[0] - off) * dt
                swing_durations.append(dur)

        stance_durations = np.array(stance_durations)
        swing_durations = np.array(swing_durations)

        print(f"    {side_name} stance duration: mean={np.mean(stance_durations):.4f}s, std={np.std(stance_durations):.4f}s, median={np.median(stance_durations):.4f}s")
        print(f"    {side_name} swing duration:  mean={np.mean(swing_durations):.4f}s, std={np.std(swing_durations):.4f}s, median={np.median(swing_durations):.4f}s")

    # Contact ratio
    l_contact_frac = df_trimmed["contact_left"].mean()
    r_contact_frac = df_trimmed["contact_right"].mean()
    print(f"    L contact fraction: {l_contact_frac*100:.1f}%")
    print(f"    R contact fraction: {r_contact_frac*100:.1f}%")
    both_contact = ((df_trimmed["contact_left"] == 1) & (df_trimmed["contact_right"] == 1)).mean()
    print(f"    Both feet contact: {both_contact*100:.1f}%")


# ============================================================
# TASK 3: YAW DRIFT CAUSAL ANALYSIS
# ============================================================
print("\n\n" + "=" * 80)
print("TASK 3: YAW DRIFT CAUSAL ANALYSIS")
print("=" * 80)

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n{'='*60}")
    print(f"  {version_name} - YAW ANALYSIS")
    print(f"{'='*60}")

    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)
    l_onsets = detect_contact_onsets(df_trimmed["contact_left"])

    # 3.1: Per-gait-cycle Yaw change
    print(f"\n  --- 3.1: Per-gait-cycle Yaw change ---")

    yaw_changes = []
    cycle_times = []
    for i in range(len(l_onsets) - 1):
        start = l_onsets[i]
        end = l_onsets[i + 1]
        yaw_start = df_trimmed["yaw_deg"].iloc[start]
        yaw_end = df_trimmed["yaw_deg"].iloc[end]
        yaw_changes.append(yaw_end - yaw_start)
        cycle_times.append(df_trimmed["timestamp"].iloc[start])

    yaw_changes = np.array(yaw_changes)
    cycle_times = np.array(cycle_times)

    print(f"    Number of gait cycles: {len(yaw_changes)}")
    print(f"    Yaw change per cycle: mean={np.mean(yaw_changes):.4f}°, std={np.std(yaw_changes):.4f}°")
    print(f"    Cumulative Yaw at end: {df_trimmed['yaw_deg'].iloc[-1]:.2f}°")

    # Check if constant (systematic) or growing (instability)
    if len(yaw_changes) > 10:
        first_half = yaw_changes[:len(yaw_changes)//2]
        second_half = yaw_changes[len(yaw_changes)//2:]
        print(f"    First half mean: {np.mean(first_half):.4f}°/cycle")
        print(f"    Second half mean: {np.mean(second_half):.4f}°/cycle")

        # Linear regression on yaw_changes vs cycle index
        from numpy.polynomial.polynomial import polyfit
        cycle_idx = np.arange(len(yaw_changes))
        coefs = polyfit(cycle_idx, yaw_changes, 1)
        print(f"    Trend (linear fit): intercept={coefs[0]:.4f}°, slope={coefs[1]:.6f}°/cycle")
        if abs(coefs[1]) < 0.01:
            print(f"    → SYSTEMATIC BIAS (constant per-cycle drift)")
        else:
            print(f"    → GROWING INSTABILITY (drift accelerates)")

    # 3.2: Correlation: ankle asymmetry → hip_yaw → Yaw
    print(f"\n  --- 3.2: Ankle asymmetry → hip_yaw → Yaw correlation ---")

    ankle_asym_per_cycle = []
    hip_yaw_asym_per_cycle = []
    hip_roll_asym_per_cycle = []
    hip_pitch_asym_per_cycle = []
    yaw_rate_per_cycle = []

    for i in range(len(l_onsets) - 1):
        start = l_onsets[i]
        end = l_onsets[i + 1]
        if end - start < 5:
            continue

        segment = df_trimmed.iloc[start:end]

        l_ankle_range = segment["dof_pos_L_ankle_pitch"].max() - segment["dof_pos_L_ankle_pitch"].min()
        r_ankle_range = segment["dof_pos_R_ankle_pitch"].max() - segment["dof_pos_R_ankle_pitch"].min()
        ankle_asym_per_cycle.append(l_ankle_range - r_ankle_range)

        l_hip_yaw_range = segment["dof_pos_L_hip_yaw"].max() - segment["dof_pos_L_hip_yaw"].min()
        r_hip_yaw_range = segment["dof_pos_R_hip_yaw"].max() - segment["dof_pos_R_hip_yaw"].min()
        hip_yaw_asym_per_cycle.append(l_hip_yaw_range - r_hip_yaw_range)

        l_hip_roll_range = segment["dof_pos_L_hip_roll"].max() - segment["dof_pos_L_hip_roll"].min()
        r_hip_roll_range = segment["dof_pos_R_hip_roll"].max() - segment["dof_pos_R_hip_roll"].min()
        hip_roll_asym_per_cycle.append(l_hip_roll_range - r_hip_roll_range)

        l_hip_pitch_range = segment["dof_pos_L_hip_pitch"].max() - segment["dof_pos_L_hip_pitch"].min()
        r_hip_pitch_range = segment["dof_pos_R_hip_pitch"].max() - segment["dof_pos_R_hip_pitch"].min()
        hip_pitch_asym_per_cycle.append(l_hip_pitch_range - r_hip_pitch_range)

        cycle_dur = (end - start) * dt
        yaw_rate = (segment["yaw_deg"].iloc[-1] - segment["yaw_deg"].iloc[0]) / cycle_dur
        yaw_rate_per_cycle.append(yaw_rate)

    ankle_asym_per_cycle = np.array(ankle_asym_per_cycle)
    hip_yaw_asym_per_cycle = np.array(hip_yaw_asym_per_cycle)
    hip_roll_asym_per_cycle = np.array(hip_roll_asym_per_cycle)
    hip_pitch_asym_per_cycle = np.array(hip_pitch_asym_per_cycle)
    yaw_rate_per_cycle = np.array(yaw_rate_per_cycle)

    print(f"    Cycles analyzed: {len(yaw_rate_per_cycle)}")
    print(f"    Ankle asymmetry (L-R range): mean={np.mean(ankle_asym_per_cycle):.4f} rad, std={np.std(ankle_asym_per_cycle):.4f}")
    print(f"    Hip_yaw asymmetry (L-R range): mean={np.mean(hip_yaw_asym_per_cycle):.4f} rad, std={np.std(hip_yaw_asym_per_cycle):.4f}")
    print(f"    Hip_roll asymmetry (L-R range): mean={np.mean(hip_roll_asym_per_cycle):.4f} rad, std={np.std(hip_roll_asym_per_cycle):.4f}")
    print(f"    Hip_pitch asymmetry (L-R range): mean={np.mean(hip_pitch_asym_per_cycle):.4f} rad, std={np.std(hip_pitch_asym_per_cycle):.4f}")
    print(f"    Yaw rate: mean={np.mean(yaw_rate_per_cycle):.4f}°/s, std={np.std(yaw_rate_per_cycle):.4f}°/s")

    # Correlations
    if len(yaw_rate_per_cycle) > 5:
        corr_ankle_yaw = np.corrcoef(ankle_asym_per_cycle, yaw_rate_per_cycle)[0, 1]
        corr_hipyaw_yaw = np.corrcoef(hip_yaw_asym_per_cycle, yaw_rate_per_cycle)[0, 1]
        corr_hiproll_yaw = np.corrcoef(hip_roll_asym_per_cycle, yaw_rate_per_cycle)[0, 1]
        corr_hippitch_yaw = np.corrcoef(hip_pitch_asym_per_cycle, yaw_rate_per_cycle)[0, 1]
        corr_ankle_hipyaw = np.corrcoef(ankle_asym_per_cycle, hip_yaw_asym_per_cycle)[0, 1]

        print(f"    Correlations with Yaw rate:")
        print(f"      ankle_asym → Yaw rate:     r={corr_ankle_yaw:.4f}")
        print(f"      hip_yaw_asym → Yaw rate:   r={corr_hipyaw_yaw:.4f}")
        print(f"      hip_roll_asym → Yaw rate:   r={corr_hiproll_yaw:.4f}")
        print(f"      hip_pitch_asym → Yaw rate:  r={corr_hippitch_yaw:.4f}")
        print(f"      ankle_asym → hip_yaw_asym:  r={corr_ankle_hipyaw:.4f}")

    # 3.3: Action asymmetry
    print(f"\n  --- 3.3: Action asymmetry (L vs R) ---")
    print(f"    Action mapping: 0-4 = L(hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch)")
    print(f"                    5-9 = R(hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch)")

    joint_names = ["hip_yaw", "hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch"]
    for j, jname in enumerate(joint_names):
        l_action = df_trimmed[f"action_{j}"].values
        r_action = df_trimmed[f"action_{j+5}"].values
        l_mean = np.mean(l_action)
        r_mean = np.mean(r_action)
        l_std = np.std(l_action)
        r_std = np.std(r_action)
        diff_mean = l_mean - r_mean
        # For a "symmetric" gait, L and R should be mirror images
        # hip_yaw: L positive ↔ R negative (or vice versa)
        # hip_roll: L and R should be similar (both legs roll same way relative to body)
        # hip_pitch, knee_pitch, ankle_pitch: L and R should be anti-phase but similar magnitude
        print(f"    {jname:15s}: L_mean={l_mean:+.4f}(std={l_std:.3f}), R_mean={r_mean:+.4f}(std={r_std:.3f}), diff={diff_mean:+.4f}")

# ============================================================
# TASK 3.4: Contact velocity per direction (XYZ proxy)
# ============================================================
print("\n" + "=" * 80)
print("TASK 3.4: VELOCITY MAGNITUDE AT CONTACT (all joints summed)")
print("(Proxy for foot impact velocity)")
print("=" * 80)

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n  --- {version_name} ---")
    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    for side in ["L", "R"]:
        contact_col = f"contact_{'left' if side == 'L' else 'right'}"
        onsets = detect_contact_onsets(df_trimmed[contact_col])

        # Total kinetic energy proxy: sum of squared joint velocities at contact
        joints = ["hip_yaw", "hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch"]
        total_vel_sq = []
        for idx in onsets:
            if idx < len(df_trimmed):
                sq_sum = sum(df_trimmed[f"dof_vel_{side}_{j}"].iloc[idx]**2 for j in joints)
                total_vel_sq.append(np.sqrt(sq_sum))
        total_vel_sq = np.array(total_vel_sq)
        print(f"  {side} foot: ||joint_vel|| at contact = mean={np.mean(total_vel_sq):.3f}, std={np.std(total_vel_sq):.3f}")


# ============================================================
# TASK 4: V24 contact_no_vel effectiveness check
# ============================================================
print("\n\n" + "=" * 80)
print("TASK 4: contact_no_vel EFFECTIVENESS - WHY HEEL TAPPING PERSISTS")
print("=" * 80)

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n{'='*60}")
    print(f"  {version_name}")
    print(f"{'='*60}")

    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    for side in ["L", "R"]:
        contact_col = f"contact_{'left' if side == 'L' else 'right'}"

        # During stance: analyze velocity patterns
        stance_mask = df_trimmed[contact_col] == 1.0

        # ankle_pitch position during stance
        ankle_pos = df_trimmed.loc[stance_mask, f"dof_pos_{side}_ankle_pitch"]
        ankle_vel = df_trimmed.loc[stance_mask, f"dof_vel_{side}_ankle_pitch"]
        knee_vel = df_trimmed.loc[stance_mask, f"dof_vel_{side}_knee_pitch"]

        print(f"\n  {side} foot during STANCE:")
        print(f"    ankle_pitch pos: mean={ankle_pos.mean():.4f} ({np.degrees(ankle_pos.mean()):.2f}°), std={ankle_pos.std():.4f}")
        print(f"    ankle_pitch vel: std={ankle_vel.std():.4f}, |mean|={ankle_vel.abs().mean():.4f}")
        print(f"    knee_pitch vel:  std={knee_vel.std():.4f}, |mean|={knee_vel.abs().mean():.4f}")

        # Check if ankle is saturating at position limits
        print(f"    ankle_pitch pos min={ankle_pos.min():.4f} ({np.degrees(ankle_pos.min()):.2f}°), max={ankle_pos.max():.4f} ({np.degrees(ankle_pos.max()):.2f}°)")

        # ankle_pitch action during stance
        action_idx = 4 if side == "L" else 9
        ankle_action = df_trimmed.loc[stance_mask, f"action_{action_idx}"]
        print(f"    ankle_pitch action: mean={ankle_action.mean():.4f}, std={ankle_action.std():.4f}")

        # Percentage of stance with high ankle velocity (>2 rad/s)
        high_vel_frac = (ankle_vel.abs() > 2.0).mean() * 100
        very_high_vel_frac = (ankle_vel.abs() > 5.0).mean() * 100
        print(f"    Stance time with |ankle_vel| > 2 rad/s: {high_vel_frac:.1f}%")
        print(f"    Stance time with |ankle_vel| > 5 rad/s: {very_high_vel_frac:.1f}%")

# ============================================================
# TASK 5: CONTACT DURATION vs ANKLE BEHAVIOR
# ============================================================
print("\n\n" + "=" * 80)
print("TASK 5: MICRO-CONTACT / CHATTERING ANALYSIS")
print("(Detecting very short contact durations that indicate heel tapping)")
print("=" * 80)

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n  --- {version_name} ---")
    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    for side in ["L", "R"]:
        contact_col = f"contact_{'left' if side == 'L' else 'right'}"
        onsets = detect_contact_onsets(df_trimmed[contact_col])
        offsets = detect_contact_offsets(df_trimmed[contact_col])

        # Compute all contact durations
        contact_durations = []
        for on in onsets:
            next_off = offsets[offsets > on]
            if len(next_off) > 0:
                dur_steps = next_off[0] - on
                contact_durations.append(dur_steps)

        contact_durations = np.array(contact_durations)

        print(f"\n  {side} foot contact durations ({len(contact_durations)} contacts):")
        print(f"    Mean: {np.mean(contact_durations):.1f} steps ({np.mean(contact_durations)*dt:.3f}s)")
        print(f"    Std:  {np.std(contact_durations):.1f} steps")
        print(f"    Min:  {np.min(contact_durations)} steps ({np.min(contact_durations)*dt:.3f}s)")
        print(f"    Max:  {np.max(contact_durations)} steps ({np.max(contact_durations)*dt:.3f}s)")

        # Distribution of contact durations
        bins = [1, 2, 3, 5, 10, 20, 50, 100]
        print(f"    Duration distribution:")
        for i in range(len(bins) - 1):
            count = np.sum((contact_durations >= bins[i]) & (contact_durations < bins[i+1]))
            pct = count / len(contact_durations) * 100
            print(f"      {bins[i]}-{bins[i+1]-1} steps ({bins[i]*dt:.2f}-{(bins[i+1]-1)*dt:.2f}s): {count} ({pct:.1f}%)")
        count = np.sum(contact_durations >= bins[-1])
        pct = count / len(contact_durations) * 100
        print(f"      {bins[-1]}+ steps ({bins[-1]*dt:.2f}s+): {count} ({pct:.1f}%)")

        # Micro-contacts (1-2 steps): these are the "heel taps"
        micro_contacts = np.sum(contact_durations <= 2)
        print(f"    MICRO-CONTACTS (≤2 steps, ≤0.04s): {micro_contacts} ({micro_contacts/len(contact_durations)*100:.1f}%)")

        # Inter-contact gaps (swing durations)
        swing_durations = []
        for off in offsets:
            next_on = onsets[onsets > off]
            if len(next_on) > 0:
                dur_steps = next_on[0] - off
                swing_durations.append(dur_steps)
        swing_durations = np.array(swing_durations)

        if len(swing_durations) > 0:
            micro_swings = np.sum(swing_durations <= 2)
            print(f"    MICRO-SWINGS (≤2 steps gap): {micro_swings} ({micro_swings/len(swing_durations)*100:.1f}%) ← contact chattering")

# ============================================================
# TASK 6: Ankle_pitch ACTION vs VELOCITY phase relationship
# ============================================================
print("\n\n" + "=" * 80)
print("TASK 6: ANKLE_PITCH ACTION vs POSITION DURING STANCE")
print("(Does the policy actively drive heel tapping?)")
print("=" * 80)

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n  --- {version_name} ---")
    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    for side in ["L", "R"]:
        contact_col = f"contact_{'left' if side == 'L' else 'right'}"
        ankle_pos_col = f"dof_pos_{side}_ankle_pitch"
        ankle_vel_col = f"dof_vel_{side}_ankle_pitch"
        action_idx = 4 if side == "L" else 9
        action_col = f"action_{action_idx}"

        stance_mask = df_trimmed[contact_col] == 1.0

        # Correlation between action and velocity during stance
        stance_action = df_trimmed.loc[stance_mask, action_col].values
        stance_vel = df_trimmed.loc[stance_mask, ankle_vel_col].values
        stance_pos = df_trimmed.loc[stance_mask, ankle_pos_col].values

        corr_act_vel = np.corrcoef(stance_action, stance_vel)[0, 1]
        corr_act_pos = np.corrcoef(stance_action, stance_pos)[0, 1]

        print(f"  {side} foot (stance only):")
        print(f"    action-velocity correlation: {corr_act_vel:.4f}")
        print(f"    action-position correlation: {corr_act_pos:.4f}")
        print(f"    action mean={np.mean(stance_action):.4f}, std={np.std(stance_action):.4f}")

        # Check if action oscillates (sign changes)
        action_sign_changes = np.sum(np.abs(np.diff(np.sign(stance_action))) > 0)
        total_steps = len(stance_action) - 1
        print(f"    action sign changes: {action_sign_changes}/{total_steps} ({action_sign_changes/total_steps*100:.1f}%)")

# ============================================================
# SUMMARY
# ============================================================
print("\n\n" + "=" * 80)
print("SUMMARY TABLE: V23 vs V24")
print("=" * 80)

# Collect key metrics
for version_name, df in [("V23", v23), ("V24", v24)]:
    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    print(f"\n  {version_name}:")
    print(f"    Final Yaw: {df_trimmed['yaw_deg'].iloc[-1]:.2f}°")
    print(f"    X velocity mean: {df_trimmed['base_vel_x'].mean():.4f} m/s")
    print(f"    Roll std: {df_trimmed['roll_deg'].std():.2f}°")
    print(f"    Pitch std: {df_trimmed['pitch_deg'].std():.2f}°")

    for side in ["L", "R"]:
        ankle_vel = df_trimmed[f"dof_vel_{side}_ankle_pitch"]
        print(f"    {side} ankle_pitch vel: full_std={ankle_vel.std():.4f}, |mean|={ankle_vel.abs().mean():.4f}")

        contact_col = f"contact_{'left' if side == 'L' else 'right'}"
        stance_mask = df_trimmed[contact_col] == 1.0
        stance_vel = df_trimmed.loc[stance_mask, f"dof_vel_{side}_ankle_pitch"]
        print(f"    {side} ankle_pitch vel (stance): std={stance_vel.std():.4f}, |mean|={stance_vel.abs().mean():.4f}")

print("\n\nAnalysis complete.")
