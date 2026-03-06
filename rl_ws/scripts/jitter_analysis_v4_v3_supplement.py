#!/usr/bin/env python3
"""Supplementary jitter analysis: deeper dives into key findings."""

import numpy as np

DT = 0.02
STEADY_START = 100

JOINT_NAMES = [
    "L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee_pitch", "L_ankle_pitch",
    "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee_pitch", "R_ankle_pitch",
]

def load_data(path):
    data = np.loadtxt(path, delimiter=',', skiprows=1)
    return data

def get_header(path):
    with open(path, 'r') as f:
        return f.readline()

def get_columns(header_line):
    cols = header_line.strip().split(',')
    return {name: i for i, name in enumerate(cols)}

v4_path = "/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs/droid-walking-omni-v4/eval_1999.csv"
v3_path = "/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs/droid-walking-omni-v3/eval_1999.csv"

header = get_header(v4_path)
col_idx = get_columns(header)

v4_all = load_data(v4_path)
v3_all = load_data(v3_path)
v4 = v4_all[STEADY_START:]
v3 = v3_all[STEADY_START:]

# ============================================================
# SUPPLEMENTARY A: Micro-swing/stance chattering analysis
# ============================================================
print("=" * 80)
print("SUPPLEMENT A: CONTACT CHATTERING DEEP DIVE")
print("=" * 80)
print()

contact_l_idx = col_idx["contact_left"]
contact_r_idx = col_idx["contact_right"]

def analyze_chattering(contact, label):
    """Detect chattering: rapid on/off transitions."""
    transitions = np.abs(np.diff(contact)) > 0.5

    # Find consecutive transition distances
    trans_indices = np.where(transitions)[0]
    if len(trans_indices) < 2:
        return

    inter_trans = np.diff(trans_indices) * DT  # time between transitions

    print(f"  {label}:")
    print(f"    Total transitions: {len(trans_indices)}")
    print(f"    Inter-transition times:")
    print(f"      Min: {np.min(inter_trans):.4f}s")
    print(f"      Mean: {np.mean(inter_trans):.4f}s")
    print(f"      Median: {np.median(inter_trans):.4f}s")
    print(f"      < 0.04s (2 steps): {np.sum(inter_trans < 0.04)} ({100*np.sum(inter_trans < 0.04)/len(inter_trans):.1f}%)")
    print(f"      < 0.06s (3 steps): {np.sum(inter_trans < 0.06)} ({100*np.sum(inter_trans < 0.06)/len(inter_trans):.1f}%)")
    print(f"      < 0.10s (5 steps): {np.sum(inter_trans < 0.10)} ({100*np.sum(inter_trans < 0.10)/len(inter_trans):.1f}%)")

    # Chattering bursts: sequences of rapid transitions
    chatter_threshold = 0.06  # transitions faster than 0.06s
    in_chatter = inter_trans < chatter_threshold
    chatter_count = 0
    chatter_durations = []
    current_duration = 0
    for i, is_chatter in enumerate(in_chatter):
        if is_chatter:
            if current_duration == 0:
                chatter_count += 1
            current_duration += inter_trans[i]
        else:
            if current_duration > 0:
                chatter_durations.append(current_duration)
            current_duration = 0
    if current_duration > 0:
        chatter_durations.append(current_duration)

    print(f"    Chatter bursts (<0.06s threshold): {chatter_count}")
    if chatter_durations:
        print(f"    Chatter burst durations: {[f'{d:.3f}s' for d in chatter_durations]}")

print("V4:")
analyze_chattering(v4[:, contact_l_idx], "Left foot")
analyze_chattering(v4[:, contact_r_idx], "Right foot")
print("\nV3:")
analyze_chattering(v3[:, contact_l_idx], "Left foot")
analyze_chattering(v3[:, contact_r_idx], "Right foot")
print()

# ============================================================
# SUPPLEMENT B: action_1 (L_hip_roll) deep dive
# ============================================================
print("=" * 80)
print("SUPPLEMENT B: action_1 (L_hip_roll) DEEP DIVE - Worst Jitter Action")
print("=" * 80)
print()

a1_idx = col_idx["action_1"]
v4_a1 = v4[:, a1_idx]
v3_a1 = v3[:, a1_idx]

# Action statistics
print(f"  {'Metric':<30} {'V4':<14} {'V3':<14} {'V4/V3':<10}")
print(f"  {'-'*68}")
print(f"  {'Mean':<30} {np.mean(v4_a1):<14.4f} {np.mean(v3_a1):<14.4f}")
print(f"  {'Std':<30} {np.std(v4_a1):<14.4f} {np.std(v3_a1):<14.4f} {np.std(v4_a1)/np.std(v3_a1):.3f}")
print(f"  {'Range':<30} {np.ptp(v4_a1):<14.4f} {np.ptp(v3_a1):<14.4f} {np.ptp(v4_a1)/np.ptp(v3_a1):.3f}")
print(f"  {'Max abs':<30} {np.max(np.abs(v4_a1)):<14.4f} {np.max(np.abs(v3_a1)):<14.4f}")

# Derivative statistics
v4_a1_d = np.diff(v4_a1) / DT
v3_a1_d = np.diff(v3_a1) / DT
print(f"  {'Deriv RMS':<30} {np.sqrt(np.mean(v4_a1_d**2)):<14.2f} {np.sqrt(np.mean(v3_a1_d**2)):<14.2f} {np.sqrt(np.mean(v4_a1_d**2))/np.sqrt(np.mean(v3_a1_d**2)):.3f}")
print(f"  {'Deriv max abs':<30} {np.max(np.abs(v4_a1_d)):<14.2f} {np.max(np.abs(v3_a1_d)):<14.2f}")
print(f"  {'Deriv > 50 /s count':<30} {np.sum(np.abs(v4_a1_d) > 50):<14} {np.sum(np.abs(v3_a1_d) > 50):<14}")
print(f"  {'Deriv > 100 /s count':<30} {np.sum(np.abs(v4_a1_d) > 100):<14} {np.sum(np.abs(v3_a1_d) > 100):<14}")

# Zero crossings (sign changes)
v4_zc = np.sum(np.diff(np.sign(v4_a1_d)) != 0)
v3_zc = np.sum(np.diff(np.sign(v3_a1_d)) != 0)
print(f"  {'Derivative zero-crossings':<30} {v4_zc:<14} {v3_zc:<14} {v4_zc/v3_zc if v3_zc>0 else 'inf'}")
print()

# ============================================================
# SUPPLEMENT C: L/R hip_yaw dominant frequency shift
# ============================================================
print("=" * 80)
print("SUPPLEMENT C: HIP_YAW FREQUENCY ANALYSIS")
print("=" * 80)
print()

N = len(v4)
freqs = np.fft.rfftfreq(N, d=DT)

for jn in ["L_hip_yaw", "R_hip_yaw"]:
    jcol = col_idx[f"dof_pos_{jn}"]
    v4_sig = v4[:, jcol] - np.mean(v4[:, jcol])
    v3_sig = v3[:, jcol] - np.mean(v3[:, jcol])

    v4_fft = np.fft.rfft(v4_sig)
    v3_fft = np.fft.rfft(v3_sig)
    v4_power = np.abs(v4_fft)**2
    v3_power = np.abs(v3_fft)**2

    # Top 5 frequencies
    v4_top5 = np.argsort(v4_power[1:])[-5:][::-1] + 1
    v3_top5 = np.argsort(v3_power[1:])[-5:][::-1] + 1

    print(f"  {jn}:")
    print(f"    V4 top-5 frequencies: {', '.join([f'{freqs[i]:.2f}Hz ({v4_power[i]/np.sum(v4_power)*100:.1f}%)' for i in v4_top5])}")
    print(f"    V3 top-5 frequencies: {', '.join([f'{freqs[i]:.2f}Hz ({v3_power[i]/np.sum(v3_power)*100:.1f}%)' for i in v3_top5])}")

    # 2-5Hz band power
    band_mid = (freqs >= 2) & (freqs < 5)
    v4_mid = np.sum(v4_power[band_mid]) / np.sum(v4_power) * 100
    v3_mid = np.sum(v3_power[band_mid]) / np.sum(v3_power) * 100
    print(f"    2-5Hz band: V4={v4_mid:.1f}%  V3={v3_mid:.1f}%  V4/V3={v4_mid/v3_mid if v3_mid>0 else 'inf':.2f}")
    print()

# ============================================================
# SUPPLEMENT D: DOF velocity per-joint analysis
# ============================================================
print("=" * 80)
print("SUPPLEMENT D: DOF VELOCITY PER-JOINT ANALYSIS")
print("=" * 80)
print()

print(f"{'Joint':<18} {'V4 vel RMS':<14} {'V3 vel RMS':<14} {'V4/V3':<10} {'V4 vel max':<14} {'V3 vel max':<14}")
print("-" * 84)
for jn in JOINT_NAMES:
    vcol = col_idx[f"dof_vel_{jn}"]
    v4_rms = np.sqrt(np.mean(v4[:, vcol]**2))
    v3_rms = np.sqrt(np.mean(v3[:, vcol]**2))
    v4_max = np.max(np.abs(v4[:, vcol]))
    v3_max = np.max(np.abs(v3[:, vcol]))
    ratio = v4_rms / v3_rms if v3_rms > 0 else float('inf')
    print(f"{jn:<18} {v4_rms:<14.3f} {v3_rms:<14.3f} {ratio:<10.3f} {v4_max:<14.3f} {v3_max:<14.3f}")
print()

# ============================================================
# SUPPLEMENT E: Command velocity ranges
# ============================================================
print("=" * 80)
print("SUPPLEMENT E: COMMAND VELOCITY RANGES")
print("=" * 80)
print()

cmd_x = col_idx["cmd_vel_x"]
cmd_y = col_idx["cmd_vel_y"]
cmd_yaw = col_idx["cmd_vel_yaw"]

for label, data in [("V4", v4), ("V3", v3)]:
    print(f"  {label}:")
    print(f"    cmd_x:   min={np.min(data[:,cmd_x]):.3f}, max={np.max(data[:,cmd_x]):.3f}, mean={np.mean(data[:,cmd_x]):.3f}")
    print(f"    cmd_y:   min={np.min(data[:,cmd_y]):.3f}, max={np.max(data[:,cmd_y]):.3f}, mean={np.mean(data[:,cmd_y]):.3f}")
    print(f"    cmd_yaw: min={np.min(data[:,cmd_yaw]):.3f}, max={np.max(data[:,cmd_yaw]):.3f}, mean={np.mean(data[:,cmd_yaw]):.3f}")
    cmd_norm = np.sqrt(data[:,cmd_x]**2 + data[:,cmd_y]**2)
    print(f"    ||cmd_xy||: min={np.min(cmd_norm):.3f}, max={np.max(cmd_norm):.3f}, mean={np.mean(cmd_norm):.3f}")
print()

# ============================================================
# SUPPLEMENT F: L_hip_pitch and R_hip_pitch detailed FFT comparison
# ============================================================
print("=" * 80)
print("SUPPLEMENT F: HIP PITCH DETAILED FFT (KEY JITTER INDICATOR)")
print("=" * 80)
print()

for jn in ["L_hip_pitch", "R_hip_pitch"]:
    jcol = col_idx[f"dof_pos_{jn}"]
    v4_sig = v4[:, jcol] - np.mean(v4[:, jcol])
    v3_sig = v3[:, jcol] - np.mean(v3[:, jcol])

    v4_fft = np.fft.rfft(v4_sig)
    v3_fft = np.fft.rfft(v3_sig)
    v4_power = np.abs(v4_fft)**2
    v3_power = np.abs(v3_fft)**2

    v4_top5 = np.argsort(v4_power[1:])[-5:][::-1] + 1
    v3_top5 = np.argsort(v3_power[1:])[-5:][::-1] + 1

    band_0_1 = (freqs >= 0) & (freqs < 1)
    band_1_2 = (freqs >= 1) & (freqs < 2)
    band_2_3 = (freqs >= 2) & (freqs < 3)
    band_3_5 = (freqs >= 3) & (freqs < 5)
    band_5_10 = (freqs >= 5) & (freqs < 10)
    band_10plus = freqs >= 10

    v4_total = np.sum(v4_power)
    v3_total = np.sum(v3_power)

    print(f"  {jn}:")
    print(f"    V4 top-5: {', '.join([f'{freqs[i]:.2f}Hz ({v4_power[i]/v4_total*100:.1f}%)' for i in v4_top5])}")
    print(f"    V3 top-5: {', '.join([f'{freqs[i]:.2f}Hz ({v3_power[i]/v3_total*100:.1f}%)' for i in v3_top5])}")

    print(f"    Band distribution:")
    print(f"    {'Band':<12} {'V4 %':<10} {'V3 %':<10} {'V4/V3 power':<14}")
    for band_label, band_mask in [("0-1Hz", band_0_1), ("1-2Hz", band_1_2), ("2-3Hz", band_2_3),
                                   ("3-5Hz", band_3_5), ("5-10Hz", band_5_10), ("10+Hz", band_10plus)]:
        v4_pct = np.sum(v4_power[band_mask]) / v4_total * 100
        v3_pct = np.sum(v3_power[band_mask]) / v3_total * 100
        v4_abs = np.sum(v4_power[band_mask])
        v3_abs = np.sum(v3_power[band_mask])
        ratio = v4_abs / v3_abs if v3_abs > 0 else float('inf')
        print(f"    {band_label:<12} {v4_pct:<10.1f} {v3_pct:<10.1f} {ratio:<14.2f}")
    print()

# ============================================================
# SUPPLEMENT G: Micro-stance analysis (key jitter indicator)
# ============================================================
print("=" * 80)
print("SUPPLEMENT G: MICRO-EVENT TIMELINE (first 2s of steady state)")
print("=" * 80)
print()

# Show contact pattern for first 2s of steady state (rows 100-200)
window = v4_all[100:200]
print("V4 Contact pattern (t=2.0-4.0s, 1=contact, 0=swing):")
print("Time   Left  Right  Both  None")
for i in range(0, len(window), 5):  # every 0.1s
    t = 2.0 + i * DT
    left = int(window[i, contact_l_idx] > 0.5)
    right = int(window[i, contact_r_idx] > 0.5)
    both = left and right
    none = not left and not right
    print(f"{t:.2f}   {left}     {right}      {both}     {int(none)}")

print()
window3 = v3_all[100:200]
print("V3 Contact pattern (t=2.0-4.0s, 1=contact, 0=swing):")
print("Time   Left  Right  Both  None")
for i in range(0, len(window3), 5):  # every 0.1s
    t = 2.0 + i * DT
    left = int(window3[i, contact_l_idx] > 0.5)
    right = int(window3[i, contact_r_idx] > 0.5)
    both = left and right
    none = not left and not right
    print(f"{t:.2f}   {left}     {right}      {both}     {int(none)}")

print()
print("Analysis complete.")
