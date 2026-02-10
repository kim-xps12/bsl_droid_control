#!/usr/bin/env python3
"""Jitter-specific quantitative analysis: exp009-V4 vs V3."""

import numpy as np
import sys

DT = 0.02  # 20ms timestep
STEADY_START = 100  # row index for t > 2s

# Column names for reference
ACTION_NAMES = [f"action_{i}" for i in range(10)]
JOINT_NAMES = [
    "L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee_pitch", "L_ankle_pitch",
    "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee_pitch", "R_ankle_pitch",
]

def load_data(path):
    """Load CSV, return header and data."""
    data = np.loadtxt(path, delimiter=',', skiprows=1)
    return data

def get_columns(header_line):
    """Parse header to get column indices."""
    cols = header_line.strip().split(',')
    return {name: i for i, name in enumerate(cols)}

def get_header(path):
    with open(path, 'r') as f:
        return f.readline()

# ---- Load data ----
v4_path = "/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs/droid-walking-omni-v4/eval_1999.csv"
v3_path = "/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs/droid-walking-omni-v3/eval_1999.csv"

header = get_header(v4_path)
col_idx = get_columns(header)

v4_all = load_data(v4_path)
v3_all = load_data(v3_path)

# Steady-state data (t > 2s)
v4 = v4_all[STEADY_START:]
v3 = v3_all[STEADY_START:]

print(f"V4 total rows: {len(v4_all)}, steady-state rows: {len(v4)}")
print(f"V3 total rows: {len(v3_all)}, steady-state rows: {len(v3)}")
print(f"Steady-state time range: {v4[0, 0]:.2f}s - {v4[-1, 0]:.2f}s")
print()

# ============================================================
# 1. Action Rate-of-Change Analysis
# ============================================================
print("=" * 80)
print("1. ACTION RATE-OF-CHANGE ANALYSIS")
print("=" * 80)
print()

action_cols = [col_idx[f"action_{i}"] for i in range(10)]

print(f"{'Action':<12} {'V4 RMS (1/s)':<16} {'V3 RMS (1/s)':<16} {'V4/V3 ratio':<14} {'V4-V3 diff':<14}")
print("-" * 72)

v4_action_rms_list = []
v3_action_rms_list = []

for i, acol in enumerate(action_cols):
    v4_diff = np.diff(v4[:, acol]) / DT
    v3_diff = np.diff(v3[:, acol]) / DT
    v4_rms = np.sqrt(np.mean(v4_diff**2))
    v3_rms = np.sqrt(np.mean(v3_diff**2))
    ratio = v4_rms / v3_rms if v3_rms > 0 else float('inf')
    diff = v4_rms - v3_rms
    v4_action_rms_list.append(v4_rms)
    v3_action_rms_list.append(v3_rms)
    print(f"action_{i:<4} {v4_rms:<16.2f} {v3_rms:<16.2f} {ratio:<14.3f} {diff:+.2f}")

v4_mean_rms = np.mean(v4_action_rms_list)
v3_mean_rms = np.mean(v3_action_rms_list)
print("-" * 72)
print(f"{'MEAN':<12} {v4_mean_rms:<16.2f} {v3_mean_rms:<16.2f} {v4_mean_rms/v3_mean_rms:<14.3f} {v4_mean_rms - v3_mean_rms:+.2f}")
print()

# ============================================================
# 2. Action FFT Analysis
# ============================================================
print("=" * 80)
print("2. ACTION FFT ANALYSIS")
print("=" * 80)
print()

# Find top 3 actions by V4 RMS
top_actions = np.argsort(v4_action_rms_list)[::-1][:4]
print(f"Analyzing top-4 actions by V4 RMS: {['action_' + str(i) for i in top_actions]}")
print()

N = len(v4)
freqs = np.fft.rfftfreq(N, d=DT)

# Frequency bands
band_low = (freqs >= 0) & (freqs < 2)
band_mid = (freqs >= 2) & (freqs < 5)
band_high = (freqs >= 5)

for ai in top_actions:
    acol = action_cols[ai]

    # FFT
    v4_fft = np.fft.rfft(v4[:, acol] - np.mean(v4[:, acol]))
    v3_fft = np.fft.rfft(v3[:, acol] - np.mean(v3[:, acol]))

    v4_power = np.abs(v4_fft)**2
    v3_power = np.abs(v3_fft)**2

    # Band powers
    v4_low = np.sum(v4_power[band_low])
    v4_mid = np.sum(v4_power[band_mid])
    v4_high = np.sum(v4_power[band_high])
    v4_total = v4_low + v4_mid + v4_high

    v3_low = np.sum(v3_power[band_low])
    v3_mid = np.sum(v3_power[band_mid])
    v3_high = np.sum(v3_power[band_high])
    v3_total = v3_low + v3_mid + v3_high

    # Dominant frequency
    v4_dom_idx = np.argmax(v4_power[1:]) + 1  # skip DC
    v3_dom_idx = np.argmax(v3_power[1:]) + 1

    print(f"--- action_{ai} ---")
    print(f"  Dominant freq:  V4={freqs[v4_dom_idx]:.2f}Hz  V3={freqs[v3_dom_idx]:.2f}Hz")
    print(f"  Band power distribution:")
    print(f"    {'Band':<12} {'V4 power':<16} {'V4 %':<10} {'V3 power':<16} {'V3 %':<10} {'V4/V3':<10}")
    print(f"    {'0-2Hz':<12} {v4_low:<16.1f} {100*v4_low/v4_total:<10.1f} {v3_low:<16.1f} {100*v3_low/v3_total:<10.1f} {v4_low/v3_low if v3_low>0 else float('inf'):<10.2f}")
    print(f"    {'2-5Hz':<12} {v4_mid:<16.1f} {100*v4_mid/v4_total:<10.1f} {v3_mid:<16.1f} {100*v3_mid/v3_total:<10.1f} {v4_mid/v3_mid if v3_mid>0 else float('inf'):<10.2f}")
    print(f"    {'5+Hz':<12} {v4_high:<16.1f} {100*v4_high/v4_total:<10.1f} {v3_high:<16.1f} {100*v3_high/v3_total:<10.1f} {v4_high/v3_high if v3_high>0 else float('inf'):<10.2f}")
    print(f"    {'TOTAL':<12} {v4_total:<16.1f} {'100.0':<10} {v3_total:<16.1f} {'100.0':<10} {v4_total/v3_total if v3_total>0 else float('inf'):<10.2f}")
    print()

# ============================================================
# 3. Contact State Transition Frequency
# ============================================================
print("=" * 80)
print("3. CONTACT STATE TRANSITION FREQUENCY")
print("=" * 80)
print()

contact_l_idx = col_idx["contact_left"]
contact_r_idx = col_idx["contact_right"]

duration = (len(v4) - 1) * DT  # steady-state duration

for label, data_v4, data_v3 in [("Steady-state (t>2s)", v4, v3)]:
    dur = (len(data_v4) - 1) * DT

    v4_l_trans = np.sum(np.abs(np.diff(data_v4[:, contact_l_idx])) > 0.5)
    v4_r_trans = np.sum(np.abs(np.diff(data_v4[:, contact_r_idx])) > 0.5)
    v3_l_trans = np.sum(np.abs(np.diff(data_v3[:, contact_l_idx])) > 0.5)
    v3_r_trans = np.sum(np.abs(np.diff(data_v3[:, contact_r_idx])) > 0.5)

    print(f"{label} (duration={dur:.1f}s):")
    print(f"  {'Foot':<12} {'V4 count':<12} {'V4 rate/s':<12} {'V3 count':<12} {'V3 rate/s':<12} {'V4/V3':<10}")
    print(f"  {'Left':<12} {v4_l_trans:<12} {v4_l_trans/dur:<12.2f} {v3_l_trans:<12} {v3_l_trans/dur:<12.2f} {(v4_l_trans/dur)/(v3_l_trans/dur) if v3_l_trans>0 else float('inf'):<10.2f}")
    print(f"  {'Right':<12} {v4_r_trans:<12} {v4_r_trans/dur:<12.2f} {v3_r_trans:<12} {v3_r_trans/dur:<12.2f} {(v4_r_trans/dur)/(v3_r_trans/dur) if v3_r_trans>0 else float('inf'):<10.2f}")
    print(f"  {'Total':<12} {v4_l_trans+v4_r_trans:<12} {(v4_l_trans+v4_r_trans)/dur:<12.2f} {v3_l_trans+v3_r_trans:<12} {(v3_l_trans+v3_r_trans)/dur:<12.2f} {((v4_l_trans+v4_r_trans)/dur)/((v3_l_trans+v3_r_trans)/dur) if (v3_l_trans+v3_r_trans)>0 else float('inf'):<10.2f}")
    print()

# Also analyze both_feet_contact and no_feet_contact proportions
v4_both = np.sum((v4[:, contact_l_idx] > 0.5) & (v4[:, contact_r_idx] > 0.5)) / len(v4) * 100
v4_none = np.sum((v4[:, contact_l_idx] < 0.5) & (v4[:, contact_r_idx] < 0.5)) / len(v4) * 100
v3_both = np.sum((v3[:, contact_l_idx] > 0.5) & (v3[:, contact_r_idx] > 0.5)) / len(v3) * 100
v3_none = np.sum((v3[:, contact_l_idx] < 0.5) & (v3[:, contact_r_idx] < 0.5)) / len(v3) * 100

print(f"Contact patterns:")
print(f"  {'Pattern':<20} {'V4 %':<10} {'V3 %':<10}")
print(f"  {'Both feet':<20} {v4_both:<10.1f} {v3_both:<10.1f}")
print(f"  {'No feet (airborne)':<20} {v4_none:<10.1f} {v3_none:<10.1f}")
print(f"  {'Single foot (gait)':<20} {100-v4_both-v4_none:<10.1f} {100-v3_both-v3_none:<10.1f}")
print()

# ============================================================
# 4. All-Joint FFT Analysis
# ============================================================
print("=" * 80)
print("4. ALL-JOINT FFT ANALYSIS")
print("=" * 80)
print()

dof_pos_cols = [col_idx[f"dof_pos_{jn}"] for jn in JOINT_NAMES]

print(f"{'Joint':<18} {'V4 dom Hz':<12} {'V4 0-2Hz%':<12} {'V4 2-5Hz%':<12} {'V4 5+Hz%':<12} {'V3 dom Hz':<12} {'V3 0-2Hz%':<12} {'V3 2-5Hz%':<12} {'V3 5+Hz%':<12} {'HF ratio V4/V3':<16}")
print("-" * 140)

v4_hf_ratios = []
v3_hf_ratios = []

for ji, jn in enumerate(JOINT_NAMES):
    jcol = dof_pos_cols[ji]

    v4_sig = v4[:, jcol] - np.mean(v4[:, jcol])
    v3_sig = v3[:, jcol] - np.mean(v3[:, jcol])

    v4_fft = np.fft.rfft(v4_sig)
    v3_fft = np.fft.rfft(v3_sig)

    v4_power = np.abs(v4_fft)**2
    v3_power = np.abs(v3_fft)**2

    v4_dom_idx = np.argmax(v4_power[1:]) + 1
    v3_dom_idx = np.argmax(v3_power[1:]) + 1

    v4_low = np.sum(v4_power[band_low])
    v4_mid = np.sum(v4_power[band_mid])
    v4_high = np.sum(v4_power[band_high])
    v4_total = v4_low + v4_mid + v4_high

    v3_low = np.sum(v3_power[band_low])
    v3_mid = np.sum(v3_power[band_mid])
    v3_high = np.sum(v3_power[band_high])
    v3_total = v3_low + v3_mid + v3_high

    v4_hf_pct = 100 * (v4_mid + v4_high) / v4_total if v4_total > 0 else 0
    v3_hf_pct = 100 * (v3_mid + v3_high) / v3_total if v3_total > 0 else 0

    v4_hf_ratios.append(v4_hf_pct)
    v3_hf_ratios.append(v3_hf_pct)

    hf_ratio_v4v3 = v4_hf_pct / v3_hf_pct if v3_hf_pct > 0 else float('inf')

    print(f"{jn:<18} {freqs[v4_dom_idx]:<12.2f} {100*v4_low/v4_total:<12.1f} {100*v4_mid/v4_total:<12.1f} {100*v4_high/v4_total:<12.1f} {freqs[v3_dom_idx]:<12.2f} {100*v3_low/v3_total:<12.1f} {100*v3_mid/v3_total:<12.1f} {100*v3_high/v3_total:<12.1f} {hf_ratio_v4v3:<16.2f}")

print("-" * 140)
v4_hf_mean = np.mean(v4_hf_ratios)
v3_hf_mean = np.mean(v3_hf_ratios)
print(f"{'MEAN HF (>2Hz)%':<18} {'':12} {'':12} {'':12} {v4_hf_mean:<12.1f} {'':12} {'':12} {'':12} {v3_hf_mean:<12.1f} {v4_hf_mean/v3_hf_mean if v3_hf_mean>0 else float('inf'):<16.2f}")
print()

# ============================================================
# 5. Swing Duration Distribution
# ============================================================
print("=" * 80)
print("5. SWING DURATION DISTRIBUTION")
print("=" * 80)
print()

def compute_swing_durations(contact_signal):
    """Extract individual swing durations (contact=0 periods)."""
    durations = []
    in_swing = False
    swing_start = 0
    for i in range(len(contact_signal)):
        if contact_signal[i] < 0.5:  # not in contact = swing
            if not in_swing:
                swing_start = i
                in_swing = True
        else:
            if in_swing:
                dur = (i - swing_start) * DT
                durations.append(dur)
                in_swing = False
    # Handle case where swing extends to end
    if in_swing:
        dur = (len(contact_signal) - swing_start) * DT
        durations.append(dur)
    return np.array(durations)

for foot, cidx in [("Left", contact_l_idx), ("Right", contact_r_idx)]:
    v4_swings = compute_swing_durations(v4[:, cidx])
    v3_swings = compute_swing_durations(v3[:, cidx])

    print(f"--- {foot} Foot ---")
    print(f"  {'Metric':<20} {'V4':<14} {'V3':<14}")
    print(f"  {'Count':<20} {len(v4_swings):<14} {len(v3_swings):<14}")

    if len(v4_swings) > 0 and len(v3_swings) > 0:
        print(f"  {'Min (s)':<20} {np.min(v4_swings):<14.4f} {np.min(v3_swings):<14.4f}")
        print(f"  {'25th pct (s)':<20} {np.percentile(v4_swings, 25):<14.4f} {np.percentile(v3_swings, 25):<14.4f}")
        print(f"  {'Median (s)':<20} {np.median(v4_swings):<14.4f} {np.median(v3_swings):<14.4f}")
        print(f"  {'75th pct (s)':<20} {np.percentile(v4_swings, 75):<14.4f} {np.percentile(v3_swings, 75):<14.4f}")
        print(f"  {'Max (s)':<20} {np.max(v4_swings):<14.4f} {np.max(v3_swings):<14.4f}")
        print(f"  {'Mean (s)':<20} {np.mean(v4_swings):<14.4f} {np.mean(v3_swings):<14.4f}")
        print(f"  {'Std (s)':<20} {np.std(v4_swings):<14.4f} {np.std(v3_swings):<14.4f}")

        v4_micro = np.sum(v4_swings < 0.1) / len(v4_swings) * 100
        v3_micro = np.sum(v3_swings < 0.1) / len(v3_swings) * 100
        v4_micro_count = np.sum(v4_swings < 0.1)
        v3_micro_count = np.sum(v3_swings < 0.1)
        print(f"  {'Micro (<0.1s) count':<20} {v4_micro_count:<14} {v3_micro_count:<14}")
        print(f"  {'Micro (<0.1s) %':<20} {v4_micro:<14.1f} {v3_micro:<14.1f}")

        v4_very_micro = np.sum(v4_swings < 0.06)
        v3_very_micro = np.sum(v3_swings < 0.06)
        print(f"  {'Very micro (<0.06s)':<20} {v4_very_micro:<14} {v3_very_micro:<14}")
    print()

# Also check stance durations
print("--- Stance Duration Distribution ---")
def compute_stance_durations(contact_signal):
    durations = []
    in_stance = False
    stance_start = 0
    for i in range(len(contact_signal)):
        if contact_signal[i] > 0.5:
            if not in_stance:
                stance_start = i
                in_stance = True
        else:
            if in_stance:
                dur = (i - stance_start) * DT
                durations.append(dur)
                in_stance = False
    if in_stance:
        dur = (len(contact_signal) - stance_start) * DT
        durations.append(dur)
    return np.array(durations)

for foot, cidx in [("Left", contact_l_idx), ("Right", contact_r_idx)]:
    v4_stances = compute_stance_durations(v4[:, cidx])
    v3_stances = compute_stance_durations(v3[:, cidx])

    print(f"  {foot} Stance:")
    if len(v4_stances) > 0 and len(v3_stances) > 0:
        v4_micro_stance = np.sum(v4_stances < 0.1)
        v3_micro_stance = np.sum(v3_stances < 0.1)
        print(f"    Count: V4={len(v4_stances)}, V3={len(v3_stances)}")
        print(f"    Min: V4={np.min(v4_stances):.4f}s, V3={np.min(v3_stances):.4f}s")
        print(f"    Median: V4={np.median(v4_stances):.4f}s, V3={np.median(v3_stances):.4f}s")
        print(f"    Max: V4={np.max(v4_stances):.4f}s, V3={np.max(v3_stances):.4f}s")
        print(f"    Micro-stance (<0.1s): V4={v4_micro_stance} ({100*v4_micro_stance/len(v4_stances):.1f}%), V3={v3_micro_stance} ({100*v3_micro_stance/len(v3_stances):.1f}%)")
print()

# ============================================================
# 6. Static Behavior Analysis
# ============================================================
print("=" * 80)
print("6. STATIC BEHAVIOR ANALYSIS")
print("=" * 80)
print()

# 6a. First 1 second (transient, rows 0-49)
print("--- 6a. First 1 second (transient, rows 0-49) ---")
v4_trans = v4_all[:50]
v3_trans = v3_all[:50]

for label, d4, d3 in [("Transient (0-1s)", v4_trans, v3_trans)]:
    v4_action_rms = []
    v3_action_rms = []
    for acol in action_cols:
        v4_action_rms.append(np.sqrt(np.mean(d4[:, acol]**2)))
        v3_action_rms.append(np.sqrt(np.mean(d3[:, acol]**2)))

    v4_dof_vel_rms = []
    v3_dof_vel_rms = []
    dof_vel_cols = [col_idx[f"dof_vel_{jn}"] for jn in JOINT_NAMES]
    for vcol in dof_vel_cols:
        v4_dof_vel_rms.append(np.sqrt(np.mean(d4[:, vcol]**2)))
        v3_dof_vel_rms.append(np.sqrt(np.mean(d3[:, vcol]**2)))

    print(f"  {label}:")
    print(f"    Action RMS (mean over 10):  V4={np.mean(v4_action_rms):.3f}  V3={np.mean(v3_action_rms):.3f}  ratio={np.mean(v4_action_rms)/np.mean(v3_action_rms):.3f}")
    print(f"    DOF Vel RMS (mean over 10): V4={np.mean(v4_dof_vel_rms):.3f}  V3={np.mean(v3_dof_vel_rms):.3f}  ratio={np.mean(v4_dof_vel_rms)/np.mean(v3_dof_vel_rms):.3f}")
    print()

# 6b. Low-command periods (XY command norm < 0.15)
print("--- 6b. Low-command periods (||cmd_xy|| < 0.15) ---")
cmd_x_idx = col_idx["cmd_vel_x"]
cmd_y_idx = col_idx["cmd_vel_y"]

v4_cmd_norm = np.sqrt(v4[:, cmd_x_idx]**2 + v4[:, cmd_y_idx]**2)
v3_cmd_norm = np.sqrt(v3[:, cmd_x_idx]**2 + v3[:, cmd_y_idx]**2)

v4_low_cmd = v4[v4_cmd_norm < 0.15]
v3_low_cmd = v3[v3_cmd_norm < 0.15]

print(f"  Low-command samples: V4={len(v4_low_cmd)} ({100*len(v4_low_cmd)/len(v4):.1f}%), V3={len(v3_low_cmd)} ({100*len(v3_low_cmd)/len(v3):.1f}%)")

if len(v4_low_cmd) > 5 and len(v3_low_cmd) > 5:
    v4_action_rms_lc = []
    v3_action_rms_lc = []
    for acol in action_cols:
        v4_action_rms_lc.append(np.sqrt(np.mean(v4_low_cmd[:, acol]**2)))
        v3_action_rms_lc.append(np.sqrt(np.mean(v3_low_cmd[:, acol]**2)))

    v4_dof_vel_rms_lc = []
    v3_dof_vel_rms_lc = []
    for vcol in dof_vel_cols:
        v4_dof_vel_rms_lc.append(np.sqrt(np.mean(v4_low_cmd[:, vcol]**2)))
        v3_dof_vel_rms_lc.append(np.sqrt(np.mean(v3_low_cmd[:, vcol]**2)))

    print(f"    Action RMS (mean):  V4={np.mean(v4_action_rms_lc):.3f}  V3={np.mean(v3_action_rms_lc):.3f}  ratio={np.mean(v4_action_rms_lc)/np.mean(v3_action_rms_lc):.3f}")
    print(f"    DOF Vel RMS (mean): V4={np.mean(v4_dof_vel_rms_lc):.3f}  V3={np.mean(v3_dof_vel_rms_lc):.3f}  ratio={np.mean(v4_dof_vel_rms_lc)/np.mean(v3_dof_vel_rms_lc):.3f}")

    # Per-joint DOF vel RMS for low-command
    print(f"\n    Per-joint DOF Vel RMS during low-command:")
    print(f"    {'Joint':<18} {'V4':<12} {'V3':<12} {'V4/V3':<10}")
    for ji, jn in enumerate(JOINT_NAMES):
        ratio = v4_dof_vel_rms_lc[ji] / v3_dof_vel_rms_lc[ji] if v3_dof_vel_rms_lc[ji] > 0 else float('inf')
        print(f"    {jn:<18} {v4_dof_vel_rms_lc[ji]:<12.3f} {v3_dof_vel_rms_lc[ji]:<12.3f} {ratio:<10.3f}")
else:
    print("    Insufficient low-command samples for analysis.")

print()

# Also check with threshold 0.05 for "near-zero" command
v4_near_zero = v4[v4_cmd_norm < 0.05]
v3_near_zero = v3[v3_cmd_norm < 0.05]
print(f"  Near-zero command (||cmd_xy|| < 0.05): V4={len(v4_near_zero)} samples, V3={len(v3_near_zero)} samples")
if len(v4_near_zero) > 5 and len(v3_near_zero) > 5:
    v4_act_rms_nz = np.mean([np.sqrt(np.mean(v4_near_zero[:, ac]**2)) for ac in action_cols])
    v3_act_rms_nz = np.mean([np.sqrt(np.mean(v3_near_zero[:, ac]**2)) for ac in action_cols])
    v4_vel_rms_nz = np.mean([np.sqrt(np.mean(v4_near_zero[:, vc]**2)) for vc in dof_vel_cols])
    v3_vel_rms_nz = np.mean([np.sqrt(np.mean(v3_near_zero[:, vc]**2)) for vc in dof_vel_cols])
    print(f"    Action RMS: V4={v4_act_rms_nz:.3f}  V3={v3_act_rms_nz:.3f}  ratio={v4_act_rms_nz/v3_act_rms_nz:.3f}")
    print(f"    DOF Vel RMS: V4={v4_vel_rms_nz:.3f}  V3={v3_vel_rms_nz:.3f}  ratio={v4_vel_rms_nz/v3_vel_rms_nz:.3f}")
print()

# ============================================================
# 7. Overall Jitter Score Comparison
# ============================================================
print("=" * 80)
print("7. OVERALL JITTER SCORE COMPARISON")
print("=" * 80)
print()

# 7a. Total action derivative RMS
total_v4_act_deriv = v4_mean_rms
total_v3_act_deriv = v3_mean_rms

# 7b. High-frequency power ratio (>3Hz / total) for hip_pitch joints
hip_pitch_joints = ["L_hip_pitch", "R_hip_pitch"]
v4_hf3_ratio_hp = []
v3_hf3_ratio_hp = []
band_above3 = freqs >= 3

for jn in hip_pitch_joints:
    jcol = col_idx[f"dof_pos_{jn}"]
    v4_sig = v4[:, jcol] - np.mean(v4[:, jcol])
    v3_sig = v3[:, jcol] - np.mean(v3[:, jcol])
    v4_fft = np.fft.rfft(v4_sig)
    v3_fft = np.fft.rfft(v3_sig)
    v4_power = np.abs(v4_fft)**2
    v3_power = np.abs(v3_fft)**2
    v4_hf3_ratio_hp.append(np.sum(v4_power[band_above3]) / np.sum(v4_power) * 100)
    v3_hf3_ratio_hp.append(np.sum(v3_power[band_above3]) / np.sum(v3_power) * 100)

v4_hf3_hp_mean = np.mean(v4_hf3_ratio_hp)
v3_hf3_hp_mean = np.mean(v3_hf3_ratio_hp)

# 7c. Contact transition rate
v4_total_trans = (v4_l_trans + v4_r_trans) / duration
v3_total_trans = (v3_l_trans + v3_r_trans) / duration

print(f"{'Metric':<40} {'V4':<16} {'V3':<16} {'V4/V3':<12} {'Assessment':<20}")
print("-" * 104)
print(f"{'Action deriv RMS (mean, 1/s)':<40} {total_v4_act_deriv:<16.2f} {total_v3_act_deriv:<16.2f} {total_v4_act_deriv/total_v3_act_deriv:<12.3f} {'WORSE' if total_v4_act_deriv > total_v3_act_deriv * 1.1 else 'SIMILAR' if total_v4_act_deriv > total_v3_act_deriv * 0.9 else 'BETTER'}")
print(f"{'Hip pitch HF ratio (>3Hz, %)':<40} {v4_hf3_hp_mean:<16.2f} {v3_hf3_hp_mean:<16.2f} {v4_hf3_hp_mean/v3_hf3_hp_mean if v3_hf3_hp_mean>0 else float('inf'):<12.3f} {'WORSE' if v4_hf3_hp_mean > v3_hf3_hp_mean * 1.1 else 'SIMILAR' if v4_hf3_hp_mean > v3_hf3_hp_mean * 0.9 else 'BETTER'}")
print(f"{'Contact transitions (total/s)':<40} {v4_total_trans:<16.2f} {v3_total_trans:<16.2f} {v4_total_trans/v3_total_trans if v3_total_trans>0 else float('inf'):<12.3f} {'WORSE' if v4_total_trans > v3_total_trans * 1.1 else 'SIMILAR' if v4_total_trans > v3_total_trans * 0.9 else 'BETTER'}")

# Additional composite
v4_joint_hf_mean = np.mean(v4_hf_ratios)
v3_joint_hf_mean = np.mean(v3_hf_ratios)
print(f"{'All-joint HF ratio (>2Hz, %)':<40} {v4_joint_hf_mean:<16.2f} {v3_joint_hf_mean:<16.2f} {v4_joint_hf_mean/v3_joint_hf_mean if v3_joint_hf_mean>0 else float('inf'):<12.3f} {'WORSE' if v4_joint_hf_mean > v3_joint_hf_mean * 1.1 else 'SIMILAR' if v4_joint_hf_mean > v3_joint_hf_mean * 0.9 else 'BETTER'}")

# DOF velocity RMS (all joints, steady-state)
v4_dof_vel_rms_all = np.mean([np.sqrt(np.mean(v4[:, vc]**2)) for vc in dof_vel_cols])
v3_dof_vel_rms_all = np.mean([np.sqrt(np.mean(v3[:, vc]**2)) for vc in dof_vel_cols])
print(f"{'DOF vel RMS (mean, rad/s)':<40} {v4_dof_vel_rms_all:<16.3f} {v3_dof_vel_rms_all:<16.3f} {v4_dof_vel_rms_all/v3_dof_vel_rms_all:<12.3f} {'WORSE' if v4_dof_vel_rms_all > v3_dof_vel_rms_all * 1.1 else 'SIMILAR' if v4_dof_vel_rms_all > v3_dof_vel_rms_all * 0.9 else 'BETTER'}")

# Action amplitude RMS (steady-state)
v4_action_amp = np.mean([np.sqrt(np.mean(v4[:, ac]**2)) for ac in action_cols])
v3_action_amp = np.mean([np.sqrt(np.mean(v3[:, ac]**2)) for ac in action_cols])
print(f"{'Action amplitude RMS (mean)':<40} {v4_action_amp:<16.3f} {v3_action_amp:<16.3f} {v4_action_amp/v3_action_amp:<12.3f} {'WORSE' if v4_action_amp > v3_action_amp * 1.1 else 'SIMILAR' if v4_action_amp > v3_action_amp * 0.9 else 'BETTER'}")

print()
print("=" * 80)
print("JITTER DIAGNOSIS SUMMARY")
print("=" * 80)
print()

# Compute overall jitter score (normalized composite)
jitter_metrics_v4 = [
    total_v4_act_deriv / 100,  # normalize action deriv
    v4_hf3_hp_mean / 10,       # normalize HF ratio
    v4_total_trans / 10,       # normalize contact rate
    v4_joint_hf_mean / 10,     # normalize joint HF
]
jitter_metrics_v3 = [
    total_v3_act_deriv / 100,
    v3_hf3_hp_mean / 10,
    v3_total_trans / 10,
    v3_joint_hf_mean / 10,
]

v4_composite = np.mean(jitter_metrics_v4)
v3_composite = np.mean(jitter_metrics_v3)

print(f"Composite Jitter Score: V4={v4_composite:.4f}  V3={v3_composite:.4f}  V4/V3={v4_composite/v3_composite:.3f}")
print()

# Per-joint action derivative breakdown
print("Per-action derivative RMS breakdown:")
print(f"{'Action':<12} {'Joint':<18} {'V4 RMS':<14} {'V3 RMS':<14} {'V4/V3':<10} {'Flag':<8}")
print("-" * 76)
action_joint_map = [
    "L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee_pitch", "L_ankle_pitch",
    "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee_pitch", "R_ankle_pitch",
]
for i in range(10):
    ratio = v4_action_rms_list[i] / v3_action_rms_list[i] if v3_action_rms_list[i] > 0 else float('inf')
    flag = "***" if ratio > 1.5 else "**" if ratio > 1.2 else "*" if ratio > 1.1 else ""
    print(f"action_{i:<4} {action_joint_map[i]:<18} {v4_action_rms_list[i]:<14.2f} {v3_action_rms_list[i]:<14.2f} {ratio:<10.3f} {flag}")

print()
print("Flag legend: *** = >50% worse, ** = >20% worse, * = >10% worse")
print()
print("Analysis complete.")
