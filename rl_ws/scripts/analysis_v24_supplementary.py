#!/usr/bin/env python3
"""
Supplementary analysis: Focus on pre-contact ankle dynamics and the mechanism
of heel tapping that contact_no_vel fails to address.
"""

import pandas as pd
import numpy as np
from pathlib import Path

base = Path("/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs")
v24 = pd.read_csv(base / "droid-walking-narrow-v24" / "eval_499.csv")
v23 = pd.read_csv(base / "droid-walking-narrow-v23" / "eval_499.csv")

dt = 0.02
skip_steps = int(2.0 / dt)

def detect_contact_onsets(contact_signal):
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
    offsets = []
    in_stance = False
    for i in range(1, len(contact_signal)):
        if in_stance and contact_signal.iloc[i] == 0.0:
            offsets.append(i)
            in_stance = False
        elif contact_signal.iloc[i] == 1.0:
            in_stance = True
    return np.array(offsets)

# ============================================================
# KEY ANALYSIS: Pre-contact ankle velocity is the "heel tapping" signal
# ============================================================
print("=" * 80)
print("ANALYSIS A: PRE-CONTACT ANKLE VELOCITY DECOMPOSITION")
print("(The 'heel tapping' is the ankle slapping down BEFORE contact is detected)")
print("=" * 80)

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n{'='*60}")
    print(f"  {version_name}")
    print(f"{'='*60}")
    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    for side in ["L", "R"]:
        contact_col = f"contact_{'left' if side == 'L' else 'right'}"
        onsets = detect_contact_onsets(df_trimmed[contact_col])

        # Pre-contact velocities (3 steps before contact)
        pre_3_vels = []
        pre_2_vels = []
        pre_1_vels = []
        at_contact_vels = []
        post_1_vels = []

        for idx in onsets:
            if idx >= 3 and idx + 1 < len(df_trimmed):
                pre_3_vels.append(df_trimmed[f"dof_vel_{side}_ankle_pitch"].iloc[idx-3])
                pre_2_vels.append(df_trimmed[f"dof_vel_{side}_ankle_pitch"].iloc[idx-2])
                pre_1_vels.append(df_trimmed[f"dof_vel_{side}_ankle_pitch"].iloc[idx-1])
                at_contact_vels.append(df_trimmed[f"dof_vel_{side}_ankle_pitch"].iloc[idx])
                post_1_vels.append(df_trimmed[f"dof_vel_{side}_ankle_pitch"].iloc[idx+1])

        print(f"\n  {side} ankle_pitch velocity around contact ({len(pre_3_vels)} events):")
        print(f"    t-3: mean={np.mean(pre_3_vels):+.3f}, |mean|={np.mean(np.abs(pre_3_vels)):.3f}")
        print(f"    t-2: mean={np.mean(pre_2_vels):+.3f}, |mean|={np.mean(np.abs(pre_2_vels)):.3f}")
        print(f"    t-1: mean={np.mean(pre_1_vels):+.3f}, |mean|={np.mean(np.abs(pre_1_vels)):.3f}  ← LAST SWING STEP")
        print(f"    t=0: mean={np.mean(at_contact_vels):+.3f}, |mean|={np.mean(np.abs(at_contact_vels)):.3f}  ← FIRST STANCE STEP (contact_no_vel applies here)")
        print(f"    t+1: mean={np.mean(post_1_vels):+.3f}, |mean|={np.mean(np.abs(post_1_vels)):.3f}")

        # Position drop (from pre-contact to contact)
        pre_pos = []
        at_pos = []
        for idx in onsets:
            if idx >= 3:
                pre_pos.append(df_trimmed[f"dof_pos_{side}_ankle_pitch"].iloc[idx-3])
                at_pos.append(df_trimmed[f"dof_pos_{side}_ankle_pitch"].iloc[idx])

        pre_pos = np.array(pre_pos)
        at_pos = np.array(at_pos)
        drop = pre_pos - at_pos
        print(f"    Position drop (t-3 to t=0): {np.mean(drop):.4f} rad ({np.degrees(np.mean(drop)):.2f}°)")

# ============================================================
# ANALYSIS B: contact_no_vel penalty window vs heel-tapping dynamics
# ============================================================
print("\n\n" + "=" * 80)
print("ANALYSIS B: contact_no_vel ACTS ON STANCE ONLY - TIMING MISMATCH")
print("(Heel tapping = high velocity at t-1 to t+0, but penalty starts at t=0)")
print("=" * 80)

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n  --- {version_name} ---")
    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    for side in ["L", "R"]:
        contact_col = f"contact_{'left' if side == 'L' else 'right'}"
        ankle_vel_col = f"dof_vel_{side}_ankle_pitch"

        onsets = detect_contact_onsets(df_trimmed[contact_col])

        # Peak velocity in different windows
        peak_pre = []  # 3 steps before contact
        peak_at = []   # first step of contact
        peak_post = [] # 3 steps after first contact

        for idx in onsets:
            if idx >= 3 and idx + 4 < len(df_trimmed):
                pre_window = df_trimmed[ankle_vel_col].iloc[idx-3:idx].abs().values
                at_step = abs(df_trimmed[ankle_vel_col].iloc[idx])
                post_window = df_trimmed[ankle_vel_col].iloc[idx+1:idx+4].abs().values

                peak_pre.append(np.max(pre_window))
                peak_at.append(at_step)
                peak_post.append(np.max(post_window))

        print(f"  {side} foot:")
        print(f"    Peak |vel| pre-contact  (t-3 to t-1): mean={np.mean(peak_pre):.3f} rad/s")
        print(f"    |vel| at contact        (t=0):        mean={np.mean(peak_at):.3f} rad/s")
        print(f"    Peak |vel| post-contact (t+1 to t+3): mean={np.mean(peak_post):.3f} rad/s")
        print(f"    Pre/At ratio: {np.mean(peak_pre)/np.mean(peak_at):.2f}x")
        print(f"    Pre/Post ratio: {np.mean(peak_pre)/np.mean(peak_post):.2f}x")

# ============================================================
# ANALYSIS C: Swing-phase ankle dynamics (where heel tapping originates)
# ============================================================
print("\n\n" + "=" * 80)
print("ANALYSIS C: SWING-PHASE ANKLE DYNAMICS")
print("(Heel tapping originates in swing phase - contact_no_vel cannot penalize this)")
print("=" * 80)

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n  --- {version_name} ---")
    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    for side in ["L", "R"]:
        contact_col = f"contact_{'left' if side == 'L' else 'right'}"
        ankle_vel_col = f"dof_vel_{side}_ankle_pitch"
        ankle_pos_col = f"dof_pos_{side}_ankle_pitch"

        swing_mask = df_trimmed[contact_col] == 0.0
        stance_mask = df_trimmed[contact_col] == 1.0

        swing_vel = df_trimmed.loc[swing_mask, ankle_vel_col]
        stance_vel = df_trimmed.loc[stance_mask, ankle_vel_col]

        print(f"\n  {side} ankle_pitch:")
        print(f"    SWING:  vel_std={swing_vel.std():.4f}, |vel|_mean={swing_vel.abs().mean():.4f}, vel_range=[{swing_vel.min():.2f}, {swing_vel.max():.2f}]")
        print(f"    STANCE: vel_std={stance_vel.std():.4f}, |vel|_mean={stance_vel.abs().mean():.4f}, vel_range=[{stance_vel.min():.2f}, {stance_vel.max():.2f}]")
        print(f"    SWING/STANCE |vel| ratio: {swing_vel.abs().mean()/stance_vel.abs().mean():.2f}x")

        # Ankle position: swing end position is what determines contact velocity
        swing_pos = df_trimmed.loc[swing_mask, ankle_pos_col]
        stance_pos = df_trimmed.loc[stance_mask, ankle_pos_col]
        print(f"    SWING pos:  mean={swing_pos.mean():.4f} ({np.degrees(swing_pos.mean()):.2f}°), range=[{swing_pos.min():.3f}, {swing_pos.max():.3f}]")
        print(f"    STANCE pos: mean={stance_pos.mean():.4f} ({np.degrees(stance_pos.mean()):.2f}°), range=[{stance_pos.min():.3f}, {stance_pos.max():.3f}]")

# ============================================================
# ANALYSIS D: Hip_roll action asymmetry → Yaw drift mechanism
# ============================================================
print("\n\n" + "=" * 80)
print("ANALYSIS D: HIP_ROLL ACTION ASYMMETRY → YAW DRIFT MECHANISM")
print("(V24 hip_roll action diff increased from -5.03 to -6.60)")
print("=" * 80)

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n  --- {version_name} ---")
    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    l_onsets = detect_contact_onsets(df_trimmed["contact_left"])

    # Per-cycle hip_roll action asymmetry vs yaw change
    hip_roll_asym_per_cycle = []
    ankle_action_asym_per_cycle = []
    yaw_change_per_cycle = []

    for i in range(len(l_onsets) - 1):
        start = l_onsets[i]
        end = l_onsets[i + 1]
        if end - start < 5:
            continue

        seg = df_trimmed.iloc[start:end]

        # hip_roll action: action_1 (L), action_6 (R)
        l_hr = seg["action_1"].mean()
        r_hr = seg["action_6"].mean()
        hip_roll_asym_per_cycle.append(l_hr - r_hr)

        # ankle action: action_4 (L), action_9 (R)
        l_aa = seg["action_4"].mean()
        r_aa = seg["action_9"].mean()
        ankle_action_asym_per_cycle.append(l_aa - r_aa)

        # Yaw change
        yaw_change = seg["yaw_deg"].iloc[-1] - seg["yaw_deg"].iloc[0]
        yaw_change_per_cycle.append(yaw_change)

    hip_roll_asym_per_cycle = np.array(hip_roll_asym_per_cycle)
    ankle_action_asym_per_cycle = np.array(ankle_action_asym_per_cycle)
    yaw_change_per_cycle = np.array(yaw_change_per_cycle)

    print(f"  hip_roll action asym per cycle: mean={np.mean(hip_roll_asym_per_cycle):.4f}, std={np.std(hip_roll_asym_per_cycle):.4f}")
    print(f"  ankle action asym per cycle: mean={np.mean(ankle_action_asym_per_cycle):.4f}, std={np.std(ankle_action_asym_per_cycle):.4f}")
    print(f"  yaw change per cycle: mean={np.mean(yaw_change_per_cycle):.4f}°")

    if len(yaw_change_per_cycle) > 3:
        corr_hr = np.corrcoef(hip_roll_asym_per_cycle, yaw_change_per_cycle)[0, 1]
        corr_aa = np.corrcoef(ankle_action_asym_per_cycle, yaw_change_per_cycle)[0, 1]
        print(f"  Correlation: hip_roll_action_asym → yaw_change: r={corr_hr:.4f}")
        print(f"  Correlation: ankle_action_asym → yaw_change: r={corr_aa:.4f}")

# ============================================================
# ANALYSIS E: V24 ankle_pitch position RANGE reduction
# ============================================================
print("\n\n" + "=" * 80)
print("ANALYSIS E: ANKLE_PITCH POSITION RANGE COMPARISON")
print("(V24 reduced overall ankle range - potentially hides tapping in position space)")
print("=" * 80)

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n  --- {version_name} ---")
    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    for side in ["L", "R"]:
        pos = df_trimmed[f"dof_pos_{side}_ankle_pitch"]
        vel = df_trimmed[f"dof_vel_{side}_ankle_pitch"]

        print(f"  {side} ankle_pitch:")
        print(f"    Full trajectory pos range: [{pos.min():.4f}, {pos.max():.4f}] = {pos.max()-pos.min():.4f} rad ({np.degrees(pos.max()-pos.min()):.2f}°)")
        print(f"    Full trajectory vel range: [{vel.min():.4f}, {vel.max():.4f}] = {vel.max()-vel.min():.4f} rad/s")
        print(f"    Full trajectory pos mean:  {pos.mean():.4f} rad ({np.degrees(pos.mean()):.2f}°)")

# ============================================================
# ANALYSIS F: V24 contact_no_vel 3D effect - check velocity components
# ============================================================
print("\n\n" + "=" * 80)
print("ANALYSIS F: BASE VELOCITY AT CONTACT (Z-component is the heel tapping proxy)")
print("=" * 80)

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n  --- {version_name} ---")
    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    for side in ["L", "R"]:
        contact_col = f"contact_{'left' if side == 'L' else 'right'}"
        onsets = detect_contact_onsets(df_trimmed[contact_col])

        # Base velocity at contact
        vx_at_contact = [df_trimmed["base_vel_x"].iloc[idx] for idx in onsets if idx < len(df_trimmed)]
        vy_at_contact = [df_trimmed["base_vel_y"].iloc[idx] for idx in onsets if idx < len(df_trimmed)]
        vz_at_contact = [df_trimmed["base_vel_z"].iloc[idx] for idx in onsets if idx < len(df_trimmed)]

        print(f"  {side} foot contact ({len(vx_at_contact)} events):")
        print(f"    base_vel_x at contact: mean={np.mean(vx_at_contact):.4f}, std={np.std(vx_at_contact):.4f}")
        print(f"    base_vel_y at contact: mean={np.mean(vy_at_contact):.4f}, std={np.std(vy_at_contact):.4f}")
        print(f"    base_vel_z at contact: mean={np.mean(vz_at_contact):.4f}, std={np.std(vz_at_contact):.4f}")

# ============================================================
# ANALYSIS G: Post-contact ankle_pitch REBOUND pattern
# ============================================================
print("\n\n" + "=" * 80)
print("ANALYSIS G: POST-CONTACT ANKLE REBOUND PATTERN")
print("(Does ankle 'bounce' after contact? This is the visible 'tapping')")
print("=" * 80)

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n  --- {version_name} ---")
    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    for side in ["L", "R"]:
        contact_col = f"contact_{'left' if side == 'L' else 'right'}"
        ankle_pos_col = f"dof_pos_{side}_ankle_pitch"
        onsets = detect_contact_onsets(df_trimmed[contact_col])

        # Check for position rebound: contact position → minimum → bounce back
        rebound_magnitudes = []
        rebound_timings = []  # steps to first local minimum after contact

        for idx in onsets:
            end_check = min(idx + 15, len(df_trimmed))
            if end_check - idx < 5:
                continue

            pos_window = df_trimmed[ankle_pos_col].iloc[idx:end_check].values
            contact_pos = pos_window[0]

            # Find first local minimum
            for t in range(1, len(pos_window) - 1):
                if pos_window[t] <= pos_window[t-1] and pos_window[t] <= pos_window[t+1]:
                    min_pos = pos_window[t]
                    # Find subsequent maximum
                    for t2 in range(t+1, len(pos_window) - 1):
                        if pos_window[t2] >= pos_window[t2-1] and pos_window[t2] >= pos_window[t2+1]:
                            rebound = pos_window[t2] - min_pos
                            rebound_magnitudes.append(rebound)
                            rebound_timings.append(t)
                            break
                    break

        if len(rebound_magnitudes) > 0:
            rebound_magnitudes = np.array(rebound_magnitudes)
            rebound_timings = np.array(rebound_timings)
            print(f"  {side} post-contact rebound ({len(rebound_magnitudes)} events):")
            print(f"    Rebound magnitude: mean={np.mean(rebound_magnitudes):.4f} rad ({np.degrees(np.mean(rebound_magnitudes)):.2f}°)")
            print(f"    Rebound timing: mean={np.mean(rebound_timings):.1f} steps ({np.mean(rebound_timings)*dt:.3f}s)")
        else:
            print(f"  {side}: No clear rebound detected")

# ============================================================
# ANALYSIS H: Ankle position at steady stance (not transition)
# ============================================================
print("\n\n" + "=" * 80)
print("ANALYSIS H: STANCE-PHASE ANKLE POSITION EVOLUTION")
print("(Track ankle position from contact onset through full stance)")
print("=" * 80)

for version_name, df in [("V23", v23), ("V24", v24)]:
    print(f"\n  --- {version_name} ---")
    df_trimmed = df.iloc[skip_steps:].reset_index(drop=True)

    for side in ["L", "R"]:
        contact_col = f"contact_{'left' if side == 'L' else 'right'}"
        ankle_pos_col = f"dof_pos_{side}_ankle_pitch"
        ankle_vel_col = f"dof_vel_{side}_ankle_pitch"
        onsets = detect_contact_onsets(df_trimmed[contact_col])
        offsets = detect_contact_offsets(df_trimmed[contact_col])

        # For each stance phase, track normalized trajectory
        stance_pos_at_5pct = []
        stance_pos_at_50pct = []
        stance_pos_at_95pct = []
        stance_vel_std_first_quarter = []
        stance_vel_std_last_quarter = []

        for on in onsets:
            next_off = offsets[offsets > on]
            if len(next_off) == 0:
                continue
            off = next_off[0]
            dur = off - on
            if dur < 10:
                continue

            pos_traj = df_trimmed[ankle_pos_col].iloc[on:off].values
            vel_traj = df_trimmed[ankle_vel_col].iloc[on:off].values

            idx_5 = int(0.05 * dur)
            idx_50 = int(0.50 * dur)
            idx_95 = min(int(0.95 * dur), dur - 1)

            stance_pos_at_5pct.append(pos_traj[idx_5])
            stance_pos_at_50pct.append(pos_traj[idx_50])
            stance_pos_at_95pct.append(pos_traj[idx_95])

            q1 = dur // 4
            stance_vel_std_first_quarter.append(np.std(vel_traj[:q1]))
            stance_vel_std_last_quarter.append(np.std(vel_traj[-q1:]))

        if len(stance_pos_at_5pct) > 0:
            print(f"\n  {side} stance trajectory ({len(stance_pos_at_5pct)} stances):")
            print(f"    Pos at  5%: mean={np.mean(stance_pos_at_5pct):.4f} ({np.degrees(np.mean(stance_pos_at_5pct)):.2f}°)")
            print(f"    Pos at 50%: mean={np.mean(stance_pos_at_50pct):.4f} ({np.degrees(np.mean(stance_pos_at_50pct)):.2f}°)")
            print(f"    Pos at 95%: mean={np.mean(stance_pos_at_95pct):.4f} ({np.degrees(np.mean(stance_pos_at_95pct)):.2f}°)")
            print(f"    Position change (5% → 95%): {np.mean(stance_pos_at_95pct) - np.mean(stance_pos_at_5pct):.4f} rad ({np.degrees(np.mean(stance_pos_at_95pct) - np.mean(stance_pos_at_5pct)):.2f}°)")
            print(f"    Vel std first quarter: {np.mean(stance_vel_std_first_quarter):.4f}")
            print(f"    Vel std last quarter:  {np.mean(stance_vel_std_last_quarter):.4f}")
            print(f"    Last/First quarter ratio: {np.mean(stance_vel_std_last_quarter)/np.mean(stance_vel_std_first_quarter):.2f}x")

print("\n\nSupplementary analysis complete.")
