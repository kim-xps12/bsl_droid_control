"""Stance leg posture analysis: V1 vs V2 comparison for exp008.
Focuses on hip_roll/hip_yaw/knee/ankle during stance phase."""

import numpy as np
import pandas as pd

def load_data(path):
    df = pd.read_csv(path)
    # Filter to steady state (t > 2s)
    return df[df['timestamp'] > 2.0].copy()

def rad2deg(x):
    return np.degrees(x)

def stance_stats(df, col, contact_col):
    """Get stats for a joint during stance (contact > 0)."""
    vals = df.loc[df[contact_col] > 0, col]
    if len(vals) == 0:
        return {'mean': np.nan, 'std': np.nan, 'min': np.nan, 'max': np.nan, 'count': 0}
    return {
        'mean': vals.mean(),
        'std': vals.std(),
        'min': vals.min(),
        'max': vals.max(),
        'count': len(vals),
    }

def find_touchdown_values(df, contact_col, joint_col):
    """Find joint values at the moment of touchdown (contact 0->1 transition)."""
    contact = df[contact_col].values
    joint = df[joint_col].values
    timestamps = df['timestamp'].values

    touchdowns = []
    for i in range(1, len(contact)):
        if contact[i-1] == 0 and contact[i] > 0:
            touchdowns.append({
                'timestamp': timestamps[i],
                'joint_value_rad': joint[i],
                'joint_value_deg': rad2deg(joint[i]),
            })
    return touchdowns

def print_section(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")

# Load data
v1 = load_data('/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs/droid-walking-narrow-v1/eval_499.csv')
v2 = load_data('/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs/droid-walking-narrow-v2/eval_499.csv')

print(f"V1: {len(v1)} frames (t={v1['timestamp'].min():.2f}~{v1['timestamp'].max():.2f}s)")
print(f"V2: {len(v2)} frames (t={v2['timestamp'].min():.2f}~{v2['timestamp'].max():.2f}s)")
print(f"V1 L contact frames: {(v1['contact_left']>0).sum()}, R contact frames: {(v1['contact_right']>0).sum()}")
print(f"V2 L contact frames: {(v2['contact_left']>0).sum()}, R contact frames: {(v2['contact_right']>0).sum()}")

# =========================================================
# 1. Stance hip_roll statistics
# =========================================================
print_section("1. Stance hip_roll Statistics (rad / deg)")

for label, side, contact in [("Left", "L", "contact_left"), ("Right", "R", "contact_right")]:
    col = f"dof_pos_{side}_hip_roll"
    s1 = stance_stats(v1, col, contact)
    s2 = stance_stats(v2, col, contact)

    print(f"\n  {label} leg stance hip_roll:")
    print(f"  {'':15s} {'V1':>12s} {'V2':>12s} {'Delta':>12s}")
    print(f"  {'Mean (rad)':15s} {s1['mean']:12.4f} {s2['mean']:12.4f} {s2['mean']-s1['mean']:+12.4f}")
    print(f"  {'Mean (deg)':15s} {rad2deg(s1['mean']):12.2f} {rad2deg(s2['mean']):12.2f} {rad2deg(s2['mean']-s1['mean']):+12.2f}")
    print(f"  {'Std (rad)':15s} {s1['std']:12.4f} {s2['std']:12.4f} {s2['std']-s1['std']:+12.4f}")
    print(f"  {'Min (deg)':15s} {rad2deg(s1['min']):12.2f} {rad2deg(s2['min']):12.2f}")
    print(f"  {'Max (deg)':15s} {rad2deg(s1['max']):12.2f} {rad2deg(s2['max']):12.2f}")
    print(f"  {'N frames':15s} {s1['count']:12d} {s2['count']:12d}")

# =========================================================
# 2. Stance hip_yaw statistics
# =========================================================
print_section("2. Stance hip_yaw Statistics (rad / deg)")

for label, side, contact in [("Left", "L", "contact_left"), ("Right", "R", "contact_right")]:
    col = f"dof_pos_{side}_hip_yaw"
    s1 = stance_stats(v1, col, contact)
    s2 = stance_stats(v2, col, contact)

    print(f"\n  {label} leg stance hip_yaw:")
    print(f"  {'':15s} {'V1':>12s} {'V2':>12s} {'Delta':>12s}")
    print(f"  {'Mean (rad)':15s} {s1['mean']:12.4f} {s2['mean']:12.4f} {s2['mean']-s1['mean']:+12.4f}")
    print(f"  {'Mean (deg)':15s} {rad2deg(s1['mean']):12.2f} {rad2deg(s2['mean']):12.2f} {rad2deg(s2['mean']-s1['mean']):+12.2f}")
    print(f"  {'Std (rad)':15s} {s1['std']:12.4f} {s2['std']:12.4f} {s2['std']-s1['std']:+12.4f}")
    print(f"  {'Min (deg)':15s} {rad2deg(s1['min']):12.2f} {rad2deg(s2['min']):12.2f}")
    print(f"  {'Max (deg)':15s} {rad2deg(s1['max']):12.2f} {rad2deg(s2['max']):12.2f}")
    print(f"  {'N frames':15s} {s1['count']:12d} {s2['count']:12d}")

# =========================================================
# 3. Stance knee_pitch / ankle_pitch statistics
# =========================================================
print_section("3. Stance knee_pitch / ankle_pitch Statistics (deg)")

for joint in ['knee_pitch', 'ankle_pitch']:
    print(f"\n  --- {joint} ---")
    for label, side, contact in [("Left", "L", "contact_left"), ("Right", "R", "contact_right")]:
        col = f"dof_pos_{side}_{joint}"
        s1 = stance_stats(v1, col, contact)
        s2 = stance_stats(v2, col, contact)

        print(f"\n  {label} leg stance {joint}:")
        print(f"  {'':15s} {'V1':>12s} {'V2':>12s} {'Delta':>12s}")
        print(f"  {'Mean (deg)':15s} {rad2deg(s1['mean']):12.2f} {rad2deg(s2['mean']):12.2f} {rad2deg(s2['mean']-s1['mean']):+12.2f}")
        print(f"  {'Std (deg)':15s} {rad2deg(s1['std']):12.2f} {rad2deg(s2['std']):12.2f} {rad2deg(s2['std']-s1['std']):+12.2f}")
        print(f"  {'Min (deg)':15s} {rad2deg(s1['min']):12.2f} {rad2deg(s2['min']):12.2f}")
        print(f"  {'Max (deg)':15s} {rad2deg(s1['max']):12.2f} {rad2deg(s2['max']):12.2f}")

# =========================================================
# 4. Touchdown hip_roll values (swing -> stance transition)
# =========================================================
print_section("4. Touchdown hip_roll (at contact 0->1 transition)")

for label, side, contact in [("Left", "L", "contact_left"), ("Right", "R", "contact_right")]:
    col = f"dof_pos_{side}_hip_roll"
    td1 = find_touchdown_values(v1, contact, col)
    td2 = find_touchdown_values(v2, contact, col)

    print(f"\n  {label} leg touchdowns:")
    print(f"  V1: {len(td1)} touchdowns")
    if td1:
        vals1 = [t['joint_value_deg'] for t in td1]
        print(f"    hip_roll at touchdown (deg): {', '.join(f'{v:.2f}' for v in vals1)}")
        print(f"    Mean: {np.mean(vals1):.2f}°, Std: {np.std(vals1):.2f}°")

    print(f"  V2: {len(td2)} touchdowns")
    if td2:
        vals2 = [t['joint_value_deg'] for t in td2]
        print(f"    hip_roll at touchdown (deg): {', '.join(f'{v:.2f}' for v in vals2)}")
        print(f"    Mean: {np.mean(vals2):.2f}°, Std: {np.std(vals2):.2f}°")

# Also check hip_yaw at touchdown
print_section("4b. Touchdown hip_yaw (at contact 0->1 transition)")

for label, side, contact in [("Left", "L", "contact_left"), ("Right", "R", "contact_right")]:
    col = f"dof_pos_{side}_hip_yaw"
    td1 = find_touchdown_values(v1, contact, col)
    td2 = find_touchdown_values(v2, contact, col)

    print(f"\n  {label} leg touchdowns:")
    print(f"  V1: {len(td1)} touchdowns")
    if td1:
        vals1 = [t['joint_value_deg'] for t in td1]
        print(f"    hip_yaw at touchdown (deg): {', '.join(f'{v:.2f}' for v in vals1)}")
        print(f"    Mean: {np.mean(vals1):.2f}°, Std: {np.std(vals1):.2f}°")

    print(f"  V2: {len(td2)} touchdowns")
    if td2:
        vals2 = [t['joint_value_deg'] for t in td2]
        print(f"    hip_yaw at touchdown (deg): {', '.join(f'{v:.2f}' for v in vals2)}")
        print(f"    Mean: {np.mean(vals2):.2f}°, Std: {np.std(vals2):.2f}°")

# =========================================================
# 5. Time series table: t=3~5s, 100ms sampling
# =========================================================
print_section("5. hip_roll Time Series (t=3~5s, 100ms sampling)")

for ver_label, df in [("V1", v1), ("V2", v2)]:
    print(f"\n  --- {ver_label} ---")
    print(f"  {'t(s)':>6s} {'L_hip_roll':>12s} {'R_hip_roll':>12s} {'contact_L':>10s} {'contact_R':>10s} {'L_hip_yaw':>12s} {'R_hip_yaw':>12s}")

    # Sample at 100ms intervals from 3.0 to 5.0
    for t_target in np.arange(3.0, 5.01, 0.1):
        # Find closest frame
        idx = (df['timestamp'] - t_target).abs().idxmin()
        row = df.loc[idx]
        t_actual = row['timestamp']
        if abs(t_actual - t_target) > 0.05:
            continue  # skip if no close frame

        l_hr = rad2deg(row['dof_pos_L_hip_roll'])
        r_hr = rad2deg(row['dof_pos_R_hip_roll'])
        cl = int(row['contact_left'])
        cr = int(row['contact_right'])
        l_hy = rad2deg(row['dof_pos_L_hip_yaw'])
        r_hy = rad2deg(row['dof_pos_R_hip_yaw'])

        print(f"  {t_actual:6.2f} {l_hr:12.2f} {r_hr:12.2f} {cl:10d} {cr:10d} {l_hy:12.2f} {r_hy:12.2f}")

# =========================================================
# Summary: "内股" (pigeon-toe) quantification
# =========================================================
print_section("6. Summary: 内股 (Pigeon-toe) Quantification")

# hip_roll sign convention: negative L = inward, positive R = inward
# "内股着地" means the stance leg hip_roll pushes foot inward toward centerline

for ver_label, df in [("V1", v1), ("V2", v2)]:
    print(f"\n  --- {ver_label} ---")

    # Left stance: hip_roll should be negative (inward) for pigeon-toe
    l_stance = df.loc[df['contact_left'] > 0]
    r_stance = df.loc[df['contact_right'] > 0]

    l_hr_mean = l_stance['dof_pos_L_hip_roll'].mean()
    r_hr_mean = r_stance['dof_pos_R_hip_roll'].mean()
    l_hy_mean = l_stance['dof_pos_L_hip_yaw'].mean()
    r_hy_mean = r_stance['dof_pos_R_hip_yaw'].mean()

    print(f"  L stance hip_roll mean: {rad2deg(l_hr_mean):+.2f}° (negative=inward)")
    print(f"  R stance hip_roll mean: {rad2deg(r_hr_mean):+.2f}° (positive=inward)")
    print(f"  |L| + |R| hip_roll offset: {rad2deg(abs(l_hr_mean) + abs(r_hr_mean)):.2f}° (total inward displacement)")
    print(f"  L stance hip_yaw mean: {rad2deg(l_hy_mean):+.2f}°")
    print(f"  R stance hip_yaw mean: {rad2deg(r_hy_mean):+.2f}°")

    # Knee/ankle during stance
    l_kp = rad2deg(l_stance['dof_pos_L_knee_pitch'].mean())
    r_kp = rad2deg(r_stance['dof_pos_R_knee_pitch'].mean())
    l_ap = rad2deg(l_stance['dof_pos_L_ankle_pitch'].mean())
    r_ap = rad2deg(r_stance['dof_pos_R_ankle_pitch'].mean())
    print(f"  L stance knee_pitch mean: {l_kp:.2f}°, ankle_pitch mean: {l_ap:.2f}°")
    print(f"  R stance knee_pitch mean: {r_kp:.2f}°, ankle_pitch mean: {r_ap:.2f}°")
