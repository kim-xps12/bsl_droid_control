"""
Deep analysis: V2 Yaw feedback loop mechanism.
Focus on:
- L_hip_yaw mean drift over time (positive trend visible in main analysis)
- Stance/swing asymmetry evolution
- Body Yaw → hip_yaw interaction (is hip_yaw compensating or amplifying?)
- hip_roll interaction with Yaw
"""

import pandas as pd
import numpy as np
from scipy import stats

def load_data(path):
    df = pd.read_csv(path)
    for col in ['dof_pos_L_hip_yaw', 'dof_pos_R_hip_yaw',
                'dof_pos_L_hip_roll', 'dof_pos_R_hip_roll',
                'dof_pos_L_hip_pitch', 'dof_pos_R_hip_pitch']:
        df[f'{col}_deg'] = np.degrees(df[col])
    return df

def analyze_hip_yaw_mean_drift(df, label):
    """Track L_hip_yaw and R_hip_yaw mean values in 1s windows"""
    print(f"\n{'='*60}")
    print(f"L/R hip_yaw MEAN drift over time ({label})")
    print(f"{'='*60}")

    max_t = df['timestamp'].max()
    print(f"  Time  | L_hy_mean | R_hy_mean | L+R (same_dir) | L-R (asym) | Yaw")
    for t_start in np.arange(0, max_t, 1.0):
        t_end = t_start + 1.0
        mask = (df['timestamp'] >= t_start) & (df['timestamp'] < t_end)
        w = df[mask]
        if len(w) < 5:
            continue
        l_mean = w['dof_pos_L_hip_yaw_deg'].mean()
        r_mean = w['dof_pos_R_hip_yaw_deg'].mean()
        yaw_mid = w['yaw_deg'].mean()
        # Same direction component (both positive = net body rotation torque)
        same_dir = l_mean + r_mean
        # Asymmetry
        asym = l_mean - r_mean
        print(f"  {t_start:4.0f}s | {l_mean:+8.2f}° | {r_mean:+8.2f}° | {same_dir:+10.2f}°    | {asym:+8.2f}°  | {yaw_mid:+7.1f}°")

def analyze_net_hip_yaw_torque(df, label):
    """
    When both hip_yaw joints are positive simultaneously, net angular momentum
    is generated. Analyze the instantaneous net torque direction.
    """
    print(f"\n{'='*60}")
    print(f"Net hip_yaw angular momentum analysis ({label})")
    print(f"{'='*60}")

    df = df.copy()
    # Angular velocity of hip_yaw joints
    dt = df['timestamp'].diff().median()

    # L_hip_yaw + R_hip_yaw = net yaw torque direction
    # When stance foot is on ground, hip_yaw rotation creates body yaw torque
    # L foot on ground + L_hip_yaw positive → body rotates positive
    # R foot on ground + R_hip_yaw positive → body rotates negative (opposite hip)

    # Stance-weighted hip_yaw: stance leg's hip_yaw contributes to body yaw
    df['stance_net_yaw'] = (
        df['contact_left'] * df['dof_pos_L_hip_yaw_deg'] -  # L stance: L_hip_yaw → +Yaw
        df['contact_right'] * df['dof_pos_R_hip_yaw_deg']   # R stance: R_hip_yaw → -Yaw (reaction)
    )

    # Alternative: during R stance, R_hip_yaw positive → body rotates in +Yaw direction
    # Actually let's think more carefully:
    # - Robot stands on right foot. R_hip_yaw rotates the hip joint.
    # - If R_hip_yaw increases (positive), the RIGHT thigh rotates outward
    # - By reaction, the torso rotates in +Yaw direction
    # So: R_stance × R_hip_yaw_positive → +Yaw body rotation
    # Similarly: L_stance × L_hip_yaw_positive → -Yaw body rotation (or +Yaw?)
    # Actually the sign depends on convention. Let's just check correlation.

    max_t = df['timestamp'].max()
    print(f"  Time  | stance_L | stance_R | L_hy_stance | R_hy_stance | Yaw_rate | Both_stance%")
    for t_start in np.arange(0, max_t, 1.0):
        t_end = t_start + 1.0
        mask = (df['timestamp'] >= t_start) & (df['timestamp'] < t_end)
        w = df[mask]
        if len(w) < 5:
            continue

        l_st = w[w['contact_left'] > 0.5]
        r_st = w[w['contact_right'] > 0.5]
        both = w[(w['contact_left'] > 0.5) & (w['contact_right'] > 0.5)]

        l_hy_st = l_st['dof_pos_L_hip_yaw_deg'].mean() if len(l_st) > 0 else 0
        r_hy_st = r_st['dof_pos_R_hip_yaw_deg'].mean() if len(r_st) > 0 else 0

        yaw_rate = (w['yaw_deg'].iloc[-1] - w['yaw_deg'].iloc[0]) / (w['timestamp'].iloc[-1] - w['timestamp'].iloc[0])
        both_pct = len(both) / len(w) * 100

        print(f"  {t_start:4.0f}s | {len(l_st)/len(w)*100:5.1f}%  | {len(r_st)/len(w)*100:5.1f}%  | {l_hy_st:+10.2f}° | {r_hy_st:+10.2f}° | {yaw_rate:+7.2f}  | {both_pct:5.1f}%")

def analyze_same_sign_problem(df, label):
    """
    Key insight: In V2, BOTH L and R hip_yaw are positive on average.
    This means a net torque in one direction.
    Analyze frames where L and R have the same sign.
    """
    print(f"\n{'='*60}")
    print(f"Same-sign hip_yaw problem ({label})")
    print(f"{'='*60}")

    df_active = df[df['timestamp'] >= 1.0].copy()

    l = df_active['dof_pos_L_hip_yaw_deg']
    r = df_active['dof_pos_R_hip_yaw_deg']

    both_pos = ((l > 0) & (r > 0)).sum()
    both_neg = ((l < 0) & (r < 0)).sum()
    l_pos_r_neg = ((l > 0) & (r < 0)).sum()
    l_neg_r_pos = ((l < 0) & (r > 0)).sum()
    total = len(df_active)

    print(f"  Both positive: {both_pos}/{total} ({both_pos/total*100:.1f}%)")
    print(f"  Both negative: {both_neg}/{total} ({both_neg/total*100:.1f}%)")
    print(f"  L+/R-: {l_pos_r_neg}/{total} ({l_pos_r_neg/total*100:.1f}%)")
    print(f"  L-/R+: {l_neg_r_pos}/{total} ({l_neg_r_pos/total*100:.1f}%)")

    # Mean hip_yaw in each configuration
    for name, mask in [("Both+", (l > 0) & (r > 0)),
                       ("Both-", (l < 0) & (r < 0)),
                       ("L+R-", (l > 0) & (r < 0)),
                       ("L-R+", (l < 0) & (r > 0))]:
        sub = df_active[mask]
        if len(sub) > 0:
            print(f"    {name}: L_mean={sub['dof_pos_L_hip_yaw_deg'].mean():+.2f}°, "
                  f"R_mean={sub['dof_pos_R_hip_yaw_deg'].mean():+.2f}°, "
                  f"avg Yaw_at_time={sub['yaw_deg'].mean():+.1f}°")

def analyze_hip_roll_interaction(df, label):
    """Analyze hip_roll changes and their interaction with Yaw"""
    print(f"\n{'='*60}")
    print(f"hip_roll vs Yaw interaction ({label})")
    print(f"{'='*60}")

    max_t = df['timestamp'].max()
    print(f"  Time  | L_hr_mean | R_hr_mean | L_hr_stance | R_hr_stance | Yaw")
    for t_start in np.arange(0, max_t, 2.0):
        t_end = t_start + 2.0
        mask = (df['timestamp'] >= t_start) & (df['timestamp'] < t_end)
        w = df[mask]
        if len(w) < 5:
            continue

        l_hr = w['dof_pos_L_hip_roll_deg'].mean()
        r_hr = w['dof_pos_R_hip_roll_deg'].mean()

        l_st = w[w['contact_left'] > 0.5]
        r_st = w[w['contact_right'] > 0.5]
        l_hr_st = l_st['dof_pos_L_hip_roll_deg'].mean() if len(l_st) > 0 else np.nan
        r_hr_st = r_st['dof_pos_R_hip_roll_deg'].mean() if len(r_st) > 0 else np.nan

        yaw_mid = w['yaw_deg'].mean()
        print(f"  {t_start:4.0f}s | {l_hr:+8.2f}° | {r_hr:+8.2f}° | {l_hr_st:+10.2f}° | {r_hr_st:+10.2f}° | {yaw_mid:+7.1f}°")

def analyze_yaw_hip_yaw_phase(df, label):
    """
    Check if hip_yaw leads or lags body Yaw.
    If hip_yaw LAGS Yaw, the controller isn't compensating - it's being dragged.
    If hip_yaw LEADS Yaw, the hip_yaw is CAUSING the drift.
    """
    print(f"\n{'='*60}")
    print(f"Phase analysis: does hip_yaw lead or lag body Yaw? ({label})")
    print(f"{'='*60}")

    dt = df['timestamp'].diff().median()

    # Cross-correlation between Yaw and L_hip_yaw
    yaw = df['yaw_deg'].values
    l_hy = df['dof_pos_L_hip_yaw_deg'].values
    r_hy = df['dof_pos_R_hip_yaw_deg'].values

    # Detrend to focus on oscillations
    from scipy.signal import detrend
    yaw_dt = detrend(yaw)
    l_hy_dt = detrend(l_hy)
    r_hy_dt = detrend(r_hy)

    # Cross-correlation at different lags
    max_lag = int(0.5 / dt)  # up to 0.5s lag
    print(f"  Cross-correlation (detrended, lag in frames, dt={dt*1000:.1f}ms):")
    print(f"  {'Lag(ms)':>8s} | {'Yaw×L_hy':>10s} | {'Yaw×R_hy':>10s}")

    for lag in range(-max_lag, max_lag+1, max(1, max_lag//5)):
        if lag >= 0:
            y = yaw_dt[lag:]
            lh = l_hy_dt[:len(y)]
            rh = r_hy_dt[:len(y)]
        else:
            lh = l_hy_dt[-lag:]
            rh = r_hy_dt[-lag:]
            y = yaw_dt[:len(lh)]

        if len(y) > 10:
            r_l = np.corrcoef(y, lh)[0,1]
            r_r = np.corrcoef(y, rh)[0,1]
            print(f"  {lag*dt*1000:+7.0f}  | {r_l:+10.4f} | {r_r:+10.4f}")

def main():
    v1_path = '/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs/droid-walking-narrow-v1/eval_499.csv'
    v2_path = '/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs/droid-walking-narrow-v2/eval_499.csv'

    df_v1 = load_data(v1_path)
    df_v2 = load_data(v2_path)

    # 1. hip_yaw mean drift
    analyze_hip_yaw_mean_drift(df_v1, "V1")
    analyze_hip_yaw_mean_drift(df_v2, "V2")

    # 2. Same-sign problem
    analyze_same_sign_problem(df_v1, "V1")
    analyze_same_sign_problem(df_v2, "V2")

    # 3. Net torque analysis
    analyze_net_hip_yaw_torque(df_v1, "V1")
    analyze_net_hip_yaw_torque(df_v2, "V2")

    # 4. hip_roll interaction
    analyze_hip_roll_interaction(df_v1, "V1")
    analyze_hip_roll_interaction(df_v2, "V2")

    # 5. Phase analysis
    analyze_yaw_hip_yaw_phase(df_v1, "V1")
    analyze_yaw_hip_yaw_phase(df_v2, "V2")

    # Key metric: L_hip_yaw mean over time - is it drifting?
    print(f"\n{'='*60}")
    print(f"KEY FINDING: L_hip_yaw mean trend")
    print(f"{'='*60}")
    for label, df in [("V1", df_v1), ("V2", df_v2)]:
        df_active = df[df['timestamp'] >= 1.0]
        slope_l, intercept_l, r_l, p_l, _ = stats.linregress(
            df_active['timestamp'], df_active['dof_pos_L_hip_yaw_deg'])
        slope_r, intercept_r, r_r, p_r, _ = stats.linregress(
            df_active['timestamp'], df_active['dof_pos_R_hip_yaw_deg'])
        slope_yaw, _, r_yaw, _, _ = stats.linregress(
            df_active['timestamp'], df_active['yaw_deg'])
        print(f"  {label}: L_hip_yaw slope={slope_l:+.3f}°/s (r²={r_l**2:.4f}, p={p_l:.2e})")
        print(f"  {label}: R_hip_yaw slope={slope_r:+.3f}°/s (r²={r_r**2:.4f}, p={p_r:.2e})")
        print(f"  {label}: Yaw slope={slope_yaw:+.3f}°/s (r²={r_yaw**2:.4f})")
        print(f"  {label}: L_hy/Yaw slope ratio={slope_l/slope_yaw:.4f}")
        print()

if __name__ == '__main__':
    main()
