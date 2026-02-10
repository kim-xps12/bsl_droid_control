"""
Yaw drift acceleration mechanism analysis for exp008 V2 vs V1.
Analyzes:
1. Yaw acceleration over time (°/s per 1-second windows)
2. hip_yaw L/R stance statistics
3. hip_yaw L/R swing statistics
4. Yaw drift vs hip_yaw asymmetry correlation
5. Critical point analysis (feedback loop evidence)
"""

import pandas as pd
import numpy as np
from scipy import stats

def load_data(path):
    df = pd.read_csv(path)
    # Convert dof_pos from radians to degrees for hip_yaw
    for col in ['dof_pos_L_hip_yaw', 'dof_pos_R_hip_yaw',
                'dof_pos_L_hip_roll', 'dof_pos_R_hip_roll']:
        df[f'{col}_deg'] = np.degrees(df[col])
    return df

def analyze_yaw_rate(df, label):
    """1. Yaw acceleration over time - 1-second windows"""
    print(f"\n{'='*60}")
    print(f"1. Yaw Rate Analysis ({label})")
    print(f"{'='*60}")

    dt = df['timestamp'].diff().median()
    max_t = df['timestamp'].max()

    results = []
    for t_start in np.arange(0, max_t, 1.0):
        t_end = t_start + 1.0
        mask = (df['timestamp'] >= t_start) & (df['timestamp'] < t_end)
        window = df[mask]
        if len(window) < 2:
            continue
        yaw_start = window['yaw_deg'].iloc[0]
        yaw_end = window['yaw_deg'].iloc[-1]
        yaw_rate = (yaw_end - yaw_start) / (window['timestamp'].iloc[-1] - window['timestamp'].iloc[0])
        results.append({
            't_start': t_start,
            'yaw_start': yaw_start,
            'yaw_end': yaw_end,
            'yaw_rate': yaw_rate,
        })

    results_df = pd.DataFrame(results)
    print(f"  Time(s) | Yaw_start(°) | Yaw_end(°) | Yaw_rate(°/s)")
    print(f"  --------|--------------|------------|---------------")
    for _, r in results_df.iterrows():
        print(f"  {r['t_start']:5.0f}-{r['t_start']+1:.0f}  | {r['yaw_start']:+11.2f}  | {r['yaw_end']:+9.2f}  | {r['yaw_rate']:+11.3f}")

    # Acceleration: rate of change of yaw_rate
    if len(results_df) >= 3:
        yaw_rates = results_df['yaw_rate'].values
        yaw_accel = np.diff(yaw_rates)
        print(f"\n  Yaw acceleration (change in rate °/s²):")
        for i, a in enumerate(yaw_accel):
            print(f"    {i+1}s→{i+2}s: {a:+.3f} °/s²")

    return results_df

def analyze_stance_hip_yaw(df, label):
    """2. hip_yaw L/R stance statistics"""
    print(f"\n{'='*60}")
    print(f"2. Stance hip_yaw Statistics ({label})")
    print(f"{'='*60}")

    # Filter out first 1s (initial settling)
    df_active = df[df['timestamp'] >= 1.0].copy()

    # Left stance = contact_left > 0.5
    l_stance = df_active[df_active['contact_left'] > 0.5]
    r_stance = df_active[df_active['contact_right'] > 0.5]

    # Left hip_yaw during left stance
    l_hy_l_stance = l_stance['dof_pos_L_hip_yaw_deg']
    r_hy_r_stance = r_stance['dof_pos_R_hip_yaw_deg']

    # Also get stance leg hip_yaw (the grounded leg's hip_yaw)
    print(f"  Left hip_yaw during LEFT stance:")
    print(f"    mean: {l_hy_l_stance.mean():+.3f}°, std: {l_hy_l_stance.std():.3f}°")
    print(f"    min: {l_hy_l_stance.min():+.3f}°, max: {l_hy_l_stance.max():+.3f}°")
    print(f"    range: {l_hy_l_stance.max() - l_hy_l_stance.min():.3f}°")

    print(f"  Right hip_yaw during RIGHT stance:")
    print(f"    mean: {r_hy_r_stance.mean():+.3f}°, std: {r_hy_r_stance.std():.3f}°")
    print(f"    min: {r_hy_r_stance.min():+.3f}°, max: {r_hy_r_stance.max():+.3f}°")
    print(f"    range: {r_hy_r_stance.max() - r_hy_r_stance.min():.3f}°")

    print(f"\n  Asymmetry (|L_mean| vs |R_mean|): L={abs(l_hy_l_stance.mean()):.3f}° vs R={abs(r_hy_r_stance.mean()):.3f}°")
    print(f"  Asymmetry ratio: {abs(l_hy_l_stance.mean()) / max(abs(r_hy_r_stance.mean()), 0.001):.3f}")

    # Time-resolved stance hip_yaw (2-second windows)
    print(f"\n  Time-resolved stance hip_yaw (2s windows):")
    max_t = df_active['timestamp'].max()
    for t_start in np.arange(1, max_t, 2.0):
        t_end = t_start + 2.0
        mask = (df_active['timestamp'] >= t_start) & (df_active['timestamp'] < t_end)
        window = df_active[mask]
        l_st = window[window['contact_left'] > 0.5]['dof_pos_L_hip_yaw_deg']
        r_st = window[window['contact_right'] > 0.5]['dof_pos_R_hip_yaw_deg']
        if len(l_st) > 0 and len(r_st) > 0:
            print(f"    {t_start:.0f}-{t_end:.0f}s: L_stance_hip_yaw={l_st.mean():+.2f}°, R_stance_hip_yaw={r_st.mean():+.2f}°")

    return {
        'L_stance_mean': l_hy_l_stance.mean(),
        'R_stance_mean': r_hy_r_stance.mean(),
        'L_stance_range': l_hy_l_stance.max() - l_hy_l_stance.min(),
        'R_stance_range': r_hy_r_stance.max() - r_hy_r_stance.min(),
    }

def analyze_swing_hip_yaw(df, label):
    """3. hip_yaw L/R swing statistics"""
    print(f"\n{'='*60}")
    print(f"3. Swing hip_yaw Statistics ({label})")
    print(f"{'='*60}")

    df_active = df[df['timestamp'] >= 1.0].copy()

    # Left swing = contact_left < 0.5
    l_swing = df_active[df_active['contact_left'] < 0.5]
    r_swing = df_active[df_active['contact_right'] < 0.5]

    l_hy_swing = l_swing['dof_pos_L_hip_yaw_deg']
    r_hy_swing = r_swing['dof_pos_R_hip_yaw_deg']

    print(f"  Left hip_yaw during LEFT swing:")
    print(f"    mean: {l_hy_swing.mean():+.3f}°, std: {l_hy_swing.std():.3f}°")
    print(f"    range: {l_hy_swing.max() - l_hy_swing.min():.3f}°")
    print(f"    N frames: {len(l_hy_swing)}")

    print(f"  Right hip_yaw during RIGHT swing:")
    print(f"    mean: {r_hy_swing.mean():+.3f}°, std: {r_hy_swing.std():.3f}°")
    print(f"    range: {r_hy_swing.max() - r_hy_swing.min():.3f}°")
    print(f"    N frames: {len(r_hy_swing)}")

    # Time-resolved swing hip_yaw
    print(f"\n  Time-resolved swing hip_yaw (2s windows):")
    max_t = df_active['timestamp'].max()
    for t_start in np.arange(1, max_t, 2.0):
        t_end = t_start + 2.0
        mask = (df_active['timestamp'] >= t_start) & (df_active['timestamp'] < t_end)
        window = df_active[mask]
        l_sw = window[window['contact_left'] < 0.5]['dof_pos_L_hip_yaw_deg']
        r_sw = window[window['contact_right'] < 0.5]['dof_pos_R_hip_yaw_deg']
        if len(l_sw) > 0 and len(r_sw) > 0:
            print(f"    {t_start:.0f}-{t_end:.0f}s: L_swing_hip_yaw={l_sw.mean():+.2f}°, R_swing_hip_yaw={r_sw.mean():+.2f}°")

def analyze_yaw_asymmetry_correlation(df, label):
    """4. Yaw drift vs hip_yaw asymmetry correlation"""
    print(f"\n{'='*60}")
    print(f"4. Yaw Drift vs hip_yaw Asymmetry Correlation ({label})")
    print(f"{'='*60}")

    max_t = df['timestamp'].max()
    results = []

    for t_start in np.arange(0, max_t, 1.0):
        t_end = t_start + 1.0
        mask = (df['timestamp'] >= t_start) & (df['timestamp'] < t_end)
        window = df[mask]
        if len(window) < 10:
            continue

        # Yaw rate
        yaw_rate = (window['yaw_deg'].iloc[-1] - window['yaw_deg'].iloc[0]) / \
                   (window['timestamp'].iloc[-1] - window['timestamp'].iloc[0])

        # hip_yaw L/R ranges in this window
        l_range = window['dof_pos_L_hip_yaw_deg'].max() - window['dof_pos_L_hip_yaw_deg'].min()
        r_range = window['dof_pos_R_hip_yaw_deg'].max() - window['dof_pos_R_hip_yaw_deg'].min()
        asymmetry = l_range - r_range  # positive = L has larger range
        abs_asymmetry = abs(asymmetry)

        # hip_yaw mean difference (signed)
        l_mean = window['dof_pos_L_hip_yaw_deg'].mean()
        r_mean = window['dof_pos_R_hip_yaw_deg'].mean()
        mean_diff = l_mean - r_mean  # signed mean difference

        # Stance leg hip_yaw means
        l_stance = window[window['contact_left'] > 0.5]['dof_pos_L_hip_yaw_deg']
        r_stance = window[window['contact_right'] > 0.5]['dof_pos_R_hip_yaw_deg']
        stance_l_mean = l_stance.mean() if len(l_stance) > 0 else np.nan
        stance_r_mean = r_stance.mean() if len(r_stance) > 0 else np.nan

        results.append({
            't_start': t_start,
            'yaw_rate': yaw_rate,
            'l_range': l_range,
            'r_range': r_range,
            'asymmetry': asymmetry,
            'abs_asymmetry': abs_asymmetry,
            'mean_diff': mean_diff,
            'l_mean': l_mean,
            'r_mean': r_mean,
            'stance_l_mean': stance_l_mean,
            'stance_r_mean': stance_r_mean,
        })

    results_df = pd.DataFrame(results)

    # Correlation: yaw_rate vs range asymmetry
    valid = results_df.dropna()
    if len(valid) >= 3:
        r_range_asym, p_range = stats.pearsonr(valid['yaw_rate'], valid['asymmetry'])
        r_mean_diff, p_mean = stats.pearsonr(valid['yaw_rate'], valid['mean_diff'])
        r_abs_asym, p_abs = stats.pearsonr(valid['yaw_rate'].abs(), valid['abs_asymmetry'])

        print(f"  Pearson correlation (yaw_rate vs L-R range asymmetry): r={r_range_asym:.4f}, p={p_range:.4f}")
        print(f"  Pearson correlation (yaw_rate vs L-R mean diff): r={r_mean_diff:.4f}, p={p_mean:.4f}")
        print(f"  Pearson correlation (|yaw_rate| vs |asymmetry|): r={r_abs_asym:.4f}, p={p_abs:.4f}")

    print(f"\n  Per-second details:")
    print(f"  Time  | Yaw_rate | L_range | R_range | Asym  | L_mean  | R_mean  | StanceL | StanceR")
    for _, r in results_df.iterrows():
        sl = f"{r['stance_l_mean']:+.2f}" if not np.isnan(r['stance_l_mean']) else "  N/A"
        sr = f"{r['stance_r_mean']:+.2f}" if not np.isnan(r['stance_r_mean']) else "  N/A"
        print(f"  {r['t_start']:4.0f}s | {r['yaw_rate']:+7.2f} | {r['l_range']:6.2f}  | {r['r_range']:6.2f}  | {r['asymmetry']:+5.2f} | {r['l_mean']:+6.2f}  | {r['r_mean']:+6.2f}  | {sl}  | {sr}")

    return results_df

def analyze_critical_point(df, label):
    """5. Critical point analysis - feedback loop evidence"""
    print(f"\n{'='*60}")
    print(f"5. Critical Point Analysis ({label})")
    print(f"{'='*60}")

    dt = df['timestamp'].diff().median()

    # Compute instantaneous yaw rate (smoothed over ~0.2s window)
    window_size = max(1, int(0.2 / dt))
    df = df.copy()
    df['yaw_rate'] = df['yaw_deg'].diff().rolling(window=window_size, center=True).mean() / dt

    # Find where yaw_rate acceleration is maximum
    df['yaw_accel'] = df['yaw_rate'].diff().rolling(window=window_size, center=True).mean() / dt

    # Find the timestamp where |yaw_rate| first exceeds thresholds
    thresholds = [5, 10, 15, 20, 30, 50]
    print(f"  Yaw rate threshold crossings:")
    for thresh in thresholds:
        exceeds = df[df['yaw_rate'].abs() > thresh]
        if len(exceeds) > 0:
            t = exceeds['timestamp'].iloc[0]
            row = exceeds.iloc[0]
            print(f"    |yaw_rate| > {thresh}°/s at t={t:.3f}s: "
                  f"Yaw={row['yaw_deg']:+.2f}°, "
                  f"L_hip_yaw={row['dof_pos_L_hip_yaw_deg']:+.2f}°, "
                  f"R_hip_yaw={row['dof_pos_R_hip_yaw_deg']:+.2f}°, "
                  f"L_hip_roll={row['dof_pos_L_hip_roll_deg']:+.2f}°, "
                  f"R_hip_roll={row['dof_pos_R_hip_roll_deg']:+.2f}°")
        else:
            print(f"    |yaw_rate| > {thresh}°/s: never reached")

    # Check for feedback loop: does hip_yaw asymmetry grow with yaw?
    print(f"\n  Feedback loop analysis (Yaw vs hip_yaw asymmetry over time):")
    df['hip_yaw_asym'] = df['dof_pos_L_hip_yaw_deg'].abs() - df['dof_pos_R_hip_yaw_deg'].abs()

    # Sample at 0.5s intervals
    max_t = df['timestamp'].max()
    for t in np.arange(0, max_t, 0.5):
        idx = (df['timestamp'] - t).abs().idxmin()
        row = df.loc[idx]
        if not np.isnan(row.get('yaw_rate', np.nan)):
            print(f"    t={t:5.1f}s: Yaw={row['yaw_deg']:+8.2f}°, "
                  f"rate={row['yaw_rate']:+8.2f}°/s, "
                  f"L_hy={row['dof_pos_L_hip_yaw_deg']:+7.2f}°, "
                  f"R_hy={row['dof_pos_R_hip_yaw_deg']:+7.2f}°, "
                  f"|L|-|R|={row['hip_yaw_asym']:+6.2f}°")

    # Check if hip_yaw range grows over time (loss of control)
    print(f"\n  hip_yaw range evolution (2s windows):")
    for t_start in np.arange(0, max_t, 2.0):
        t_end = t_start + 2.0
        mask = (df['timestamp'] >= t_start) & (df['timestamp'] < t_end)
        window = df[mask]
        if len(window) < 10:
            continue
        l_range = window['dof_pos_L_hip_yaw_deg'].max() - window['dof_pos_L_hip_yaw_deg'].min()
        r_range = window['dof_pos_R_hip_yaw_deg'].max() - window['dof_pos_R_hip_yaw_deg'].min()
        l_std = window['dof_pos_L_hip_yaw_deg'].std()
        r_std = window['dof_pos_R_hip_yaw_deg'].std()
        yaw_at_start = window['yaw_deg'].iloc[0]
        print(f"    {t_start:.0f}-{t_end:.0f}s: Yaw={yaw_at_start:+.1f}°, "
              f"L_range={l_range:.2f}°, R_range={r_range:.2f}°, "
              f"L_std={l_std:.2f}°, R_std={r_std:.2f}°")

    # Correlation between cumulative yaw and hip_yaw values
    valid = df.dropna(subset=['yaw_rate'])
    if len(valid) > 10:
        r_yaw_lhy, p1 = stats.pearsonr(valid['yaw_deg'], valid['dof_pos_L_hip_yaw_deg'])
        r_yaw_rhy, p2 = stats.pearsonr(valid['yaw_deg'], valid['dof_pos_R_hip_yaw_deg'])
        r_yaw_asym, p3 = stats.pearsonr(valid['yaw_deg'], valid['hip_yaw_asym'])
        print(f"\n  Correlation (cumulative Yaw vs joint values):")
        print(f"    Yaw vs L_hip_yaw: r={r_yaw_lhy:.4f}, p={p1:.4e}")
        print(f"    Yaw vs R_hip_yaw: r={r_yaw_rhy:.4f}, p={p2:.4e}")
        print(f"    Yaw vs |L|-|R| asymmetry: r={r_yaw_asym:.4f}, p={p3:.4e}")


def main():
    v1_path = '/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs/droid-walking-narrow-v1/eval_499.csv'
    v2_path = '/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs/droid-walking-narrow-v2/eval_499.csv'

    print("Loading data...")
    df_v1 = load_data(v1_path)
    df_v2 = load_data(v2_path)

    print(f"V1: {len(df_v1)} frames, {df_v1['timestamp'].max():.2f}s")
    print(f"V2: {len(df_v2)} frames, {df_v2['timestamp'].max():.2f}s")
    print(f"V1 final Yaw: {df_v1['yaw_deg'].iloc[-1]:+.2f}°")
    print(f"V2 final Yaw: {df_v2['yaw_deg'].iloc[-1]:+.2f}°")

    # 1. Yaw rate analysis
    rates_v1 = analyze_yaw_rate(df_v1, "V1")
    rates_v2 = analyze_yaw_rate(df_v2, "V2")

    # 2. Stance hip_yaw
    stance_v1 = analyze_stance_hip_yaw(df_v1, "V1")
    stance_v2 = analyze_stance_hip_yaw(df_v2, "V2")

    # Comparison
    print(f"\n{'='*60}")
    print(f"Stance hip_yaw Comparison (V1 vs V2)")
    print(f"{'='*60}")
    print(f"  L_stance mean: V1={stance_v1['L_stance_mean']:+.3f}° → V2={stance_v2['L_stance_mean']:+.3f}° (Δ={stance_v2['L_stance_mean']-stance_v1['L_stance_mean']:+.3f}°)")
    print(f"  R_stance mean: V1={stance_v1['R_stance_mean']:+.3f}° → V2={stance_v2['R_stance_mean']:+.3f}° (Δ={stance_v2['R_stance_mean']-stance_v1['R_stance_mean']:+.3f}°)")
    print(f"  L_stance range: V1={stance_v1['L_stance_range']:.3f}° → V2={stance_v2['L_stance_range']:.3f}°")
    print(f"  R_stance range: V1={stance_v1['R_stance_range']:.3f}° → V2={stance_v2['R_stance_range']:.3f}°")

    # 3. Swing hip_yaw
    analyze_swing_hip_yaw(df_v1, "V1")
    analyze_swing_hip_yaw(df_v2, "V2")

    # 4. Correlation analysis
    corr_v1 = analyze_yaw_asymmetry_correlation(df_v1, "V1")
    corr_v2 = analyze_yaw_asymmetry_correlation(df_v2, "V2")

    # 5. Critical point analysis
    analyze_critical_point(df_v1, "V1")
    analyze_critical_point(df_v2, "V2")

    # Final summary
    print(f"\n{'='*60}")
    print(f"SUMMARY")
    print(f"{'='*60}")
    print(f"V1: Final Yaw={df_v1['yaw_deg'].iloc[-1]:+.2f}°, avg rate={df_v1['yaw_deg'].iloc[-1]/df_v1['timestamp'].max():.2f}°/s")
    print(f"V2: Final Yaw={df_v2['yaw_deg'].iloc[-1]:+.2f}°, avg rate={df_v2['yaw_deg'].iloc[-1]/df_v2['timestamp'].max():.2f}°/s")
    print(f"V2/V1 drift rate ratio: {(df_v2['yaw_deg'].iloc[-1]/df_v2['timestamp'].max()) / (df_v1['yaw_deg'].iloc[-1]/df_v1['timestamp'].max()):.2f}x")

if __name__ == '__main__':
    main()
