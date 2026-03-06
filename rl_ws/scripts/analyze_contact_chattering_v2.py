"""
着地時足首タッピング - 深層分析
接地フラグにはチャタリングが見られなかったため、
足首の動きの質的分析と着地衝撃パターンを詳細に分析
"""

import pandas as pd
import numpy as np


def load_data(path):
    return pd.read_csv(path)


def detect_transitions(contact_series, timestamps):
    transitions = []
    for i in range(1, len(contact_series)):
        prev = contact_series.iloc[i - 1]
        curr = contact_series.iloc[i]
        if prev != curr:
            transitions.append({
                'idx': i,
                'time': timestamps.iloc[i],
                'type': 'landing' if curr == 1.0 else 'liftoff',
            })
    return transitions


def analyze_stance_ankle_oscillation(df, contact_col, side, label):
    """
    接地中の足首ピッチ振動パターンを分析。
    「地面を叩く」= ankle_pitchが接地中に上下に大きく振動するパターン
    """
    print(f"\n--- {label} {side}脚: 接地中ankle_pitch振動分析 ---")

    vel_col = f'dof_vel_{side}_ankle_pitch'
    pos_col = f'dof_pos_{side}_ankle_pitch'

    df_ss = df[df['timestamp'] > 2.0].copy()
    transitions = detect_transitions(df_ss[contact_col], df_ss['timestamp'])

    # Find stance phases (landing to liftoff)
    landings = [t for t in transitions if t['type'] == 'landing']
    liftoffs = [t for t in transitions if t['type'] == 'liftoff']

    stance_results = []
    for landing in landings:
        # Find next liftoff
        next_liftoffs = [lo for lo in liftoffs if lo['time'] > landing['time']]
        if not next_liftoffs:
            continue
        liftoff = next_liftoffs[0]

        # Extract stance phase data
        mask = (df_ss['timestamp'] >= landing['time']) & (df_ss['timestamp'] <= liftoff['time'])
        stance = df_ss[mask]

        if len(stance) < 3:
            continue

        vel = stance[vel_col].values
        pos = stance[pos_col].values
        times = stance['timestamp'].values
        duration = times[-1] - times[0]

        # Count zero crossings of velocity (oscillation indicator)
        zero_crossings = 0
        for i in range(1, len(vel)):
            if vel[i - 1] * vel[i] < 0:
                zero_crossings += 1

        # Count direction reversals in position
        pos_reversals = 0
        for i in range(1, len(pos) - 1):
            if (pos[i] - pos[i - 1]) * (pos[i + 1] - pos[i]) < 0:
                pos_reversals += 1

        # Ankle pitch range during stance
        pos_range = np.max(pos) - np.min(pos)
        vel_rms = np.sqrt(np.mean(vel ** 2))

        # First 0.1s after landing - "impact phase"
        impact_mask = (df_ss['timestamp'] >= landing['time']) & (
            df_ss['timestamp'] <= landing['time'] + 0.1
        )
        impact = df_ss[impact_mask]
        if len(impact) > 0:
            impact_vel_max = np.max(np.abs(impact[vel_col].values))
            impact_pos_range = np.max(impact[pos_col].values) - np.min(impact[pos_col].values)
        else:
            impact_vel_max = 0
            impact_pos_range = 0

        stance_results.append({
            'landing_time': landing['time'],
            'duration': duration,
            'vel_zero_crossings': zero_crossings,
            'pos_reversals': pos_reversals,
            'pos_range_rad': pos_range,
            'pos_range_deg': np.degrees(pos_range),
            'vel_rms': vel_rms,
            'vel_max': np.max(np.abs(vel)),
            'impact_vel_max': impact_vel_max,
            'impact_pos_range_rad': impact_pos_range,
            'n_steps': len(stance),
        })

    # Print results
    print(f"  {'landing_t':>10s} | {'dur(s)':>6s} | {'vel_0x':>6s} | {'pos_rev':>7s} | {'pos_rng°':>8s} | {'vel_rms':>7s} | {'vel_max':>7s} | {'imp_vel':>7s} | {'imp_rng°':>8s}")
    print("  " + "-" * 95)
    for r in stance_results:
        print(
            f"  {r['landing_time']:10.3f} | {r['duration']:6.3f} | {r['vel_zero_crossings']:6d} | {r['pos_reversals']:7d} | {r['pos_range_deg']:8.2f} | {r['vel_rms']:7.3f} | {r['vel_max']:7.3f} | {r['impact_vel_max']:7.3f} | {np.degrees(r['impact_pos_range_rad']):8.2f}"
        )

    if stance_results:
        avg_zc = np.mean([r['vel_zero_crossings'] for r in stance_results])
        avg_rev = np.mean([r['pos_reversals'] for r in stance_results])
        avg_range = np.mean([r['pos_range_deg'] for r in stance_results])
        avg_vel_rms = np.mean([r['vel_rms'] for r in stance_results])
        avg_vel_max = np.mean([r['vel_max'] for r in stance_results])
        avg_imp_vel = np.mean([r['impact_vel_max'] for r in stance_results])
        print(f"\n  平均: vel_0x={avg_zc:.1f}, pos_rev={avg_rev:.1f}, pos_range={avg_range:.2f}°, vel_rms={avg_vel_rms:.3f}, vel_max={avg_vel_max:.3f}, impact_vel={avg_imp_vel:.3f}")

    return stance_results


def analyze_both_feet_phase(df, label):
    """両足接地フェーズの分析 - タッピングは両足接地フェーズで起きやすい"""
    print(f"\n--- {label}: 両足接地フェーズ分析 ---")

    df_ss = df[df['timestamp'] > 2.0].copy()

    both = (df_ss['contact_left'] == 1.0) & (df_ss['contact_right'] == 1.0)
    both_indices = df_ss[both].index

    if len(both_indices) == 0:
        print("  両足接地フレームなし")
        return

    # Group consecutive both-feet frames
    groups = []
    current_group = [both_indices[0]]
    for i in range(1, len(both_indices)):
        if both_indices[i] == both_indices[i - 1] + 1:
            current_group.append(both_indices[i])
        else:
            groups.append(current_group)
            current_group = [both_indices[i]]
    groups.append(current_group)

    print(f"  両足接地フレーム数: {len(both_indices)} / {len(df_ss)} ({100 * len(both_indices) / len(df_ss):.1f}%)")
    print(f"  両足接地イベント数: {len(groups)}")

    for i, g in enumerate(groups):
        t_start = df_ss.loc[g[0], 'timestamp']
        t_end = df_ss.loc[g[-1], 'timestamp']
        dur = t_end - t_start
        n_frames = len(g)

        # Ankle pitch during this phase
        l_vel = df_ss.loc[g, 'dof_vel_L_ankle_pitch'].values
        r_vel = df_ss.loc[g, 'dof_vel_R_ankle_pitch'].values
        l_pos = df_ss.loc[g, 'dof_pos_L_ankle_pitch'].values
        r_pos = df_ss.loc[g, 'dof_pos_R_ankle_pitch'].values

        print(
            f"  #{i + 1}: t={t_start:.3f}-{t_end:.3f}s ({n_frames} frames, {dur * 1000:.0f}ms)"
            f"  L_ankle: pos={l_pos[0]:.3f}->{l_pos[-1]:.3f}, vel_max={np.max(np.abs(l_vel)):.3f}"
            f"  R_ankle: pos={r_pos[0]:.3f}->{r_pos[-1]:.3f}, vel_max={np.max(np.abs(r_vel)):.3f}"
        )


def analyze_swing_ankle_pumping(df, contact_col, side, label):
    """
    遊脚中のankle_pitchポンピング（繰り返し屈伸）を検出。
    「足首で地面を叩く」は遊脚中に足首が動いて接地の瞬間に衝撃が生まれることかも
    """
    print(f"\n--- {label} {side}脚: 遊脚中ankle_pitchポンピング ---")

    vel_col = f'dof_vel_{side}_ankle_pitch'
    pos_col = f'dof_pos_{side}_ankle_pitch'

    df_ss = df[df['timestamp'] > 2.0].copy()
    transitions = detect_transitions(df_ss[contact_col], df_ss['timestamp'])

    liftoffs = [t for t in transitions if t['type'] == 'liftoff']
    landings = [t for t in transitions if t['type'] == 'landing']

    for lo in liftoffs:
        next_landings = [l for l in landings if l['time'] > lo['time']]
        if not next_landings:
            continue
        landing = next_landings[0]

        mask = (df_ss['timestamp'] >= lo['time']) & (df_ss['timestamp'] <= landing['time'])
        swing = df_ss[mask]

        if len(swing) < 3:
            continue

        pos = swing[pos_col].values
        vel = swing[vel_col].values
        times = swing['timestamp'].values

        # Direction reversals
        reversals = 0
        for i in range(1, len(pos) - 1):
            if (pos[i] - pos[i - 1]) * (pos[i + 1] - pos[i]) < 0:
                reversals += 1

        pos_range = np.max(pos) - np.min(pos)

        # velocity at the very end (approaching landing)
        vel_at_end = vel[-1]

        print(
            f"  swing t={lo['time']:.3f}→{landing['time']:.3f}s "
            f"({(landing['time'] - lo['time']) * 1000:.0f}ms): "
            f"reversals={reversals}, range={np.degrees(pos_range):.1f}°, "
            f"vel_at_end={vel_at_end:+.3f} rad/s, "
            f"pos: {np.degrees(pos[0]):.1f}→{np.degrees(pos[-1]):.1f}°"
        )


def analyze_base_vel_z_at_landing(df, label):
    """着地時のbase_vel_z（垂直速度）- 衝撃の大きさの指標"""
    print(f"\n--- {label}: 着地時base_vel_z（衝撃指標）---")

    df_ss = df[df['timestamp'] > 2.0].copy()

    for side, contact_col in [('L', 'contact_left'), ('R', 'contact_right')]:
        transitions = detect_transitions(df_ss[contact_col], df_ss['timestamp'])
        landings = [t for t in transitions if t['type'] == 'landing']

        vel_z_at_landing = []
        for l in landings:
            vel_z = df_ss.iloc[l['idx'] - df_ss.index[0]]['base_vel_z'] if l['idx'] >= df_ss.index[0] else 0
            vel_z_at_landing.append(vel_z)
            print(f"  {side} landing t={l['time']:.3f}s: base_vel_z={vel_z:.4f} m/s")

        if vel_z_at_landing:
            print(f"  {side} 平均 base_vel_z at landing: {np.mean(vel_z_at_landing):.4f} m/s")


def main():
    v1_path = '/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs/droid-walking-narrow-v1/eval_499.csv'
    v2_path = '/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs/droid-walking-narrow-v2/eval_499.csv'

    df_v1 = load_data(v1_path)
    df_v2 = load_data(v2_path)

    # 1. 接地中の足首振動分析
    for side, cc in [('L', 'contact_left'), ('R', 'contact_right')]:
        analyze_stance_ankle_oscillation(df_v1, cc, side, 'V1')
        analyze_stance_ankle_oscillation(df_v2, cc, side, 'V2')

    # 2. 両足接地フェーズ
    analyze_both_feet_phase(df_v1, 'V1')
    analyze_both_feet_phase(df_v2, 'V2')

    # 3. 遊脚中ankle_pitchポンピング
    for side, cc in [('L', 'contact_left'), ('R', 'contact_right')]:
        analyze_swing_ankle_pumping(df_v1, cc, side, 'V1')
        analyze_swing_ankle_pumping(df_v2, cc, side, 'V2')

    # 4. 着地時の垂直速度
    analyze_base_vel_z_at_landing(df_v1, 'V1')
    analyze_base_vel_z_at_landing(df_v2, 'V2')

    # サマリー比較テーブル
    print("\n" + "=" * 80)
    print("  V1 vs V2 足首タッピング関連指標 サマリー")
    print("=" * 80)

    for version, df in [('V1', df_v1), ('V2', df_v2)]:
        df_ss = df[df['timestamp'] > 2.0]
        print(f"\n{version}:")
        for side, cc in [('L', 'contact_left'), ('R', 'contact_right')]:
            vel_col = f'dof_vel_{side}_ankle_pitch'
            pos_col = f'dof_pos_{side}_ankle_pitch'

            # Stance ankle stats
            stance_mask = df_ss[cc] == 1.0
            if stance_mask.sum() > 0:
                stance_vel_rms = np.sqrt(np.mean(df_ss.loc[stance_mask, vel_col] ** 2))
                stance_pos_range = np.degrees(
                    df_ss.loc[stance_mask, pos_col].max() - df_ss.loc[stance_mask, pos_col].min()
                )
                stance_vel_max = np.max(np.abs(df_ss.loc[stance_mask, vel_col]))
            else:
                stance_vel_rms = stance_pos_range = stance_vel_max = 0

            # Swing ankle stats
            swing_mask = df_ss[cc] == 0.0
            if swing_mask.sum() > 0:
                swing_vel_rms = np.sqrt(np.mean(df_ss.loc[swing_mask, vel_col] ** 2))
                swing_pos_range = np.degrees(
                    df_ss.loc[swing_mask, pos_col].max() - df_ss.loc[swing_mask, pos_col].min()
                )
            else:
                swing_vel_rms = swing_pos_range = 0

            print(
                f"  {side}: stance_vel_rms={stance_vel_rms:.3f}, stance_vel_max={stance_vel_max:.3f}, "
                f"stance_pos_range={stance_pos_range:.1f}°, "
                f"swing_vel_rms={swing_vel_rms:.3f}, swing_pos_range={swing_pos_range:.1f}°"
            )


if __name__ == '__main__':
    main()
