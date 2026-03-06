"""
着地時足首タッピング（チャタリング）の定量分析
V1 vs V2 比較
"""

import pandas as pd
import numpy as np

def load_data(path):
    df = pd.read_csv(path)
    return df

def detect_transitions(contact_series, timestamps):
    """接地フラグの遷移を検出"""
    transitions = []
    for i in range(1, len(contact_series)):
        prev = contact_series.iloc[i-1]
        curr = contact_series.iloc[i]
        if prev != curr:
            transitions.append({
                'idx': i,
                'time': timestamps.iloc[i],
                'type': 'landing' if curr == 1.0 else 'liftoff',
                'from': prev,
                'to': curr,
            })
    return transitions

def detect_chattering(transitions, threshold=0.1):
    """
    着地(0→1)後、threshold秒以内に再度0→1が発生するパターンを検出
    """
    landings = [t for t in transitions if t['type'] == 'landing']
    chattering_events = []
    for i in range(1, len(landings)):
        dt = landings[i]['time'] - landings[i-1]['time']
        if dt <= threshold:
            chattering_events.append({
                'first_landing_time': landings[i-1]['time'],
                'second_landing_time': landings[i]['time'],
                'interval': dt,
            })
    return chattering_events

def detect_swing_contacts(contact_series, timestamps, transitions):
    """
    遊脚中（離地後、次の着地前）に短時間接触する回数をカウント
    swing phase中にcontact=1が短時間(<0.06s = 3ステップ)出現するイベント
    """
    # Find swing phases (liftoff to landing)
    swing_contacts = []
    liftoffs = [t for t in transitions if t['type'] == 'liftoff']
    landings = [t for t in transitions if t['type'] == 'landing']

    for lo in liftoffs:
        # Find next landing after this liftoff
        next_landings = [l for l in landings if l['time'] > lo['time']]
        if not next_landings:
            continue
        next_landing = next_landings[0]

        # Check for brief contacts during this swing phase
        swing_start_idx = lo['idx']
        swing_end_idx = next_landing['idx']

        swing_segment = contact_series.iloc[swing_start_idx:swing_end_idx]
        swing_times = timestamps.iloc[swing_start_idx:swing_end_idx]

        # Count brief contact episodes within swing
        in_contact = False
        contact_start = None
        for j in range(len(swing_segment)):
            val = swing_segment.iloc[j]
            if val == 1.0 and not in_contact:
                in_contact = True
                contact_start = j
            elif val == 0.0 and in_contact:
                in_contact = False
                duration = (j - contact_start) * 0.02
                if duration <= 0.06:  # Brief contact
                    swing_contacts.append({
                        'swing_start_time': lo['time'],
                        'contact_time': swing_times.iloc[contact_start],
                        'duration_s': duration,
                    })
    return swing_contacts


def analyze_ankle_pitch_at_landing(df, transitions, side):
    """着地の瞬間の前後5ステップのankle_pitch角速度"""
    vel_col = f'dof_vel_{side}_ankle_pitch'
    pos_col = f'dof_pos_{side}_ankle_pitch'
    results = []
    for t in transitions:
        if t['type'] != 'landing':
            continue
        idx = t['idx']
        start = max(0, idx - 5)
        end = min(len(df), idx + 6)

        vel_window = df[vel_col].iloc[start:end].values
        pos_window = df[pos_col].iloc[start:end].values
        time_window = df['timestamp'].iloc[start:end].values

        results.append({
            'landing_time': t['time'],
            'landing_idx': idx,
            'vel_window': vel_window,
            'pos_window': pos_window,
            'time_window': time_window,
            'vel_at_landing': df[vel_col].iloc[idx],
            'pos_at_landing': df[pos_col].iloc[idx],
            'max_abs_vel': np.max(np.abs(vel_window)),
        })
    return results


def print_contact_timeline(df, t_start, t_end):
    """接地パターンの時系列を表として出力"""
    mask = (df['timestamp'] >= t_start) & (df['timestamp'] <= t_end)
    subset = df[mask]

    print(f"\n{'time':>8s} | {'c_L':>4s} | {'c_R':>4s} | {'ankle_L_vel':>11s} | {'ankle_R_vel':>11s} | {'ankle_L_pos':>11s} | {'ankle_R_pos':>11s}")
    print("-" * 80)
    for _, row in subset.iterrows():
        cl = int(row['contact_left'])
        cr = int(row['contact_right'])
        marker = ""
        print(f"{row['timestamp']:8.3f} | {cl:>4d} | {cr:>4d} | {row['dof_vel_L_ankle_pitch']:>11.3f} | {row['dof_vel_R_ankle_pitch']:>11.3f} | {row['dof_pos_L_ankle_pitch']:>11.4f} | {row['dof_pos_R_ankle_pitch']:>11.4f}")


def analyze_version(df, label):
    """1バージョンの完全分析"""
    print(f"\n{'='*80}")
    print(f"  {label}")
    print(f"{'='*80}")

    # Filter to steady state (t > 2s)
    df_ss = df[df['timestamp'] > 2.0].copy()
    print(f"\nデータ範囲: t={df_ss['timestamp'].min():.2f}s ~ {df_ss['timestamp'].max():.2f}s")
    print(f"データ点数: {len(df_ss)}")

    results = {}

    for side, contact_col in [('L', 'contact_left'), ('R', 'contact_right')]:
        print(f"\n--- {side}脚 ---")

        # 1. 接地フラグの遷移パターン
        transitions = detect_transitions(df_ss[contact_col], df_ss['timestamp'])
        landings = [t for t in transitions if t['type'] == 'landing']
        liftoffs = [t for t in transitions if t['type'] == 'liftoff']

        print(f"\n[1] 接地遷移パターン:")
        print(f"  着地回数 (0→1): {len(landings)}")
        print(f"  離地回数 (1→0): {len(liftoffs)}")

        if len(landings) > 1:
            landing_intervals = [landings[i+1]['time'] - landings[i]['time'] for i in range(len(landings)-1)]
            print(f"  着地間隔: mean={np.mean(landing_intervals):.4f}s, std={np.std(landing_intervals):.4f}s, min={np.min(landing_intervals):.4f}s, max={np.max(landing_intervals):.4f}s")

        # Stance/swing durations
        for lo in liftoffs:
            pass  # just count

        # 2. チャタリング検出
        chattering = detect_chattering(transitions, threshold=0.1)
        print(f"\n[2] チャタリング検出 (着地後0.1s以内に再着地):")
        print(f"  チャタリングイベント数: {len(chattering)}")
        if chattering:
            for i, c in enumerate(chattering):
                print(f"    #{i+1}: t={c['first_landing_time']:.3f}s → {c['second_landing_time']:.3f}s (間隔 {c['interval']*1000:.1f}ms)")

        # Also check with tighter threshold
        chattering_tight = detect_chattering(transitions, threshold=0.06)
        print(f"  チャタリング(0.06s以内): {len(chattering_tight)}回")

        # 3. 着地時ankle_pitch角速度
        ankle_data = analyze_ankle_pitch_at_landing(df_ss, transitions, side)
        print(f"\n[3] 着地時ankle_pitch角速度:")
        if ankle_data:
            vel_at_landings = [a['vel_at_landing'] for a in ankle_data]
            max_abs_vels = [a['max_abs_vel'] for a in ankle_data]
            print(f"  着地時角速度: mean={np.mean(vel_at_landings):.3f}, std={np.std(vel_at_landings):.3f}")
            print(f"  着地前後最大|角速度|: mean={np.mean(max_abs_vels):.3f}, max={np.max(max_abs_vels):.3f}")

            # Print detail for each landing
            print(f"  各着地の詳細:")
            for a in ankle_data:
                print(f"    t={a['landing_time']:.3f}s: vel={a['vel_at_landing']:+.3f} rad/s, pos={a['pos_at_landing']:.4f} rad, max|vel|={a['max_abs_vel']:.3f}")

        # 4. 遊脚中の短時間接触
        swing_contacts = detect_swing_contacts(df_ss[contact_col], df_ss['timestamp'], transitions)
        print(f"\n[4] 遊脚中の短時間接触:")
        print(f"  遊脚中接触イベント数: {len(swing_contacts)}")
        if swing_contacts:
            for i, sc in enumerate(swing_contacts):
                print(f"    #{i+1}: swing開始={sc['swing_start_time']:.3f}s, 接触={sc['contact_time']:.3f}s, 持続={sc['duration_s']*1000:.0f}ms")

        results[side] = {
            'landings': len(landings),
            'liftoffs': len(liftoffs),
            'chattering_count': len(chattering),
            'chattering_tight': len(chattering_tight),
            'swing_contacts': len(swing_contacts),
            'ankle_vel_mean': np.mean(vel_at_landings) if ankle_data else 0,
            'ankle_vel_std': np.std(vel_at_landings) if ankle_data else 0,
            'ankle_max_abs_vel': np.max(max_abs_vels) if ankle_data else 0,
        }

    # 5. 時系列表示 (t=3s~5s)
    print(f"\n[5] 接地パターン時系列 (t=3.0s ~ 5.0s):")
    print_contact_timeline(df, 3.0, 5.0)

    return results


def main():
    v1_path = '/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs/droid-walking-narrow-v1/eval_499.csv'
    v2_path = '/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs/droid-walking-narrow-v2/eval_499.csv'

    df_v1 = load_data(v1_path)
    df_v2 = load_data(v2_path)

    print(f"V1: {len(df_v1)} rows, duration={df_v1['timestamp'].max():.2f}s")
    print(f"V2: {len(df_v2)} rows, duration={df_v2['timestamp'].max():.2f}s")

    results_v1 = analyze_version(df_v1, "exp008 V1 (narrow URDF, baseline)")
    results_v2 = analyze_version(df_v2, "exp008 V2 (hip_pos=-0.5)")

    # Summary comparison
    print(f"\n{'='*80}")
    print(f"  V1 vs V2 比較サマリー")
    print(f"{'='*80}")

    print(f"\n{'指標':<30s} | {'V1_L':>8s} | {'V1_R':>8s} | {'V2_L':>8s} | {'V2_R':>8s}")
    print("-" * 80)
    for key in ['landings', 'liftoffs', 'chattering_count', 'chattering_tight', 'swing_contacts']:
        v1l = results_v1['L'][key]
        v1r = results_v1['R'][key]
        v2l = results_v2['L'][key]
        v2r = results_v2['R'][key]
        print(f"{key:<30s} | {v1l:>8} | {v1r:>8} | {v2l:>8} | {v2r:>8}")

    for key in ['ankle_vel_mean', 'ankle_vel_std', 'ankle_max_abs_vel']:
        v1l = results_v1['L'][key]
        v1r = results_v1['R'][key]
        v2l = results_v2['L'][key]
        v2r = results_v2['R'][key]
        print(f"{key:<30s} | {v1l:>8.3f} | {v1r:>8.3f} | {v2l:>8.3f} | {v2r:>8.3f}")


if __name__ == '__main__':
    main()
