"""Analyze direction-specific evaluation CSVs for exp009 V4 vs V3."""
import numpy as np
import os

BASE = "/Users/yutaro.kimura/bsl_droid_control/rl_ws/logs"

DIRECTIONS = {
    "FWD": "0.30_0.00_0.00",
    "BWD": "-0.30_0.00_0.00",
    "LFT": "0.00_0.30_0.00",
    "RGT": "0.00_-0.30_0.00",
}

SEEDS = [1, 2, 3]
STEADY_STATE_ROW = 100  # t >= 2.0s at dt=0.02s


def load_csv(path):
    """Load CSV, return header list and data array."""
    with open(path, "r") as f:
        header = f.readline().strip().split(",")
    data = np.loadtxt(path, skiprows=1, delimiter=",")
    col = {name: i for i, name in enumerate(header)}
    return col, data


def compute_metrics(col, data, direction):
    """Compute all metrics for a single CSV using steady-state data."""
    ss = data[STEADY_STATE_ROW:]  # steady-state rows

    # Velocities
    vel_x = ss[:, col["base_vel_x"]]
    vel_y = ss[:, col["base_vel_y"]]

    if direction in ("FWD", "BWD"):
        cmd_vel = vel_x.mean()
        orth_vel = vel_y.mean()
    elif direction == "LFT":
        cmd_vel = vel_y.mean()
        orth_vel = vel_x.mean()
    else:  # RGT
        # cmd is -0.3, so negate to get positive tracking
        cmd_vel = -vel_y.mean()
        orth_vel = vel_x.mean()

    tracking_rate = abs(cmd_vel) / 0.3

    # Yaw drift: final yaw - yaw at t=2s
    yaw = ss[:, col["yaw_deg"]]
    yaw_drift = yaw[-1] - yaw[0]

    # Roll std
    roll = ss[:, col["roll_deg"]]
    roll_std = roll.std()

    # hip_pitch correlation
    L_hip_pitch = ss[:, col["dof_pos_L_hip_pitch"]]
    R_hip_pitch = ss[:, col["dof_pos_R_hip_pitch"]]
    hip_corr = np.corrcoef(L_hip_pitch, R_hip_pitch)[0, 1]

    # Action RMS: sqrt(mean(action_i^2)) averaged over 10 actions
    action_rms_per_act = []
    for i in range(10):
        act = ss[:, col[f"action_{i}"]]
        action_rms_per_act.append(np.sqrt(np.mean(act**2)))
    action_rms = np.mean(action_rms_per_act)

    # DOF vel RMS: sqrt(mean(dof_vel^2)) averaged over 10 joints
    joint_names = [
        "L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee_pitch", "L_ankle_pitch",
        "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee_pitch", "R_ankle_pitch",
    ]
    dof_vel_rms_per_joint = []
    for jname in joint_names:
        vel = ss[:, col[f"dof_vel_{jname}"]]
        dof_vel_rms_per_joint.append(np.sqrt(np.mean(vel**2)))
    dof_vel_rms = np.mean(dof_vel_rms_per_joint)

    # Falls detection: base_pos_z < 0.15 at any point (full trajectory, not just steady-state)
    z_full = data[:, col["base_pos_z"]]
    fell = z_full.min() < 0.15

    return {
        "cmd_vel": cmd_vel,
        "orth_vel": orth_vel,
        "tracking_rate": tracking_rate,
        "yaw_drift": yaw_drift,
        "roll_std": roll_std,
        "hip_corr": hip_corr,
        "action_rms": action_rms,
        "dof_vel_rms": dof_vel_rms,
        "fell": fell,
    }


def analyze_version(version):
    """Analyze all directions and seeds for a given version."""
    results = {}
    for dname, cmd_str in DIRECTIONS.items():
        seed_metrics = []
        for seed in SEEDS:
            fname = f"eval_1999_cmd_{cmd_str}_s{seed}.csv"
            path = os.path.join(BASE, f"droid-walking-omni-v{version}", fname)
            col, data = load_csv(path)
            metrics = compute_metrics(col, data, dname)
            seed_metrics.append(metrics)

        # Aggregate: mean ± std across seeds
        agg = {}
        for key in seed_metrics[0]:
            vals = [m[key] for m in seed_metrics]
            if key == "fell":
                agg["falls"] = sum(vals)
                agg["falls_total"] = len(vals)
            else:
                agg[f"{key}_mean"] = np.mean(vals)
                agg[f"{key}_std"] = np.std(vals)
        results[dname] = agg
    return results


def print_results(version, results):
    """Print results in the requested format."""
    print(f"=== V{version} Direction-Specific Results ===")
    for dname in ["FWD", "BWD", "LFT", "RGT"]:
        r = results[dname]
        falls_str = f"{r['falls']}/{r['falls_total']}"
        print(
            f"{dname}: "
            f"cmd_vel={r['cmd_vel_mean']:+.3f}\u00b1{r['cmd_vel_std']:.3f}, "
            f"orth_vel={r['orth_vel_mean']:+.3f}\u00b1{r['orth_vel_std']:.3f}, "
            f"tracking={r['tracking_rate_mean']*100:.1f}%\u00b1{r['tracking_rate_std']*100:.1f}%, "
            f"yaw={r['yaw_drift_mean']:+.2f}\u00b1{r['yaw_drift_std']:.2f}\u00b0, "
            f"roll_std={r['roll_std_mean']:.2f}\u00b1{r['roll_std_std']:.2f}\u00b0, "
            f"hip_corr={r['hip_corr_mean']:.3f}\u00b1{r['hip_corr_std']:.3f}, "
            f"act_rms={r['action_rms_mean']:.3f}\u00b1{r['action_rms_std']:.3f}, "
            f"dof_vel_rms={r['dof_vel_rms_mean']:.3f}\u00b1{r['dof_vel_rms_std']:.3f}, "
            f"falls={falls_str}"
        )
    print()


def print_balance(version, results):
    """Print direction balance metrics."""
    tracking_rates = [results[d]["tracking_rate_mean"] for d in ["FWD", "BWD", "LFT", "RGT"]]
    mean_tr = np.mean(tracking_rates)
    std_tr = np.std(tracking_rates)
    fwd_bwd_diff = results["FWD"]["tracking_rate_mean"] - results["BWD"]["tracking_rate_mean"]
    lft_rgt_diff = results["LFT"]["tracking_rate_mean"] - results["RGT"]["tracking_rate_mean"]
    print(f"  V{version}: mean_tracking={mean_tr*100:.1f}%, std={std_tr*100:.1f}%, "
          f"FWD-BWD={fwd_bwd_diff*100:+.1f}%, LFT-RGT={lft_rgt_diff*100:+.1f}%")
    return mean_tr, std_tr


def main():
    v4 = analyze_version(4)
    v3 = analyze_version(3)

    print_results(4, v4)
    print_results(3, v3)

    print("=== Direction Balance ===")
    print_balance(4, v4)
    print_balance(3, v3)
    print()

    print("=== V4 vs V3 Changes (per direction) ===")
    for dname in ["FWD", "BWD", "LFT", "RGT"]:
        t4 = v4[dname]["tracking_rate_mean"] * 100
        t3 = v3[dname]["tracking_rate_mean"] * 100
        diff = t4 - t3
        pct = (t4 - t3) / t3 * 100 if t3 != 0 else float("inf")
        print(f"  {dname} tracking: {t3:.1f}% -> {t4:.1f}% ({diff:+.1f}pp, {pct:+.1f}%)")

    # Additional: cmd_vel, orth_vel, yaw, roll_std, hip_corr changes
    print()
    print("=== V4 vs V3 Detailed Changes ===")
    for dname in ["FWD", "BWD", "LFT", "RGT"]:
        r4 = v4[dname]
        r3 = v3[dname]
        print(f"  {dname}:")
        print(f"    cmd_vel:     {r3['cmd_vel_mean']:+.3f} -> {r4['cmd_vel_mean']:+.3f} ({r4['cmd_vel_mean']-r3['cmd_vel_mean']:+.3f})")
        print(f"    orth_vel:    {r3['orth_vel_mean']:+.3f} -> {r4['orth_vel_mean']:+.3f} ({r4['orth_vel_mean']-r3['orth_vel_mean']:+.3f})")
        yaw3, yaw4 = r3["yaw_drift_mean"], r4["yaw_drift_mean"]
        print(f"    yaw_drift:   {yaw3:+.2f} -> {yaw4:+.2f} ({yaw4-yaw3:+.2f})")
        roll3, roll4 = r3["roll_std_mean"], r4["roll_std_mean"]
        pct_roll = (roll4 - roll3) / roll3 * 100 if roll3 != 0 else 0
        print(f"    roll_std:    {roll3:.2f} -> {roll4:.2f} ({pct_roll:+.1f}%)")
        hip3, hip4 = r3["hip_corr_mean"], r4["hip_corr_mean"]
        print(f"    hip_corr:    {hip3:.3f} -> {hip4:.3f} ({hip4-hip3:+.3f})")
        act3, act4 = r3["action_rms_mean"], r4["action_rms_mean"]
        pct_act = (act4 - act3) / act3 * 100 if act3 != 0 else 0
        print(f"    action_rms:  {act3:.3f} -> {act4:.3f} ({pct_act:+.1f}%)")
        dv3, dv4 = r3["dof_vel_rms_mean"], r4["dof_vel_rms_mean"]
        pct_dv = (dv4 - dv3) / dv3 * 100 if dv3 != 0 else 0
        print(f"    dof_vel_rms: {dv3:.3f} -> {dv4:.3f} ({pct_dv:+.1f}%)")

    # Summary table for markdown
    print()
    print("=== Markdown Summary Table ===")
    print("| Direction | Metric | V3 | V4 | Change |")
    print("|-----------|--------|-----|-----|--------|")
    for dname in ["FWD", "BWD", "LFT", "RGT"]:
        r4 = v4[dname]
        r3 = v3[dname]
        t4 = r4["tracking_rate_mean"]*100
        t3 = r3["tracking_rate_mean"]*100
        print(f"| {dname} | Tracking | {t3:.1f}% | {t4:.1f}% | {t4-t3:+.1f}pp |")
        print(f"| | cmd_vel | {r3['cmd_vel_mean']:+.3f} | {r4['cmd_vel_mean']:+.3f} | {r4['cmd_vel_mean']-r3['cmd_vel_mean']:+.3f} |")
        print(f"| | orth_vel | {r3['orth_vel_mean']:+.3f} | {r4['orth_vel_mean']:+.3f} | {r4['orth_vel_mean']-r3['orth_vel_mean']:+.3f} |")
        print(f"| | yaw_drift | {r3['yaw_drift_mean']:+.1f}\u00b0 | {r4['yaw_drift_mean']:+.1f}\u00b0 | {r4['yaw_drift_mean']-r3['yaw_drift_mean']:+.1f}\u00b0 |")
        print(f"| | roll_std | {r3['roll_std_mean']:.2f}\u00b0 | {r4['roll_std_mean']:.2f}\u00b0 | {(r4['roll_std_mean']-r3['roll_std_mean'])/r3['roll_std_mean']*100:+.1f}% |")
        print(f"| | hip_corr | {r3['hip_corr_mean']:.3f} | {r4['hip_corr_mean']:.3f} | {r4['hip_corr_mean']-r3['hip_corr_mean']:+.3f} |")
        print(f"| | falls | {r3['falls']}/3 | {r4['falls']}/3 | |")


if __name__ == "__main__":
    main()
