# Copyright (c) 2024-2025, Yutaro KIMURA (B-SKY Lab)
# SPDX-License-Identifier: MIT

"""
plot_recording.py — sysid_recorder の CSV を時系列で可視化する

励振品質の確認に使う。同定の前段階で:
  - cmd_torque が想定振幅か
  - est_torque とのギャップ（電流ループ帯域の影響）
  - position が drift guard ±2π に迫っていないか
  - velocity が RS-02 限界 ±44 rad/s に近づいていないか

sysid モード／validate モード両方に対応。validate モードでは target_position
が階段線で重ね描きされる。

Usage:
    uv run python sysid/plot_recording.py \\
        --csv recorded/sysid_prod.csv \\
        --output-png /tmp/recording.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# RS-02 物理リミットと recorder 側の安全網
TORQUE_LIMIT_NM = 12.0
DRIFT_LIMIT_RAD = 2.0 * np.pi
VEL_GUARD_RAD_S = 30.0
VEL_MAX_RAD_S = 44.0


def plot_recording(csv_path: str, output_png: str | None) -> None:
    df = pd.read_csv(csv_path)
    t = df["timestamp"].to_numpy()
    valid_rate = float(df["valid"].mean()) if "valid" in df.columns else 1.0
    initial_pos = float(df["position"].iloc[0])
    has_target = "target_position" in df.columns and df["target_position"].abs().max() > 1e-6

    print(f"Loaded: {csv_path}")
    print(f"  samples={len(df)}  duration={t[-1] - t[0]:.2f}s  valid_rate={valid_rate:.4f}")
    print(f"  cmd_torque:         [{df['cmd_torque'].min():.3f}, {df['cmd_torque'].max():.3f}] Nm")
    if "cmd_torque_clamped" in df.columns:
        c = df["cmd_torque_clamped"]
        clamp_hits = int((c.abs() >= TORQUE_LIMIT_NM - 1e-6).sum())
        print(f"  cmd_torque_clamped: [{c.min():.3f}, {c.max():.3f}] Nm  "
              f"(saturated samples: {clamp_hits} / {len(df)} = {100.0*clamp_hits/len(df):.2f}%)")
    if "estimated_torque" in df.columns:
        print(f"  estimated_torque:   [{df['estimated_torque'].min():.3f}, {df['estimated_torque'].max():.3f}] Nm")
    print(f"  position:           [{df['position'].min():.3f}, {df['position'].max():.3f}] rad  "
          f"(initial={initial_pos:.3f})")
    print(f"  velocity:           [{df['velocity'].min():.3f}, {df['velocity'].max():.3f}] rad/s")

    fig, axes = plt.subplots(4, 1, figsize=(14, 9), sharex=True)
    title_mode = "validate (PD)" if has_target else "sysid (multi-sine)"
    fig.suptitle(f"recording  [{title_mode}]  {Path(csv_path).name}", fontsize=11)

    # cmd_torque (raw) と cmd_torque_clamped (実際にmotorに届く値) を重ね描き
    axes[0].plot(t, df["cmd_torque"], color="tab:blue", lw=0.5, label="cmd (raw)")
    if "cmd_torque_clamped" in df.columns:
        axes[0].plot(t, df["cmd_torque_clamped"], color="tab:red", lw=0.5,
                     alpha=0.7, label="cmd (clamped)")
    axes[0].axhline(0, color="k", lw=0.3)
    axes[0].axhline(TORQUE_LIMIT_NM, color="r", lw=0.5, ls="--", label="torque_limit")
    axes[0].axhline(-TORQUE_LIMIT_NM, color="r", lw=0.5, ls="--")
    axes[0].set_ylabel("cmd_torque [Nm]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right", fontsize=8)

    # est_torque
    if "estimated_torque" in df.columns:
        axes[1].plot(t, df["estimated_torque"], color="tab:green", lw=0.5)
        axes[1].axhline(0, color="k", lw=0.3)
        axes[1].axhline(TORQUE_LIMIT_NM, color="r", lw=0.5, ls="--")
        axes[1].axhline(-TORQUE_LIMIT_NM, color="r", lw=0.5, ls="--")
        axes[1].set_ylabel("est_torque [Nm]")
        axes[1].grid(True, alpha=0.3)
    else:
        axes[1].set_visible(False)

    # position
    axes[2].plot(t, df["position"], color="tab:purple", lw=0.7, label="real")
    if has_target:
        axes[2].plot(t, df["target_position"], color="0.5", lw=0.7,
                     drawstyle="steps-post", label="target")
    axes[2].axhline(initial_pos, color="gray", lw=0.5, ls=":", label="initial_pos")
    axes[2].axhline(initial_pos + DRIFT_LIMIT_RAD, color="r", lw=0.5, ls="--", label="drift_guard")
    axes[2].axhline(initial_pos - DRIFT_LIMIT_RAD, color="r", lw=0.5, ls="--")
    axes[2].set_ylabel("position [rad]")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="upper right", fontsize=8)

    # velocity
    axes[3].plot(t, df["velocity"], color="tab:orange", lw=0.5)
    axes[3].axhline(0, color="k", lw=0.3)
    axes[3].axhline(VEL_GUARD_RAD_S, color="r", lw=0.5, ls="--", label="vel_guard")
    axes[3].axhline(-VEL_GUARD_RAD_S, color="r", lw=0.5, ls="--")
    axes[3].axhline(VEL_MAX_RAD_S, color="m", lw=0.5, ls=":", label="RS-02 max")
    axes[3].axhline(-VEL_MAX_RAD_S, color="m", lw=0.5, ls=":")
    axes[3].set_ylabel("velocity [rad/s]")
    axes[3].set_xlabel("time [s]")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(loc="upper right", fontsize=8)

    plt.tight_layout()

    if output_png:
        Path(output_png).parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(output_png, dpi=150)
        print(f"Saved: {output_png}")
    else:
        plt.show()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Plot a sysid_recorder CSV time-series")
    p.add_argument("--csv", required=True, help="Path to recording CSV")
    p.add_argument("--output-png", default=None, help="Save plot to PNG (default: show)")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    plot_recording(args.csv, args.output_png)


if __name__ == "__main__":
    main()
