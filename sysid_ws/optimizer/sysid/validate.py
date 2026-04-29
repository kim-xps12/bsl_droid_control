# Copyright (c) 2024-2025, Yutaro KIMURA (B-SKY Lab)
# SPDX-License-Identifier: MIT

"""
validate.py — 同定パラメータの検証（初期値 vs 同定値 vs 実機）

重要: 検証シミュレーションはオープンループトルクリプレイではなく
クローズドループ PD シミュレーション。
CSVの target_position をシミュレーション内の PD コントローラに入力し、
シミュレーション自身の位置・速度でフィードバック演算する。

Usage:
    uv run python sysid/validate.py \\
        --csv data/recording_validate.csv \\
        --model ../models/rs02_joint.xml \\
        --params results/identified_params.json \\
        --output-png results/validation.png
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np
import pandas as pd
from numpy.typing import NDArray


# =============================================================================
# データ読み込み
# =============================================================================


def detect_torque_limit(df: pd.DataFrame, fallback: float = 17.0) -> float:
    """CSV の cmd_torque vs cmd_torque_clamped の差から実機側 torque_limit を推定する。

    motor 側で `set_torque_limit(L)` が効いている場合、cmd_torque_clamped は ±L で
    頭打ちされる。abs(cmd_torque) > abs(cmd_torque_clamped) となるサンプルが飽和イベント
    なので、その時の abs(cmd_torque_clamped) の最大値が L そのもの。

    `cmd_torque_clamped` 列がない（旧 schema）または飽和イベントが観測されない場合は
    fallback（driver の scale::TORQUE = 17 Nm）を返す。
    """
    if "cmd_torque_clamped" not in df.columns:
        return fallback
    cmd = df["cmd_torque"].to_numpy(dtype=np.float64)
    clamped = df["cmd_torque_clamped"].to_numpy(dtype=np.float64)
    saturated = np.abs(cmd) - np.abs(clamped) > 1e-6
    if not saturated.any():
        return fallback
    return float(np.abs(clamped[saturated]).max())


def load_recording(csv_path: str) -> tuple[NDArray[np.float64], NDArray[np.float64],
                                             NDArray[np.float64], NDArray[np.float64],
                                             NDArray[np.float64], float]:
    """CSV を読み込み (t, cmd_torque, target_position, q, v, torque_limit) を返す。

    valid==0 行は除外する。
    torque_limit は `cmd_torque_clamped` 列から自動検出する（detect_torque_limit 参照）。
    """
    df = pd.read_csv(csv_path)
    if "valid" in df.columns:
        df = df[df["valid"] == 1].reset_index(drop=True)
    t = df["timestamp"].to_numpy(dtype=np.float64)
    tau = df["cmd_torque"].to_numpy(dtype=np.float64)
    q_target = df["target_position"].to_numpy(dtype=np.float64)
    q = df["position"].to_numpy(dtype=np.float64)
    v = df["velocity"].to_numpy(dtype=np.float64)
    torque_limit = detect_torque_limit(df)
    return t, tau, q_target, q, v, torque_limit


def load_params(json_path: str) -> dict[str, float]:
    with open(json_path) as f:
        return json.load(f)


# =============================================================================
# クローズドループ PD シミュレーション
# =============================================================================


def simulate_pd(
    model: mujoco.MjModel,
    q_targets: NDArray[np.float64],
    q0: float,
    v0: float,
    kp: float,
    kd: float,
    torque_limit: float = 17.0,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """クローズドループ PD シミュレーション。

    各ステップでシミュレーション内の qpos/qvel を使って
    PD 制御量を計算することで、実機と同条件のフィードバックを再現する。

    target_position 列（= 録画時のランダム目標位置）をそのまま入力する。

    torque_limit は実機側 `set_torque_limit` の値に合わせる（CSV から自動検出）。
    sim と real のクランプ値を一致させないと、飽和サンプルで sim が過剰トルクを
    出して RMSE が膨らむ。
    """
    data = mujoco.MjData(model)
    data.qpos[0] = q0
    data.qvel[0] = v0
    mujoco.mj_forward(model, data)

    n = len(q_targets)
    q_sim = np.empty(n, dtype=np.float64)
    v_sim = np.empty(n, dtype=np.float64)

    for i in range(n):
        # PD トルク: シミュレーション内の状態でフィードバック
        tau = kp * (q_targets[i] - data.qpos[0]) + kd * (0.0 - data.qvel[0])
        # 実機側 torque_limit と同じ値でクランプ
        tau = float(np.clip(tau, -torque_limit, torque_limit))
        data.ctrl[0] = tau
        mujoco.mj_step(model, data)
        q_sim[i] = data.qpos[0]
        v_sim[i] = data.qvel[0]

    return q_sim, v_sim


# =============================================================================
# 統計
# =============================================================================


def rmse(a: NDArray[np.float64], b: NDArray[np.float64]) -> float:
    return float(np.sqrt(np.mean((a - b) ** 2)))


# =============================================================================
# プロット
# =============================================================================


def plot_comparison(
    t: NDArray[np.float64],
    q_target: NDArray[np.float64],
    q_real: NDArray[np.float64],
    v_real: NDArray[np.float64],
    q_init: NDArray[np.float64],
    v_init: NDArray[np.float64],
    q_iden: NDArray[np.float64],
    v_iden: NDArray[np.float64],
    params_init: dict[str, float],
    params_iden: dict[str, float],
    kp: float,
    kd: float,
    output_png: str | None = None,
) -> None:
    """before sysid / after sysid を縦に積んだ比較プロットを出力する。

    各行は (位置, 速度) の 2 列。位置パネルにはコマンド (target_position) を
    グレーの階段線、real を青、sim をオレンジ破線で重ねる。左下にその行で
    使ったパラメータを注記する。
    """

    def _annotate_params(ax, params: dict[str, float]) -> None:
        text = (
            f"armature={params['armature']:.4f}  "
            f"frictionloss={params['frictionloss']:.3f}  "
            f"damping={params['damping']:.3f}"
        )
        ax.text(
            0.01, 0.02, text,
            transform=ax.transAxes,
            fontsize=8, color="tab:orange",
            ha="left", va="bottom",
        )

    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    fig.suptitle(f"RS-02 sysid validation  [kp={kp} kd={kd}]", fontsize=12)

    rows = [
        ("before sysid", q_init, v_init, params_init),
        ("after sysid",  q_iden, v_iden, params_iden),
    ]

    for i, (label, q_sim, v_sim, params) in enumerate(rows):
        rmse_q = rmse(q_real, q_sim)
        rmse_v = rmse(v_real, v_sim)

        # ── 位置パネル: cmd / real / sim ─────────────────────────────────────
        ax_q = axes[i, 0]
        ax_q.plot(t, q_target, color="0.6", lw=0.7, drawstyle="steps-post", label="cmd")
        ax_q.plot(t, q_real,   color="tab:blue",   lw=1.5, label="real")
        ax_q.plot(t, q_sim,    color="tab:orange", lw=1.0, ls="--", label="sim")
        ax_q.set_ylabel("Position [rad]")
        ax_q.set_title(f"{label}    pos RMSE = {rmse_q:.4f} rad", loc="left", fontsize=10)
        ax_q.legend(loc="upper right", fontsize=8)
        ax_q.grid(True, alpha=0.3)
        _annotate_params(ax_q, params)

        # ── 速度パネル: real / sim (cmd は PD 設定で 0 一定なので省略) ─────
        ax_v = axes[i, 1]
        ax_v.plot(t, v_real, color="tab:blue",   lw=1.5, label="real")
        ax_v.plot(t, v_sim,  color="tab:orange", lw=1.0, ls="--", label="sim")
        ax_v.set_ylabel("Velocity [rad/s]")
        ax_v.set_title(f"vel RMSE = {rmse_v:.4f} rad/s", loc="left", fontsize=10)
        ax_v.legend(loc="upper right", fontsize=8)
        ax_v.grid(True, alpha=0.3)

    axes[1, 0].set_xlabel("Time [s]")
    axes[1, 1].set_xlabel("Time [s]")

    plt.tight_layout()

    if output_png:
        Path(output_png).parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(output_png, dpi=150)
        print(f"Saved: {output_png}")
    else:
        plt.show()


# =============================================================================
# メイン
# =============================================================================


def validate(
    csv_path: str,
    model_path: str,
    params_path: str,
    kp: float = 8.0,
    kd: float = 0.5,
    output_png: str | None = None,
) -> None:
    print(f"Loading CSV: {csv_path}")
    t, _tau, q_target, q_real, v_real, torque_limit = load_recording(csv_path)
    print(f"  {len(t)} valid samples, duration={float(t[-1] - t[0]):.2f} s")
    print(f"  Detected torque_limit: {torque_limit:.2f} Nm "
          f"(used to match sim clamp to real motor)")

    q0, v0 = float(q_real[0]), float(v_real[0])

    # 初期パラメータでシミュレーション (XML default をそのまま使う)
    print("\nSimulating with initial parameters ...")
    model_init = mujoco.MjModel.from_xml_path(model_path)
    params_init = {
        "armature": float(model_init.dof_armature[0]),
        "frictionloss": float(model_init.dof_frictionloss[0]),
        "damping": float(model_init.dof_damping[0]),
    }
    q_init, v_init = simulate_pd(model_init, q_target, q0, v0, kp, kd, torque_limit)

    # 同定パラメータでシミュレーション
    print(f"Loading identified params: {params_path}")
    params_iden = load_params(params_path)
    print(f"  armature={params_iden['armature']:.6f}  "
          f"frictionloss={params_iden['frictionloss']:.6f}  "
          f"damping={params_iden['damping']:.6f}")

    model_iden = mujoco.MjModel.from_xml_path(model_path)
    model_iden.dof_armature[0] = params_iden["armature"]
    model_iden.dof_frictionloss[0] = params_iden["frictionloss"]
    model_iden.dof_damping[0] = params_iden["damping"]
    print("\nSimulating with identified parameters ...")
    q_iden, v_iden = simulate_pd(model_iden, q_target, q0, v0, kp, kd, torque_limit)

    # RMSE レポート
    rmse_q_init = rmse(q_real, q_init)
    rmse_q_iden = rmse(q_real, q_iden)
    rmse_v_init = rmse(v_real, v_init)
    rmse_v_iden = rmse(v_real, v_iden)

    print("\n=== Validation Results ===")
    print(f"  Position RMSE:  initial={rmse_q_init:.4f} rad  →  identified={rmse_q_iden:.4f} rad  "
          f"(improvement: {100.0 * (1.0 - rmse_q_iden / max(rmse_q_init, 1e-9)):.1f}%)")
    print(f"  Velocity RMSE:  initial={rmse_v_init:.4f} rad/s  →  identified={rmse_v_iden:.4f} rad/s  "
          f"(improvement: {100.0 * (1.0 - rmse_v_iden / max(rmse_v_init, 1e-9)):.1f}%)")

    plot_comparison(
        t, q_target,
        q_real, v_real,
        q_init, v_init,
        q_iden, v_iden,
        params_init, params_iden,
        kp, kd,
        output_png,
    )


# =============================================================================
# CLI
# =============================================================================


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="RS-02 QDD sysid validation")
    p.add_argument("--csv", required=True, help="Path to validation recording CSV (--validate mode)")
    p.add_argument("--model", required=True, help="Path to rs02_joint.xml")
    p.add_argument("--params", required=True, help="Path to identified_params.json")
    p.add_argument("--kp", type=float, default=8.0, help="PD position gain used during recording (default: 8.0)")
    p.add_argument("--kd", type=float, default=0.5, help="PD damping gain used during recording (default: 0.5)")
    p.add_argument("--output-png", default=None, help="Save comparison plot to PNG (default: show)")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    validate(args.csv, args.model, args.params, args.kp, args.kd, args.output_png)


if __name__ == "__main__":
    main()
