# Copyright (c) 2024-2025, Yutaro KIMURA (B-SKY Lab)
# SPDX-License-Identifier: MIT

"""
optimize.py — RS-02 QDD システム同定（シミュレーションリプレイ法）

アルゴリズム:
  1. CSVからマルチサイン励振の記録を読み込む（valid==1 の行のみ）
  2. MuJoCo モデルを 1 度だけパースし、最適化中はパラメータフィールドを直接書き換える
  3. residual(θ): 同じトルクコマンドをシミュレーションで再生 → 重み付き残差ベクトルを返す
  4. scipy.optimize.least_squares (Trust Region Reflective) で θ を最適化
  5. 結果を JSON に保存

mujoco-sysid ライブラリは使わない。
理由: MJX の dof_frictionloss 勾配バグを回避するため CPU MuJoCo + 数値ヤコビアンを採用。

最小二乗構造を直接 least_squares に渡すことで:
  - ガウス・ニュートン的な曲率近似で L-BFGS-B より収束が速い
  - x_scale="jac" でパラメータのスケール差（armature ~10⁻², frictionloss ~10⁻¹）を自動正規化

Usage:
    uv run python sysid/optimize.py \\
        --csv data/recording_sysid.csv \\
        --model ../models/rs02_joint.xml \\
        --output results/identified_params.json
"""

from __future__ import annotations

import argparse
import json
from collections.abc import Callable
from pathlib import Path

import mujoco
import numpy as np
import pandas as pd
from numpy.typing import NDArray
from scipy.optimize import least_squares


# =============================================================================
# データ読み込み
# =============================================================================


def load_recording(csv_path: str) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """CSV を読み込み (timestamps, cmd_torques, positions, velocities) を返す。

    valid==0 の行（タイムアウト応答）は除外する。
    valid 列が存在しない古い記録はそのまま全行使用する。
    """
    df = pd.read_csv(csv_path)
    if "valid" in df.columns:
        df = df[df["valid"] == 1].reset_index(drop=True)
    t = df["timestamp"].to_numpy(dtype=np.float64)
    tau = df["cmd_torque"].to_numpy(dtype=np.float64)
    q = df["position"].to_numpy(dtype=np.float64)
    v = df["velocity"].to_numpy(dtype=np.float64)
    return t, tau, q, v


# =============================================================================
# シミュレーションリプレイ
# =============================================================================


def simulate_replay(
    model: mujoco.MjModel,
    tau_cmds: NDArray[np.float64],
    q0: float,
    v0: float,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """トルクコマンド列をオープンループでシミュレーション再生し (q_sim, v_sim) を返す。

    モデルのパラメータは呼び出し前に設定済みであること。
    毎呼び出しで MjData を新規作成し状態のリークを防ぐ。
    """
    data = mujoco.MjData(model)
    data.qpos[0] = q0
    data.qvel[0] = v0
    mujoco.mj_forward(model, data)

    n = len(tau_cmds)
    q_sim = np.empty(n, dtype=np.float64)
    v_sim = np.empty(n, dtype=np.float64)

    for i in range(n):
        data.ctrl[0] = tau_cmds[i]
        mujoco.mj_step(model, data)
        q_sim[i] = data.qpos[0]
        v_sim[i] = data.qvel[0]

    return q_sim, v_sim


# =============================================================================
# 残差関数ファクトリ
# =============================================================================


def make_residual_fn(
    model: mujoco.MjModel,
    tau_cmds: NDArray[np.float64],
    q_real: NDArray[np.float64],
    v_real: NDArray[np.float64],
    q0: float,
    v0: float,
    pos_weight: float = 1.0,
    vel_weight: float = 0.1,
) -> Callable[[NDArray[np.float64]], NDArray[np.float64]]:
    """least_squares に渡す重み付き残差関数を生成する。

    θ = [armature, frictionloss, damping]

    残差ベクトル長 2N。0.5·sum(residual²) = 0.5·(pos_weight·MSE_q + vel_weight·MSE_v)
    と一致するように 1/√N の正規化を残差に組み込む。

    vel_weight=0.1 の根拠: CAN の速度フィードバックは 12bit / ±44 rad/s
    → LSB ≈ 0.021 rad/s で位置より分解能が低い。
    """
    n = len(tau_cmds)
    sqrt_pw = float(np.sqrt(pos_weight / n))
    sqrt_vw = float(np.sqrt(vel_weight / n))
    call_count = [0]

    def residual(theta: NDArray[np.float64]) -> NDArray[np.float64]:
        armature, frictionloss, damping = theta

        model.dof_armature[0] = armature
        model.dof_frictionloss[0] = frictionloss
        model.dof_damping[0] = damping

        q_sim, v_sim = simulate_replay(model, tau_cmds, q0, v0)

        r = np.empty(2 * n, dtype=np.float64)
        r[:n] = sqrt_pw * (q_sim - q_real)
        r[n:] = sqrt_vw * (v_sim - v_real)

        call_count[0] += 1
        if call_count[0] % 5 == 1:
            cost = 0.5 * float(r @ r)
            print(
                f"  iter={call_count[0]:4d}  "
                f"armature={armature:.6f}  "
                f"frictionloss={frictionloss:.6f}  "
                f"damping={damping:.6f}  "
                f"cost={cost:.8f}"
            )
        return r

    return residual


# =============================================================================
# 最適化エントリポイント
# =============================================================================


def optimize(
    csv_path: str,
    model_path: str,
    output_path: str | None = None,
    pos_weight: float = 1.0,
    vel_weight: float = 0.1,
) -> dict[str, float]:
    """フル最適化パイプライン。同定パラメータ辞書を返す。"""
    print(f"Loading CSV: {csv_path}")
    t, tau, q, v = load_recording(csv_path)
    n_valid = len(t)
    duration = float(t[-1] - t[0]) if n_valid > 1 else 0.0
    print(f"  {n_valid} valid samples, duration={duration:.2f} s")

    q0, v0 = float(q[0]), float(v[0])

    # 初期パラメータ（モデルのデフォルト値と一致）
    x0 = np.array([0.01, 0.10, 0.05])

    # 物理的に意味のある下限・上限
    #   armature:     0.001 – 0.5  kg·m²
    #   frictionloss: 0.001 – 2.0  Nm
    #   damping:      0.001 – 1.0  Nm/(rad/s)
    lb = np.array([0.001, 0.001, 0.001])
    ub = np.array([0.500, 2.000, 1.000])

    # XML は最適化開始前に 1 度だけパースし、以降はフィールドを直接書き換える
    model = mujoco.MjModel.from_xml_path(model_path)
    residual_fn = make_residual_fn(model, tau, q, v, q0, v0, pos_weight, vel_weight)

    print(f"\nOptimizing (least_squares / TRF, x0={x0}) ...")
    result = least_squares(
        residual_fn,
        x0,
        method="trf",
        bounds=(lb, ub),
        x_scale="jac",
        max_nfev=600,
        ftol=1e-12,
        xtol=1e-12,
        gtol=1e-9,
    )

    params = {
        "armature": float(result.x[0]),
        "frictionloss": float(result.x[1]),
        "damping": float(result.x[2]),
    }

    print("\n=== Identified Parameters ===")
    for k, val in params.items():
        print(f"  {k}: {val:.6f}")
    # result.cost = 0.5 * sum(residual**2); residual に正規化を組み込んでいるので
    # 2*cost = pos_weight*MSE_q + vel_weight*MSE_v
    print(f"  Final cost:  {result.cost:.10f}")
    print(f"  Converged:   {result.success}")
    print(f"  Iterations:  {result.nfev}")
    if not result.success:
        print(f"  Message:     {result.message}")

    if output_path:
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, "w") as f:
            json.dump(params, f, indent=2)
        print(f"\nSaved: {output_path}")

    return params


# =============================================================================
# CLI
# =============================================================================


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="RS-02 QDD system identification optimizer")
    p.add_argument("--csv", required=True, help="Path to sysid recording CSV")
    p.add_argument("--model", required=True, help="Path to rs02_joint.xml")
    p.add_argument("--output", default=None, help="JSON output path for identified parameters")
    p.add_argument("--pos-weight", type=float, default=1.0, help="Position MSE weight (default: 1.0)")
    p.add_argument("--vel-weight", type=float, default=0.1, help="Velocity MSE weight (default: 0.1)")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    optimize(args.csv, args.model, args.output, args.pos_weight, args.vel_weight)


if __name__ == "__main__":
    main()
