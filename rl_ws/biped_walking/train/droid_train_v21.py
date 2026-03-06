#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト V21

============================================================
【V21: V20ベース + 胴体高さ報酬（沈み込み防止）】
============================================================

【V20の問題分析】
V20では以下の問題が観察された：
- 胴体高さが0.183mまで沈み込み（初期0.35mの52%）
- 小刻み歩行（ストライド・足上げ幅が小さい）
- feet_air_time報酬が機能していない（接地検出の問題）

【V21の設計方針】
接地検出に依存せず、胴体高さ報酬で沈み込みを改善：

1. 終了条件: V20と同じ
2. 報酬変更:
   - feet_air_time: 削除（接地検出が機能していないため）
   - base_height: -2.0 追加（target=0.25m）
3. 報酬バランス: 主報酬優位を維持

【報酬バランス検証】
主報酬: tracking_lin_vel(2.0) + tracking_ang_vel(0.5) + alive(1.0) = 3.5
ペナルティ: orientation(-1.0) + base_height(-2.0) + action_rate(-0.02) ≈ -3.0
比率: 3.5 / 3.0 ≈ 1.2 （ペナルティが強めだが許容範囲）

【期待効果】
- Base height: > 0.22m（V20: 0.183m）
- X移動距離: ≥ 2.5m（V20: 2.72m）

【次のステップ】
V21がダメなら足先位置空間での探索へシフト（IK実装）
============================================================
"""

from __future__ import annotations

import argparse
import math
import os
import pickle
import shutil
from pathlib import Path
from typing import Any

import genesis as gs

# rsl-rl-lib==2.2.4のインポート
from rsl_rl.runners.on_policy_runner import OnPolicyRunner

from biped_walking.envs.droid_env import DroidEnv


def get_train_cfg(exp_name: str, max_iterations: int) -> dict[str, Any]:
    """訓練設定を取得"""
    train_cfg_dict = {
        "algorithm": {
            "class_name": "PPO",
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.01,  # 探索促進のため維持
            "gamma": 0.99,
            "lam": 0.95,
            "learning_rate": 0.001,
            "max_grad_norm": 1.0,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "schedule": "adaptive",
            "use_clipped_value_loss": True,
            "value_loss_coef": 1.0,
        },
        "init_member_classes": {},
        "policy": {
            "activation": "elu",
            "actor_hidden_dims": [512, 256, 128],
            "critic_hidden_dims": [512, 256, 128],
            "init_noise_std": 1.0,
            "class_name": "ActorCritic",
        },
        "runner": {
            "checkpoint": -1,
            "experiment_name": exp_name,
            "load_run": -1,
            "log_interval": 1,
            "max_iterations": max_iterations,
            "record_interval": -1,
            "resume": False,
            "resume_path": None,
            "run_name": "",
        },
        "runner_class_name": "OnPolicyRunner",
        "num_steps_per_env": 24,
        "save_interval": 100,
        "empirical_normalization": None,
        "seed": 1,
    }

    return train_cfg_dict


def get_cfgs() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any]]:
    """環境設定を取得"""
    script_dir = Path(__file__).parent
    rl_ws_dir = script_dir.parent.parent
    urdf_path = rl_ws_dir / "assets" / "bsl_droid_simplified.urdf"

    # V9と同じ初期姿勢（実績あり）
    hip_pitch_rad = 60 * math.pi / 180  # 1.047 rad
    knee_pitch_rad = -100 * math.pi / 180  # -1.745 rad
    ankle_pitch_rad = 45 * math.pi / 180  # 0.785 rad

    env_cfg = {
        "num_actions": 10,
        "urdf_path": str(urdf_path),
        "joint_names": [
            "left_hip_yaw_joint",
            "left_hip_roll_joint",
            "left_hip_pitch_joint",
            "left_knee_pitch_joint",
            "left_ankle_pitch_joint",
            "right_hip_yaw_joint",
            "right_hip_roll_joint",
            "right_hip_pitch_joint",
            "right_knee_pitch_joint",
            "right_ankle_pitch_joint",
        ],
        "default_joint_angles": {
            "left_hip_yaw_joint": 0.0,
            "left_hip_roll_joint": 0.0,
            "left_hip_pitch_joint": hip_pitch_rad,
            "left_knee_pitch_joint": knee_pitch_rad,
            "left_ankle_pitch_joint": ankle_pitch_rad,
            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_pitch_joint": hip_pitch_rad,
            "right_knee_pitch_joint": knee_pitch_rad,
            "right_ankle_pitch_joint": ankle_pitch_rad,
        },
        "feet_names": ["left_foot_link", "right_foot_link"],
        "kp": 35.0,
        "kd": 2.0,
        # ============================================================
        # 【V20】終了条件 - V18と同じ（V19の厳格化は行わない）
        # ============================================================
        # V19では20°/0.15mに厳格化して失敗 → V18レベルに維持
        "termination_if_roll_greater_than": 25,  # V18と同じ
        "termination_if_pitch_greater_than": 25,  # V18と同じ（V19の20°は厳しすぎ）
        "termination_if_height_lower_than": 0.12,  # V18と同じ（V19の0.15mは厳しすぎ）
        "termination_if_knee_positive": True,  # V18と同じ
        "base_init_pos": [0.0, 0.0, 0.35],
        "base_init_quat": [1.0, 0.0, 0.0, 0.0],
        "episode_length_s": 20.0,
        "resampling_time_s": 4.0,
        "action_scale": 0.4,
        "simulate_action_latency": True,
        "clip_actions": 10.0,
    }

    obs_cfg = {
        "num_obs": 39,
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }

    # ============================================================
    # V21: V20ベース + 胴体高さ報酬（沈み込み防止）
    # ============================================================
    # V20で胴体が0.183mまで沈み込む問題が発生。
    # 接地検出に依存するfeet_air_timeは削除し、
    # base_height報酬で沈み込みを防止する。
    #
    # 【報酬バランス検証】
    # 主報酬: tracking_lin_vel(2.0) + tracking_ang_vel(0.5) + alive(1.0) = 3.5
    # ペナルティ: orientation(-1.0) + base_height(-2.0) + action_rate(-0.02) ≈ -3.0
    # 比率: 3.5 / 3.0 ≈ 1.2（ペナルティ強めだが許容範囲）
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.25,  # V20の0.22 → 0.25（目標を明示）
        "gait_frequency": 1.5,
        "reward_scales": {
            # ============================================================
            # 【タスク報酬】（3項目）: V20と同じ
            # ============================================================
            "tracking_lin_vel": 2.0,  # 前進速度追従（最重要）
            "tracking_ang_vel": 0.5,  # 旋回速度追従
            "alive": 1.0,  # 生存報酬（倒れない動機）
            # ============================================================
            # 【エネルギー効率】（2項目）: V20と同じ
            # ============================================================
            "torques": -1e-4,  # トルク最小化 → 省エネ
            "dof_acc": -1e-6,  # 加速度最小化 → 滑らか
            # ============================================================
            # 【安定性】（2項目）: V20 + base_height追加
            # ============================================================
            "orientation": -1.0,  # 傾きペナルティ（緩め）
            "base_height": -2.0,  # 【V21追加】沈み込み防止
            # ============================================================
            # 【振動抑制】（1項目）: V20と同じ
            # ============================================================
            "action_rate": -0.02,  # アクション変化率ペナルティ
            # feet_air_time: 削除（接地検出が機能していないため）
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.3, 0.3],  # 目標速度 0.3 m/s
        "lin_vel_y_range": [0, 0],
        "ang_vel_range": [0, 0],
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v21")
    parser.add_argument("-B", "--num_envs", type=int, default=4096)
    parser.add_argument("--max_iterations", type=int, default=500)
    args = parser.parse_args()

    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg = get_cfgs()
    train_cfg = get_train_cfg(args.exp_name, args.max_iterations)

    if os.path.exists(log_dir):
        shutil.rmtree(log_dir)
    os.makedirs(log_dir, exist_ok=True)

    with open(f"{log_dir}/cfgs.pkl", "wb") as f:
        pickle.dump(
            [env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg],
            f,
        )

    gs.init(backend=gs.gpu, precision="32", logging_level="warning", seed=train_cfg["seed"], performance_mode=True)

    env = DroidEnv(
        num_envs=args.num_envs, env_cfg=env_cfg, obs_cfg=obs_cfg, reward_cfg=reward_cfg, command_cfg=command_cfg
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()
