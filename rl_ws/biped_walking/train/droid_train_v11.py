"""BSL-Droid Simplified 二脚ロボット Genesis訓練スクリプト v11

============================================================
V11: ハイブリッドアプローチ（V9ペナルティ + V10参照軌道 + 接地強制）
============================================================

コンセプト: V9とV10の長所を組み合わせ、接地問題を根本解決

背景（V9/V10評価結果）:
- V9: 交互歩行優秀（相関-0.516）、直進性良好（Y=0.01m）
- V10: 重心安定（std=0.009m）、脚傾斜改善
- 両方: 滑走歩行100%（接地判定の問題）

V11のアプローチ:
1. 接地判定閾値を修正（0.025→0.04m）
2. V9のペナルティ復活（hip_pitch_sync_penalty強化）
3. V10のPhase-based報酬を維持（やや弱める）
4. 斜行対策（symmetry_hip_roll追加）
5. 接地促進（ground_contact_bonus追加、no_fly強化）

サーベイからの採用項目:
- Legged Gym (ETH): feet_air_time報酬、接地判定修正
- Cassie Biped: no_fly強化（-0.5→-3.0）
- Low-Frequency Control (Oxford): gait_frequency低下（1.2→1.0Hz）

新規報酬関数:
- symmetry_hip_roll: 斜行対策
- ground_contact_bonus: 接地促進

Usage:
    cd rl_ws
    uv run python biped_walking/train/droid_train_v11.py --max_iterations 500
"""

from __future__ import annotations

import argparse
import os
import pickle
import shutil
from importlib import metadata
from pathlib import Path
from typing import Any


try:
    try:
        if metadata.version("rsl-rl"):
            raise ImportError
    except metadata.PackageNotFoundError:
        if metadata.version("rsl-rl-lib") != "2.2.4":
            raise ImportError from None
except (metadata.PackageNotFoundError, ImportError) as e:
    raise ImportError("Please uninstall 'rsl_rl' and install 'rsl-rl-lib==2.2.4'.") from e
import sys

import genesis as gs
from rsl_rl.runners import OnPolicyRunner


# envsパッケージへのパスを追加
rl_ws_dir = Path(__file__).parent.parent
sys.path.insert(0, str(rl_ws_dir))
from biped_walking.envs.droid_env import DroidEnv


def get_train_cfg(exp_name: str, max_iterations: int) -> dict[str, Any]:
    """訓練設定を取得"""
    train_cfg_dict = {
        "algorithm": {
            "class_name": "PPO",
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.01,
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

    # V8/V9/V10と同じ初期姿勢を継続
    import math

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
        "termination_if_roll_greater_than": 30,
        "termination_if_pitch_greater_than": 30,
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
    # V11: ハイブリッドアプローチ報酬設定
    # ============================================================
    # V9（ペナルティベース）+ V10（Phase-based）+ 接地強制
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.22,
        "feet_air_time_target": 0.25,
        "gait_frequency": 1.0,  # V10: 1.2 → V11: 1.0 Hz（低周波でゆったり）
        # Phase-based trajectory parameters
        "ref_hip_pitch_amplitude": 0.30,  # V10: 0.25 → V11: 0.30 rad（大きめストライド）
        "ref_hip_pitch_offset": 0.0,
        "phase_tracking_sigma": 0.1,
        # 接地判定閾値（V11で修正）
        "contact_threshold": 0.04,  # V10: 0.025 → V11: 0.04m
        "reward_scales": {
            # ========== Phase-based報酬（V10から継承、調整） ==========
            "phase_hip_pitch_tracking": 2.0,  # V10: 3.0 → V11: 2.0
            "phase_contact_sync": 1.5,
            "phase_velocity_sync": 0.5,  # V10: 1.0 → V11: 0.5
            "smooth_action": 1.5,  # V10: 1.0 → V11: 1.5
            "periodic_foot_lift": 1.5,  # V10: 2.0 → V11: 1.5
            "natural_rhythm": 0.3,  # V10: 0.5 → V11: 0.3
            # ========== V11新規報酬 ==========
            "symmetry_hip_roll": 1.0,  # 斜行対策（新規）
            "ground_contact_bonus": 1.5,  # 接地促進（新規）
            # ========== 主タスク報酬 ==========
            "tracking_lin_vel": 1.5,
            "tracking_ang_vel": 0.5,
            "alive": 0.1,
            "forward_progress": 0.3,
            # ========== 交互歩行報酬（V9から復活強化） ==========
            "alternating_gait": 1.0,  # V10: 0.5 → V11: 1.0
            "foot_swing": 0.3,
            "feet_air_time": 2.0,  # V10: 1.0 → V11: 2.0（Legged Gym参考）
            "single_stance": 0.5,  # V10: 0.3 → V11: 0.5
            "no_fly": -3.0,  # V10: -0.5 → V11: -3.0（Cassie参考、強化）
            # ========== hip_pitch動作報酬 ==========
            "hip_pitch_alternation": 1.5,  # V10: 1.0 → V11: 1.5
            "hip_pitch_velocity": 0.3,
            "contact_alternation": 1.0,  # V10: 0.5 → V11: 1.0
            # ========== 同期ペナルティ（V9から復活強化） ==========
            "hip_pitch_sync_penalty": -2.0,  # V10: -1.0 → V11: -2.0
            # ========== 姿勢・安定性ペナルティ ==========
            "orientation": -2.5,
            "base_height": -10.0,
            "base_height_high": -5.0,
            "base_height_low": -10.0,
            "lin_vel_z": -2.0,
            "ang_vel_xy": -0.05,
            "pitch_penalty": -3.0,
            "roll_penalty": -3.0,
            # ========== Yawドリフト対策（強化） ==========
            "yaw_rate": -1.5,  # V10: -1.0 → V11: -1.5
            "symmetry": -0.8,  # V10: -0.5 → V11: -0.8
            # ========== 膝角度制約 ==========
            "dof_pos_limits": -5.0,
            "knee_negative": -3.0,
            "knee_max_angle": -3.0,
            # ========== 後退ペナルティ ==========
            "backward_velocity": -2.0,
            # ========== 振動抑制ペナルティ（強化） ==========
            "action_rate": -0.05,  # V10: -0.02 → V11: -0.05
            "dof_vel": -1e-3,
            "dof_acc": -5e-7,
            "torques": -5e-5,
            "similar_to_default": -0.01,
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
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v11")
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
