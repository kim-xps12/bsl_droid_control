"""
BSL-Droid二脚ロボット Genesis訓練スクリプト v17

============================================================
低重心歩行の実現 - 3つの方針を統合
============================================================

V17の設計方針:

1. 初期姿勢を低く設定（補助）
   - hip_pitch: -0.35 rad（前傾）
   - knee_pitch: -0.78 rad（-45°、深く曲げる）
   - ankle_pitch: 0.43 rad（バランス補正）
   - base_init_pos: 0.35m（目標高さ）

2. 参照軌道に低姿勢を組み込む（動的）
   - base_height_target: 0.35m

3. 非対称報酬（最重要）
   - 高すぎる: -40.0 * error² → 強いペナルティ
   - 低すぎる: -5.0 * error² → 弱いペナルティ
   - 8:1の比率で低い方向への探索を許容

V13の動的報酬構造を維持（V12の教訓）

Usage:
    cd rl_ws
    uv run python scripts/biped_train_v17.py --max_iterations 500
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
from biped_walking.envs.biped_env import BipedEnv


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
    rl_ws_dir = script_dir.parent
    urdf_path = rl_ws_dir / "assets" / "biped_digitigrade.urdf"

    # ============================================================
    # V17: 低重心歩行のための設定
    # ============================================================
    #
    # 1. 初期姿勢を低く設定
    #    - hip_pitch: -0.35 rad（-20°、前傾）
    #    - knee_pitch: -0.78 rad（-45°、深く曲げる）
    #    - ankle_pitch: 0.43 rad（+25°、バランス補正）
    #
    # 2. 初期高さを目標値に合わせる
    #    - base_init_pos: 0.35m
    #
    # 物理的検証（check_height.pyより）:
    #    - 0.35m達成: hip=-40°, knee=-5° で可能
    #    - 設定姿勢（hip=-20°, knee=-45°）での予想高さ: ~0.34m
    #
    # ============================================================

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
        # V17: 低重心姿勢（V13からの変更）
        "default_joint_angles": {
            "left_hip_yaw_joint": 0.0,
            "left_hip_roll_joint": 0.0,
            "left_hip_pitch_joint": -0.35,  # V13: 0.0 → V17: -0.35 rad (-20°, 前傾)
            "left_knee_pitch_joint": -0.78,  # V13: -0.52 → V17: -0.78 rad (-45°, 深く曲げる)
            "left_ankle_pitch_joint": 0.43,  # V13: 0.52 → V17: 0.43 rad (バランス補正)
            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_pitch_joint": -0.35,  # 同上
            "right_knee_pitch_joint": -0.78,  # 同上
            "right_ankle_pitch_joint": 0.43,  # 同上
        },
        "feet_names": ["left_foot_link", "right_foot_link"],
        # PD gains
        "kp": 35.0,
        "kd": 2.0,
        "termination_if_roll_greater_than": 30,
        "termination_if_pitch_greater_than": 30,
        # V17: 初期高さを目標に合わせる
        "base_init_pos": [0.0, 0.0, 0.35],  # V13: 0.45 → V17: 0.35
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
    # V17: 非対称報酬による低重心歩行の促進
    # ============================================================
    #
    # 主要変更点:
    # 1. base_height_target: 0.40 → 0.35m
    # 2. height_penalty_high: 40.0（高すぎる場合の強いペナルティ）
    # 3. height_penalty_low: 5.0（低すぎる場合の弱いペナルティ）
    #
    # ペナルティ比較（target=0.35m）:
    # | 高さ | 従来(-15*e²) | V17非対称 |
    # |------|-------------|-----------|
    # | 0.45m| -0.15       | -0.40     | ← 高い: 2.7倍強化
    # | 0.35m| 0           | 0         |
    # | 0.30m| -0.04       | -0.01     | ← 低い: 1/4に緩和
    #
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        # V17: 目標高さを低く設定
        "base_height_target": 0.35,  # V13: 0.40 → V17: 0.35
        # V17: 非対称ペナルティのパラメータ
        "height_penalty_high": 40.0,  # 高すぎる場合（新規）
        "height_penalty_low": 5.0,  # 低すぎる場合（新規）
        "feet_air_time_target": 0.25,
        "gait_frequency": 1.5,
        "reward_scales": {
            # ========== 主タスク報酬 ==========
            "tracking_lin_vel": 1.5,
            "tracking_ang_vel": 0.5,
            "alive": 0.1,
            "forward_progress": 0.3,
            # ========== 交互歩行報酬（V13継承）==========
            "alternating_gait": 1.5,
            "foot_swing": 0.8,
            "feet_air_time": 1.0,
            "single_stance": 0.5,
            "no_fly": -1.0,
            # ========== 動的歩行報酬（V13継承）==========
            "hip_pitch_alternation": 2.0,
            "hip_pitch_velocity": 0.5,
            "contact_alternation": 0.8,
            # ========== 姿勢・安定性ペナルティ ==========
            "orientation": -2.5,
            # V17: 非対称base_height報酬（_reward_base_height内で実装）
            # 従来の-15.0 * error² → 高い:-40.0 * error², 低い:-5.0 * error²
            "base_height": -1.0,  # スケールは関数内で適用されるため、ここは-1.0
            "lin_vel_z": -2.0,
            "ang_vel_xy": -0.05,
            "pitch_penalty": -3.0,
            "roll_penalty": -3.0,
            # ========== 振動抑制ペナルティ（V13継承）==========
            "action_rate": -0.03,
            "dof_vel": -1e-3,
            "dof_acc": -5e-7,
            "torques": -5e-5,
            "similar_to_default": -0.02,
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
    parser.add_argument("-e", "--exp_name", type=str, default="biped-walking-v17")
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

    env = BipedEnv(
        num_envs=args.num_envs, env_cfg=env_cfg, obs_cfg=obs_cfg, reward_cfg=reward_cfg, command_cfg=command_cfg
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()
