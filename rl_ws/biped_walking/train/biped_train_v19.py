"""
BSL-Droid二脚ロボット Genesis訓練スクリプト v19

============================================================
V13ベース + 穏やかな非対称高さ報酬（低重心化）
============================================================

V17/V18の失敗分析:
- V17: 低重心化は成功（0.371m）だが後退・横移動
- V18: その場ジャンプのみ、歩行完全失敗
  - 原因: 過剰な方向制御ペナルティ、初期位置の不整合

V19の設計方針:

1. V13をベースに戴る（歩行成功実績あり）
   - 動的報酬構造を維持
   - default_joint_anglesはV13のまま
   - base_init_posはV13のまま（0.45m）
   - 方向制御ペナルティは追加しない

2. 穏やかな非対称高さ報酬のみ追加
   - base_height_target: 0.35m（V18と同じ）
   - 非対称比率: 2:1（V17の8:1より穏やか）
   - height_penalty_high: 15.0（高いとき）
   - height_penalty_low: 10.0（低いとき）

3. num_envs=4096（V13と同じ、高速化）

原則: 「一度に1つだけ変える」

Usage:
    cd rl_ws
    uv run python scripts/biped_train_v19.py --max_iterations 500
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
            "left_hip_pitch_joint": 0.0,
            "left_knee_pitch_joint": -0.52,
            "left_ankle_pitch_joint": 0.52,
            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_pitch_joint": 0.0,
            "right_knee_pitch_joint": -0.52,
            "right_ankle_pitch_joint": 0.52,
        },
        # 正しいリンク名
        "feet_names": ["left_foot_link", "right_foot_link"],
        # PD gains
        "kp": 35.0,
        "kd": 2.0,
        "termination_if_roll_greater_than": 30,
        "termination_if_pitch_greater_than": 30,
        "base_init_pos": [0.0, 0.0, 0.45],
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
    # V19: V13ベース + 穏やかな非対称高さ報酬
    # ============================================================
    #
    # 変更方針:
    # 1. V13の動的報酬構造を維持
    # 2. base_height_targetを0.35mに変更（V18と同じ）
    # 3. 非対称高さペナルティを追加（比率2:1）
    # 4. 方向制御ペナルティは追加しない（V18の失敗原因）
    #
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        # V19: 目標高さ0.35m（V18と同じ）
        "base_height_target": 0.35,
        # V19: 非対称ペナルティのパラメータ（比率2:1に緩和）
        "height_penalty_high": 15.0,  # V17: 40.0 → V19: 15.0
        "height_penalty_low": 10.0,  # V17: 5.0 → V19: 10.0
        "feet_air_time_target": 0.25,
        "gait_frequency": 1.5,  # Hz
        "reward_scales": {
            # ========== 主タスク報酬 ==========
            "tracking_lin_vel": 1.5,
            "tracking_ang_vel": 0.5,
            "alive": 0.1,
            "forward_progress": 0.3,
            # ========== 交互歩行報酬（V11ベース）==========
            "alternating_gait": 1.5,
            "foot_swing": 0.8,
            "feet_air_time": 1.0,
            "single_stance": 0.5,
            "no_fly": -1.0,
            # ========== V13: 動的歩行報酬（速度ベース）==========
            "hip_pitch_alternation": 2.0,  # V11: 1.0 → 強化
            "hip_pitch_velocity": 0.5,  # 新規：hip_pitchが動いていることを報酬
            "contact_alternation": 0.8,  # 新規：接地タイミングの交互性
            # ========== 姿勢・安定性ペナルティ ==========
            "orientation": -2.5,
            "base_height": -15.0,
            "lin_vel_z": -2.0,
            "ang_vel_xy": -0.05,
            "pitch_penalty": -3.0,
            "roll_penalty": -3.0,  # V13新規：V12のRoll傾斜対策
            # ========== 振動抑制ペナルティ（V11維持）==========
            "action_rate": -0.03,
            "dof_vel": -1e-3,
            "dof_acc": -5e-7,
            "torques": -5e-5,
            "similar_to_default": -0.02,  # V13で復元（V12で削除したら失敗）
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
    parser.add_argument("-e", "--exp_name", type=str, default="biped-walking-v19")
    # V19: num_envs=4096（V13と同じ、高速化）
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
