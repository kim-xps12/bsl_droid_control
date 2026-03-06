"""BSL-Droid Simplified 二脚ロボット Genesis訓練スクリプト v8

============================================================
V8: 立位姿勢での再学習
============================================================

V7の問題:
- 初期高さ0.25mがhip_pitch=-0.5, knee=-1.2の姿勢に対して低すぎた
- base_height_target=0.22mも低すぎて「座り込み」が報酬的に有利だった
- 膝が-2 rad（-115°）まで深く曲がり、完全にしゃがみ込んだ状態になった
- 結果: z座標0.08mで静止、前進ゼロ

V8での変更:
1. 初期姿勢を「立っている」姿勢に変更:
   - hip_pitch: 60° (1.047 rad) - 脚を後ろに引いた姿勢
   - knee_pitch: -100° (-1.745 rad) - 適度な屈曲
   - ankle_pitch: 45° (0.785 rad) - 足裏水平に近づける
2. 初期高さを上げる: 0.25m → 0.35m
3. base_height_targetを上げる: 0.22m → 0.28m
4. knee_max_angle（膝の過度な屈曲）へのペナルティ追加

採用している規約（Unitree/ANYmal標準、V7から継続）:
- 全ピッチ軸: (0, 1, 0) で統一
- hip_pitch: 正で前方振り出し
- knee_pitch: 負で後方屈曲（逆関節）
- ankle_pitch: 正でつま先上げ

Usage:
    cd rl_ws
    uv run python biped_walking/train/droid_train_v8.py --max_iterations 500
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

    # ============================================================
    # V8: 立位姿勢での再学習
    # ============================================================
    # 角度設定（度→ラジアン）:
    #   hip_pitch: 60° = 1.047 rad（脚を後ろに引いた姿勢）
    #   knee_pitch: -100° = -1.745 rad（適度な屈曲）
    #   ankle_pitch: 45° = 0.785 rad（足裏を水平に近づける）
    # ============================================================
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
        # ============================================================
        # V8: 「立っている」初期姿勢
        # ============================================================
        # V7では hip_pitch=-0.5, knee=-1.2 で低すぎた
        # V8では脚を後ろに引いた立位姿勢を採用
        "default_joint_angles": {
            "left_hip_yaw_joint": 0.0,
            "left_hip_roll_joint": 0.0,
            "left_hip_pitch_joint": hip_pitch_rad,  # 60° = 1.047 rad
            "left_knee_pitch_joint": knee_pitch_rad,  # -100° = -1.745 rad
            "left_ankle_pitch_joint": ankle_pitch_rad,  # 45° = 0.785 rad
            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_pitch_joint": hip_pitch_rad,
            "right_knee_pitch_joint": knee_pitch_rad,
            "right_ankle_pitch_joint": ankle_pitch_rad,
        },
        # 正しいリンク名
        "feet_names": ["left_foot_link", "right_foot_link"],
        # PD gains
        "kp": 35.0,
        "kd": 2.0,
        "termination_if_roll_greater_than": 30,
        "termination_if_pitch_greater_than": 30,
        # V8: 初期配置は少し高めに（落下して姿勢が安定する）
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
    # V8: 報酬設定
    # ============================================================
    # V7からの変更:
    # - base_height_target: 幾何学的計算値に基づく
    #   股関節→足首: thigh*cos(60°) + shank*cos(40°) = 0.055 + 0.092 = 0.147m
    #   足首→地面: 0.035m, 胴体中心→股関節: 0.08m
    #   合計: 0.147 + 0.035 + 0.08 = 0.262m
    # - knee_max_angle: 追加（膝が過度に曲がりすぎるのを抑制）
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.26,  # V7: 0.22 → V8: 0.26（幾何学的計算値）
        "feet_air_time_target": 0.25,
        "gait_frequency": 1.5,  # Hz
        "reward_scales": {
            # ========== 主タスク報酬 ==========
            "tracking_lin_vel": 1.5,
            "tracking_ang_vel": 0.5,
            "alive": 0.1,
            "forward_progress": 0.3,
            # ========== 交互歩行報酬 ==========
            "alternating_gait": 1.5,
            "foot_swing": 0.8,
            "feet_air_time": 1.0,
            "single_stance": 0.5,
            "no_fly": -1.0,
            # ========== 動的歩行報酬 (速度ベース) ==========
            "hip_pitch_alternation": 2.0,
            "hip_pitch_velocity": 0.5,
            "contact_alternation": 0.8,
            # ========== 姿勢・安定性ペナルティ ==========
            "orientation": -2.5,
            "base_height": -10.0,
            "base_height_high": -5.0,
            "lin_vel_z": -2.0,
            "ang_vel_xy": -0.05,
            "pitch_penalty": -3.0,
            "roll_penalty": -3.0,
            # ========== 膝角度制約（V7から継続 + V8追加） ==========
            # knee_negative: 正角度侵入にペナルティ（0°より大きくならない）
            # knee_max_angle: 過度な屈曲にペナルティ（-2 rad以下に曲がりすぎない）
            "dof_pos_limits": -5.0,
            "knee_negative": -3.0,
            "knee_max_angle": -3.0,  # V8追加: 過度な膝屈曲にペナルティ
            # ========== 後退ペナルティ ==========
            "backward_velocity": -2.0,
            # ========== 振動抑制ペナルティ ==========
            "action_rate": -0.03,
            "dof_vel": -1e-3,
            "dof_acc": -5e-7,
            "torques": -5e-5,
            "similar_to_default": -0.05,  # V7: -0.1 → V8: -0.05（デフォルト姿勢固着を緩和）
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
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v8")
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
