"""BSL-Droid Simplified 二脚ロボット Genesis訓練スクリプト v9

============================================================
V9: 交互歩行とストライド改善
============================================================

V8の成果:
- 前進を達成（10秒で2.58m、目標速度の86%）
- 姿勢安定（Roll/Pitch変動小）
- 膝角度正常（-1.7〜-2.1 rad）

V8の残存課題:
1. 左右同期（並行移動）: hip_pitch相関+0.772（理想は-1.0）
2. 小さいストライド: hip_pitch range L=0.19, R=0.69 rad
3. 足の持ち上げ量不足: 両足宙浮き100%（滑走歩行）
4. Yawドリフト: -15.7°/10s
5. 目標高さ未達: 実測0.214m vs 目標0.26m

V9での変更:
1. 交互歩行強化:
   - hip_pitch_alternation: 2.0 → 4.0
   - hip_pitch_sync_penalty: 追加（同期動作にペナルティ）
   - contact_alternation: 0.8 → 1.5
2. 足の持ち上げ改善:
   - foot_clearance: 追加（足のクリアランス報酬）
   - feet_air_time: 1.0 → 2.0
3. ストライド改善:
   - hip_pitch_range: 追加（動作範囲報酬）
   - similar_to_default: -0.05 → -0.02（動作範囲を広げる）
4. 高さ維持:
   - base_height_target: 0.26 → 0.22（現実的な値に調整）
   - base_height_low: 追加（低すぎペナルティ）
5. Yawドリフト対策:
   - yaw_rate: 追加（角速度ペナルティ）
   - symmetry: 追加（左右対称性報酬）

Usage:
    cd rl_ws
    uv run python biped_walking/train/droid_train_v9.py --max_iterations 500
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
    # V9: V8と同じ初期姿勢を継続
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
        # V8と同じ初期姿勢
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
        "feet_names": ["left_foot_link", "right_foot_link"],
        "kp": 35.0,
        "kd": 2.0,
        "termination_if_roll_greater_than": 30,
        "termination_if_pitch_greater_than": 30,
        # V8と同じ初期配置
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
    # V9: 報酬設定 - 交互歩行とストライド改善
    # ============================================================
    # V8の問題点に対処:
    # 1. hip_pitch相関+0.772 → 交互歩行報酬強化 + 同期ペナルティ
    # 2. 小さいストライド → hip_pitch_range報酬 + similar_to_default緩和
    # 3. 滑走歩行 → foot_clearance報酬 + feet_air_time強化
    # 4. Yawドリフト → yaw_rate + symmetry
    # 5. 低すぎる高さ → base_height_target調整 + base_height_low
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        # V9: 目標高さを現実的な値に調整（V8実測0.214mに近づける）
        "base_height_target": 0.22,  # V8: 0.26 → V9: 0.22
        "feet_air_time_target": 0.25,
        "gait_frequency": 1.5,  # Hz
        "reward_scales": {
            # ========== 主タスク報酬 ==========
            "tracking_lin_vel": 1.5,
            "tracking_ang_vel": 0.5,
            "alive": 0.1,
            "forward_progress": 0.3,
            # ========== 交互歩行報酬（V9強化） ==========
            "alternating_gait": 1.5,
            "foot_swing": 0.8,
            "feet_air_time": 2.0,  # V8: 1.0 → V9: 2.0
            "single_stance": 0.5,
            "no_fly": -1.0,
            # ========== 動的歩行報酬（V9強化） ==========
            "hip_pitch_alternation": 4.0,  # V8: 2.0 → V9: 4.0（交互動作を強化）
            "hip_pitch_velocity": 0.5,
            "contact_alternation": 1.5,  # V8: 0.8 → V9: 1.5
            # ========== V9追加: 同期ペナルティ ==========
            "hip_pitch_sync_penalty": -3.0,  # 左右が同時に同方向に動くとペナルティ
            # ========== V9追加: 足の持ち上げ ==========
            "foot_clearance": 2.0,  # 足のクリアランス報酬
            # ========== V9追加: ストライド改善 ==========
            "hip_pitch_range": 1.0,  # hip_pitchの動作範囲を報酬化
            # ========== 姿勢・安定性ペナルティ ==========
            "orientation": -2.5,
            "base_height": -10.0,
            "base_height_high": -5.0,
            "base_height_low": -10.0,  # V9追加: 低すぎペナルティ
            "lin_vel_z": -2.0,
            "ang_vel_xy": -0.05,
            "pitch_penalty": -3.0,
            "roll_penalty": -3.0,
            # ========== V9追加: Yawドリフト対策 ==========
            "yaw_rate": -1.0,  # Yaw角速度ペナルティ
            "symmetry": -0.5,  # 左右対称性ペナルティ
            # ========== 膝角度制約 ==========
            "dof_pos_limits": -5.0,
            "knee_negative": -3.0,
            "knee_max_angle": -3.0,
            # ========== 後退ペナルティ ==========
            "backward_velocity": -2.0,
            # ========== 振動抑制ペナルティ ==========
            "action_rate": -0.03,
            "dof_vel": -1e-3,
            "dof_acc": -5e-7,
            "torques": -5e-5,
            "similar_to_default": -0.02,  # V8: -0.05 → V9: -0.02（動作範囲を広げる）
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
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v9")
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
