"""BSL-Droid V25: 報酬バランス改善版 End-to-End

============================================================
変更履歴（V24からの差分）
============================================================
- tracking_lin_vel: 2.0 → 5.0（前進報酬を大幅強化）
- alive: 0.5 → 0.1（静止が最適解にならないよう削減）
- base_height: 2.0 → 0.5（静止が最適解にならないよう削減）
- foot_height_diff: 1.5 → 2.0（交互歩行誘導強化）
- forward_progress: 新規追加（X方向移動を直接報酬化）
- 環境: DroidEnvTaskSpaceE2E → DroidEnvTaskSpaceE2EV25

V24失敗の根本原因:
- 静止が最適解になっていた（alive+base_height > tracking_lin_vel）
- 報酬バランスを見直し、前進を強力に報酬化

実行:
    cd rl_ws
    uv run python biped_walking/train/droid_train_v25.py --max_iterations 500
"""

import argparse
import math
import os
import pickle

import torch

import genesis as gs
from biped_walking.envs.droid_env_taskspace_e2e_v25 import DroidEnvTaskSpaceE2EV25

# rsl-rl imports
from rsl_rl.runners import OnPolicyRunner


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v25")
    parser.add_argument("-B", "--num_envs", type=int, default=4096)
    parser.add_argument("--max_iterations", type=int, default=500)
    args = parser.parse_args()

    gs.init(logging_level="warning")

    # ログディレクトリ
    log_dir = f"logs/{args.exp_name}"
    os.makedirs(log_dir, exist_ok=True)

    # ============================================================
    # 環境設定（V24: End-to-End用）
    # ============================================================
    # 初期姿勢（V21と同じ）
    hip_pitch_rad = 60 * math.pi / 180    # 1.047 rad
    knee_pitch_rad = -100 * math.pi / 180  # -1.745 rad
    ankle_pitch_rad = 45 * math.pi / 180   # 0.785 rad

    env_cfg = {
        # 基本設定
        "urdf_path": "assets/bsl_droid_simplified.urdf",
        "num_actions": 4,  # 足先残差 (left_x, left_z, right_x, right_z)
        "episode_length_s": 20.0,
        "resampling_time_s": 4.0,

        # 関節名（V21と同じ）
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

        # デフォルト関節角度（V21と同じ）
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

        # V24: 初期位置を修正（足が地面に着くように）
        # default_foot_z = -0.182m なので、base_height = 0.19mで足が地面に着く
        "base_init_pos": [0.0, 0.0, 0.19],
        "base_init_quat": [1.0, 0.0, 0.0, 0.0],

        # PD制御パラメータ（V21と同じ）
        "kp": 35.0,
        "kd": 2.0,
        "torque_limit": 20.0,

        # V24: 行動スケール拡大（直接探索のため）
        "action_scale": 0.05,  # 5cm（V22の2cmから拡大）
        "clip_actions": 3.0,
        "clip_observations": 100.0,

        # V24: 終了条件（高さを初期位置に合わせて調整）
        "termination_if_pitch_greater_than": 30.0,  # deg
        "termination_if_roll_greater_than": 30.0,   # deg
        "termination_if_height_lower_than": 0.10,   # m（0.19mから約9cm落下で終了）

        # 足先リンク名
        "feet_names": ["left_foot_link", "right_foot_link"],

        # 運動学パラメータ（URDFから）
        "thigh_length": 0.11,
        "shank_length": 0.12,
        "foot_height": 0.035,
        "ankle_offset_x": 0.02,

        # V24: 歩容パラメータ廃止（ベース軌道を使用しないため）
        # "gait_frequency": 2.0,
        # "swing_height": 0.03,
        # "stride_length": 0.06,

        # デフォルト足先位置（股関節フレーム）
        "default_foot_x": 0.0381,
        "default_foot_z": -0.1820,
    }

    # V24: 観測次元 39→37（gait_phase 2次元削除）
    obs_cfg = {
        "num_obs": 37,  # 3+3+3+10+10+4+4 = 37
        "obs_scales": {
            "ang_vel": 0.25,
            "lin_vel": 2.0,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }

    # V25: 報酬バランス改善（前進を強力に報酬化）
    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.19,
        "target_air_time": 0.25,
        "contact_threshold": 0.03,
        "reward_scales": {
            # 追従報酬（V25: 前進を大幅強化）
            "tracking_lin_vel": 5.0,  # V24: 2.0 → V25: 5.0
            "tracking_ang_vel": 0.5,

            # V25新規: 前進距離を直接報酬化
            "forward_progress": 3.0,

            # 姿勢維持（V25: 静止が有利になりすぎないよう削減）
            "orientation": -1.0,
            "base_height": 0.5,  # V24: 2.0 → V25: 0.5

            # ペナルティ
            "lin_vel_z": -0.5,
            "ang_vel_xy": -0.05,
            "torques": -1e-5,
            "dof_vel": -1e-4,
            "dof_acc": -1e-7,
            "action_rate": -0.01,

            # 交互歩行誘導（V25: 強化）
            "foot_height_diff": 2.0,  # V24: 1.5 → V25: 2.0
            "hip_pitch_alternation": 1.0,  # V24: 0.5 → V25: 1.0

            # 生存・終了（V25: alive削減）
            "alive": 0.1,  # V24: 0.5 → V25: 0.1
            "termination": -100.0,
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x": [0.2, 0.5],    # 前進コマンド
        "lin_vel_y": [0.0, 0.0],    # 横方向は0
        "ang_vel_yaw": [0.0, 0.0],  # 回転は0
    }

    # ============================================================
    # 学習設定
    # ============================================================
    train_cfg = {
        "algorithm": {
            "class_name": "PPO",
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.01,
            "gamma": 0.99,
            "lam": 0.95,
            "learning_rate": 1e-3,
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
            "actor_hidden_dims": [256, 256, 128],
            "critic_hidden_dims": [256, 256, 128],
            "class_name": "ActorCritic",
            "init_noise_std": 1.0,
        },
        "runner": {
            "checkpoint": -1,
            "experiment_name": args.exp_name,
            "load_run": -1,
            "log_interval": 1,
            "max_iterations": args.max_iterations,
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

    # 設定を保存
    with open(os.path.join(log_dir, "cfgs.pkl"), "wb") as f:
        pickle.dump(
            {"env_cfg": env_cfg, "obs_cfg": obs_cfg, "reward_cfg": reward_cfg,
             "command_cfg": command_cfg, "train_cfg": train_cfg},
            f,
        )

    # ============================================================
    # 環境と学習の実行（V25: 報酬バランス改善版）
    # ============================================================
    env = DroidEnvTaskSpaceE2EV25(
        num_envs=args.num_envs,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        command_cfg=command_cfg,
        show_viewer=False,
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir=log_dir, device=gs.device)
    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()
