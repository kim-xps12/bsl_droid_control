"""BSL-Droid V22: 足先空間 Residual Policy トレーニング

============================================================
変更履歴（V21からの差分）
============================================================
- 環境: DroidEnv → DroidEnvTaskSpace
- 行動空間: 10次元（関節角度）→ 4次元（足先XZ残差）
- 観測空間: 39次元 → 39次元（actions 10→4、gait_phase 2追加、foot_pos 4追加）
- ベース軌道: 正弦波スイングパターン（stride_length=0.06m, swing_height=0.03m）
- IK: 解析的逆運動学で足先→関節変換

実験目的:
- 関節空間での探索限界を超えるため、足先空間での探索に移行
- ベース軌道が最低限の歩容を提供し、残差ポリシーで適応調整

実行:
    cd rl_ws
    uv run python biped_walking/train/droid_train_v22.py --max_iterations 500
"""

import argparse
import math
import os
import pickle

import torch

import genesis as gs
from biped_walking.envs.droid_env_taskspace import DroidEnvTaskSpace

# rsl-rl imports
from rsl_rl.runners import OnPolicyRunner


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v22")
    parser.add_argument("-B", "--num_envs", type=int, default=4096)
    parser.add_argument("--max_iterations", type=int, default=500)
    args = parser.parse_args()

    gs.init(logging_level="warning")

    # ログディレクトリ
    log_dir = f"logs/{args.exp_name}"
    os.makedirs(log_dir, exist_ok=True)

    # ============================================================
    # 環境設定
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

        # 初期位置・姿勢（V21と同じ高さ）
        "base_init_pos": [0.0, 0.0, 0.35],
        "base_init_quat": [1.0, 0.0, 0.0, 0.0],

        # PD制御パラメータ（V21と同じ）
        "kp": 35.0,
        "kd": 2.0,
        "torque_limit": 20.0,

        # 行動スケール（足先残差のスケール: 2cm）
        "action_scale": 0.02,
        "clip_actions": 3.0,
        "clip_observations": 100.0,

        # 終了条件（V21と同じ）
        "termination_if_pitch_greater_than": 25.0,  # deg
        "termination_if_roll_greater_than": 25.0,   # deg
        "termination_if_height_lower_than": 0.12,   # m

        # 足先リンク名
        "feet_names": ["left_foot_link", "right_foot_link"],

        # 運動学パラメータ（URDFから）
        "thigh_length": 0.11,
        "shank_length": 0.12,
        "foot_height": 0.035,
        "ankle_offset_x": 0.02,

        # 歩容パラメータ
        "gait_frequency": 2.0,     # Hz
        "swing_height": 0.03,      # m（足の持ち上げ高さ）
        "stride_length": 0.06,     # m（ストライド長）

        # デフォルト足先位置（股関節フレーム）
        "default_foot_x": 0.0381,
        "default_foot_z": -0.1820,
    }

    obs_cfg = {
        "num_obs": 39,  # 3+3+3+10+10+4+1+1+4
        "obs_scales": {
            "ang_vel": 0.25,
            "lin_vel": 2.0,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }

    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.25,  # 目標高さ
        "target_air_time": 0.25,
        "contact_threshold": 0.03,
        "reward_scales": {
            # 追従報酬
            "tracking_lin_vel": 2.0,
            "tracking_ang_vel": 0.5,

            # 姿勢維持
            "orientation": -1.0,
            "base_height": 2.0,  # 高さ維持を重視

            # ペナルティ
            "lin_vel_z": -0.5,
            "ang_vel_xy": -0.05,
            "torques": -1e-5,
            "dof_vel": -1e-4,
            "dof_acc": -1e-7,
            "action_rate": -0.01,

            # 足先追従
            "foot_tracking": 1.0,

            # 残差ペナルティ（ベース軌道からの逸脱を抑制）
            "residual_penalty": -0.1,

            # 生存・終了
            "alive": 0.5,
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
    # 環境と学習の実行
    # ============================================================
    env = DroidEnvTaskSpace(
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
