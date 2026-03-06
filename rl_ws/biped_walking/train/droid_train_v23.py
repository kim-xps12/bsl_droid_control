"""BSL-Droid V23: 足先空間 Residual Policy トレーニング（改善版）

============================================================
変更履歴（V22からの差分）
============================================================
- FKのhip_roll Z方向影響を修正（droid_kinematics.py）
- 報酬設計の改善（V21の成功パターンを参考）
  - 交互歩行を促進するhip_pitch_alternation報酬を追加
  - ベース軌道パラメータを調整（歩幅拡大）
- 行動スケールの調整（探索範囲の拡大）

V22からの主な改善点:
1. DroidKinematicsのFK精度向上（hip_rollのZ影響考慮）
2. 交互歩行報酬の追加（同期歩行防止）
3. 歩幅の拡大（stride_length: 0.06 → 0.08m）
4. 行動スケールの拡大（0.02 → 0.03m）

実験目的:
- 足先空間アプローチでの交互歩行獲得
- V21より大きな歩幅での安定歩行

実行:
    cd rl_ws
    uv run python biped_walking/train/droid_train_v23.py --max_iterations 500
"""

from __future__ import annotations

import argparse
import math
import os
import pickle

import genesis as gs

# rsl-rl imports
from rsl_rl.runners import OnPolicyRunner

from biped_walking.envs.droid_env_taskspace import DroidEnvTaskSpace


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v23")
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
    hip_pitch_rad = 60 * math.pi / 180  # 1.047 rad
    knee_pitch_rad = -100 * math.pi / 180  # -1.745 rad
    ankle_pitch_rad = 45 * math.pi / 180  # 0.785 rad

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
        # 行動スケール（V22: 0.02 → V23: 0.03m に拡大）
        "action_scale": 0.03,
        "clip_actions": 3.0,
        "clip_observations": 100.0,
        # 終了条件（V21と同じ）
        "termination_if_pitch_greater_than": 25.0,  # deg
        "termination_if_roll_greater_than": 25.0,  # deg
        "termination_if_height_lower_than": 0.12,  # m
        # 足先リンク名
        "feet_names": ["left_foot_link", "right_foot_link"],
        # 運動学パラメータ（URDFから）
        "thigh_length": 0.11,
        "shank_length": 0.12,
        "foot_height": 0.035,
        "ankle_offset_x": 0.02,
        # 歩容パラメータ（V22より歩幅拡大）
        "gait_frequency": 2.0,  # Hz
        "swing_height": 0.035,  # m（V22: 0.03 → 0.035）
        "stride_length": 0.08,  # m（V22: 0.06 → 0.08）
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

    # ============================================================
    # 報酬設計（V21の成功パターンを参考に改善）
    # ============================================================
    # V22の問題点:
    # - 交互歩行を促す報酬がない
    # - 残差ペナルティが探索を阻害する可能性
    #
    # V23の改善:
    # - hip_pitch_alternation報酬を追加（V21から導入）
    # - residual_penaltyを緩和
    # - 姿勢維持報酬を強化
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.26,  # 目標高さ（V21: 0.25 → 0.26）
        "target_air_time": 0.25,
        "contact_threshold": 0.04,  # V21と同じ
        "reward_scales": {
            # ============================================================
            # 【タスク報酬】
            # ============================================================
            "tracking_lin_vel": 2.0,  # 前進速度追従（最重要）
            "tracking_ang_vel": 0.5,  # 旋回速度追従
            "alive": 1.0,  # 生存報酬
            # ============================================================
            # 【姿勢維持】
            # ============================================================
            "orientation": -1.5,  # 姿勢ペナルティ（V22: -1.0 → -1.5）
            "base_height": 3.0,  # 高さ維持報酬（V22: 2.0 → 3.0）
            # ============================================================
            # 【交互歩行促進】（V21から導入）
            # ============================================================
            "hip_pitch_alternation": 2.0,  # 交互歩行報酬（NEW）
            # ============================================================
            # 【エネルギー効率】
            # ============================================================
            "lin_vel_z": -0.3,  # 上下動抑制（V22: -0.5 → -0.3）
            "ang_vel_xy": -0.05,  # 回転抑制
            "torques": -1e-5,  # トルク最小化
            "dof_vel": -1e-4,  # 関節速度最小化
            "dof_acc": -1e-7,  # 加速度最小化
            "action_rate": -0.005,  # アクション変化率（V22: -0.01 → -0.005）
            # ============================================================
            # 【足先空間制御】
            # ============================================================
            "foot_tracking": 1.5,  # 足先追従（V22: 1.0 → 1.5）
            "residual_penalty": -0.05,  # 残差ペナルティ（V22: -0.1 → -0.05、緩和）
            # ============================================================
            # 【終了】
            # ============================================================
            "termination": -100.0,
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x": [0.2, 0.5],  # 前進コマンド
        "lin_vel_y": [0.0, 0.0],  # 横方向は0
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
            {
                "env_cfg": env_cfg,
                "obs_cfg": obs_cfg,
                "reward_cfg": reward_cfg,
                "command_cfg": command_cfg,
                "train_cfg": train_cfg,
            },
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
