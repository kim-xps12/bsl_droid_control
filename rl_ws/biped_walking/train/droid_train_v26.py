"""BSL-Droid V26: 足先空間改善版

============================================================
変更履歴（V25からの差分）
============================================================
V25失敗の根本原因:
1. hip_roll異常: IK=0だがPDで追従失敗 → 脚が八の字に開く
2. 震え: foot_height_diff等を高周波振動でexploit
3. Yaw drift: 左右非対称な関節動作

V26の改善点:
1. 行動空間拡張: 4次元 → 6次元（hip_roll追加）
2. gait_phase復活 → 周期的歩容報酬で自然なリズム誘導
3. action_rate/dof_acc大幅強化 → 震え対策
4. gait_cycle報酬追加 → 高周波振動抑制
5. hip_roll_penaltyで開脚を抑制
6. gait_frequency低下: 2.0 → 1.5 Hz（ゆっくり歩行）

実行:
    cd rl_ws
    uv run python biped_walking/train/droid_train_v26.py --max_iterations 500
"""

from __future__ import annotations

import argparse
import math
import os
import pickle

import genesis as gs

# rsl-rl imports
from rsl_rl.runners import OnPolicyRunner

from biped_walking.envs.droid_env_taskspace_e2e_v26 import DroidEnvTaskSpaceE2EV26


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v26")
    parser.add_argument("-B", "--num_envs", type=int, default=4096)
    parser.add_argument("--max_iterations", type=int, default=500)
    args = parser.parse_args()

    gs.init(logging_level="warning")

    # ログディレクトリ
    log_dir = f"logs/{args.exp_name}"
    os.makedirs(log_dir, exist_ok=True)

    # ============================================================
    # 環境設定（V26: 足先空間改善版）
    # ============================================================
    # 初期姿勢
    hip_pitch_rad = 60 * math.pi / 180  # 1.047 rad
    knee_pitch_rad = -100 * math.pi / 180  # -1.745 rad
    ankle_pitch_rad = 45 * math.pi / 180  # 0.785 rad

    env_cfg = {
        # 基本設定
        "urdf_path": "assets/bsl_droid_simplified.urdf",
        "num_actions": 6,  # V26: 足先4 + hip_roll 2
        "episode_length_s": 20.0,
        "resampling_time_s": 4.0,
        # 関節名
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
        # デフォルト関節角度
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
        # 初期位置
        "base_init_pos": [0.0, 0.0, 0.19],
        "base_init_quat": [1.0, 0.0, 0.0, 0.0],
        # PD制御パラメータ
        "kp": 35.0,
        "kd": 2.0,
        "torque_limit": 20.0,
        # 行動スケール
        "action_scale": 0.05,  # 足先XZ: 5cm
        "hip_roll_scale": 0.1,  # hip_roll: 約5.7°
        "clip_actions": 3.0,
        "clip_observations": 100.0,
        # 終了条件
        "termination_if_pitch_greater_than": 30.0,  # deg
        "termination_if_roll_greater_than": 30.0,  # deg
        "termination_if_height_lower_than": 0.10,  # m
        # 足先リンク名
        "feet_names": ["left_foot_link", "right_foot_link"],
        # 運動学パラメータ
        "thigh_length": 0.11,
        "shank_length": 0.12,
        "foot_height": 0.035,
        "ankle_offset_x": 0.02,
        # V26: 歩容パラメータ（周期報酬用）
        "gait_frequency": 1.5,  # Hz（V25の2.0より低め、ゆっくり歩行）
        "swing_height": 0.03,  # m
        # デフォルト足先位置
        "default_foot_x": 0.0381,
        "default_foot_z": -0.1820,
    }

    # V26: 観測次元 41（gait_phase復活、action6次元）
    obs_cfg = {
        "num_obs": 41,  # 3+3+3+10+10+6+2+4 = 41
        "obs_scales": {
            "ang_vel": 0.25,
            "lin_vel": 2.0,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }

    # V26: 報酬設計（震え対策、周期的歩容誘導）
    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.19,
        "target_air_time": 0.25,
        "contact_threshold": 0.03,
        "reward_scales": {
            # 追従報酬
            "tracking_lin_vel": 5.0,
            "tracking_ang_vel": 0.5,
            # 前進距離
            "forward_progress": 3.0,
            # 姿勢維持
            "orientation": -1.0,
            "base_height": 0.5,
            # V26: ペナルティ大幅強化（震え対策）
            "lin_vel_z": -0.5,
            "ang_vel_xy": -0.05,
            "torques": -1e-5,
            "dof_vel": -1e-4,
            "dof_acc": -1e-5,  # V25: -1e-7 → V26: -1e-5（100倍強化）
            "action_rate": -0.1,  # V25: -0.01 → V26: -0.1（10倍強化）
            # 交互歩行誘導
            "foot_height_diff": 1.0,  # V25: 2.0 → V26: 1.0（gait_cycleと役割分担）
            "hip_pitch_alternation": 0.5,  # V25: 1.0 → V26: 0.5
            # V26新規: 周期的歩容報酬
            "gait_cycle": 2.0,  # 高周波振動を抑制
            # V26新規: hip_rollペナルティ
            "hip_roll_penalty": -1.0,  # 開脚を抑制
            # 生存・終了
            "alive": 0.1,
            "termination": -100.0,
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x": [0.15, 0.3],  # V25より低速（0.2-0.5 → 0.15-0.3）
        "lin_vel_y": [0.0, 0.0],
        "ang_vel_yaw": [0.0, 0.0],
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
    env = DroidEnvTaskSpaceE2EV26(
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
