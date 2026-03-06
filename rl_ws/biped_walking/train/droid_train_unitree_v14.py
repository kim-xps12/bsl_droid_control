#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V14）

============================================================
【EXP007 V14: V1ベース + 速度目標のみ低下】
============================================================

【設計方針】
V13レポートの教訓「1変更1検証」を厳守し、V1から速度目標のみを変更する。
V2-V13で複数変更を同時に行い、V1の歩行パターンを壊してしまった反省に基づく。

【V1からの変更点】（1点のみ）
- lin_vel_x_range: [0.2, 0.3] → [0.15, 0.20]（速度目標を下げる）

【V1と同じ設定（維持）】
- gait_frequency: 1.5 Hz
- 報酬項目数: 15項目
- tracking_sigma: 0.25
- action_rate: -0.01
- hip_pos: -0.5
- その他全てのパラメータ

【成功基準】
- X速度: 0.15-0.20 m/s（目標速度に追従）
- 歩行品質: V1レベルを維持
- 目視評価: V1と同等

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

from biped_walking.envs.droid_env_unitree import DroidEnvUnitree


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

    # V1と同じ初期姿勢
    hip_pitch_rad = 60 * math.pi / 180
    knee_pitch_rad = -100 * math.pi / 180
    ankle_pitch_rad = 45 * math.pi / 180

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
        "termination_if_roll_greater_than": 25,
        "termination_if_pitch_greater_than": 25,
        "termination_if_height_lower_than": 0.12,
        "termination_if_knee_positive": True,
        "base_init_pos": [0.0, 0.0, 0.35],
        "base_init_quat": [1.0, 0.0, 0.0, 0.0],
        "episode_length_s": 20.0,
        "resampling_time_s": 4.0,
        "action_scale": 0.25,  # rad（約14°）
        "simulate_action_latency": True,
        "clip_actions": 10.0,
    }

    obs_cfg = {
        "num_obs": 50,  # Unitree方式の観測空間（V1と同じ）
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }

    # ============================================================
    # V1と同じ報酬設計（15項目）
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.25,  # V1と同じ
        "base_height_target": 0.20,  # V1と同じ
        "swing_height_target": 0.03,  # V1と同じ
        "gait_frequency": 1.5,  # V1と同じ（周期0.67秒）
        "contact_threshold": 0.025,  # V1と同じ
        "reward_scales": {
            # ============================================================
            # 【主報酬】速度追従（V1と同じ）
            # ============================================================
            "tracking_lin_vel": 1.0,  # V1と同じ
            "tracking_ang_vel": 0.5,  # V1と同じ
            # ============================================================
            # 【歩行品質報酬】（V1と同じ）
            # ============================================================
            "feet_air_time": 1.0,  # V1と同じ
            "contact": 0.2,  # V1と同じ
            "alive": 0.1,  # V1と同じ
            # ============================================================
            # 【安定性ペナルティ】（V1と同じ）
            # ============================================================
            "lin_vel_z": -2.0,  # V1と同じ
            "ang_vel_xy": -0.05,  # V1と同じ
            "orientation": -0.5,  # V1と同じ
            "base_height": -5.0,  # V1と同じ
            # ============================================================
            # 【歩行品質ペナルティ】（V1と同じ）
            # ============================================================
            "feet_swing_height": -5.0,  # V1と同じ
            "contact_no_vel": -0.1,  # V1と同じ
            "hip_pos": -0.5,  # V1と同じ
            # ============================================================
            # 【エネルギー効率ペナルティ】（V1と同じ）
            # ============================================================
            "torques": -1e-5,  # V1と同じ
            "action_rate": -0.01,  # V1と同じ
            "dof_acc": -2.5e-7,  # V1と同じ
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.15, 0.20],  # ★唯一の変更点: V1 [0.2, 0.3] → V14 [0.15, 0.20]
        "lin_vel_y_range": [0, 0],  # V1と同じ
        "ang_vel_range": [0, 0],  # V1と同じ
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main() -> None:
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V14)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v14")
    parser.add_argument("--num_envs", type=int, default=4096)
    parser.add_argument("--max_iterations", type=int, default=500)
    args = parser.parse_args()

    # Genesis初期化
    gs.init(logging_level="warning")

    # 設定読み込み
    env_cfg, obs_cfg, reward_cfg, command_cfg = get_cfgs()
    train_cfg = get_train_cfg(args.exp_name, args.max_iterations)

    # 環境作成
    env = DroidEnvUnitree(
        num_envs=args.num_envs,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        command_cfg=command_cfg,
        show_viewer=False,
    )

    # ログディレクトリ
    log_root = Path(__file__).resolve().parent.parent.parent / "logs"
    log_dir = log_root / args.exp_name

    # 既存ログのバックアップ
    if log_dir.exists():
        backup_dir = log_root / f"{args.exp_name}_backup_{os.getpid()}"
        shutil.move(str(log_dir), str(backup_dir))
        print(f"既存ログを {backup_dir} にバックアップしました")

    log_dir.mkdir(parents=True, exist_ok=True)

    # 設定を保存
    cfgs_path = log_dir / "cfgs.pkl"
    with open(cfgs_path, "wb") as f:
        pickle.dump((env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg), f)
    print(f"設定を保存: {cfgs_path}")

    # ランナー作成
    runner = OnPolicyRunner(env, train_cfg, log_dir=str(log_dir), device="mps")

    # 訓練開始
    print(f"\n{'=' * 60}")
    print("EXP007 V14: V1ベース + 速度目標のみ低下")
    print(f"{'=' * 60}")
    print("【V1からの変更点】（1点のみ）")
    print("- lin_vel_x_range: [0.2, 0.3] → [0.15, 0.20]")
    print(f"{'=' * 60}")
    print("【V1と同じ設定（維持）】")
    print("- gait_frequency: 1.5 Hz")
    print(f"- 報酬項目数: {len(reward_cfg['reward_scales'])}項目")
    print("- tracking_sigma: 0.25")
    print("- action_rate: -0.01")
    print("- hip_pos: -0.5")
    print(f"{'=' * 60}")
    print(f"観測空間: {obs_cfg['num_obs']}次元")
    print(f"行動空間: {env_cfg['num_actions']}次元")
    print(f"{'=' * 60}\n")

    # 報酬スケール表示
    print("報酬スケール:")
    for name, scale in reward_cfg["reward_scales"].items():
        print(f"  {name}: {scale}")
    print()

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)

    print(f"\n訓練完了: {args.exp_name}")
    print(f"モデル保存先: {log_dir}")


if __name__ == "__main__":
    main()
