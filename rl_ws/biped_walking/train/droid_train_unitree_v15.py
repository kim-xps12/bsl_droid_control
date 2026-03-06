#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V15）

============================================================
【EXP007 V15: V3ベース + ゆったり大股歩行】
============================================================

【設計方針】
V3は0.246 m/sを達成し、静止ポリシー回避に成功した実績がある。
V15では、V3の成功要因を維持しつつ「ゆったり大股」を目指す。

【V3からの変更点】（3点）
1. gait_frequency: 1.5 Hz → 1.0 Hz（歩行周期を0.67秒から1.0秒に）
2. lin_vel_x_range: [0.2, 0.3] → [0.15, 0.25]（目標速度をやや低下）
3. feet_air_time: 1.0 → 1.5（滞空時間報酬を強化して大股を誘導）

【V3から維持する設定】（静止ポリシー回避の要）
- 報酬項目数: 16項目
- tracking_lin_vel: 1.5（動く動機を維持）
- single_foot_contact: 0.3（片足接地報酬）
- velocity_deficit: -0.5（速度未達ペナルティ）
- alive: 削除（静止の報酬価値を除去）
- その他全てV3と同じ

【V13との違い】
V13は「V1ベース + gait_frequency 0.8Hz + 速度低下」で失敗した。
V15は「V3ベース（静止回避機構あり）+ gait_frequency 1.0Hz + やや速度低下」。
V3の静止回避機構（single_foot_contact, velocity_deficit）を維持することで、
V13のような「動かない」問題を回避することを期待。

【成功基準】
- X速度: 0.15-0.20 m/s（目標速度に追従、V3の0.246より低い）
- 歩幅: V3より大きい（feet_air_time強化の効果）
- 目視評価: ゆったりとした歩行

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

    # V3と同じ初期姿勢
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
        "num_obs": 50,  # V3と同じ観測空間
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }

    # ============================================================
    # V15: V3ベース + ゆったり大股歩行
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.25,  # V3と同じ
        "base_height_target": 0.20,  # V3と同じ
        "swing_height_target": 0.03,  # V3と同じ
        "gait_frequency": 1.0,  # ★変更: V3 1.5Hz → V15 1.0Hz（ゆっくり）
        "contact_threshold": 0.025,  # V3と同じ
        "reward_scales": {
            # ============================================================
            # 【主報酬】速度追従（V3と同じ）
            # ============================================================
            "tracking_lin_vel": 1.5,  # V3と同じ（動く動機を維持）
            "tracking_ang_vel": 0.5,  # V3と同じ
            # ============================================================
            # 【歩行品質報酬】（V3の静止ポリシー回避機構を維持）
            # ============================================================
            "feet_air_time": 1.5,  # ★変更: V3 1.0 → V15 1.5（大股誘導）
            "contact": 0.2,  # V3と同じ
            # alive: 削除（V3と同じ、静止の報酬価値を除去）
            # V3の静止ポリシー回避機構を維持
            "single_foot_contact": 0.3,  # V3と同じ（片足接地報酬）
            # ============================================================
            # 【安定性ペナルティ】（V3と同じ）
            # ============================================================
            "lin_vel_z": -2.0,  # V3と同じ
            "ang_vel_xy": -0.05,  # V3と同じ
            "orientation": -0.5,  # V3と同じ
            "base_height": -5.0,  # V3と同じ
            # ============================================================
            # 【歩行品質ペナルティ】（V3と同じ）
            # ============================================================
            "feet_swing_height": -5.0,  # V3と同じ
            "contact_no_vel": -0.1,  # V3と同じ
            "hip_pos": -0.5,  # V3と同じ
            # V3の静止ポリシー回避機構を維持
            "velocity_deficit": -0.5,  # V3と同じ（速度未達ペナルティ）
            # ============================================================
            # 【エネルギー効率ペナルティ】（V3と同じ）
            # ============================================================
            "torques": -1e-5,  # V3と同じ
            "action_rate": -0.01,  # V3と同じ
            "dof_acc": -2.5e-7,  # V3と同じ
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.15, 0.25],  # ★変更: V3 [0.2, 0.3] → V15 [0.15, 0.25]
        "lin_vel_y_range": [0, 0],  # V3と同じ
        "ang_vel_range": [0, 0],  # V3と同じ
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main() -> None:
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V15)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v15")
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
    print(f"\n{'=' * 70}")
    print("EXP007 V15: V3ベース + ゆったり大股歩行")
    print(f"{'=' * 70}")
    print("【V3からの変更点】（3点）")
    print("  1. gait_frequency: 1.5 Hz → 1.0 Hz（ゆっくり）")
    print("  2. lin_vel_x_range: [0.2, 0.3] → [0.15, 0.25]（やや低速）")
    print("  3. feet_air_time: 1.0 → 1.5（大股誘導）")
    print(f"{'=' * 70}")
    print("【V3から維持する設定】（静止ポリシー回避の要）")
    print("  - tracking_lin_vel: 1.5（動く動機を維持）")
    print("  - single_foot_contact: 0.3（片足接地報酬）")
    print("  - velocity_deficit: -0.5（速度未達ペナルティ）")
    print("  - alive: 削除（静止の報酬価値を除去）")
    print(f"{'=' * 70}")
    print(f"観測空間: {obs_cfg['num_obs']}次元")
    print(f"行動空間: {env_cfg['num_actions']}次元")
    print(f"報酬項目数: {len(reward_cfg['reward_scales'])}")
    print(f"{'=' * 70}\n")

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
