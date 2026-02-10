#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V16）

============================================================
【EXP007 V16: V15ベース + 振動抑制・大股歩行強化】
============================================================

【設計方針】
V15は0.198 m/sで歩行に成功したが、「小刻み振動」問題が残った。
V16では**関節角速度の制御**と**ストライド長の増加**の2軸で
「ゆったりと大股の歩容」を実現する。

【問題の根本原因】（V15レポートより）
- knee_pitch可動域: 0.099 rad（V3の0.305 radの約1/3）
- dof_velペナルティ未使用により高周波振動が抑制されていない
- feet_air_timeオフセット0.5秒がBSL-Droidには大きすぎ、滞空報酬を獲得しにくい
- action_rate=-0.01が大きな関節動作を抑制している

【V15からの変更点】（5点）
1. dof_vel: (なし) → -0.005（振動抑制）
2. step_length: (なし) → 0.5（ストライド長報酬）
3. action_rate: -0.01 → -0.005（大きな動作許容）
4. swing_height_target: 0.03 m → 0.04 m（足上げ誘導）
5. air_time_offset: 0.5秒 → 0.3秒（滞空報酬獲得しやすく）

【V15から維持する設定】（静止ポリシー回避の要）
- 報酬項目数: 16 → 18項目（+2: dof_vel, step_length）
- tracking_lin_vel: 1.5（動く動機を維持）
- single_foot_contact: 0.3（片足接地報酬）
- velocity_deficit: -0.5（速度未達ペナルティ）
- alive: 削除（静止の報酬価値を除去）
- gait_frequency: 1.0 Hz（V15と同じ）
- lin_vel_x_range: [0.15, 0.25]（V15と同じ）

【成功基準】
- X速度: 0.15-0.20 m/s（V15と同等を維持）
- knee_pitch可動域: > 0.20 rad（V15の0.099 radから改善）
- DOF range sum: > 2.0 rad（V15の1.834 radから改善）
- 関節速度std: < 1.0 rad/s（振動抑制確認）
- 目視評価: ゆったり大股

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
    # V16: V15ベース + 振動抑制・大股歩行強化
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.25,  # V15と同じ
        "base_height_target": 0.20,  # V15と同じ
        "swing_height_target": 0.04,  # ★変更: V15 0.03 → V16 0.04 m（足上げ誘導）
        "gait_frequency": 1.0,  # V15と同じ
        "contact_threshold": 0.025,  # V15と同じ
        "air_time_offset": 0.3,  # ★変更: デフォルト0.5 → V16 0.3秒（滞空報酬獲得しやすく）
        "reward_scales": {
            # ============================================================
            # 【主報酬】速度追従（V15と同じ）
            # ============================================================
            "tracking_lin_vel": 1.5,  # V15と同じ（動く動機を維持）
            "tracking_ang_vel": 0.5,  # V15と同じ
            # ============================================================
            # 【歩行品質報酬】（V15の静止ポリシー回避機構を維持）
            # ============================================================
            "feet_air_time": 1.5,  # V15と同じ
            "contact": 0.2,  # V15と同じ
            # alive: 削除（V15と同じ、静止の報酬価値を除去）
            # V15の静止ポリシー回避機構を維持
            "single_foot_contact": 0.3,  # V15と同じ（片足接地報酬）
            # ============================================================
            # 【大股歩行強化】（V16新規）
            # ============================================================
            "step_length": 0.5,  # ★新規: ストライド長報酬
            # ============================================================
            # 【安定性ペナルティ】（V15と同じ）
            # ============================================================
            "lin_vel_z": -2.0,  # V15と同じ
            "ang_vel_xy": -0.05,  # V15と同じ
            "orientation": -0.5,  # V15と同じ
            "base_height": -5.0,  # V15と同じ
            # ============================================================
            # 【歩行品質ペナルティ】（V15と同じ）
            # ============================================================
            "feet_swing_height": -5.0,  # V15と同じ
            "contact_no_vel": -0.1,  # V15と同じ
            "hip_pos": -0.5,  # V15と同じ
            # V15の静止ポリシー回避機構を維持
            "velocity_deficit": -0.5,  # V15と同じ（速度未達ペナルティ）
            # ============================================================
            # 【エネルギー効率・振動抑制ペナルティ】
            # ============================================================
            "torques": -1e-5,  # V15と同じ
            "action_rate": -0.005,  # ★変更: V15 -0.01 → V16 -0.005（大きな動作許容）
            "dof_acc": -2.5e-7,  # V15と同じ
            "dof_vel": -0.005,  # ★新規: 関節角速度ペナルティ（振動抑制）
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
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V16)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v16")
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
    print("EXP007 V16: V15ベース + 振動抑制・大股歩行強化")
    print(f"{'=' * 70}")
    print("【V15からの変更点】（5点）")
    print("  1. dof_vel: (なし) → -0.005（振動抑制）")
    print("  2. step_length: (なし) → 0.5（ストライド長報酬）")
    print("  3. action_rate: -0.01 → -0.005（大きな動作許容）")
    print("  4. swing_height_target: 0.03 → 0.04 m（足上げ誘導）")
    print("  5. air_time_offset: 0.5 → 0.3秒（滞空報酬獲得しやすく）")
    print(f"{'=' * 70}")
    print("【V15から維持する設定】（静止ポリシー回避の要）")
    print("  - tracking_lin_vel: 1.5（動く動機を維持）")
    print("  - single_foot_contact: 0.3（片足接地報酬）")
    print("  - velocity_deficit: -0.5（速度未達ペナルティ）")
    print("  - alive: 削除（静止の報酬価値を除去）")
    print("  - gait_frequency: 1.0 Hz（V15と同じ）")
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
