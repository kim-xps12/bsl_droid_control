#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V18）

============================================================
【EXP007 V18: V3ベース + RobStride RS-02実機パラメータ適合】
============================================================

【V3からの変更点】
V3の報酬設計をベースに、RobStride RS-02モータの実機パラメータ（最大角速度 ±44 rad/s）
に適合した動作を学習させる。シミュレーション環境で速度制限を超える動作を学習すると、
実機展開時に速度飽和による動作不安定・性能劣化が発生する。

| パラメータ             | V3値       | V18値        | 変更理由                        |
|-----------------------|------------|-------------|--------------------------------|
| dof_vel_limits        | 設定なし   | 44.0 rad/s  | RobStride RS-02実機仕様         |
| soft_dof_vel_limit    | 設定なし   | 0.9 (90%)   | 制限の90%でペナルティ開始       |
| reward: dof_vel_limits| 設定なし   | -0.3        | 速度制限超過ペナルティ          |

【実機パラメータの根拠】
RobStride RS-02仕様（ros2_ws/src/robstride_hardware/include/robstride_hardware/robstride_driver.hpp）:
- Max Velocity: ±44 rad/s
- Max Torque: ±17 Nm

【期待される効果】
1. 実機パラメータ遵守: シミュレーションで実機の物理制約を学習
2. sim-to-real transfer: 速度飽和による性能劣化を事前に防止
3. 安全性向上: 過度な速度指令を回避

【参考文献】
- ETH Zurich Legged Gym: https://github.com/leggedrobotics/legged_gym
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

    # V9以降と同じ初期姿勢
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
        "num_obs": 50,  # Unitree方式の観測空間（3+3+3+3+10+10+10+1+1+2+2+2=50）
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }

    # ============================================================
    # V18: V3ベース + RobStride RS-02実機パラメータ適合
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.25,  # 速度追従のガウシアン幅
        "base_height_target": 0.20,  # 目標胴体高さ（BSL-Droid用に調整）
        "swing_height_target": 0.03,  # 遊脚の目標高さ
        "gait_frequency": 1.5,  # 歩行周波数
        "contact_threshold": 0.025,  # 接地判定閾値 m
        # V18: RobStride RS-02実機パラメータ
        "dof_vel_limits": 44.0,  # ±44 rad/s (RS-02 spec)
        "soft_dof_vel_limit": 0.9,  # 制限の90%でペナルティ開始
        "reward_scales": {
            # ============================================================
            # 【主報酬】速度追従（V3強化）
            # ============================================================
            "tracking_lin_vel": 1.5,  # 線速度追従（V2: 1.0 → V3: 1.5、動く動機を強化）
            "tracking_ang_vel": 0.5,  # 角速度追従（V2: 0.8 → V3: 0.5、V1に復元）
            # ============================================================
            # 【歩行品質報酬】（V3: 静止ポリシー対策）
            # ============================================================
            "feet_air_time": 1.0,  # 滞空時間報酬
            "contact": 0.2,  # 接地フェーズ整合性（V2: 0.5 → V3: 0.2、V1に復元）
            # alive: 削除（V2: 0.15 → V3: 0.0）
            # 静止でも報酬を獲得できる項目を削除
            # 【V3新規】片足接地報酬
            "single_foot_contact": 0.3,  # 移動コマンド時に片足のみ接地を報酬化
            # ============================================================
            # 【安定性ペナルティ】（Unitree方式）
            # ============================================================
            "lin_vel_z": -2.0,  # Z軸速度ペナルティ
            "ang_vel_xy": -0.05,  # XY角速度ペナルティ
            "orientation": -0.5,  # 姿勢ペナルティ（BSL-Droid向け緩和）
            "base_height": -5.0,  # 高さ維持（BSL-Droid向け緩和）
            # ============================================================
            # 【歩行品質ペナルティ】
            # ============================================================
            "feet_swing_height": -5.0,  # 遊脚高さ（V2: -10.0 → V3: -5.0、V1に復元）
            "contact_no_vel": -0.1,  # 接地時足速度
            "hip_pos": -0.5,  # 股関節位置（開脚抑制）
            # 【V3新規】速度未達ペナルティ
            "velocity_deficit": -0.5,  # 目標速度を下回るとペナルティ
            # ============================================================
            # 【V18新規】関節角速度制限
            # ============================================================
            "dof_vel_limits": -0.3,  # 実機パラメータ超過ペナルティ
            # ============================================================
            # 【エネルギー効率ペナルティ】
            # ============================================================
            "torques": -1e-5,  # トルクペナルティ
            "action_rate": -0.01,  # アクション変化率
            "dof_acc": -2.5e-7,  # 関節加速度
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.2, 0.3],  # 目標前進速度（V2: 0.10-0.15 → V3: 0.2-0.3、V1に復元）
        "lin_vel_y_range": [0, 0],  # 横移動なし
        "ang_vel_range": [0, 0],  # 旋回なし
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main() -> None:
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V18)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v18")
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
    print("EXP007 V18: V3ベース + RobStride RS-02実機パラメータ適合")
    print(f"{'=' * 70}")
    print("【V18の改善点】")
    print("  V3の報酬設計を維持しつつ、実機パラメータ制約を追加")
    print(f"{'=' * 70}")
    print("【V18での追加項目】")
    print("  - dof_vel_limits: 44.0 rad/s（RobStride RS-02実機仕様）")
    print("  - soft_dof_vel_limit: 0.9（制限の90%でペナルティ開始）")
    print("  - reward: dof_vel_limits: -0.3（速度制限超過ペナルティ）")
    print(f"{'=' * 70}")
    print("【期待される効果】")
    print("  - 実機パラメータ内での動作学習")
    print("  - sim-to-real gapの削減")
    print("  - 速度飽和による性能劣化の防止")
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
