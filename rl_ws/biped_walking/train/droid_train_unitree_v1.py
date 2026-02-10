#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V1）

============================================================
【EXP007 V1: Unitree RL Gym参考実装】
============================================================

【設計方針】
Unitree RL Gym（G1/H1）の報酬設計を参考に、関節空間制御に戻す。
exp006（足先空間）の失敗から学び、Unitreeの成功事例を導入。

【Unitreeから導入した要素】
1. contact報酬: 歩行フェーズと接地状態の整合性（0.2）
2. feet_swing_height報酬: 遊脚期の足上げ高さ制御（-5.0）
3. contact_no_vel報酬: 接地時の足滑り防止（-0.1）
4. hip_pos報酬: 股関節位置の過度な変位抑制（-0.5）
5. 観測空間にbase_lin_vel, leg_phase追加（49次元）

【BSL-Droid向け調整】
- base_height_target: 0.78m → 0.20m（脚長の違い）
- swing_height_target: 0.08m → 0.03m（ロボットサイズに比例）
- feet_swing_height: -20.0 → -5.0（Unitreeより緩和）
- orientation: -1.0 → -0.5（小型ロボット向け緩和）

【報酬項目数】
15項目（Unitree G1/H1と同等の構成）
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
    # Unitree RL Gym参考 報酬設計
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.25,  # 速度追従のガウシアン幅
        "base_height_target": 0.20,  # 目標胴体高さ（BSL-Droid用に調整）
        "swing_height_target": 0.03,  # 遊脚の目標高さ（BSL-Droid用に調整）
        "gait_frequency": 1.5,  # 歩行周波数 Hz
        "contact_threshold": 0.025,  # 接地判定閾値 m
        "reward_scales": {
            # ============================================================
            # 【主報酬】速度追従（Unitree方式）
            # ============================================================
            "tracking_lin_vel": 1.0,  # 線速度追従（exp関数）
            "tracking_ang_vel": 0.5,  # 角速度追従（exp関数）
            # ============================================================
            # 【歩行品質報酬】（Unitree G1/H1から導入）
            # ============================================================
            "feet_air_time": 1.0,  # 滞空時間報酬
            "contact": 0.2,  # 接地フェーズ整合性（Unitree: 0.18）
            "alive": 0.1,  # 生存報酬（Unitree: 0.15、控えめに設定）
            # ============================================================
            # 【安定性ペナルティ】（Unitree方式）
            # ============================================================
            "lin_vel_z": -2.0,  # Z軸速度ペナルティ
            "ang_vel_xy": -0.05,  # XY角速度ペナルティ
            "orientation": -0.5,  # 姿勢ペナルティ（Unitree: -1.0、BSL-Droid向け緩和）
            "base_height": -5.0,  # 高さ維持（Unitree: -10.0、BSL-Droid向け緩和）
            # ============================================================
            # 【歩行品質ペナルティ】（Unitree G1/H1から導入）
            # ============================================================
            "feet_swing_height": -5.0,  # 遊脚高さ（Unitree: -20.0、大幅緩和）
            "contact_no_vel": -0.1,  # 接地時足速度（Unitree H1: -0.2、緩和）
            "hip_pos": -0.5,  # 股関節位置（開脚抑制）
            # ============================================================
            # 【エネルギー効率ペナルティ】
            # ============================================================
            "torques": -1e-5,  # トルクペナルティ（Unitree: -1e-5）
            "action_rate": -0.01,  # アクション変化率（Unitree: -0.01）
            "dof_acc": -2.5e-7,  # 関節加速度（Unitree: -2.5e-7）
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.2, 0.3],  # 目標前進速度（低速から開始）
        "lin_vel_y_range": [0, 0],  # 横移動なし
        "ang_vel_range": [0, 0],  # 旋回なし
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main() -> None:
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V1)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v1")
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
    print("EXP007 V1: Unitree RL Gym参考実装")
    print(f"{'=' * 60}")
    print("【Unitreeから導入した報酬項目】")
    print("- contact: 歩行フェーズと接地状態の整合性")
    print("- feet_swing_height: 遊脚期の足上げ高さ制御")
    print("- contact_no_vel: 接地時の足滑り防止")
    print("- hip_pos: 股関節位置の過度な変位抑制")
    print(f"{'=' * 60}")
    print("【BSL-Droid向け調整】")
    print("- base_height_target: 0.20m（Unitree G1: 0.78m）")
    print("- swing_height_target: 0.03m（Unitree G1: 0.08m）")
    print("- feet_swing_height: -5.0（Unitree: -20.0）")
    print(f"{'=' * 60}")
    print(f"観測空間: {obs_cfg['num_obs']}次元")
    print(f"行動空間: {env_cfg['num_actions']}次元")
    print(f"報酬項目数: {len(reward_cfg['reward_scales'])}")
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
