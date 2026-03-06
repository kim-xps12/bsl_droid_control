#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V4）

============================================================
【EXP007 V4: 交互歩行・大股歩行への改善】
============================================================

【V3の成功と課題】
V3は静止ポリシー問題を解決し、0.246 m/sで前進するようになった。
しかし以下の問題が残った:
1. hip_pitch相関 +0.886（両脚同期、理想は-1.0の交互）
2. 歩幅が小刻み（hip_pitch可動域 約8°）
3. 接地検出の問題（contact_threshold=0.025mが厳しすぎる）
   → 足先Z座標の実測値は0.063-0.071m（閾値の2.5倍）

【V4での対策】
| パラメータ             | V3値       | V4値        | 変更理由                        |
|-----------------------|------------|-------------|--------------------------------|
| contact_threshold     | 0.025 m    | 0.08 m      | 接地検出を修正（実測値に基づく） |
| single_foot_contact   | 0.3        | 0.8         | 交互歩行誘導を大幅強化           |
| gait_frequency        | 1.5 Hz     | 1.0 Hz      | 歩幅拡大のため周波数低下         |
| action_rate           | -0.01      | -0.005      | 大きな関節動作を許容             |
| feet_air_time         | 1.0        | 1.5         | 滞空時間報酬を強化               |
| alive                 | 0.0        | 0.03        | 小量復活（安定性補助）           |
| dof_vel               | -          | -0.001      | 関節速度ペナルティ追加（新規）   |

【期待される効果】
1. 接地検出の修正: contact_threshold=0.08mでsingle_foot_contactが機能
2. 交互歩行の実現: single_foot_contact=0.8で片足接地を強く誘導
3. 歩幅の拡大: gait_frequency=1.0Hz + action_rate緩和で大きな動作
4. 動作のスムーズ化: dof_vel追加で高速振動を抑制
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
    # V4: 交互歩行・大股歩行への改善
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.25,  # 速度追従のガウシアン幅
        "base_height_target": 0.20,  # 目標胴体高さ（BSL-Droid用に調整）
        "swing_height_target": 0.03,  # 遊脚の目標高さ
        "gait_frequency": 1.0,  # 歩行周波数（V3: 1.5Hz → V4: 1.0Hz、歩幅拡大）
        "contact_threshold": 0.08,  # 接地判定閾値（V3: 0.025m → V4: 0.08m、実測値に基づく修正）
        "reward_scales": {
            # ============================================================
            # 【主報酬】速度追従
            # ============================================================
            "tracking_lin_vel": 1.5,  # 線速度追従
            "tracking_ang_vel": 0.5,  # 角速度追従
            # ============================================================
            # 【歩行品質報酬】（V4: 交互歩行強化）
            # ============================================================
            "feet_air_time": 1.5,  # 滞空時間報酬（V3: 1.0 → V4: 1.5、強化）
            "contact": 0.2,  # 接地フェーズ整合性
            "alive": 0.03,  # 生存報酬（V3: 0.0 → V4: 0.03、小量復活）
            # 【V4強化】片足接地報酬
            "single_foot_contact": 0.8,  # 交互歩行誘導（V3: 0.3 → V4: 0.8、大幅強化）
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
            "feet_swing_height": -5.0,  # 遊脚高さ
            "contact_no_vel": -0.1,  # 接地時足速度
            "hip_pos": -0.5,  # 股関節位置（開脚抑制）
            # 速度未達ペナルティ
            "velocity_deficit": -0.5,  # 目標速度を下回るとペナルティ
            # ============================================================
            # 【エネルギー効率ペナルティ】（V4: 動作スムーズ化）
            # ============================================================
            "torques": -1e-5,  # トルクペナルティ
            "action_rate": -0.005,  # アクション変化率（V3: -0.01 → V4: -0.005、大動作許容）
            "dof_acc": -2.5e-7,  # 関節加速度
            "dof_vel": -0.001,  # 関節速度（V4新規、高速振動抑制）
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
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V4)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v4")
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
    print("EXP007 V4: 交互歩行・大股歩行への改善")
    print(f"{'=' * 70}")
    print("【V3の成功と課題】")
    print("  成功: 静止ポリシー問題を解決、0.246 m/sで前進")
    print("  課題: hip_pitch相関 +0.886（両脚同期、理想は-1.0）")
    print("        歩幅が小刻み、接地検出の問題")
    print(f"{'=' * 70}")
    print("【V4での対策】")
    print("  - contact_threshold: 0.025 → 0.08 m（接地検出修正）")
    print("  - single_foot_contact: 0.3 → 0.8（交互歩行強化）")
    print("  - gait_frequency: 1.5 → 1.0 Hz（歩幅拡大）")
    print("  - action_rate: -0.01 → -0.005（大動作許容）")
    print("  - feet_air_time: 1.0 → 1.5（滞空時間強化）")
    print("  - alive: 0.0 → 0.03（小量復活）")
    print("  - dof_vel: 新規追加 -0.001（高速振動抑制）")
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
