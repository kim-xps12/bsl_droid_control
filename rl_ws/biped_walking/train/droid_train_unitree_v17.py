#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V17）

============================================================
【EXP007 V17: 「ゆったり大股で歩く」の実現】
============================================================

【設計方針】
V16は数値的には改善したが、目視で「振動・片脚固定」問題が継続した。
根本原因は左右hip_pitch可動域の非対称（L=0.524 vs R=0.197 rad, 2.7倍差）。
V17では**左右対称性の明示的強化**と**階層的報酬設計**により
「ゆったり大股で歩く」を実現する。

【問題の根本原因】（V16レポートより）
1. 左右非対称: hip_pitch可動域 L=0.524 vs R=0.197 rad（2.7倍の差）
2. 接地検出失敗: feet_air_time=0, single_foot_contact=0（100%空中判定）
3. Yawドリフト悪化: V15 -4.92° → V16 +13.57°
4. 既存symmetry報酬はhip_pitchを除外しているため問題に対処できない

【V16からの変更点】（4点、項目数維持18項目）
1. symmetry_range: (なし) → 0.5（hip_pitch可動域の左右対称性）★追加
2. step_length: 0.5 → 削除（symmetry_rangeで対称性確保すれば自然と歩幅が出る）
3. tracking_ang_vel: 0.5 → 0.8（Yawドリフト対策）
4. contact_threshold: 0.025 → 0.035 m（接地検出改善）
5. dof_vel: -0.005 → -0.01（振動抑制強化）

【報酬項目数】18項目（V16と同数、推奨範囲15-17に近い）
- V16: 18項目（step_length含む）
- V17: 18項目（step_length削除、symmetry_range追加）

【階層的報酬設計】（学術サーベイに基づく）
- Tier 1（主報酬）: tracking_lin_vel=1.5, tracking_ang_vel=0.8
- Tier 2（歩行品質）: feet_air_time=1.5, symmetry_range=0.5, contact, single_foot_contact
- Tier 3（安定性）: lin_vel_z, ang_vel_xy, orientation, base_height
- Tier 4（動作品質）: feet_swing_height, contact_no_vel, hip_pos, velocity_deficit
- Tier 5（エネルギー）: torques, action_rate, dof_acc, dof_vel

【報酬バランス検証】
- 正報酬合計（理想状態）: ~3.8
- ペナルティ合計（通常歩行）: ~-0.9
- 純報酬: ~2.9（正、動作を奨励）

【成功基準】
- X速度: 0.15-0.20 m/s
- hip_pitch可動域 L/R比: < 1.5x（V16: 2.7x）
- Yawドリフト(10s): < 5°（V16: +13.57°）
- feet_air_time報酬: > 0.1（V16: 0.0）
- 目視評価: ゆったり大股

【学術的根拠】
- 階層的報酬構造: STRIDE (arXiv:2502.04692)
- 対称性強制: Leveraging Symmetry in RL (IROS 2024, arXiv:2403.17320)
- 報酬バランス: ペナルティ合計 < 正報酬合計（静止ポリシー防止）

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
    # V17: 「ゆったり大股で歩く」- 階層的報酬設計
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.25,  # V16と同じ
        "base_height_target": 0.20,  # V16と同じ
        "swing_height_target": 0.04,  # V16と同じ
        "gait_frequency": 1.0,  # V16と同じ
        "contact_threshold": 0.035,  # ★変更: V16 0.025 → V17 0.035 m（接地検出改善）
        "air_time_offset": 0.3,  # V16と同じ
        "reward_scales": {
            # ============================================================
            # 【Tier 1: 主報酬】速度追従
            # ============================================================
            "tracking_lin_vel": 1.5,  # V16と同じ（動く動機を維持）
            "tracking_ang_vel": 0.8,  # ★変更: V16 0.5 → V17 0.8（Yawドリフト対策）
            # ============================================================
            # 【Tier 2: 歩行品質報酬】
            # ============================================================
            "feet_air_time": 1.5,  # V16と同じ
            "contact": 0.2,  # V16と同じ
            "single_foot_contact": 0.3,  # V16と同じ（片足接地報酬）
            "symmetry_range": 0.5,  # ★新規: hip_pitch可動域の左右対称性（step_lengthと入替）
            # step_length: 削除 - 対称性確保で自然と歩幅が出る、V16で効果なし
            # ============================================================
            # 【Tier 3: 安定性ペナルティ】
            # ============================================================
            "lin_vel_z": -2.0,  # V16と同じ
            "ang_vel_xy": -0.05,  # V16と同じ
            "orientation": -0.5,  # V16と同じ
            "base_height": -5.0,  # V16と同じ
            # ============================================================
            # 【Tier 4: 動作品質ペナルティ】
            # ============================================================
            "feet_swing_height": -5.0,  # V16と同じ
            "contact_no_vel": -0.1,  # V16と同じ
            "hip_pos": -0.5,  # V16と同じ
            "velocity_deficit": -0.5,  # V16と同じ（速度未達ペナルティ）
            # ============================================================
            # 【Tier 5: エネルギー効率・振動抑制ペナルティ】
            # ============================================================
            "torques": -1e-5,  # V16と同じ
            "action_rate": -0.005,  # V16と同じ（大きな動作許容）
            "dof_acc": -2.5e-7,  # V16と同じ
            "dof_vel": -0.01,  # ★変更: V16 -0.005 → V17 -0.01（振動抑制強化）
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
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V17)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v17")
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
    print("EXP007 V17: 「ゆったり大股で歩く」の実現")
    print(f"{'=' * 70}")
    print("【V16からの変更点】（5点）")
    print("  1. symmetry_range: (なし) → 0.5（hip_pitch可動域の左右対称性）★新規")
    print("  2. tracking_ang_vel: 0.5 → 0.8（Yawドリフト対策）")
    print("  3. contact_threshold: 0.025 → 0.035 m（接地検出改善）")
    print("  4. dof_vel: -0.005 → -0.01（振動抑制強化）")
    print("  5. step_length: 0.5 → 0.3（非対称収束防止）")
    print(f"{'=' * 70}")
    print("【階層的報酬設計】")
    print("  Tier 1（主報酬）: tracking_lin_vel=1.5, tracking_ang_vel=0.8")
    print("  Tier 2（歩行品質）: feet_air_time=1.5, symmetry_range=0.5")
    print("  Tier 3-5: 安定性・動作品質・エネルギーペナルティ")
    print(f"{'=' * 70}")
    print("【成功基準】")
    print("  - X速度: 0.15-0.20 m/s")
    print("  - hip_pitch可動域 L/R比: < 1.5x（V16: 2.7x）")
    print("  - Yawドリフト(10s): < 5°（V16: +13.57°）")
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
