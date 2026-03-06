#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V9）

============================================================
【EXP007 V9: 報酬項目の整理と両脚対称性強化】
============================================================

【V8の結果と課題】
V8はhip_pitch相関を+0.828から-0.382に改善したが、
hip_pitch_antiphase報酬の設計欠陥により「右脚だけ動かす」局所最適に収束：

- 成功点:
  - hip_pitch相関: -0.382（逆相関を獲得、V7から大幅改善）
  - Roll std: 4.41°（V7の6.21°から29%改善）
  - X速度: 0.108 m/s（目標0.10 m/s達成）

- 課題点:
  - 左右非対称: 右脚hip_pitch range = 0.358 rad、左脚 = 0.237 rad（1.5倍の差）
  - 右脚hip_pitch vel std = 1.137 rad/s、左脚 = 0.521 rad/s（2.2倍の差）
  - Yawドリフト: +113°（V7の-7.56°から15倍悪化）
  - 報酬項目数: 23（V1の14から64%増加、複雑化）

【V9の改善方針】

1. **報酬項目の整理（ロールバック方向）**
   - 削除: step_length, foot_flat, ankle_roll, hip_pitch_antiphase（4項目）
   - 報酬項目数: 23 → 21に削減（約9%減）

2. **hip_pitch_antiphase報酬の修正**
   - V8版の欠陥: 片脚静止でも報酬0（ペナルティなし）
   - V9版: 両脚が動いている場合のみ報酬を付与
   - 「右脚だけ動かす」局所最適を回避

3. **両脚動作報酬の新規追加**
   - both_legs_active: min(左脚速度, 右脚速度)を報酬化
   - 片脚静止戦略を明示的に抑制

4. **対称性・Yaw追従の強化**
   - symmetry: 0.3 → 1.0（3.3倍強化）
   - tracking_ang_vel: 0.5 → 1.0（2倍強化）
   - ang_vel_range: [0, 0] → [-0.2, 0.2]（Yaw訓練追加）

【V9パラメータ変更一覧】
| パラメータ                 | V8値       | V9値         | 変更理由                          |
|---------------------------|------------|--------------|----------------------------------|
| step_length               | 0.3        | 削除         | 効果未確認、複雑化の原因          |
| foot_flat                 | -5.0       | 削除         | 設計ミス（ankle_pitchのみ）       |
| ankle_roll                | -2.0       | 削除         | hip_posと重複                     |
| hip_pitch_antiphase       | 0.8        | 削除         | V9で修正版に置換                  |
| hip_pitch_antiphase_v2    | (なし)     | 0.8          | 【新規】両脚動作必須版            |
| both_legs_active          | (なし)     | 0.5          | 【新規】両脚動作報酬              |
| symmetry                  | 0.3        | 1.0          | ★強化（3.3倍）                    |
| tracking_ang_vel          | 0.5        | 1.0          | ★強化（2倍）、Yawドリフト対策    |
| ang_vel_range             | [0, 0]     | [-0.2, 0.2]  | Yaw訓練追加                       |

【成功基準】
| 指標 | V8値 | V9目標 | 判定基準 |
|------|------|--------|---------|
| X速度 | 0.108 m/s | > 0.10 m/s | 維持 |
| hip_pitch相関 | -0.382 | < -0.3 | 逆相関を維持 |
| hip_pitch L/R比 | 1.5倍 | < 1.2倍 | 左右対称性改善 |
| Yawドリフト | +113° | < ±30° | 旋回抑制 |
| 報酬項目数 | 23 | 21 | 複雑化抑制 |
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
        "num_obs": 50,  # Unitree方式の観測空間
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }

    # ============================================================
    # V9: 報酬項目の整理と両脚対称性強化
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.10,  # V6から継続（ガウシアン鋭化）
        "base_height_target": 0.20,  # 目標胴体高さ
        "swing_height_target": 0.03,  # 遊脚の目標高さ
        "gait_frequency": 0.8,  # 歩行周波数（V6と同じ）
        "contact_threshold": 0.08,  # 接地判定閾値
        "air_time_offset": 0.20,  # V7から継続
        "reward_scales": {
            # ============================================================
            # 【主報酬】速度追従
            # ============================================================
            "tracking_lin_vel": 1.5,  # 線速度追従
            "tracking_ang_vel": 1.0,  # ★強化: V8: 0.5 → V9: 1.0（Yawドリフト対策）
            # ============================================================
            # 【歩行品質報酬】
            # ============================================================
            "feet_air_time": 1.5,  # 滞空時間報酬
            "contact": 0.3,  # V7から継続
            "alive": 0.03,  # 生存報酬
            # 片足接地報酬
            "single_foot_contact": 1.0,  # V7から継続
            # 左右対称性報酬
            "symmetry": 1.0,  # ★強化: V8: 0.3 → V9: 1.0（3.3倍）
            # ★【新規】hip_pitch速度逆相関報酬・修正版（V9追加）
            # V8版の欠陥（片脚静止でも報酬0）を修正
            "hip_pitch_antiphase_v2": 0.8,  # 両脚動作必須版
            # ★【新規】両脚動作報酬（V9追加）
            "both_legs_active": 0.5,  # 両脚が共に動いていることを報酬化
            # ============================================================
            # 削除した報酬:
            #   - step_length: 効果未確認、複雑化の原因
            #   - hip_pitch_antiphase: V9でv2に置換
            # ============================================================
            # ============================================================
            # 【安定性ペナルティ】（Unitree方式）
            # ============================================================
            "lin_vel_z": -2.0,  # Z軸速度ペナルティ
            "ang_vel_xy": -0.1,  # V8から継続
            "orientation": -1.0,  # V8から継続
            "base_height": -5.0,  # 高さ維持
            # ============================================================
            # 【歩行品質ペナルティ】
            # ============================================================
            "feet_swing_height": -5.0,  # 遊脚高さ
            "contact_no_vel": -0.1,  # 接地時足速度
            "hip_pos": -0.5,  # 股関節位置（開脚抑制）
            # 速度未達ペナルティ
            "velocity_deficit": -2.0,  # V6から継続
            # ============================================================
            # 削除した報酬:
            #   - foot_flat: 設計ミス（ankle_pitchのみ対象）
            #   - ankle_roll: hip_posと重複
            # ============================================================
            # ============================================================
            # 【エネルギー効率ペナルティ】
            # ============================================================
            "torques": -1e-5,  # トルクペナルティ
            "action_rate": -0.005,  # アクション変化率
            "dof_acc": -2.5e-7,  # 関節加速度
            "dof_vel": -0.005,  # 関節速度（振動抑制）
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.10, 0.15],  # V8から継続
        "lin_vel_y_range": [0, 0],  # 横移動なし
        "ang_vel_range": [-0.2, 0.2],  # ★新規: Yaw訓練追加（V8: [0, 0]）
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main() -> None:
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V9)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v9")
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
    print("EXP007 V9: 報酬項目の整理と両脚対称性強化")
    print(f"{'=' * 70}")
    print("【V8の成功と残課題】")
    print("  成功: hip_pitch相関-0.382達成（逆相関獲得）")
    print("  課題: 左右非対称（右脚のみ動く）、Yawドリフト+113°")
    print("        報酬項目数23（複雑化）")
    print(f"{'=' * 70}")
    print("【V9の改善方針】")
    print("  1. 報酬項目の整理（23 → 21）")
    print("     - 削除: step_length, foot_flat, ankle_roll")
    print("  2. hip_pitch_antiphase報酬の修正")
    print("     - hip_pitch_antiphase_v2: 両脚動作必須版")
    print("  3. 両脚動作報酬の新規追加")
    print("     - both_legs_active: 0.5")
    print("  4. 対称性・Yaw追従の強化")
    print("     - symmetry: 0.3 → 1.0（3.3倍）")
    print("     - tracking_ang_vel: 0.5 → 1.0（2倍）")
    print("     - ang_vel_range: [0, 0] → [-0.2, 0.2]（Yaw訓練）")
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
