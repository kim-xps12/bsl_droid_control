#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V11）

============================================================
【EXP007 V11: 最小有効報酬セットによる静止ポリシー回避】
============================================================

【V10の結果と課題】
V10は静止ポリシーへの回帰という深刻な問題が発生：
- X速度: 0.092 m/s → 0.006 m/s（93%減、静止ポリシー）
- 原因: ペナルティの累積効果（5項目を同時に強化/追加）
- 報酬最大化（98.12）≠ 歩行品質最大化

【重要な教訓（V1-V10から）】
1. ペナルティの強化は一度に1-2項目まで
2. 複数のペナルティを同時に強化しない
3. 「動いていた頃」の設定をベースに最小限の調整を行う
4. 報酬の絶対値ではなく、実際の動作で評価する

【V11の改善方針】
V10レポートの「提案3: 最小有効報酬セット」に基づき、
報酬設計を大幅に簡素化する（22項目 → 15項目）。

設計原則:
1. Unitree G1/H1の成功パターンに準拠（13-14項目）
2. V3-V4の実績に基づく設定（16-17項目で0.15-0.19 m/s達成）
3. ペナルティ累積効果の回避

【削除する要素（7項目）】
| 削除要素              | 理由                     |
|----------------------|--------------------------|
| symmetry             | hip_pitch同期誘発         |
| hip_pitch_antiphase_v2 | 効果なし               |
| both_legs_active     | 効果不明                  |
| feet_stumble         | 静止誘発                  |
| hip_pos              | V3になし                  |
| action_rate          | ペナルティ累積            |
| dof_vel              | V3になく不要              |

【ペナルティ緩和】
| パラメータ         | V10値   | V11値   | 変更理由           |
|-------------------|---------|---------|-------------------|
| ang_vel_xy        | -0.1    | -0.05   | V3-V4レベルに戻す  |
| orientation       | -1.0    | -0.5    | V3-V4レベルに戻す  |
| feet_swing_height | -10.0   | -5.0    | V3-V4レベルに戻す  |
| tracking_ang_vel  | 1.0     | 0.5     | Unitree値に戻す    |
| swing_height_target| 0.05   | 0.03    | V3-V4レベルに戻す  |

【速度目標設定】
- lin_vel_x_range: [0.15, 0.25]（V7レベル、動作実績あり）
- ang_vel_range: [0, 0]（まずは直進のみ、複雑さ回避）

【成功基準】
| 指標 | V10値 | V11目標 | 判定基準 |
|------|-------|---------|---------|
| X速度 | 0.006 m/s | > 0.15 m/s | V3-V4レベルに回復 |
| hip_pitch相関 | +0.449 | < 0 | 交互歩行の回復 |
| 報酬項目数 | 22 | 15 | 簡素化 |
| エピソード長 | 1001 | > 900 | 安定性維持 |
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
    # V11: 最小有効報酬セット（15項目）
    # ============================================================
    # 設計原則:
    # 1. Unitree G1/H1の成功パターンに準拠（13-14項目）
    # 2. V3-V4の実績に基づく設定（16-17項目で0.15-0.19 m/s達成）
    # 3. ペナルティ累積効果の回避
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.10,  # V6で効果実証（静止回避）
        "base_height_target": 0.20,  # BSL-Droid向け
        "swing_height_target": 0.03,  # ★V3-V4値に戻す（V10: 0.05は過剰）
        "gait_frequency": 1.0,  # V4値
        "contact_threshold": 0.08,  # V4で修正済み
        "air_time_offset": 0.25,  # デフォルト
        "reward_scales": {
            # ============================================================
            # 【主報酬】Unitreeと同等
            # ============================================================
            "tracking_lin_vel": 1.5,  # V3-V4で実証済み
            "tracking_ang_vel": 0.5,  # ★Unitreeと同じ（V10の1.0は過剰）
            # ============================================================
            # 【歩行品質報酬】Unitree方式 + V3-V4実証済み要素
            # ============================================================
            "contact": 0.2,  # Unitree: 0.18、歩行フェーズ整合性
            "single_foot_contact": 0.8,  # V4で実証済み、交互歩行の核心
            "feet_air_time": 1.5,  # V3-V4で実証済み
            "alive": 0.03,  # 控えめに設定（Unitreeの0.15は静止誘発リスク）
            # ============================================================
            # 【安定性ペナルティ】Unitree値を使用
            # ============================================================
            "lin_vel_z": -2.0,  # Unitreeと同じ
            "ang_vel_xy": -0.05,  # ★Unitree値に戻す（V10: -0.1は過剰）
            "orientation": -0.5,  # ★V3-V4レベルに戻す（V10: -1.0は過剰）
            "base_height": -5.0,  # サーベイ6.2推奨値
            # ============================================================
            # 【歩行品質ペナルティ】
            # ============================================================
            "feet_swing_height": -5.0,  # ★V3-V4レベルに戻す（V10: -10.0は過剰）
            "contact_no_vel": -0.1,  # Unitreeと同等
            "velocity_deficit": -2.0,  # V6で効果実証済み
            # ============================================================
            # 【エネルギー効率ペナルティ】
            # ============================================================
            "torques": -1e-5,  # Unitreeと同等
            "dof_acc": -2.5e-7,  # Unitreeと同等
        },
    }
    # 報酬項目数: 15（V10の22から7項目削減）
    # 削除した要素:
    # - symmetry（hip_pitch同期誘発）
    # - hip_pitch_antiphase_v2（効果なし）
    # - both_legs_active（効果不明）
    # - feet_stumble（静止誘発）
    # - hip_pos（V3になし）
    # - action_rate（ペナルティ累積）
    # - dof_vel（V3になく不要）

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.15, 0.25],  # ★V7値（動作実績あり）
        "lin_vel_y_range": [0, 0],  # 横移動なし
        "ang_vel_range": [0, 0],  # ★まずは直進のみ（複雑さ回避）
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main() -> None:
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V11)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v11")
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
    print("EXP007 V11: 最小有効報酬セットによる静止ポリシー回避")
    print(f"{'=' * 70}")
    print("【V10の失敗原因】")
    print("  - X速度: 0.006 m/s（静止ポリシーへの回帰）")
    print("  - 原因: ペナルティの累積効果（5項目を同時に強化/追加）")
    print("  - 報酬最大化（98.12）≠ 歩行品質最大化")
    print(f"{'=' * 70}")
    print("【V11の改善方針: 最小有効報酬セット】")
    print("  1. 報酬項目数: 22 → 15（7項目削減）")
    print("  2. Unitree G1/H1の成功パターンに準拠")
    print("  3. V3-V4の実績に基づく設定")
    print("  4. ペナルティ累積効果の回避")
    print(f"{'=' * 70}")
    print("【削除した要素（7項目）】")
    print("  - symmetry（hip_pitch同期誘発）")
    print("  - hip_pitch_antiphase_v2（効果なし）")
    print("  - both_legs_active（効果不明）")
    print("  - feet_stumble（静止誘発）")
    print("  - hip_pos、action_rate、dof_vel")
    print(f"{'=' * 70}")
    print("【ペナルティ緩和】")
    print("  - ang_vel_xy: -0.1 → -0.05")
    print("  - orientation: -1.0 → -0.5")
    print("  - feet_swing_height: -10.0 → -5.0")
    print("  - tracking_ang_vel: 1.0 → 0.5")
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
