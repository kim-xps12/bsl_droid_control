#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V13）

============================================================
【EXP007 V13: V1回帰 + ゆっくり大股歩行】
============================================================

【V12の結果と課題】
V12では、hip_pos(-0.3)とfoot_flat(-0.3)を復活させて歩行品質改善を試みた。

数値上の改善:
- hip_pitch相関: +0.245 → -0.208（交互方向へ）
- 左hip_pitch range: 0.218 → 0.617 rad（約3倍）

改善されなかった問題（目視確認）:
- 内股: hip_rollは依然としてデフォルト(0)を跨がない
- 爪先立ち: 右ankle_pitchの逸脱は+28°→+29°と微増
- 定期的に胴体が大きく傾く
- ストライドと足上げ幅が小さい

【V1との比較から見える問題】
V1は以下の点で健全な歩行をしていた:
1. hip_rollがデフォルト(0)を跨いでいた（内股ではなかった）
2. ankle_pitchが安定（爪先立ちではなかった）
3. 動きが適度（DOF range sum: 2.290 rad）

V2-V12で加えた変更の副作用で、これらが失われた。

【V13の改善方針】
「余計なこと」をやめ、**V1に近い設定に戻す** + **さらにゆっくり大股へ調整**

| パラメータ | V12値 | V13値 | 変更理由 |
|-----------|-------|-------|---------|
| action_rate | なし | -0.01 | 【復活】急激な動きを抑制、V1と同じ |
| hip_pos | -0.3 | -0.5 | 【強化】内股抑制を強化、V1と同じ |
| tracking_sigma | 0.10 | 0.25 | 【緩和】緩やかな追従、V1と同じ |
| gait_frequency | 1.0 Hz | 0.8 Hz | 【調整】さらにゆっくり歩行（周期1.25秒） |
| single_foot_contact | 1.0 | 0.5 | やや緩和、過剰な動作を抑制 |
| lin_vel_x_range | [0.15, 0.25] | [0.12, 0.18] | 速度を下げて安定性優先 |
| foot_flat | -0.3 | -0.5 | 【強化】爪先立ち抑制を強化 |

報酬項目数: 17 → 18（action_rate復活）

【gait_frequency=0.8Hzの設計根拠】
歩行の基本関係: 速度 ≈ 歩幅 × 周波数

| バージョン | 周波数 | 周期 | 目標速度 | 想定歩幅 |
|-----------|--------|------|---------|---------|
| V1 | 1.5 Hz | 0.67秒 | 0.2-0.3 m/s | 0.13-0.20 m/step |
| V12 | 1.0 Hz | 1.0秒 | 0.15-0.25 m/s | 0.15-0.25 m/step |
| V13 | 0.8 Hz | 1.25秒 | 0.12-0.18 m/s | 0.15-0.23 m/step |

「ゆっくり大股」の実現:
- 速度を下げる: 0.15-0.25 → 0.12-0.18 m/s
- 歩幅を維持: 約0.15-0.20 m/step
- 周波数を下げる: 1.0 → 0.8 Hz

【成功基準】
| 指標 | V12値 | V13目標 | 判定基準 |
|------|-------|---------|---------|
| X速度 | 0.213 m/s | > 0.15 m/s | 維持（多少低下はOK） |
| hip_roll L min | -0.422 | > -0.20 | 内股解消 |
| hip_roll L max | -0.000 | > 0.00 | デフォルト跨ぐ |
| ankle_pitch R max | 1.301 | < 0.90 | 爪先立ち改善 |
| DOF range sum | 6.851 rad | 2-4 rad | 動きの適正化 |
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
    # V13: V1回帰 + ゆっくり大股歩行（18項目）
    # ============================================================
    # V12からの変更:
    # 1. action_rate復活: -0.01（V1と同じ、急激な動きを抑制）
    # 2. hip_pos強化: -0.3 → -0.5（V1と同じ、内股抑制）
    # 3. tracking_sigma緩和: 0.10 → 0.25（V1と同じ、緩やかな追従）
    # 4. gait_frequency低下: 1.0 → 0.8 Hz（ゆっくり歩行）
    # 5. single_foot_contact緩和: 1.0 → 0.5（過剰な動作を抑制）
    # 6. lin_vel_x_range低下: [0.15, 0.25] → [0.12, 0.18]（速度抑制）
    # 7. foot_flat強化: -0.3 → -0.5（爪先立ち抑制）
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.25,  # ★V12: 0.10 → V13: 0.25（V1と同じ、緩やかな追従）
        "base_height_target": 0.20,  # BSL-Droid向け
        "swing_height_target": 0.04,  # V12と同じ
        "gait_frequency": 0.8,  # ★V12: 1.0 → V13: 0.8 Hz（ゆっくり歩行、周期1.25秒）
        "contact_threshold": 0.08,  # V4で修正済み
        "air_time_offset": 0.20,  # V12と同じ
        "reward_scales": {
            # ============================================================
            # 【主報酬】Unitreeと同等
            # ============================================================
            "tracking_lin_vel": 1.5,  # V3-V4で実証済み
            "tracking_ang_vel": 0.5,  # Unitreeと同じ
            # ============================================================
            # 【歩行品質報酬】V1回帰
            # ============================================================
            "contact": 0.2,  # Unitree: 0.18、歩行フェーズ整合性
            "single_foot_contact": 0.5,  # ★V12: 1.0 → V13: 0.5（やや緩和、過剰な動作を抑制）
            "feet_air_time": 1.5,  # V3-V4で実証済み
            "alive": 0.03,  # 控えめに設定
            # ============================================================
            # 【安定性ペナルティ】Unitree値を使用
            # ============================================================
            "lin_vel_z": -2.0,  # Unitreeと同じ
            "ang_vel_xy": -0.05,  # Unitree値
            "orientation": -0.5,  # V3-V4レベル
            "base_height": -5.0,  # サーベイ6.2推奨値
            # ============================================================
            # 【歩行品質ペナルティ】
            # ============================================================
            "feet_swing_height": -5.0,  # V3-V4レベル
            "contact_no_vel": -0.1,  # Unitreeと同等
            "velocity_deficit": -2.0,  # V6で効果実証済み
            # ============================================================
            # 【V1回帰報酬】V1の設定に戻す
            # ============================================================
            "hip_pos": -0.5,  # ★V12: -0.3 → V13: -0.5（V1と同じ、内股抑制強化）
            "foot_flat": -0.5,  # ★V12: -0.3 → V13: -0.5（爪先立ち抑制強化）
            "action_rate": -0.01,  # ★【復活】V1と同じ、急激な動きを抑制
            # ============================================================
            # 【エネルギー効率ペナルティ】
            # ============================================================
            "torques": -1e-5,  # Unitreeと同等
            "dof_acc": -2.5e-7,  # Unitreeと同等
        },
    }
    # 報酬項目数: 18（V12の17から1項目追加: action_rate復活）

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.12, 0.18],  # ★V12: [0.15, 0.25] → V13: [0.12, 0.18]（速度抑制）
        "lin_vel_y_range": [0, 0],  # 横移動なし
        "ang_vel_range": [0, 0],  # まずは直進のみ（複雑さ回避）
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main() -> None:
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V13)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v13")
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
    print("EXP007 V13: V1回帰 + ゆっくり大股歩行")
    print(f"{'=' * 70}")
    print("【V12の課題（目視確認）】")
    print("  - 脚が鉛直ではなく内股になっている")
    print("  - 右足だけ爪先立ちで接地している")
    print("  - 足を動かすのが小刻みでストライドも上げ幅も小さい")
    print("  - 定期的に胴体が大きく傾く")
    print(f"{'=' * 70}")
    print("【V13の改善方針: V1回帰 + ゆっくり大股】")
    print("  1. action_rate復活(-0.01): 急激な動きを抑制（V1と同じ）")
    print("  2. hip_pos強化(-0.3→-0.5): 内股抑制を強化（V1と同じ）")
    print("  3. tracking_sigma緩和(0.10→0.25): 緩やかな追従（V1と同じ）")
    print("  4. gait_frequency低下(1.0→0.8Hz): ゆっくり歩行（周期1.25秒）")
    print("  5. single_foot_contact緩和(1.0→0.5): 過剰な動作を抑制")
    print("  6. lin_vel_x_range低下([0.15,0.25]→[0.12,0.18]): 速度抑制")
    print("  7. foot_flat強化(-0.3→-0.5): 爪先立ち抑制を強化")
    print(f"{'=' * 70}")
    print("【報酬項目数】")
    print("  V12: 17項目 → V13: 18項目（+1: action_rate復活）")
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
