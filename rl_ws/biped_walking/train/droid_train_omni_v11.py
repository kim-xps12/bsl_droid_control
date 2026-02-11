#!/usr/bin/env python3
"""
BSL-Droid Simplified V2 歩行学習スクリプト（exp009 V11）

============================================================
【EXP009 V11: vyコマンド範囲の最適化（±0.15 → ±0.20 m/s）】
============================================================

exp009 V10をベースに、以下の変更を実施:
1. lin_vel_y_range: [-0.15, 0.15] → [-0.20, 0.20]（vy範囲の最適化）

【変更根拠】
- V10でvy範囲±0.15は狭すぎ、横方向移動能力が喪失（増幅率0.63x）
- V10で報酬コンポーネントが全面低下（tracking_lin_vel -50%等）
  → penalty/positive比が実効的にoverkill域に移行
- V9(±0.3)/V10(±0.15)/V11(±0.20)の3点データでvy範囲の応答曲線を取得
- 到達率70-80%域を狙い、横方向移動能力を回復しつつ横揺れ抑制を維持
- knee Kp=50は維持（V9から継承）
- 1変更1検証原則を遵守（報酬構成・action_rate scaleは変更なし）

【PD制御ゲイン】
- 全関節デフォルト: Kp=35.0, Kd=2.0
- knee_pitch: Kp=50.0（V9から継承）
- ankle_pitch: Kd=5.0（exp008 V25から継承、PDアンダーダンピング修正）

【報酬構成】
- 16項目（V10と同じ、報酬scale変更なし）

【成功指標】
- vy増幅率 0.8〜1.2x（V10: 0.63x）
- 方向固定Yawドリフト（FWD 20s） < 30°（V10: +4.25°）
- 前進追従率 > 85%（V10: 87%）
- hip_pitch相関 < -0.2（V10: -0.249）
- 左右非対称度（hip_pitch） < 10%（V10: -15.1%）
- tracking_lin_vel報酬回復（V10: 0.479 → 目標 > 0.7）

【参考文献】
- exp009_report_v10.md（V11推奨の根拠）
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
    urdf_path = rl_ws_dir / "assets" / "bsl_droid_simplified_v2.urdf"

    # V9と同じ初期姿勢
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
        # V9から継承: knee_pitchのKpを50.0に増加（1.5Hz歩行へのknee応答性強化）
        "kp_overrides": {
            "left_knee_pitch_joint": 50.0,
            "right_knee_pitch_joint": 50.0,
        },
        # V25から継承: ankle_pitchのkdを個別に設定（PDアンダーダンピング修正）
        "kd_overrides": {
            "left_ankle_pitch_joint": 5.0,
            "right_ankle_pitch_joint": 5.0,
        },
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
        # Contact Sensor使用
        "use_contact_sensor": True,
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
    # EXP009 V11: V10ベース + vyコマンド範囲最適化
    # - 報酬項目数: 16（V10と同じ、報酬scale変更なし）
    # - PD制御: V10と同じ（knee_pitch Kp=50.0維持）
    # - コマンド: lin_vel_y_range [-0.15, 0.15] → [-0.20, 0.20]
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.20,
        "swing_height_target": 0.05,
        "gait_frequency": 1.5,  # V8から継承（1.5 Hz）
        "contact_threshold": 0.05,
        "stance_foot_lateral_min_distance": 0.050,
        "air_time_offset": 0.10,
        "contact_no_vel_dims": 2,  # XYのみ
        "hip_roll_inward_limit": -0.05,  # V14: PDターゲットの内向き制限 [rad]
        "reward_scales": {
            # ============================================================
            # 【主報酬】速度追従
            # ============================================================
            "tracking_lin_vel": 1.5,  # 線速度追従（XY両方向）
            "tracking_ang_vel": 0.5,  # 角速度追従
            # ============================================================
            # 【歩行品質報酬】
            # ============================================================
            "feet_air_time": 0,  # 無効化
            "swing_duration": 2.0,
            "swing_contact_penalty": -0.7,
            "contact": 0.4,
            "single_foot_contact": 0.5,
            "step_length": 0.8,  # 多方向対応済み（コマンド方向の足間距離）
            # ============================================================
            # 【安定性ペナルティ】
            # ============================================================
            "lin_vel_z": -2.0,
            "ang_vel_xy": -0.1,
            "orientation": -2.0,
            "base_height": -5.0,
            # ============================================================
            # 【歩行品質ペナルティ】
            # ============================================================
            "feet_swing_height": -4.0,
            "contact_no_vel": -0.1,
            "hip_yaw_pos": -0.8,
            "stance_hip_roll_target": 0,  # 無効化
            "stance_foot_lateral_position": 0,  # 無効化
            "velocity_deficit": -0.1,  # V2で-0.5→-0.1に修正済み
            # ============================================================
            # 【エネルギー効率ペナルティ】
            # ============================================================
            "action_rate": -0.01,  # V6で-0.005→-0.01に変更済み
            "swing_foot_lateral_velocity": 0,  # 無効化
            "foot_lateral_velocity": -0.5,  # 多方向対応済み（コマンド直交成分）
            "symmetry_range": 0,  # 無効化
            # ============================================================
            # 【exp009変更】base_vel_y無効化
            # exp008ではY方向並進速度を直接ペナルティ化していたが、
            # exp009ではY方向にも速度コマンドを指令するため矛盾する。
            # tracking_lin_velがY方向追従も含むため、別途ペナルティは不要。
            # ============================================================
            "base_vel_y": 0,  # exp009: 無効化（Y方向コマンドと矛盾）
        },
    }

    # ============================================================
    # EXP009 V11: vyコマンド範囲の最適化
    # V10の±0.15は狭すぎ（増幅率0.63x、横方向が実質的に死んでいる）。
    # V9(±0.3)/V10(±0.15)の中間±0.20で到達率70-80%域を狙う。
    # ============================================================
    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [-0.3, 0.3],  # exp009: 前後対称（V10と同一）
        "lin_vel_y_range": [-0.20, 0.20],  # V11変更: ±0.15→±0.20（最適化）
        "ang_vel_range": [0, 0],  # 将来バージョンで追加予定
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main() -> None:
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified V2 Walking (exp009 V11)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-omni-v11")
    parser.add_argument("--num_envs", type=int, default=4096)
    parser.add_argument("--num_mini_batches", type=int, default=4)
    parser.add_argument("--max_iterations", type=int, default=4000)
    args = parser.parse_args()

    # Genesis初期化
    gs.init(logging_level="warning")

    # 設定読み込み
    env_cfg, obs_cfg, reward_cfg, command_cfg = get_cfgs()
    train_cfg = get_train_cfg(args.exp_name, args.max_iterations)
    train_cfg["algorithm"]["num_mini_batches"] = args.num_mini_batches

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

    # ランナー作成（gs.deviceからバックエンド種別を取得: cuda / mps / cpu）
    runner = OnPolicyRunner(env, train_cfg, log_dir=str(log_dir), device=gs.device.type)

    # 訓練開始
    print(f"\n{'=' * 70}")
    print("EXP009 V11: vyコマンド範囲の最適化（±0.15 → ±0.20 m/s）")
    print(f"{'=' * 70}")
    print("【V10からの変更点】")
    print(f"  1. lin_vel_y_range: [-0.15, 0.15] → {command_cfg['lin_vel_y_range']}")
    print(f"{'=' * 70}")
    print("【PD制御ゲイン】")
    print(f"  全関節デフォルト: Kp={env_cfg['kp']}, Kd={env_cfg['kd']}")
    print(f"  knee_pitch: Kp={env_cfg['kp_overrides']['left_knee_pitch_joint']}（V9から継承）")
    print("  ankle_pitch: Kd=5.0（exp008 V25から継承）")
    print(f"  PDクランプ維持: hip_roll_inward_limit={reward_cfg['hip_roll_inward_limit']} rad")
    print(f"{'=' * 70}")
    print("【報酬構成】")
    active_rewards = len([k for k, v in reward_cfg["reward_scales"].items() if v != 0])
    print(f"  報酬項目数: {active_rewards}（V9と同じ）")
    print(f"{'=' * 70}")
    print("【コマンド設定】")
    print(f"  lin_vel_x_range: {command_cfg['lin_vel_x_range']}（V10と同一）")
    print(f"  lin_vel_y_range: {command_cfg['lin_vel_y_range']}（V11変更: 最適化）")
    print(f"{'=' * 70}")
    print(f"観測空間: {obs_cfg['num_obs']}次元")
    print(f"行動空間: {env_cfg['num_actions']}次元")
    print(f"{'=' * 70}\n")

    # 報酬スケール表示
    print("報酬スケール:")
    for name, scale in reward_cfg["reward_scales"].items():
        if scale != 0:
            print(f"  {name}: {scale}")
        else:
            print(f"  {name}: {scale} (無効)")
    print()

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)

    print(f"\n訓練完了: {args.exp_name}")
    print(f"モデル保存先: {log_dir}")
    print(
        "【注意】V11はvyコマンド範囲の最適化（±0.15→±0.20）が目的。"
        "vy増幅率0.8〜1.2x、tracking_lin_vel報酬回復を目標に評価すること。"
    )


if __name__ == "__main__":
    main()
