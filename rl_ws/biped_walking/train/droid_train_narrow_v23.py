#!/usr/bin/env python3
"""
BSL-Droid Simplified V2 歩行学習スクリプト（exp008 V23）

============================================================
【EXP008 V23: feet_swing_height scale削減 + swing_height_target復元】
============================================================

exp008 V22をベースに、以下の変更を実施:
1. swing_height_target: 0.03 → 0.05（V21値に復元）
2. feet_swing_height scale: -8.0 → -4.0（半減）

【設計意図】
- V22でswing_height_target引き下げ(0.05→0.03)が逆効果と判明（ペナルティ+25.5%悪化）
- ポリシーの自然な足高さは~0.045mで、0.05mが適切な目標値
- 問題の本質は「目標値が高すぎる」ではなく「スケールが大きすぎる」こと
- スケール半減でペナルティ寄与を-0.0659→~-0.026に低減し、ratio改善を狙う

【報酬構成】
- V22から項目数変更なし（17項目）
- 2パラメータ変更（同一報酬項目のtarget復元+scale削減）
- PDクランプ設定はV14から維持: hip_roll_inward_limit=-0.05 rad

【参考文献】
- exp008_report_v22.md
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
    # EXP008 V23: V22からswing_height_target復元 + feet_swing_height scale削減
    # - swing_height_target: 0.03 → 0.05（V21値に復元。V22で引き下げが逆効果と判明）
    # - feet_swing_height scale: -8.0 → -4.0（半減。ペナルティ過大の根本原因はスケール）
    # - 報酬項目数: 17（V22と同じ）
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.25,  # V29と同じ
        "base_height_target": 0.20,  # 目標胴体高さ（BSL-Droid用に調整）
        "swing_height_target": 0.05,  # V23変更: 0.03→0.05（V21値に復元。V22で0.03が逆効果と判明）
        # ============================================================
        # gait_frequency: V20変更（0.9→1.2Hz）
        # V19で歩幅増大のみの速度向上がhip_pitch corr悪化を引き起こした
        # 周波数引き上げで歩幅×周波数の二軸制御に転換
        # ============================================================
        "gait_frequency": 1.2,  # V20変更: 0.9→1.2 Hz
        "contact_threshold": 0.05,  # フォールバック用（Contact Sensor使用時は参照されない）
        "stance_foot_lateral_min_distance": 0.050,  # V12: 足横方向最小距離閾値 [m]
        # ============================================================
        # V30変更: air_time_offset引き下げ（0.25→0.10）
        # V29でswing_duration報酬が機能しなかった（報酬値0.0000）
        # 原因: 0.25秒の空中時間がBSL-Droidでは達成困難
        # ============================================================
        "air_time_offset": 0.10,  # V30から維持
        # ============================================================
        # V14変更: hip_roll内向きPDターゲットクランプ強化
        # -0.05 rad = -2.9°（オーバーシュート10°前提で実効値≈-13°を狙う）
        # V13: -0.20 rad → 実効値-22°（3°改善のみ）
        # ============================================================
        "hip_roll_inward_limit": -0.05,  # V14: PDターゲットの内向き制限 [rad]
        "reward_scales": {
            # ============================================================
            # 【主報酬】速度追従
            # ============================================================
            "tracking_lin_vel": 1.5,  # 線速度追従
            "tracking_ang_vel": 0.5,  # 角速度追従
            # ============================================================
            # 【歩行品質報酬】V35から維持
            # ============================================================
            "feet_air_time": 0,  # V30から維持（無効化）
            "swing_duration": 2.0,  # V30から維持
            "swing_contact_penalty": -0.7,  # V35から維持
            "contact": 0.4,  # V30から維持
            "single_foot_contact": 0.5,  # V31から維持
            "step_length": 0.8,  # V29から維持
            # ============================================================
            # 【安定性ペナルティ】（Unitree方式）
            # ============================================================
            "lin_vel_z": -2.0,  # Z軸速度ペナルティ
            "ang_vel_xy": -0.1,  # V38変更: -0.05→-0.1（Roll/Pitch角速度の直接抑制）
            "orientation": -2.0,  # V6変更: -0.5→-2.0（胴体鉛直性4x強化、ボディレベルから内股軽減）
            "base_height": -5.0,  # 高さ維持（BSL-Droid向け緩和）
            # ============================================================
            # 【歩行品質ペナルティ】
            # V36削除: ankle_pitch_range(-0.3, 寄与0.6%), dof_vel_limits(-0.3, 寄与0.0%)
            # ============================================================
            "feet_swing_height": -4.0,  # V23変更: -8.0→-4.0（スケール半減。V22でペナルティ過大が問題と判明）
            "contact_no_vel": -0.1,  # 接地時足速度
            "hip_yaw_pos": -0.8,  # V3新設: hip_yaw²のみペナルティ（hip_pos compound penaltyを分離）
            "stance_hip_roll_target": 0,  # V12で無効化（stance_foot_lateral_positionに置換）
            "stance_foot_lateral_position": 0,  # V21削除: PDクランプ(V14+)が完全に代替、V20で寄与0.6%
            "velocity_deficit": -0.5,  # 速度未達ペナルティ（静止対策）
            # ============================================================
            # 【エネルギー効率ペナルティ】
            # V36削除: torques(-1e-5, 寄与0.0%), dof_acc(-1e-7, 寄与1.1%)
            # ============================================================
            "action_rate": -0.005,  # V29から維持
            # 【V7から維持】全相足先横方向速度ペナルティ
            "swing_foot_lateral_velocity": 0,  # V7で無効化（foot_lateral_velocityに置換）
            "foot_lateral_velocity": -0.5,  # V7新設: 全相で速度指令直交成分をペナルティ化
            # 【V34から継続】左右対称性報酬は無効化
            "symmetry_range": 0,  # V34から維持（無効化）
            # ============================================================
            # 【V15新設】横方向並進速度ペナルティ
            # V14で横方向並進揺れが主課題（lateral std 48mm, Y速度std 0.225m/s）
            # base_vel_yでY方向並進速度を直接ペナルティ化
            # ============================================================
            "base_vel_y": -1.0,  # V15新設: Y方向並進速度の二乗ペナルティ
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.25, 0.35],  # V19から維持
        "lin_vel_y_range": [0, 0],  # 横移動なし
        "ang_vel_range": [0, 0],  # 旋回なし
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main() -> None:
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified V2 Walking (exp008 V23)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-narrow-v23")
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
    print("EXP008 V23: feet_swing_height scale削減 + swing_height_target復元 (V22ベース)")
    print(f"{'=' * 70}")
    print("【V22からの変更点】")
    print("  1. swing_height_target: 0.03 → 0.05（V21値に復元）")
    print("  2. feet_swing_height scale: -8.0 → -4.0（半減）")
    print(f"{'=' * 70}")
    print("【設計意図】")
    print("  - V22でswing_height_target引き下げが逆効果と判明（ペナルティ+25.5%悪化）")
    print("  - 問題の本質はスケール過大。目標値は0.05mが適切（自然な足高さ~0.045mに近い）")
    print(f"  - PDクランプ維持: hip_roll_inward_limit={reward_cfg['hip_roll_inward_limit']} rad")
    print(f"{'=' * 70}")
    print("【報酬構成】")
    print(f"  報酬項目数: {len([k for k, v in reward_cfg['reward_scales'].items() if v != 0])}")
    print("  V22から項目数変更なし（target復元+scale削減の2パラメータ変更）")
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
    print("【注意】V23はfeet_swing_height scale削減+target復元実験。ペナルティ寄与低減とratio改善を確認すること。")


if __name__ == "__main__":
    main()
