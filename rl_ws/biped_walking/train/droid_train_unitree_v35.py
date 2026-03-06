#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V35）

============================================================
【EXP007 V35: swing_contact_penalty強化（-0.5 → -0.7）】
============================================================

【V34の結果と教訓】
V34ではsymmetry_range=0でV32ベースに回帰し、振幅縮小副作用を解消:
- X速度回復: 0.173 m/s（V32: 0.175 m/s水準）
- DOF range sum回復: 6.021 rad（V33: 4.830 → V32: 5.651を上回る）
- Yawドリフト改善: +7.35°（V33: -18.69°から大幅改善、ただしV32: -2.00°には未達）
- タップダンス片側化: 左足のみに限定（V33では両足に発現）

残存課題:
- 左足タップダンスが残存（両足接地率4.4%、V32: 2.8%に未回復）
- knee_pitch L/R角速度比=2.09と左膝の過剰活動が継続
- swing_contact_penalty=-0.5が片側抑制の最低有効閾値だが、もう片側には不十分

【V35の設計原則】
1変更1検証の原則に従い、swing_contact_penaltyの強化のみを行う

| パラメータ              | V34値  | V35値  | 変更理由                                     |
|------------------------|--------|--------|----------------------------------------------|
| swing_contact_penalty  | -0.5   | -0.7   | 左足タップダンスの直接抑制                     |

【期待される効果】
1. 両足接地率の低下（4.4%→目標2%以下）
2. 左足の地面叩き動作の低減
3. 他の歩行品質指標の維持

【参考文献】
- exp007_report_v34.md: V34の結果と次バージョンへの提案（推奨案）
- V30-V31の実験: swing_contact_penalty=-0.5が最低有効閾値
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
    # V35: swing_contact_penalty強化（-0.5 → -0.7）
    # - 左足タップダンスの直接抑制
    # - 他のパラメータはV34から維持
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.25,  # V29と同じ
        "base_height_target": 0.20,  # 目標胴体高さ（BSL-Droid用に調整）
        "swing_height_target": 0.05,  # V29と同じ
        # ============================================================
        # gait_frequency: V29から維持（0.9Hz）
        # ============================================================
        "gait_frequency": 0.9,  # V29と同じ
        "contact_threshold": 0.05,  # フォールバック用（Contact Sensor使用時は参照されない）
        # ============================================================
        # V30変更: air_time_offset引き下げ（0.25→0.10）
        # V29でswing_duration報酬が機能しなかった（報酬値0.0000）
        # 原因: 0.25秒の空中時間がBSL-Droidでは達成困難
        # ============================================================
        "air_time_offset": 0.10,  # V30から維持
        # V18から継続: RobStride RS-02実機パラメータ
        "dof_vel_limits": 44.0,  # ±44 rad/s (RS-02 spec)
        "soft_dof_vel_limit": 0.9,  # 制限の90%でペナルティ開始
        # V22から継続: A案（ankle_pitch_rangeペナルティ）のパラメータ
        "ankle_pitch_limit": 0.3,  # ankle_pitchの許容範囲（rad）
        "reward_scales": {
            # ============================================================
            # 【主報酬】速度追従
            # ============================================================
            "tracking_lin_vel": 1.5,  # 線速度追従
            "tracking_ang_vel": 0.5,  # 角速度追従
            # ============================================================
            # 【歩行品質報酬】V35: swing_contact_penalty強化
            # ============================================================
            # feet_air_time: V29から継続して0（削除済み）
            "feet_air_time": 0,  # V30から維持（削除済み）
            # swing_duration: V30から維持（2.0）
            "swing_duration": 2.0,  # V30から維持
            # V35変更: swing_contact_penaltyを強化（-0.5→-0.7）
            # V34で左足タップダンスが残存、より強いペナルティで直接抑制を試みる
            "swing_contact_penalty": -0.7,  # V34: -0.5 → V35: -0.7（左足タップダンス抑制）
            "contact": 0.4,  # V30から維持
            # single_foot_contact: V31から維持（0.5）
            # V31でhip_pitch相関改善（-0.571→-0.660）に寄与
            "single_foot_contact": 0.5,  # V31から維持
            # step_length: V29から維持（0.8）
            "step_length": 0.8,  # V29と同じ
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
            "feet_swing_height": -8.0,  # 遊脚高さ目標追従
            "contact_no_vel": -0.1,  # 接地時足速度
            "hip_pos": -0.8,  # V29と同じ
            "velocity_deficit": -0.5,  # 速度未達ペナルティ（静止対策）
            # 【V22から継続】A案: 遊脚時足首角度制限
            "ankle_pitch_range": -0.3,  # 遊脚時のankle_pitch角度制限ペナルティ
            # 【V18継続】関節角速度制限
            "dof_vel_limits": -0.3,  # 実機パラメータ超過ペナルティ
            # ============================================================
            # 【エネルギー効率ペナルティ】V29値を維持
            # hip_pitch相関改善の成果を保持
            # ============================================================
            "torques": -1e-5,  # トルクペナルティ（維持）
            "action_rate": -0.005,  # V29から維持
            "dof_acc": -1.0e-7,  # V29から維持
            # 【V26から継続】遊脚横方向速度ペナルティ
            "swing_foot_lateral_velocity": -0.5,
            # 【V34から継続】左右対称性報酬は無効化を維持
            "symmetry_range": 0,  # V34から維持（無効化）
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.15, 0.25],  # V28と同じ
        "lin_vel_y_range": [0, 0],  # 横移動なし
        "ang_vel_range": [0, 0],  # 旋回なし
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main() -> None:
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V35)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v35")
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
    print("EXP007 V35: swing_contact_penalty強化（-0.5 → -0.7）")
    print(f"{'=' * 70}")
    print("【V34の結果と教訓】")
    print("  成功: V32ベース回帰でX速度0.173 m/s、DOF range 6.021 rad回復")
    print("  課題: 左足タップダンス残存（両足接地率4.4%）、Yaw +7.35°")
    print("  原因: swing_contact_penalty=-0.5が左足抑制に不十分")
    print(f"{'=' * 70}")
    print("【V35の設計原則】")
    print("  1. swing_contact_penalty: -0.5 → -0.7（左足タップダンス直接抑制）")
    print("  2. 他のパラメータはV34から維持")
    print("  ※1変更1検証の原則に従い、swing_contact_penaltyの強化のみ")
    print(f"{'=' * 70}")
    print("【期待される効果】")
    print("  - 両足接地率の低下（4.4%→目標2%以下）")
    print("  - 左足の地面叩き動作の低減")
    print("  - 他の歩行品質指標の維持")
    print(f"{'=' * 70}")
    print(f"観測空間: {obs_cfg['num_obs']}次元")
    print(f"行動空間: {env_cfg['num_actions']}次元")
    print(f"報酬項目数: {len([k for k, v in reward_cfg['reward_scales'].items() if v != 0])}")
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


if __name__ == "__main__":
    main()
