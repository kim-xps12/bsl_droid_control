#!/usr/bin/env python3
"""
BSL-Droid Simplified V2 歩行学習スクリプト（exp008 V8）

============================================================
【EXP008 V8: stance_hip_roll_targetターゲット角度設定（±5°外向き）】
============================================================

exp008 V7をベースに、以下の変更を実施:
1. stance_hip_roll_target_angle: 0.0 → 0.087 rad（±5°外向きターゲット）
   （報酬スケール-0.5は維持、報酬関数の最小点を0°→±5°に移動）

【設計意図】
- V7課題: 内股着地が7バージョン未解決。hip_roll offset L:-14.84°, R:+12.93°
- 現在のstance_hip_roll_targetはhip_roll²=0をターゲットとしているが、
  ロボットは持続的に内股offsetを維持 → 0°ターゲットでは矯正力不足
- (hip_roll - target)²に変更し、外向き±5°のターゲット角度を設定:
  - L脚: (hip_roll - 0.087)² → hip_roll=+5°で最小化（外向き）
  - R脚: (hip_roll + 0.087)² → hip_roll=-5°で最小化（外向き）
- ペナルティ最小点が0°→±5°に移動することで、内股方向への勾配が増大

【報酬構成】
- 17項目維持（スケール変更なし）
- stance_hip_roll_target=-0.5維持（スケール不変、ターゲット角度のみ変更）
- stance_hip_roll_target_angle=0.087 rad（V8新設）
- orientation=-2.0維持（V6の値）
- hip_yaw_pos=-0.8維持（Yawドリフト抑制）
- ang_vel_xy=-0.1維持（Roll/Pitch角速度の直接抑制）

【参考文献】
- exp008_report_v7.md: 代替案B（ターゲット角度設定）
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
    # EXP008 V8: stance_hip_roll_targetターゲット角度設定（±5°外向き）
    # - stance_hip_roll_target_angle: 0.0 → 0.087 rad（±5°外向き）
    # - 報酬スケールは全てV7から維持
    # - 報酬項目数17維持
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
        "stance_hip_roll_target_angle": 0.087,  # V8新設: ±5°外向きターゲット (0.087 rad ≈ 5°)
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
            "feet_swing_height": -8.0,  # 遊脚高さ目標追従
            "contact_no_vel": -0.1,  # 接地時足速度
            "hip_yaw_pos": -0.8,  # V3新設: hip_yaw²のみペナルティ（hip_pos compound penaltyを分離）
            "stance_hip_roll_target": -0.5,  # V4新設: 接地脚のみhip_roll²をペナルティ（ankle_roll置換）
            "velocity_deficit": -0.5,  # 速度未達ペナルティ（静止対策）
            # ============================================================
            # 【エネルギー効率ペナルティ】
            # V36削除: torques(-1e-5, 寄与0.0%), dof_acc(-1e-7, 寄与1.1%)
            # ============================================================
            "action_rate": -0.005,  # V29から維持
            # 【V7変更】遊脚横方向速度ペナルティ → 全相足先横方向速度ペナルティに置換
            "swing_foot_lateral_velocity": 0,  # V7で無効化（foot_lateral_velocityに置換）
            "foot_lateral_velocity": -0.5,  # V7: 全相で速度指令直交成分をペナルティ化
            # 【V34から継続】左右対称性報酬は無効化
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
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified V2 Walking (exp008 V8)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-narrow-v8")
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
    print("EXP008 V8: stance_hip_roll_targetターゲット角度設定（±5°外向き）")
    print(f"{'=' * 70}")
    print("【V7からの変更点】")
    print(f"  1. stance_hip_roll_target_angle: 0.0 → {reward_cfg.get('stance_hip_roll_target_angle', 0.0):.3f} rad（±5°外向き）")
    print(f"{'=' * 70}")
    print("【設計意図】")
    print("  - hip_roll²=0 → (hip_roll-target)²でペナルティ最小点を±5°外向きに移動")
    print("  - 内股offset(±14°)に対して外向き方向の矯正勾配を生成")
    print(f"{'=' * 70}")
    print("【報酬構成】")
    print(f"  報酬項目数: {len([k for k, v in reward_cfg['reward_scales'].items() if v != 0])}")
    print(f"  stance_hip_roll_target: -0.5（スケール維持）, target_angle: {reward_cfg.get('stance_hip_roll_target_angle', 0.0):.3f} rad")
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


if __name__ == "__main__":
    main()
