#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V40）

============================================================
【EXP007 V40: hip_pos分解（hip_roll特化ペナルティへの転換）】
============================================================

【V39の結果と教訓】
V39ではhip_posスケールを-1.0→-0.8に緩和:
- Yawドリフト: -42.71°→-19.17°（+55%改善）
- Roll std: 6.53°→6.06°（-7.2%）
- Pitch std: 3.16°→1.34°（-58%、過去最良）
- base_pos_y std: 0.138→0.090m（-35%改善）
- スイング持続: 0.30-0.35s→0.41-0.43s（回復）

残存課題:
- X速度: 0.168→0.122 m/s（-27%低下）
- hip_pitch相関: -0.801→-0.588（-27%低下）
- Yaw後半加速: 2-7sで安定だが8s以降にドリフト加速
- 接地脚内股: hip_roll offset L:-18.80° R:+17.84°→横方向並進揺れ
- 横方向振動: 脱トレンドstd 49.1mm（V38: 34.1mm、+44%悪化）

教訓:
- hip_pos(-0.8)はhip_yaw+hip_rollを同時にペナルティする構造的欠陥
- ang_vel_xyは角速度のみ制約し、hip_rollの静的mean offsetは制約しない
- 横方向並進揺れの44%はhip_roll mean offsetによる重心横移動に起因

【V40の設計原則】
hip_posをankle_rollに分解する。hip_posを無効化し、既存の_reward_ankle_roll
（hip_rollのみをペナルティ）を有効化して、hip_yaw完全自由化とhip_roll制御を両立。

変更内容:
| パラメータ  | V39値 | V40値 | 変更理由 |
|------------|-------|-------|---------|
| hip_pos    | -0.8  | 0     | hip_yaw制約を完全除去しYaw修正能力を回復 |
| ankle_roll | 0     | -1.0  | hip_rollのみを直接ペナルティし内股・横揺れを抑制 |

【期待される効果】
1. hip_yaw完全自由化→Yaw修正能力の回復→Yawドリフトのさらなる改善
2. ankle_rollがhip_roll offsetを直接ペナルティ→内股縮小→横方向並進揺れ抑制
3. ang_vel_xy=-0.1がRoll角速度安定性を担保
4. 報酬項目数は16項目を維持（-1 hip_pos + 1 ankle_roll = ±0）

【リスク】
- hip_yaw完全自由化でhip_yawが過度に大きくなる可能性
- ankle_roll=-1.0がhip_rollを強く制約しすぎる可能性

【参考文献】
- exp007_report_v39.md: V39の結果と次バージョンへの提案（推奨案）
- _reward_ankle_roll: droid_env_unitree.py:953（V8追加、hip_roll²のみをペナルティ）
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
    # V40: hip_pos分解（hip_roll特化ペナルティへの転換）
    # - hip_pos無効化（-0.8→0）+ ankle_roll有効化（0→-1.0）
    # - 16項目構成を維持（-1 + 1 = ±0）
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
            "orientation": -0.5,  # 姿勢ペナルティ（BSL-Droid向け緩和）
            "base_height": -5.0,  # 高さ維持（BSL-Droid向け緩和）
            # ============================================================
            # 【歩行品質ペナルティ】
            # V36削除: ankle_pitch_range(-0.3, 寄与0.6%), dof_vel_limits(-0.3, 寄与0.0%)
            # ============================================================
            "feet_swing_height": -8.0,  # 遊脚高さ目標追従
            "contact_no_vel": -0.1,  # 接地時足速度
            "hip_pos": 0,  # V40変更: -0.8→0（無効化、hip_yaw制約を完全除去）
            "ankle_roll": -1.0,  # V40追加: hip_rollのみを直接ペナルティ（内股・横揺れ抑制）
            "velocity_deficit": -0.5,  # 速度未達ペナルティ（静止対策）
            # ============================================================
            # 【エネルギー効率ペナルティ】
            # V36削除: torques(-1e-5, 寄与0.0%), dof_acc(-1e-7, 寄与1.1%)
            # ============================================================
            "action_rate": -0.005,  # V29から維持
            # 【V26から継続】遊脚横方向速度ペナルティ
            "swing_foot_lateral_velocity": -0.5,
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
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V40)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v40")
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
    print("EXP007 V40: hip_pos分解（hip_roll特化ペナルティへの転換）")
    print(f"{'=' * 70}")
    print("【V39の結果と教訓】")
    print("  成功: Yaw -42.71°→-19.17°(+55%改善)、Roll std 6.06°(-7%)")
    print("  成功: Pitch std 1.34°(-58%、過去最良)、base_pos_y std -35%")
    print("  課題: X速度0.122(-27%)、hip_pitch相関-0.588(-27%)")
    print("  課題: 接地脚内股(hip_roll offset)→横方向並進揺れ")
    print("  教訓: hip_pos構造的欠陥(hip_yaw+hip_roll同時ペナルティ)")
    print(f"{'=' * 70}")
    print("【V40の設計原則】")
    print("  hip_pos分解: hip_pos=-0.8→0(無効化) + ankle_roll=0→-1.0(有効化)")
    print("  目的: hip_yaw完全自由化 + hip_rollのみ直接ペナルティ")
    print("  ※ang_vel_xy=-0.1がRoll角速度安定性を担保")
    print(f"{'=' * 70}")
    print("【期待される効果】")
    print("  - hip_yaw完全自由化→Yawドリフトのさらなる改善")
    print("  - ankle_rollでhip_roll offset直接抑制→横方向並進揺れ改善")
    print("  - 報酬項目数: 16項目を維持（推奨範囲内）")
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
    print("次のステップ: biped_eval.py で評価を実行")


if __name__ == "__main__":
    main()
