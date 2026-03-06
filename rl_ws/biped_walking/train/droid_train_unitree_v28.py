#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V28）

============================================================
【EXP007 V28: air_time_offset延長 + contact報酬強化】
============================================================

【V27の結果と教訓】
V27ではエネルギーペナルティを緩和したが、以下の結果となった:
- hip_pitch相関: -0.931（V26の-0.318から大幅改善！）
- X速度: 0.217 m/s（V26の0.208から微改善）
- feet_air_time報酬: -0.0141（V26より49%改善）
- タップダンス: 再発（右足2回蹴り、左足1回の「タタン、タン」パターン）
- 内股軌道: 継続

成功点:
- エネルギーペナルティ緩和により、hip_pitch相関が大幅改善
- action_rate, dof_accの緩和が効果を発揮

失敗の原因分析:
- air_time_offset短縮(0.2→0.15)が副作用を引き起こした
- 短い滞空時間でも報酬獲得可能になり、「足を上げてすぐ下ろす」動作が有利に
- feet_air_time報酬のfirst_contact構造との相互作用でタップダンスが促進
- hip_pitch相関は改善（左右交互性）したが、「片足内での2回接地」が発生

【V28の設計原則】
air_time_offset延長 + contact報酬強化 + 内股対策強化

| パラメータ       | V27値    | V28値    | 変更理由                            |
|-----------------|----------|----------|-------------------------------------|
| air_time_offset | 0.15     | 0.25     | 短縮副作用の解消、長い滞空を要求       |
| contact         | 0.2      | 0.4      | 位相整合性の強化                      |
| hip_pos         | -0.5     | -0.8     | 内股対策強化                         |
| action_rate     | -0.005   | -0.005   | 維持（hip_pitch相関改善を保持）       |
| dof_acc         | -1.0e-7  | -1.0e-7  | 維持                                |

【期待される効果】
1. air_time_offset延長により、「足を上げてすぐ下ろす」動作では報酬獲得不可に
2. contact報酬強化により、正しいタイミングでの接地を促進
3. hip_pos強化で内股軌道を抑制
4. エネルギーペナルティ維持でhip_pitch相関改善の成果を保持

【参考文献】
- exp007_report_v27.md: V27の結果と根本原因分析
- exp007_unitree_rl_gym_survey.md: Unitree RL Gymの報酬設計
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
    # V28: air_time_offset延長 + contact報酬強化 + 内股対策強化
    # - air_time_offset: 0.15→0.25に延長（短縮副作用の解消）
    # - contact: 0.2→0.4に強化（位相整合性の強化）
    # - hip_pos: -0.5→-0.8に強化（内股対策）
    # - action_rate, dof_accは維持（hip_pitch相関改善を保持）
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.25,  # V27と同じ
        "base_height_target": 0.20,  # 目標胴体高さ（BSL-Droid用に調整）
        "swing_height_target": 0.05,  # V27と同じ
        "gait_frequency": 1.2,  # V27と同じ
        "contact_threshold": 0.05,  # フォールバック用（Contact Sensor使用時は参照されない）
        # ============================================================
        # V28変更: air_time_offset延長（0.15→0.25）
        # 長い滞空時間を要求し、タップダンスを抑制
        # ============================================================
        "air_time_offset": 0.25,  # V27: 0.15 → V28: 0.25
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
            # 【歩行品質報酬】V28: contact強化
            # ============================================================
            "feet_air_time": 1.5,  # 滞空時間報酬（維持）
            # V28変更: contact報酬強化（0.2→0.4）
            # 位相整合性を強化し、正しいタイミングでの接地を促進
            "contact": 0.4,  # V27: 0.2 → V28: 0.4
            "single_foot_contact": 0.3,  # 片足接地報酬（維持）
            # 【V19追加】歩幅促進
            "step_length": 0.5,  # 歩幅報酬
            # ============================================================
            # 【安定性ペナルティ】（Unitree方式）
            # ============================================================
            "lin_vel_z": -2.0,  # Z軸速度ペナルティ
            "ang_vel_xy": -0.05,  # XY角速度ペナルティ
            "orientation": -0.5,  # 姿勢ペナルティ（BSL-Droid向け緩和）
            "base_height": -5.0,  # 高さ維持（BSL-Droid向け緩和）
            # ============================================================
            # 【歩行品質ペナルティ】V28: hip_pos強化
            # ============================================================
            "feet_swing_height": -8.0,  # 遊脚高さ目標追従
            "contact_no_vel": -0.1,  # 接地時足速度
            # V28変更: hip_pos強化（-0.5→-0.8）
            # 内股軌道を抑制
            "hip_pos": -0.8,  # V27: -0.5 → V28: -0.8
            "velocity_deficit": -0.5,  # 速度未達ペナルティ（静止対策）
            # 【V22から継続】A案: 遊脚時足首角度制限
            "ankle_pitch_range": -0.3,  # 遊脚時のankle_pitch角度制限ペナルティ
            # 【V18継続】関節角速度制限
            "dof_vel_limits": -0.3,  # 実機パラメータ超過ペナルティ
            # ============================================================
            # 【エネルギー効率ペナルティ】V27値を維持
            # hip_pitch相関改善の成果を保持
            # ============================================================
            "torques": -1e-5,  # トルクペナルティ（維持）
            "action_rate": -0.005,  # V27から維持
            "dof_acc": -1.0e-7,  # V27から維持
            # 【V26から継続】遊脚横方向速度ペナルティ
            "swing_foot_lateral_velocity": -0.5,
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.15, 0.25],  # V27と同じ
        "lin_vel_y_range": [0, 0],  # 横移動なし
        "ang_vel_range": [0, 0],  # 旋回なし
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main() -> None:
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V28)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v28")
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
    print("EXP007 V28: air_time_offset延長 + contact報酬強化 + 内股対策強化")
    print(f"{'=' * 70}")
    print("【V27の結果と教訓】")
    print("  成功: hip_pitch相関が-0.318→-0.931に大幅改善（エネルギーペナルティ緩和の効果）")
    print("  失敗: タップダンス再発（右足2回蹴り）、内股軌道継続")
    print("  原因: air_time_offset短縮の副作用")
    print(f"{'=' * 70}")
    print("【V28の設計原則】")
    print("  1. air_time_offset: 0.15 → 0.25（長い滞空を要求）")
    print("  2. contact: 0.2 → 0.4（位相整合性の強化）")
    print("  3. hip_pos: -0.5 → -0.8（内股対策強化）")
    print("  4. action_rate, dof_acc: 維持（hip_pitch相関改善を保持）")
    print(f"{'=' * 70}")
    print("【期待される効果】")
    print("  - 「足を上げてすぐ下ろす」動作では報酬獲得不可に → タップダンス抑制")
    print("  - 正しいタイミングでの接地を促進")
    print("  - 内股軌道の抑制")
    print("  - hip_pitch相関 > -0.9を維持")
    print("  - 片足接地率 > 80%を維持")
    print("  - X速度: 0.15-0.25 m/s")
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
