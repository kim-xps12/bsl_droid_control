#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V27）

============================================================
【EXP007 V27: エネルギーペナルティ緩和 + air_time_offset短縮】
============================================================

【V26の結果と教訓】
V26ではswing_foot_lateral_velocityを-0.5に復帰し、feet_air_timeを1.5に強化したが、
以下の結果となった:
- feet_air_time報酬: -0.0275（V25より35%悪化、タップダンス継続）
- hip_pitch相関: -0.318（V25の-0.851から大幅悪化）
- hip_pos報酬: -0.0408（V25より13%改善、しかし内股継続）

根本原因の特定:
- 仮説A「左右非対称動作が原因」→ 症状
- 仮説B「エネルギー最小化が優位」→ 根本原因

エネルギー関連ペナルティ（action_rate, dof_acc）が滞空報酬より優位なため、
遊脚中にトルクを抜いて「サボる」動作が報酬最大化となり、タップダンスが発生。

【V27の設計原則】
案A（推奨）を採用: エネルギーペナルティ緩和 + air_time_offset短縮

| パラメータ      | V26値    | V27値    | 変更理由                       |
|----------------|----------|----------|-------------------------------|
| action_rate    | -0.01    | -0.005   | 遊脚維持のコストを半減          |
| dof_acc        | -2.5e-7  | -1.0e-7  | 加速度ペナルティを緩和          |
| air_time_offset| 0.2      | 0.15     | 報酬獲得閾値を下げる            |

【期待される効果】
1. エネルギーペナルティ緩和により、遊脚中にトルクを維持する動作が報酬的に有利に
2. 滞空時間が延長し、タップダンスが解消
3. air_time_offset短縮により、小型ロボットでも報酬獲得が容易に
4. 左右非対称動作も自然に改善される可能性（根本原因への対処により）

【参考文献】
- exp007_report_v26.md: V26の結果と根本原因分析
- exp007_report_v25.md: V25失敗の原因分析
- exp007_report_v24.md: 内股解消の成功事例
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
        # ============================================================
        # V22変更: Contact Sensorに復帰
        # ============================================================
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
    # V27: エネルギーペナルティ緩和 + air_time_offset短縮
    # - action_rate: -0.01→-0.005に緩和（遊脚維持コスト半減）
    # - dof_acc: -2.5e-7→-1.0e-7に緩和（加速度ペナルティ緩和）
    # - air_time_offset: 0.2→0.15に短縮（報酬獲得閾値を下げる）
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.25,  # V22と同じ
        "base_height_target": 0.20,  # 目標胴体高さ（BSL-Droid用に調整）
        "swing_height_target": 0.05,  # V22と同じ
        "gait_frequency": 1.2,  # V22と同じ
        "contact_threshold": 0.05,  # フォールバック用（Contact Sensor使用時は参照されない）
        # ============================================================
        # V27変更: air_time_offset短縮（0.2→0.15）
        # 小型ロボットでも報酬獲得が容易になる
        # ============================================================
        "air_time_offset": 0.15,  # V26: 0.2 → V27: 0.15
        # V18から継続: RobStride RS-02実機パラメータ
        "dof_vel_limits": 44.0,  # ±44 rad/s (RS-02 spec)
        "soft_dof_vel_limit": 0.9,  # 制限の90%でペナルティ開始
        # ============================================================
        # V22から継続: A案（ankle_pitch_rangeペナルティ）のパラメータ
        # ============================================================
        "ankle_pitch_limit": 0.3,  # ankle_pitchの許容範囲（rad）
        "reward_scales": {
            # ============================================================
            # 【主報酬】速度追従（V3強化）
            # ============================================================
            "tracking_lin_vel": 1.5,  # 線速度追従
            "tracking_ang_vel": 0.5,  # 角速度追従
            # ============================================================
            # 【歩行品質報酬】
            # ============================================================
            "feet_air_time": 1.5,  # 滞空時間報酬（V26から維持）
            "contact": 0.2,  # 接地フェーズ整合性
            "single_foot_contact": 0.3,  # 片足接地報酬（V3追加、静止対策）
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
            # 【歩行品質ペナルティ】
            # ============================================================
            "feet_swing_height": -8.0,  # 遊脚高さ目標追従
            "contact_no_vel": -0.1,  # 接地時足速度
            "hip_pos": -0.5,  # 股関節位置（開脚抑制）
            "velocity_deficit": -0.5,  # 速度未達ペナルティ（V3追加、静止対策）
            # ============================================================
            # 【V22から継続】A案: 遊脚時足首角度制限
            # ============================================================
            "ankle_pitch_range": -0.3,  # 遊脚時のankle_pitch角度制限ペナルティ
            # ============================================================
            # 【V18継続】関節角速度制限
            # ============================================================
            "dof_vel_limits": -0.3,  # 実機パラメータ超過ペナルティ
            # ============================================================
            # 【エネルギー効率ペナルティ】V27: 緩和
            # ============================================================
            "torques": -1e-5,  # トルクペナルティ（維持）
            # V27変更: action_rate緩和（-0.01→-0.005）
            # 遊脚維持のコストを半減し、足を上げ続ける動作を促進
            "action_rate": -0.005,  # V26: -0.01 → V27: -0.005
            # V27変更: dof_acc緩和（-2.5e-7→-1.0e-7）
            # 加速度ペナルティを緩和し、足を上げ続ける動作を促進
            "dof_acc": -1.0e-7,  # V26: -2.5e-7 → V27: -1.0e-7
            # ============================================================
            # 【V26から継続】遊脚横方向速度ペナルティ
            # ============================================================
            "swing_foot_lateral_velocity": -0.5,
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.15, 0.25],  # V20と同じ
        "lin_vel_y_range": [0, 0],  # 横移動なし
        "ang_vel_range": [0, 0],  # 旋回なし
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main() -> None:
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V27)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v27")
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
    print("EXP007 V27: エネルギーペナルティ緩和 + air_time_offset短縮")
    print(f"{'=' * 70}")
    print("【V26の結果と教訓】")
    print("  V26ではfeet_air_time強化（1.0→1.5）を試したが、報酬値は悪化")
    print("  - feet_air_time報酬: -0.0275（V25より35%悪化）")
    print("  - hip_pitch相関: -0.318（V25の-0.851から大幅悪化）")
    print("  根本原因: エネルギーペナルティが滞空報酬より優位")
    print(f"{'=' * 70}")
    print("【V27の設計原則】案A（推奨）を採用")
    print("  1. action_rate: -0.01 → -0.005（遊脚維持コスト半減）")
    print("  2. dof_acc: -2.5e-7 → -1.0e-7（加速度ペナルティ緩和）")
    print("  3. air_time_offset: 0.2 → 0.15（報酬獲得閾値を下げる）")
    print(f"{'=' * 70}")
    print("【V27での変更点（V26からの差分）】")
    print("  - action_rate: -0.01 → -0.005（緩和）")
    print("  - dof_acc: -2.5e-7 → -1.0e-7（緩和）")
    print("  - air_time_offset: 0.2 → 0.15（短縮）")
    print(f"{'=' * 70}")
    print("【期待される効果】")
    print("  - 遊脚中にトルクを維持する動作が報酬的に有利に")
    print("  - 滞空時間が延長し、タップダンスが解消")
    print("  - 左右非対称動作も自然に改善される可能性")
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
