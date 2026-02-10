#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V6）

============================================================
【EXP007 V6: tracking_sigma調整による静止ポリシー回避】
============================================================

【V5の失敗と成功】
V5はV2と同じ「静止ポリシー」に収束した（X速度: 0.003 m/s）。
- 失敗原因: 目標速度の低下が静止ポリシー収束の支配的な原因
- 成功点（V6に引き継ぐ）:
  - dof_vel: -0.005（ankle velocity std: 2.7-3.6 → 0.2-0.6 rad/s）
  - air_time_offset: 0.25 s（feet_air_time報酬正常化）
  - symmetry: 0.3（左右対称性報酬として機能）

【V6の核心: tracking_sigma調整】
問題の本質:
V5の静止ポリシー問題は、ガウシアン報酬関数（tracking_sigma=0.25）が
低目標速度において「静止」と「歩行」を十分に識別できないことに起因する。

sigma=0.25の報酬識別能力:
| 状態 | 目標速度 | 実速度 | error | 報酬 | 備考 |
|------|---------|-------|-------|------|------|
| 歩行達成 | 0.125 m/s | 0.125 m/s | 0.000 | 1.000 | 100% |
| 静止 | 0.125 m/s | 0.003 m/s | 0.122 | 0.787 | 79%（21%低下のみ） |

問題: 速度が98%低下しても報酬は21%しか低下しない。

sigma=0.10（V6案）での報酬識別能力:
| 状態 | 目標速度 | 実速度 | error | 報酬 | 備考 |
|------|---------|-------|-------|------|------|
| 歩行達成 | 0.125 m/s | 0.125 m/s | 0.000 | 1.000 | 100% |
| 静止 | 0.125 m/s | 0.003 m/s | 0.122 | 0.226 | 23%（77%低下） |

効果: sigmaを0.25→0.10に変更すると、静止と歩行の報酬差が3.5倍に拡大（21%→77%）。

【V6パラメータ変更一覧】
| パラメータ        | V5値  | V6値  | 変更理由                        |
|------------------|-------|-------|--------------------------------|
| tracking_sigma   | 0.25  | 0.10  | ガウシアン鋭化（静止識別の核心） |
| velocity_deficit | -0.5  | -2.0  | 追加保険として4倍強化           |
| 他のパラメータ    | 維持  | 維持  | V5の改善点を継続                |

【成功基準】
| 指標 | V5値 | V6目標 | 判定基準 |
|------|------|--------|---------|
| X速度 | 0.003 m/s | > 0.08 m/s | 目標の80%以上 |
| hip_pitch相関 | +0.653 | < +0.5 | 同期度低下 |
| ankle velocity std | 0.2-0.6 rad/s | < 1.0 rad/s | 振動抑制維持 |
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
        "num_obs": 50,  # Unitree方式の観測空間（3+3+3+3+10+10+10+1+1+2+2+2=50）
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }

    # ============================================================
    # V6: tracking_sigma調整による静止ポリシー回避
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.10,  # ★核心変更: V5: 0.25 → V6: 0.10（ガウシアン鋭化）
        "base_height_target": 0.20,  # 目標胴体高さ（BSL-Droid用に調整）
        "swing_height_target": 0.03,  # 遊脚の目標高さ
        "gait_frequency": 0.8,  # 歩行周波数（V5と同じ、ゆっくり歩容維持）
        "contact_threshold": 0.08,  # 接地判定閾値
        "air_time_offset": 0.25,  # 滞空時間オフセット（V5と同じ）
        "reward_scales": {
            # ============================================================
            # 【主報酬】速度追従
            # ============================================================
            "tracking_lin_vel": 1.5,  # 線速度追従
            "tracking_ang_vel": 0.5,  # 角速度追従
            # ============================================================
            # 【歩行品質報酬】（V5から継続）
            # ============================================================
            "feet_air_time": 1.5,  # 滞空時間報酬（offset=0.25sで正常化）
            "contact": 0.2,  # 接地フェーズ整合性
            "alive": 0.03,  # 生存報酬
            # 片足接地報酬
            "single_foot_contact": 0.8,  # 交互歩行誘導
            # 左右対称性報酬（V5から継続）
            "symmetry": 0.3,  # 左右脚の対称性を報酬化
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
            "feet_swing_height": -5.0,  # 遊脚高さ
            "contact_no_vel": -0.1,  # 接地時足速度
            "hip_pos": -0.5,  # 股関節位置（開脚抑制）
            # 速度未達ペナルティ（★4倍強化）
            "velocity_deficit": -2.0,  # V5: -0.5 → V6: -2.0
            # ============================================================
            # 【エネルギー効率ペナルティ】（V5から継続）
            # ============================================================
            "torques": -1e-5,  # トルクペナルティ
            "action_rate": -0.005,  # アクション変化率
            "dof_acc": -2.5e-7,  # 関節加速度
            "dof_vel": -0.005,  # 関節速度（V5から継続、振動抑制）
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.10, 0.15],  # 目標前進速度（V5と同じ、低速度維持）
        "lin_vel_y_range": [0, 0],  # 横移動なし
        "ang_vel_range": [0, 0],  # 旋回なし
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main() -> None:
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V6)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v6")
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
    print("EXP007 V6: tracking_sigma調整による静止ポリシー回避")
    print(f"{'=' * 70}")
    print("【V5の失敗と成功】")
    print("  失敗: 静止ポリシーに収束（X速度: 0.003 m/s）")
    print("  成功: 足首振動抑制、feet_air_time正常化、対称性報酬機能")
    print(f"{'=' * 70}")
    print("【V6の核心: tracking_sigma調整】")
    print("  - tracking_sigma: 0.25 → 0.10（ガウシアン鋭化）")
    print("    → 静止と歩行の報酬差が3.5倍に拡大（21%→77%）")
    print("  - velocity_deficit: -0.5 → -2.0（4倍強化、二重保険）")
    print(f"{'=' * 70}")
    print("【V5から継続】")
    print("  - lin_vel_x_range: [0.10, 0.15]（低速度維持）")
    print("  - gait_frequency: 0.8 Hz（ゆっくり歩容）")
    print("  - dof_vel: -0.005（振動抑制）")
    print("  - air_time_offset: 0.25 s")
    print("  - symmetry: 0.3")
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
