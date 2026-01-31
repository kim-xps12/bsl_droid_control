#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト V19

============================================================
【V19: V18の改善 - 姿勢制御強化 + 接地ベース歩行報酬 + 歩幅拡大】
============================================================

【V18の評価結果】
- びっこ解消 ✓
- 前後脚の固定改善 ✓
- X移動: 2.83m（V17: 2.68m）✓
- Yawドリフト: +9.8°（V17: -29.1°）✓

【V18の問題点】
1. 歩幅が小刻みすぎる（DOF range: 3.59 rad）
2. 胴体が前傾（Pitch mean: 13.7°）
3. 胴体が下がりすぎ（Base height: 0.212m、初期の61%）
4. 同期歩行（hip_pitch相関: +0.751）

【V19の改善方針】
先行研究サーベイ（exp004_reward_design_survey.md）に基づき：

1. 姿勢制御の強化:
   - orientation報酬を強化（-1.0 → -3.0）
   - base_height報酬を強化（-3.0）
   - pitch_penalty追加（-2.0、前傾13.7°対策）
   - 終了条件をさらに厳しく（pitch > 20°で終了）

2. 接地ベース歩行報酬の追加:
   - feet_air_time: Legged Gym方式の滞空時間報酬（1.0に強化）
   - single_stance: 片足接地を促進（Cassie方式）
   これにより交互歩行を誘導（関節レベル制約ではなく）

3. 歩幅拡大のための調整:
   - hip_pitch_velocity報酬追加（0.5）
   - contact_threshold上げ（0.04→0.06、接地検出改善）

【報酬項目数】
V18: 6項目 → V19: 11項目
（依然としてV17の25項目より大幅に少ない）
============================================================
"""

import argparse
import math
import os
import pickle
import shutil
from pathlib import Path

import genesis as gs

from biped_walking.envs.droid_env import DroidEnv

# rsl-rl-lib==2.2.4のインポート
from rsl_rl.runners.on_policy_runner import OnPolicyRunner


def get_train_cfg(exp_name, max_iterations):
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


def get_cfgs():
    """環境設定を取得"""
    script_dir = Path(__file__).parent
    rl_ws_dir = script_dir.parent.parent
    urdf_path = rl_ws_dir / "assets" / "bsl_droid_simplified.urdf"

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
        # ============================================================
        # 【V19】終了条件をさらに厳しく - 前傾防止
        # ============================================================
        "termination_if_roll_greater_than": 25,
        "termination_if_pitch_greater_than": 20,    # V18: 25° → V19: 20°（前傾に厳しく）
        "termination_if_height_lower_than": 0.15,   # V18: 0.12m → V19: 0.15m（沈み込み防止）
        "termination_if_knee_positive": True,
        "base_init_pos": [0.0, 0.0, 0.35],
        "base_init_quat": [1.0, 0.0, 0.0, 0.0],
        "episode_length_s": 20.0,
        "resampling_time_s": 4.0,
        "action_scale": 0.4,
        "simulate_action_latency": True,
        "clip_actions": 10.0,
    }

    obs_cfg = {
        "num_obs": 39,
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }

    # ============================================================
    # V19: 姿勢制御強化 + 接地ベース歩行報酬 + 歩幅拡大
    # ============================================================
    #
    # V18からの変更点:
    # - orientation: -1.0 → -3.0（姿勢維持をさらに強化）
    # - base_height: 0 → -3.0（沈み込みペナルティ強化）
    # - pitch_penalty: 追加 -2.0（前傾13.7°対策）
    # - feet_air_time: 0 → 1.0（滞空時間報酬強化）
    # - single_stance: 0 → 0.3（片足接地報酬追加）
    # - hip_pitch_velocity: 追加 0.5（歩幅拡大促進）
    # - contact_threshold: 0.04 → 0.06（接地検出改善）
    #
    # 報酬項目数: 6 → 11（minimalist維持）
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.25,      # V18: 0.22 → V19: 0.25（高めを目標）
        "feet_air_time_target": 0.20,    # 目標滞空時間（秒）
        "gait_frequency": 1.5,
        "contact_threshold": 0.06,       # 接地判定閾値（V18: 0.04 → V19: 0.06、接地検出改善）

        "reward_scales": {
            # ============================================================
            # 【タスク報酬】（3項目）
            # ============================================================
            "tracking_lin_vel": 2.0,     # 前進速度追従
            "tracking_ang_vel": 0.5,     # 旋回速度追従
            "alive": 1.0,                # 生存報酬

            # ============================================================
            # 【姿勢制御】（4項目）← V19で強化
            # ============================================================
            "orientation": -3.0,         # V18: -1.0 → V19: -3.0（姿勢維持をさらに強化）
            "base_height": -3.0,         # 沈み込みペナルティ（強化）
            "pitch_penalty": -2.0,       # 【NEW】前傾ペナルティ（V18の13.7°前傾対策）

            # ============================================================
            # 【エネルギー効率】（2項目）
            # ============================================================
            "torques": -1e-4,            # トルク最小化
            "dof_acc": -1e-6,            # 加速度最小化

            # ============================================================
            # 【歩行パターン誘導】（3項目）← V19で追加
            # ============================================================
            # Legged Gym / Cassie方式: 関節ではなく接地状態で歩行を誘導
            "feet_air_time": 1.0,        # 滞空時間報酬（周期的足上げ、強化）
            "single_stance": 0.3,        # 片足接地報酬（交互歩行誘導）
            "hip_pitch_velocity": 0.5,   # 【NEW】hip_pitch速度報酬（歩幅拡大促進）
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.3, 0.3],  # 目標前進速度 0.3 m/s
        "lin_vel_y_range": [0, 0],       # 横移動なし
        "ang_vel_range": [0, 0],         # 旋回なし
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main():
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking V19")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v19")
    parser.add_argument("--num_envs", type=int, default=4096)
    parser.add_argument("--max_iterations", type=int, default=500)
    args = parser.parse_args()

    # Genesis初期化
    gs.init(logging_level="warning")

    # 設定読み込み
    env_cfg, obs_cfg, reward_cfg, command_cfg = get_cfgs()
    train_cfg = get_train_cfg(args.exp_name, args.max_iterations)

    # 環境作成
    env = DroidEnv(
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
    print(f"\n{'='*60}")
    print("V19: 姿勢制御強化 + 接地ベース歩行報酬 + 歩幅拡大")
    print(f"{'='*60}")
    print("【V18からの改善】")
    print("- orientation報酬: -1.0 → -3.0 (姿勢維持さらに強化)")
    print("- base_height報酬: -3.0 (沈み込み防止強化)")
    print("- pitch_penalty報酬: 追加 -2.0 (前傾13.7°対策)")
    print("- feet_air_time報酬: 1.0 (周期的足上げ強化)")
    print("- single_stance報酬: 0.3 (片足接地・交互歩行)")
    print("- hip_pitch_velocity報酬: 追加 0.5 (歩幅拡大促進)")
    print("- contact_threshold: 0.04 → 0.06 (接地検出改善)")
    print("- pitch終了条件: 25° → 20° (前傾防止)")
    print("- height終了条件: 0.12m → 0.15m (沈み込み防止)")
    print(f"{'='*60}")
    print(f"報酬項目数: 11 (V18: 6, V17: 25)")
    print(f"{'='*60}\n")

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
