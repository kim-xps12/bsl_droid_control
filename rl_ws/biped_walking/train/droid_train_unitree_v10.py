#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V10）

============================================================
【EXP007 V10: 足上げ改善と滑らかな歩行の実現】
============================================================

【V9の結果と課題】
V9は左右対称性とYawドリフトを大幅に改善したが、
symmetry報酬の強化によりhip_pitch相関が悪化し、足引きずりも発生：

- 成功点:
  - hip_pitch L/R比: 1.5倍 → 0.91倍（ほぼ対称）
  - Yawドリフト: +113° → -30.74°（73%改善）
  - 単足接地率: 74.0% → 97.2%（交互歩行確立）
  - Roll std: 4.41° → 2.48°（44%改善）

- 課題点:
  - hip_pitch相関の悪化: -0.382 → +0.605（symmetry報酬の副作用）
  - 足先引きずり: 足上げ高さ不足、stumbleペナルティ欠如
  - 断続的な速度推移: 滑らかな歩行サイクル未確立
  - X速度の低下: 0.108 → 0.092 m/s（-15%）

【V10の改善方針】

1. **symmetry報酬の緩和**
   - V9の1.0 → V10: 0.5（半減）
   - hip_pitch同期を緩和し、逆相関を維持

2. **足上げ高さの増加**
   - swing_height_target: 0.03 → 0.05m
   - feet_swing_height: -5.0 → -10.0（ペナルティ強化）

3. **足引きずりペナルティの新規追加**
   - feet_stumble: -0.5（接地中の足の水平速度をペナルティ化）
   - 足引きずり動作を明示的に抑制

4. **動作滑らかさの改善**
   - action_rate: -0.005 → -0.01（2倍強化）
   - アクション変化を抑制し、カクカク動作を軽減

【V10パラメータ変更一覧】
| パラメータ           | V9値   | V10値      | 変更理由                          |
|---------------------|--------|------------|----------------------------------|
| symmetry            | 1.0    | 0.5        | hip_pitch同期を緩和               |
| swing_height_target | 0.03   | 0.05       | 足上げ高さ増加                    |
| feet_swing_height   | -5.0   | -10.0      | 足上げペナルティ強化              |
| feet_stumble        | (なし) | -0.5       | 【新規】足引きずり抑制            |
| action_rate         | -0.005 | -0.01      | 滑らかさ向上                      |

【成功基準】
| 指標 | V9値 | V10目標 | 判定基準 |
|------|------|---------|---------|
| X速度 | 0.092 m/s | > 0.10 m/s | 回復 |
| hip_pitch相関 | +0.605 | < 0 | 逆相関に戻す |
| hip_pitch L/R比 | 0.91倍 | < 1.2倍 | 対称性維持 |
| Yawドリフト | -30.74° | < ±30° | 維持 |
| 足引きずり | あり | なし | 解消 |
| 速度推移 | カクカク | 滑らか | 改善 |
============================================================
"""

import argparse
import math
import os
import pickle
import shutil
from pathlib import Path

import genesis as gs

# rsl-rl-lib==2.2.4のインポート
from rsl_rl.runners.on_policy_runner import OnPolicyRunner

from biped_walking.envs.droid_env_unitree import DroidEnvUnitree


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
    # V10: 足上げ改善と滑らかな歩行の実現
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.10,  # V6から継続（ガウシアン鋭化）
        "base_height_target": 0.20,  # 目標胴体高さ
        "swing_height_target": 0.05,  # ★変更: V9: 0.03 → V10: 0.05m（足上げ高さ増加）
        "gait_frequency": 0.8,  # 歩行周波数（V6と同じ）
        "contact_threshold": 0.08,  # 接地判定閾値
        "air_time_offset": 0.20,  # V7から継続
        "reward_scales": {
            # ============================================================
            # 【主報酬】速度追従
            # ============================================================
            "tracking_lin_vel": 1.5,  # 線速度追従
            "tracking_ang_vel": 1.0,  # V9から継続
            # ============================================================
            # 【歩行品質報酬】
            # ============================================================
            "feet_air_time": 1.5,  # 滞空時間報酬
            "contact": 0.3,  # V7から継続
            "alive": 0.03,  # 生存報酬
            # 片足接地報酬
            "single_foot_contact": 1.0,  # V7から継続
            # 左右対称性報酬
            "symmetry": 0.5,  # ★緩和: V9: 1.0 → V10: 0.5（hip_pitch同期を緩和）
            # hip_pitch速度逆相関報酬・修正版（V9追加）
            "hip_pitch_antiphase_v2": 0.8,  # 両脚動作必須版
            # 両脚動作報酬（V9追加）
            "both_legs_active": 0.5,  # 両脚が共に動いていることを報酬化
            # ============================================================
            # 【安定性ペナルティ】（Unitree方式）
            # ============================================================
            "lin_vel_z": -2.0,  # Z軸速度ペナルティ
            "ang_vel_xy": -0.1,  # V9から継続
            "orientation": -1.0,  # V9から継続
            "base_height": -5.0,  # 高さ維持
            # ============================================================
            # 【歩行品質ペナルティ】
            # ============================================================
            "feet_swing_height": -10.0,  # ★強化: V9: -5.0 → V10: -10.0（足上げ促進）
            "contact_no_vel": -0.1,  # 接地時足速度
            "hip_pos": -0.5,  # 股関節位置（開脚抑制）
            # 速度未達ペナルティ
            "velocity_deficit": -2.0,  # V6から継続
            # ★【新規】足引きずりペナルティ（V10追加）
            "feet_stumble": -0.5,  # 接地中の足の水平速度をペナルティ化
            # ============================================================
            # 【エネルギー効率ペナルティ】
            # ============================================================
            "torques": -1e-5,  # トルクペナルティ
            "action_rate": -0.01,  # ★強化: V9: -0.005 → V10: -0.01（滑らかさ向上）
            "dof_acc": -2.5e-7,  # 関節加速度
            "dof_vel": -0.005,  # 関節速度（振動抑制）
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.10, 0.15],  # V8から継続
        "lin_vel_y_range": [0, 0],  # 横移動なし
        "ang_vel_range": [-0.2, 0.2],  # ★新規: Yaw訓練追加（V8: [0, 0]）
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main():
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V10)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v10")
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
    print("EXP007 V10: 足上げ改善と滑らかな歩行の実現")
    print(f"{'=' * 70}")
    print("【V9の成功と残課題】")
    print("  成功: 左右対称性大幅改善（L/R比0.91）、Yawドリフト-30.74°")
    print("  課題: hip_pitch相関+0.605（同期方向へ悪化）")
    print("        足引きずり、断続的な速度推移")
    print(f"{'=' * 70}")
    print("【V10の改善方針】")
    print("  1. symmetry報酬の緩和")
    print("     - symmetry: 1.0 → 0.5（hip_pitch同期を緩和）")
    print("  2. 足上げ高さの増加")
    print("     - swing_height_target: 0.03 → 0.05m")
    print("     - feet_swing_height: -5.0 → -10.0")
    print("  3. 足引きずりペナルティの新規追加")
    print("     - feet_stumble: -0.5")
    print("  4. 動作滑らかさの改善")
    print("     - action_rate: -0.005 → -0.01")
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
