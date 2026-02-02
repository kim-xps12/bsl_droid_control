#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V12）

============================================================
【EXP007 V12: 歩行品質改善（内股・爪先立ち・小刻み歩行対策）】
============================================================

【V11の結果と課題】
V11は静止ポリシー回避に成功し、X速度0.212 m/sを達成した。
しかし、以下の歩行品質問題が顕在化した：

1. 内股（hip_roll + hip_yaw）:
   - hip_roll: 両脚ともデフォルト(0)を跨がず、常に内側に偏向
     - 左: [-0.442, 0.0]（常に負 = 内転方向のみ）
     - 右: [0.0, 0.332]（常に正 = 内転方向のみ）
   - hip_yaw: 右脚が常に負方向のみ（正方向に動かない非対称性）
   - 原因: hip_pos報酬削除による副作用

2. 爪先立ち（ankle_pitch）:
   - 右ankle_pitchがデフォルトから最大+28°逸脱（左は+2°）
   - 右ankle_pitchの可動域が左の1.78倍と非対称
   - 原因: foot_flat報酬削除による副作用

3. 小刻み歩行（hip_pitch）:
   - 左hip_pitch rangeが0.218rad（12.5°）と極小（右は0.643rad）
   - hip_pitch相関+0.245（同期方向、理想は-1.0に近い値）
   - 左脚がほぼ動いておらず、右脚主導の非対称歩行

【V12の改善方針】
V11レポートの「次バージョンへの提案」に基づき、
歩行品質を改善する。

| パラメータ | V11値 | V12値 | 変更理由 |
|-----------|-------|-------|---------|
| hip_pos | なし | -0.3 | 内股対策（hip_roll/hip_yaw偏向是正） |
| foot_flat | なし | -0.3 | 爪先立ち対策（ankle_pitch逸脱抑制） |
| single_foot_contact | 0.8 | 1.0 | 交互歩行促進（左脚の動作不足改善） |
| air_time_offset | 0.25 | 0.20 | 小刻み歩行対策（歩行サイクル促進） |
| swing_height_target | 0.03 | 0.04 | 足上げ高さ増加（引きずり防止） |

報酬項目数: 15 → 17（復活2: hip_pos, foot_flat）

【symmetryを不採用とする理由】
1. V9でsymmetry=1.0使用時、hip_pitch相関が+0.605に悪化（同期方向へ）
2. 全関節の対称性強制は交互歩行を阻害するリスク
3. 問題は関節ごとに切り分けて対処（hip_pos, foot_flat）

【成功基準】
| 指標 | V11値 | V12目標 | 判定基準 |
|------|-------|---------|---------|
| X速度 | 0.212 m/s | > 0.20 m/s | 維持 |
| hip_pitch相関 | +0.245 | < +0.15 | やや改善 |
| 内股 | hip_roll偏向あり | 偏向減少 | hip_rollがデフォルト(0)を跨ぐ |
| 爪先立ち | 右ankle+28° | 逸脱減少 | 左右差縮小 |
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
    # V12: 歩行品質改善報酬セット（17項目）
    # ============================================================
    # V11からの変更:
    # 1. hip_pos復活: -0.3（内股対策）
    # 2. foot_flat復活: -0.3（爪先立ち対策）
    # 3. single_foot_contact強化: 0.8 → 1.0（交互歩行促進）
    # 4. air_time_offset調整: 0.25 → 0.20（小刻み歩行対策）
    # 5. swing_height_target微増: 0.03 → 0.04（足上げ高さ対策）
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.10,  # V6で効果実証（静止回避）
        "base_height_target": 0.20,  # BSL-Droid向け
        "swing_height_target": 0.04,  # ★V11: 0.03 → V12: 0.04（足上げ高さ増加）
        "gait_frequency": 1.0,  # V4値
        "contact_threshold": 0.08,  # V4で修正済み
        "air_time_offset": 0.20,  # ★V11: 0.25 → V12: 0.20（歩行サイクル促進）
        "reward_scales": {
            # ============================================================
            # 【主報酬】Unitreeと同等
            # ============================================================
            "tracking_lin_vel": 1.5,  # V3-V4で実証済み
            "tracking_ang_vel": 0.5,  # Unitreeと同じ
            # ============================================================
            # 【歩行品質報酬】Unitree方式 + V3-V4実証済み要素
            # ============================================================
            "contact": 0.2,  # Unitree: 0.18、歩行フェーズ整合性
            "single_foot_contact": 1.0,  # ★V11: 0.8 → V12: 1.0（交互歩行促進）
            "feet_air_time": 1.5,  # V3-V4で実証済み
            "alive": 0.03,  # 控えめに設定
            # ============================================================
            # 【安定性ペナルティ】Unitree値を使用
            # ============================================================
            "lin_vel_z": -2.0,  # Unitreeと同じ
            "ang_vel_xy": -0.05,  # Unitree値
            "orientation": -0.5,  # V3-V4レベル
            "base_height": -5.0,  # サーベイ6.2推奨値
            # ============================================================
            # 【歩行品質ペナルティ】
            # ============================================================
            "feet_swing_height": -5.0,  # V3-V4レベル
            "contact_no_vel": -0.1,  # Unitreeと同等
            "velocity_deficit": -2.0,  # V6で効果実証済み
            # ============================================================
            # 【復活報酬】V11で削除したが必要だった報酬
            # ============================================================
            "hip_pos": -0.3,  # ★【復活】内股対策（hip_roll/hip_yaw偏向是正）
            "foot_flat": -0.3,  # ★【復活】爪先立ち対策（ankle_pitch逸脱抑制）
            # ============================================================
            # 【エネルギー効率ペナルティ】
            # ============================================================
            "torques": -1e-5,  # Unitreeと同等
            "dof_acc": -2.5e-7,  # Unitreeと同等
        },
    }
    # 報酬項目数: 17（V11の15から2項目復活: hip_pos, foot_flat）

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.15, 0.25],  # V7値（動作実績あり）
        "lin_vel_y_range": [0, 0],  # 横移動なし
        "ang_vel_range": [0, 0],  # まずは直進のみ（複雑さ回避）
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main():
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V12)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v12")
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
    print("EXP007 V12: 歩行品質改善（内股・爪先立ち・小刻み歩行対策）")
    print(f"{'=' * 70}")
    print("【V11の成果と課題】")
    print("  ✅ 静止ポリシー回避成功: X速度 0.212 m/s達成")
    print("  ⚠️ 内股: hip_roll両脚とも内側に偏向")
    print("  ⚠️ 爪先立ち: 右ankle_pitchがデフォルトから+28°逸脱")
    print("  ⚠️ 小刻み歩行: 左hip_pitch rangeが12.5°と極小")
    print(f"{'=' * 70}")
    print("【V12の改善方針】")
    print("  1. hip_pos復活(-0.3): 内股対策（hip_roll/hip_yaw偏向是正）")
    print("  2. foot_flat復活(-0.3): 爪先立ち対策（ankle_pitch逸脱抑制）")
    print("  3. single_foot_contact強化(0.8→1.0): 交互歩行促進")
    print("  4. air_time_offset調整(0.25→0.20): 歩行サイクル促進")
    print("  5. swing_height_target微増(0.03→0.04): 足上げ高さ増加")
    print(f"{'=' * 70}")
    print("【報酬項目数】")
    print("  V11: 15項目 → V12: 17項目（+2: hip_pos, foot_flat復活）")
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
