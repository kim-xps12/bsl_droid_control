#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト（Unitree参考版V8）

============================================================
【EXP007 V8: 交互歩行報酬の修正と胴体安定性強化】
============================================================

【V7の結果と課題】
V7は速度目標（0.214 m/s）を達成したが、新規報酬関数の設計ミスにより
意図した効果が得られなかった：

- 成功点:
  - X速度: 0.214 m/s（目標0.15 m/sを43%超過）
  - 単足接地率: 95.6%（維持）

- 課題点:
  - hip_pitch相関: 0.781→0.828（6%悪化、より同期）
  - Roll std: 1.09°→6.21°（470%悪化、胴体傾斜）
  - alternating_gait報酬: 位置和の報酬化は両脚静止でも達成可能
  - foot_flat報酬: ankle_pitchのみでankle_rollが未対応

【V8の改善方針】
1. **交互歩行報酬の修正**
   - alternating_gait（位置和）を削除
   - hip_pitch_antiphase（速度逆相関）を新規追加
   - 動的な交互動作を誘導

2. **胴体安定性の強化**
   - orientation: -0.5 → -1.0（姿勢ペナルティ強化）
   - ang_vel_xy: -0.05 → -0.1（角速度ペナルティ強化）
   - lin_vel_x_range: [0.15, 0.25] → [0.10, 0.15]（V6レベルに戻す）

3. **足首制御の強化**
   - foot_flat: -3.0 → -5.0（強化）
   - ankle_roll: -2.0（新規、hip_rollで代替）

【V8パラメータ変更一覧】
| パラメータ            | V7値       | V8値         | 変更理由                        |
|----------------------|------------|--------------|--------------------------------|
| lin_vel_x_range      | [0.15,0.25]| [0.10, 0.15] | V6レベルに戻し安定性優先        |
| alternating_gait     | 0.5        | 削除         | 設計ミス、機能していない        |
| hip_pitch_antiphase  | (なし)     | 0.8          | 【新規】速度逆相関報酬          |
| ankle_roll           | (なし)     | -2.0         | 【新規】足首ロールペナルティ    |
| foot_flat            | -3.0       | -5.0         | 強化（ankle_pitchペナルティ）   |
| orientation          | -0.5       | -1.0         | 胴体姿勢ペナルティ強化          |
| ang_vel_xy           | -0.05      | -0.1         | 胴体角速度ペナルティ強化        |

【成功基準】
| 指標 | V7値 | V8目標 | 判定基準 |
|------|------|--------|---------|
| X速度 | 0.214 m/s | > 0.10 m/s | 低速でも安定歩行を優先 |
| hip_pitch相関 | +0.828 | < +0.5 | 同期度改善 |
| Roll std | 6.21° | < 3.0° | 胴体傾斜抑制 |
| エピソード長 | 1001 | > 900 | 安定性維持 |
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
    # V8: 交互歩行報酬の修正と胴体安定性強化
    # ============================================================
    reward_cfg = {
        "tracking_sigma": 0.10,  # V6から継続（ガウシアン鋭化）
        "base_height_target": 0.20,  # 目標胴体高さ
        "swing_height_target": 0.03,  # 遊脚の目標高さ
        "gait_frequency": 0.8,  # 歩行周波数（V6と同じ）
        "contact_threshold": 0.08,  # 接地判定閾値
        "air_time_offset": 0.20,  # V7から継続
        "reward_scales": {
            # ============================================================
            # 【主報酬】速度追従
            # ============================================================
            "tracking_lin_vel": 1.5,  # 線速度追従
            "tracking_ang_vel": 0.5,  # 角速度追従
            # ============================================================
            # 【歩行品質報酬】
            # ============================================================
            "feet_air_time": 1.5,  # 滞空時間報酬
            "contact": 0.3,  # V7から継続
            "alive": 0.03,  # 生存報酬
            # 片足接地報酬
            "single_foot_contact": 1.0,  # V7から継続
            # 左右対称性報酬
            "symmetry": 0.3,  # V6から継続
            # ★【新規】hip_pitch速度逆相関報酬（V8追加）
            # V7のalternating_gait（位置和）は削除（設計ミス）
            "hip_pitch_antiphase": 0.8,  # 速度逆相関で動的交互歩行を誘導
            # 歩幅報酬（V7から継続）
            "step_length": 0.3,  # 足間距離を報酬化
            # ============================================================
            # 【安定性ペナルティ】（Unitree方式）★V8強化
            # ============================================================
            "lin_vel_z": -2.0,  # Z軸速度ペナルティ
            "ang_vel_xy": -0.1,  # ★強化: V7: -0.05 → V8: -0.1
            "orientation": -1.0,  # ★強化: V7: -0.5 → V8: -1.0
            "base_height": -5.0,  # 高さ維持
            # ============================================================
            # 【歩行品質ペナルティ】
            # ============================================================
            "feet_swing_height": -5.0,  # 遊脚高さ
            "contact_no_vel": -0.1,  # 接地時足速度
            "hip_pos": -0.5,  # 股関節位置（開脚抑制）
            # 速度未達ペナルティ
            "velocity_deficit": -2.0,  # V6から継続
            # 足裏水平ペナルティ ★強化
            "foot_flat": -5.0,  # ★強化: V7: -3.0 → V8: -5.0
            # ★【新規】足首ロールペナルティ（V8追加）
            "ankle_roll": -2.0,  # hip_rollで代替
            # ============================================================
            # 【エネルギー効率ペナルティ】
            # ============================================================
            "torques": -1e-5,  # トルクペナルティ
            "action_rate": -0.005,  # アクション変化率
            "dof_acc": -2.5e-7,  # 関節加速度
            "dof_vel": -0.005,  # 関節速度（振動抑制）
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.10, 0.15],  # ★変更: V7: [0.15, 0.25] → V8: [0.10, 0.15]
        "lin_vel_y_range": [0, 0],  # 横移動なし
        "ang_vel_range": [0, 0],  # 旋回なし
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main():
    """メインエントリーポイント"""
    parser = argparse.ArgumentParser(description="Train BSL-Droid Simplified Walking (Unitree Reference V8)")
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-unitree-v8")
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
    print("EXP007 V8: 交互歩行報酬の修正と胴体安定性強化")
    print(f"{'=' * 70}")
    print("【V7の成功と残課題】")
    print("  成功: X速度0.214 m/s達成（目標43%超過）")
    print("  課題: hip_pitch相関悪化（0.828）、Roll std悪化（6.21°）")
    print("        alternating_gait/foot_flat報酬の設計ミス")
    print(f"{'=' * 70}")
    print("【V8の改善方針】")
    print("  1. 交互歩行報酬の修正")
    print("     - alternating_gait: 削除（位置和→機能せず）")
    print("     - hip_pitch_antiphase: 0.8（速度逆相関で動的交互歩行）")
    print("  2. 胴体安定性の強化")
    print("     - orientation: -0.5 → -1.0")
    print("     - ang_vel_xy: -0.05 → -0.1")
    print("     - lin_vel_x_range: [0.15, 0.25] → [0.10, 0.15]（V6レベル）")
    print("  3. 足首制御の強化")
    print("     - foot_flat: -3.0 → -5.0")
    print("     - ankle_roll: -2.0（新規）")
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
