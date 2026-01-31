#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト V18

============================================================
【パラダイムシフト】ミニマリスト報酬設計
============================================================

【V17までの問題点】
- 20〜25個の報酬項目による「dense reward shaping」
- hip_pitch相関が良くても歩行性能が悪い（V17: corr=-0.637, X=2.68m, Yaw=-29°）
- 報酬項目間の競合・干渉
- 報酬ハッキングの発生

【V18の設計方針】
1. 報酬項目を最小限（6項目）に削減
2. 関節レベルの制約（hip_pitch_*, knee_*, symmetry等）を全削除
3. 制約は終了条件（termination）として実装
4. エネルギー効率報酬を追加（歩行の自然な創発を促進）

【参考】
- ETH Legged Gym: 8-10報酬項目
- Walk These Ways (MIT): タスク報酬+エネルギーペナルティ中心
- CaT (Constraint as Termination): 制約を終了条件として扱う

【報酬設計の哲学】
「どう歩くか」を報酬で教えない。
「前に進め」「倒れるな」「省エネで」だけ伝えて、
歩き方は学習に任せる。
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
            "entropy_coef": 0.01,  # 探索促進のため維持
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

    # V9と同じ初期姿勢（実績あり）
    hip_pitch_rad = 60 * math.pi / 180    # 1.047 rad
    knee_pitch_rad = -100 * math.pi / 180  # -1.745 rad
    ankle_pitch_rad = 45 * math.pi / 180   # 0.785 rad

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
        # 【NEW】終了条件による制約
        # ============================================================
        # 報酬ペナルティではなく終了条件として制約を実装
        "termination_if_roll_greater_than": 25,    # 厳しく：30° → 25°
        "termination_if_pitch_greater_than": 25,   # 厳しく：30° → 25°
        "termination_if_height_lower_than": 0.12,  # 【NEW】基本高さの下限
        "termination_if_knee_positive": True,      # 【NEW】膝が正の角度で終了
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
    # V18: ミニマリスト報酬設計（6項目のみ）
    # ============================================================
    # 【哲学】「どう歩くか」は教えない。「前に進め、省エネで」だけ。
    #
    # 削除した報酬（V17比）:
    # - hip_pitch_alternation, hip_pitch_sync_penalty  ← 交互歩行の強制を廃止
    # - contact_alternation, feet_air_time, single_stance  ← 足運びの強制を廃止
    # - foot_clearance, foot_swing, alternating_gait  ← クリアランスの強制を廃止
    # - hip_pitch_velocity  ← 速度の強制を廃止
    # - symmetry  ← 対称性の強制を廃止
    # - knee_negative, knee_max_angle  ← 終了条件に移行
    # - roll_penalty, pitch_penalty  ← 終了条件に移行
    # - base_height  ← 終了条件に移行
    # - backward_velocity  ← 前進報酬で自然に解決
    # - no_fly  ← 必要なら物理で自然に解決
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.22,  # 終了条件の参照用
        "feet_air_time_target": 0.25,  # 未使用だが互換性のため
        "gait_frequency": 1.5,  # 未使用だが互換性のため
        "contact_threshold": 0.04,  # 互換性のため

        "reward_scales": {
            # ============================================================
            # 【タスク報酬】（3項目）: これだけが「目標」
            # ============================================================
            "tracking_lin_vel": 2.0,     # 前進速度追従（最重要）
            "tracking_ang_vel": 0.5,     # 旋回速度追従
            "alive": 1.0,                # 生存報酬（倒れない動機）

            # ============================================================
            # 【エネルギー効率】（2項目）: 自然な歩行の創発を促進
            # ============================================================
            "torques": -1e-4,            # トルク最小化 → 省エネ
            "dof_acc": -1e-6,            # 加速度最小化 → 滑らか

            # ============================================================
            # 【安定性】（1項目）: 最低限の姿勢制御
            # ============================================================
            "orientation": -1.0,         # 傾きペナルティ（緩め）

            # ============================================================
            # 【振動抑制】（1項目）: アクションの急変防止
            # ============================================================
            "action_rate": -0.02,        # アクション変化率ペナルティ
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.3, 0.3],  # 目標速度 0.3 m/s
        "lin_vel_y_range": [0, 0],
        "ang_vel_range": [0, 0],
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v18")
    parser.add_argument("-B", "--num_envs", type=int, default=4096)
    parser.add_argument("--max_iterations", type=int, default=500)
    args = parser.parse_args()

    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg = get_cfgs()
    train_cfg = get_train_cfg(args.exp_name, args.max_iterations)

    if os.path.exists(log_dir):
        shutil.rmtree(log_dir)
    os.makedirs(log_dir, exist_ok=True)

    pickle.dump(
        [env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg],
        open(f"{log_dir}/cfgs.pkl", "wb"),
    )

    gs.init(backend=gs.gpu, precision="32", logging_level="warning", seed=train_cfg["seed"], performance_mode=True)

    env = DroidEnv(
        num_envs=args.num_envs, env_cfg=env_cfg, obs_cfg=obs_cfg, reward_cfg=reward_cfg, command_cfg=command_cfg
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()
