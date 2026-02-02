#!/usr/bin/env python3
"""
BSL-Droid Simplified 歩行学習スクリプト V20

============================================================
【V20: V18ベース + 交互歩行誘導（最小変更）】
============================================================

【V19の失敗分析】
V19では以下の変更を同時に行い、「動かない」が最適解になった：
- 終了条件の過剰な厳格化（pitch 30→20°, height 0.12→0.15m）
- 姿勢ペナルティの8倍強化（-1.0→-8.0）
- 報酬バランスの崩壊（主報酬2.0 vs ペナルティ-8.0）

【V20の設計方針】
V19の教訓を踏まえ、V18をベースに最小限の変更のみ行う：

1. 終了条件: V18と同じ（roll/pitch 25°, height 0.12m）
2. 報酬バランス: V18と同じ（主報酬優位を維持）
3. 追加報酬: feet_air_time のみ（0.3、控えめに）
4. target_air_time: 0.10秒（V19の0.20sより達成容易）

【報酬設計の原則（V19失敗から確立）】
- 主報酬 ≧ ペナルティ合計 × 2
- 一度に変更するのは1カテゴリのみ
- 終了条件の厳格化は5°刻みで段階的に

【期待効果】
- X移動距離: V18(2.83m)と同等以上
- hip_pitch相関: < 0（交互歩行の兆候）
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
        # 【V20】終了条件 - V18と同じ（V19の厳格化は行わない）
        # ============================================================
        # V19では20°/0.15mに厳格化して失敗 → V18レベルに維持
        "termination_if_roll_greater_than": 25,    # V18と同じ
        "termination_if_pitch_greater_than": 25,   # V18と同じ（V19の20°は厳しすぎ）
        "termination_if_height_lower_than": 0.12,  # V18と同じ（V19の0.15mは厳しすぎ）
        "termination_if_knee_positive": True,      # V18と同じ
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
    # V20: V18ベース + feet_air_time（最小変更）
    # ============================================================
    # V18の報酬バランスを維持しつつ、交互歩行を促進するために
    # feet_air_timeのみを控えめに追加する。
    #
    # 【V19からの教訓】
    # - ペナルティ合計が主報酬を超えると「動かない」が最適解になる
    # - feet_air_time_targetは達成可能な値に設定する必要がある
    # - 複数の変更を同時に行わない
    #
    # 【報酬バランス検証】
    # 主報酬: tracking_lin_vel(2.0) + tracking_ang_vel(0.5) + alive(1.0) = 3.5
    # ペナルティ: orientation(-1.0) + action_rate(-0.02) = -1.02
    # 比率: 3.5 / 1.02 ≈ 3.4 ✓（2.0以上を確保）
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.22,
        "feet_air_time_target": 0.10,  # V19の0.20s → 0.10s（達成容易に）
        "gait_frequency": 1.5,
        "contact_threshold": 0.04,

        "reward_scales": {
            # ============================================================
            # 【タスク報酬】（3項目）: V18と同じ
            # ============================================================
            "tracking_lin_vel": 2.0,     # 前進速度追従（最重要）
            "tracking_ang_vel": 0.5,     # 旋回速度追従
            "alive": 1.0,                # 生存報酬（倒れない動機）

            # ============================================================
            # 【エネルギー効率】（2項目）: V18と同じ
            # ============================================================
            "torques": -1e-4,            # トルク最小化 → 省エネ
            "dof_acc": -1e-6,            # 加速度最小化 → 滑らか

            # ============================================================
            # 【安定性】（1項目）: V18と同じ
            # ============================================================
            "orientation": -1.0,         # 傾きペナルティ（緩め）

            # ============================================================
            # 【振動抑制】（1項目）: V18と同じ
            # ============================================================
            "action_rate": -0.02,        # アクション変化率ペナルティ

            # ============================================================
            # 【V20追加】交互歩行誘導（1項目）
            # ============================================================
            # Legged Gym方式: 足の滞空時間に基づく報酬
            # V19では1.0で負の報酬が発生 → 0.3に抑制
            "feet_air_time": 0.3,        # 控えめに追加
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
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v20")
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
