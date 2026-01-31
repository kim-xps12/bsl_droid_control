"""
BSL-Droid二脚ロボット Genesis訓練スクリプト v18

============================================================
低重心歩行 + 前進方向制御の修正
============================================================

V18の設計方針:

1. default_joint_anglesをV13に戻す（観測空間の一貫性維持）
   - 観測: (dof_pos - default_dof_pos) が一貫する
   - V17で後退した原因を解消

2. 初期位置は低いまま維持（base_init_pos: 0.35m）
   - 低姿勢から開始して探索を促進

3. 前進報酬の強化
   - tracking_lin_vel: 1.5 → 2.5
   - forward_progress: 0.3 → 0.5
   - backward_penalty: -2.0（新規）

4. 方向安定性の強化
   - yaw_penalty: -3.0（新規）
   - lateral_velocity_penalty: -1.5（新規）

5. 非対称報酬は維持（V17から継承）

6. num_envs: 12288（高速化）

Usage:
    cd rl_ws
    uv run python scripts/biped_train_v18.py --max_iterations 500
"""

import argparse
import os
import pickle
import shutil
from importlib import metadata
from pathlib import Path

try:
    try:
        if metadata.version("rsl-rl"):
            raise ImportError
    except metadata.PackageNotFoundError:
        if metadata.version("rsl-rl-lib") != "2.2.4":
            raise ImportError
except (metadata.PackageNotFoundError, ImportError) as e:
    raise ImportError("Please uninstall 'rsl_rl' and install 'rsl-rl-lib==2.2.4'.") from e
from rsl_rl.runners import OnPolicyRunner

import genesis as gs

import sys
from pathlib import Path

# envsパッケージへのパスを追加
rl_ws_dir = Path(__file__).parent.parent
sys.path.insert(0, str(rl_ws_dir))
from biped_walking.envs.biped_env import BipedEnv


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
    rl_ws_dir = script_dir.parent
    urdf_path = rl_ws_dir / "assets" / "biped_digitigrade.urdf"

    # ============================================================
    # V18: 低重心歩行 + 前進方向制御の修正
    # ============================================================
    #
    # V17の問題:
    # - 低重心化は成功（0.446m → 0.371m）
    # - しかし後方移動（X=-2.832m）、横移動（Y=9.642m）
    #
    # V18の修正:
    # 1. default_joint_anglesをV13に戻す
    #    → 観測空間の一貫性維持（ポリシーが進行方向を正しく学習）
    # 2. 前進報酬を強化、後方移動にペナルティ
    # 3. Yaw・横速度にペナルティで方向安定性向上
    #
    # ============================================================

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
        # V18: default姿勢をV13に戻す（観測空間の一貫性維持）
        "default_joint_angles": {
            "left_hip_yaw_joint": 0.0,
            "left_hip_roll_joint": 0.0,
            "left_hip_pitch_joint": 0.0,    # V17: -0.35 → V18: 0.0（V13と同じ）
            "left_knee_pitch_joint": -0.52, # V17: -0.78 → V18: -0.52（V13と同じ）
            "left_ankle_pitch_joint": 0.52, # V17: 0.43 → V18: 0.52（V13と同じ）
            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_pitch_joint": 0.0,   # 同上
            "right_knee_pitch_joint": -0.52,# 同上
            "right_ankle_pitch_joint": 0.52,# 同上
        },
        "feet_names": ["left_foot_link", "right_foot_link"],
        # PD gains
        "kp": 35.0,
        "kd": 2.0,
        "termination_if_roll_greater_than": 30,
        "termination_if_pitch_greater_than": 30,
        # V18: 初期高さは低いまま維持（低姿勢からの探索促進）
        "base_init_pos": [0.0, 0.0, 0.35],  # V17と同じ（低い位置から開始）
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
    # V18: 報酬設計の修正
    # ============================================================
    #
    # 主要変更点:
    # 1. tracking_lin_vel: 1.5 → 2.5（前進報酬強化）
    # 2. forward_progress: 0.3 → 0.5（前進進捗報酬強化）
    # 3. backward_penalty: -2.0（新規、後方移動ペナルティ）
    # 4. yaw_penalty: -3.0（新規、Yaw角ペナルティ）
    # 5. lateral_velocity_penalty: -1.5（新規、横速度ペナルティ）
    #
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        # V18: 目標高さは維持
        "base_height_target": 0.35,
        # V18: 非対称ペナルティのパラメータ（V17と同じ）
        "height_penalty_high": 40.0,
        "height_penalty_low": 5.0,
        "feet_air_time_target": 0.25,
        "gait_frequency": 1.5,

        "reward_scales": {
            # ========== 主タスク報酬（V18: 強化）==========
            "tracking_lin_vel": 2.5,     # V17: 1.5 → V18: 2.5（強化）
            "tracking_ang_vel": 0.5,
            "alive": 0.1,
            "forward_progress": 0.5,     # V17: 0.3 → V18: 0.5（強化）

            # ========== V18: 方向制御報酬（新規）==========
            "backward_penalty": -2.0,    # 新規：後方移動ペナルティ
            "yaw_penalty": -3.0,         # 新規：Yaw角ペナルティ
            "lateral_velocity_penalty": -1.5,  # 新規：横速度ペナルティ

            # ========== 交互歩行報酬（V13継承）==========
            "alternating_gait": 1.5,
            "foot_swing": 0.8,
            "feet_air_time": 1.0,
            "single_stance": 0.5,
            "no_fly": -1.0,

            # ========== 動的歩行報酬（V13継承）==========
            "hip_pitch_alternation": 2.0,
            "hip_pitch_velocity": 0.5,
            "contact_alternation": 0.8,

            # ========== 姿勢・安定性ペナルティ ==========
            "orientation": -2.5,
            "base_height": -1.0,  # 非対称報酬（関数内で実装）
            "lin_vel_z": -2.0,
            "ang_vel_xy": -0.05,
            "pitch_penalty": -3.0,
            "roll_penalty": -3.0,

            # ========== 振動抑制ペナルティ（V13継承）==========
            "action_rate": -0.03,
            "dof_vel": -1e-3,
            "dof_acc": -5e-7,
            "torques": -5e-5,
            "similar_to_default": -0.02,
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
    parser.add_argument("-e", "--exp_name", type=str, default="biped-walking-v18")
    # V18: num_envsを12288に増加（メモリに余裕があるため高速化）
    parser.add_argument("-B", "--num_envs", type=int, default=12288)
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

    env = BipedEnv(
        num_envs=args.num_envs, env_cfg=env_cfg, obs_cfg=obs_cfg, reward_cfg=reward_cfg, command_cfg=command_cfg
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()
