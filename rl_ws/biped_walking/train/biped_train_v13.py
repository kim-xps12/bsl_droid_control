"""
BSL-Droid二脚ロボット Genesis訓練スクリプト v13

============================================================
V12の失敗を踏まえた改良版 - 動的な歩行サイクル報酬
============================================================

V12の失敗原因:
1. stride_length報酬 - 静的な開脚を報酬 → 脚を開いたまま固定
2. hip_pitch_opposite_sign報酬 - 静的な前後開脚を報酬
3. hip_pitch_range報酬 - 静的な前方位置を報酬
4. similar_to_default削除 - 極端な姿勢への収束を許容

V13の変更点:

1. V11ベースに戻す
   - similar_to_defaultを復元
   - 基本的な報酬構造を維持

2. V12の静的報酬を削除
   - stride_length → 削除
   - hip_pitch_opposite_sign → 削除
   - hip_pitch_range → 削除

3. 動的な変化を報酬（新規）
   - hip_pitch_alternation: V11から継続、さらに強化（1.5 → 2.0）
   - hip_pitch_velocity: hip_pitchが動いていることを報酬
   - contact_alternation: 接地タイミングの交互性を報酬
   - roll_penalty: V12のRoll傾斜対策

教訓:
- ✗ 静的な位置を報酬 → 固定姿勢に収束
- ○ 動的な速度・変化を報酬 → 歩行動作を誘導

Usage:
    cd rl_ws
    uv run python scripts/biped_train_v13.py --max_iterations 500
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
            "left_hip_pitch_joint": 0.0,
            "left_knee_pitch_joint": -0.52,
            "left_ankle_pitch_joint": 0.52,
            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_pitch_joint": 0.0,
            "right_knee_pitch_joint": -0.52,
            "right_ankle_pitch_joint": 0.52,
        },
        # 正しいリンク名
        "feet_names": ["left_foot_link", "right_foot_link"],
        # PD gains
        "kp": 35.0,
        "kd": 2.0,
        "termination_if_roll_greater_than": 30,
        "termination_if_pitch_greater_than": 30,
        "base_init_pos": [0.0, 0.0, 0.45],
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
    # V13: V12の失敗を踏まえた報酬スケール
    # ============================================================
    #
    # 変更方針:
    # 1. V11ベースに戻す（similar_to_default復元）
    # 2. V12の静的報酬を削除（stride_length等）
    # 3. 動的な変化を報酬（速度ベース）
    # 4. Roll角ペナルティを追加
    #
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.40,
        "feet_air_time_target": 0.25,
        "gait_frequency": 1.5,  # Hz

        "reward_scales": {
            # ========== 主タスク報酬 ==========
            "tracking_lin_vel": 1.5,
            "tracking_ang_vel": 0.5,
            "alive": 0.1,
            "forward_progress": 0.3,

            # ========== 交互歩行報酬（V11ベース）==========
            "alternating_gait": 1.5,
            "foot_swing": 0.8,
            "feet_air_time": 1.0,
            "single_stance": 0.5,
            "no_fly": -1.0,

            # ========== V13: 動的歩行報酬（速度ベース）==========
            "hip_pitch_alternation": 2.0,     # V11: 1.0 → 強化
            "hip_pitch_velocity": 0.5,        # 新規：hip_pitchが動いていることを報酬
            "contact_alternation": 0.8,       # 新規：接地タイミングの交互性

            # ========== 姿勢・安定性ペナルティ ==========
            "orientation": -2.5,
            "base_height": -15.0,
            "lin_vel_z": -2.0,
            "ang_vel_xy": -0.05,
            "pitch_penalty": -3.0,
            "roll_penalty": -3.0,             # V13新規：V12のRoll傾斜対策

            # ========== 振動抑制ペナルティ（V11維持）==========
            "action_rate": -0.03,
            "dof_vel": -1e-3,
            "dof_acc": -5e-7,
            "torques": -5e-5,
            "similar_to_default": -0.02,      # V13で復元（V12で削除したら失敗）
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
    parser.add_argument("-e", "--exp_name", type=str, default="biped-walking-v13")
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

    env = BipedEnv(
        num_envs=args.num_envs, env_cfg=env_cfg, obs_cfg=obs_cfg, reward_cfg=reward_cfg, command_cfg=command_cfg
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()
