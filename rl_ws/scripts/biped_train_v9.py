"""
BSL-Droid二脚ロボット Genesis訓練スクリプト v9

V7からの変更点（最小限に抑制）:
- tracking_ang_vel: 0.3 → 0.5（Yaw回転抑制、V8の1.5は過剰だった）
- orientation: -3.0 → -4.0（ロール安定性強化、V8の-8.0は過剰だった）
- symmetric_gait報酬を追加（係数0.5、控えめに）

※V8の失敗から学んだ教訓:
- ペナルティの強化は2倍以下に抑える
- 新規報酬関数は1つずつ追加して効果を検証
- 複数変更の同時導入は原因特定を困難にする

Usage:
    cd rl_ws
    uv run python scripts/biped_train_v9.py --max_iterations 500
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
from envs.biped_env_v9 import BipedEnvV9


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
        # 歩容パラメータ（V7と同じ）
        "gait_cfg": {
            "step_height": 0.04,      # 足の持ち上げ高さ [m]
            "step_length": 0.08,      # 歩幅 [m]
            "step_frequency": 0.8,    # 歩行周波数 [Hz]
            "thigh_length": 0.18,     # 大腿長 [m]
            "shank_length": 0.20,     # 下腿長 [m]
        },
    }

    # 観測空間（V7と同じ43次元）
    obs_cfg = {
        "num_obs": 43,
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }

    # 報酬設定（V7をベースに最小限の変更）
    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.40,
        "feet_air_time_target": 0.25,
        "trajectory_tracking_sigma": 0.05,
        "reward_scales": {
            # === 追従報酬 ===
            "tracking_lin_vel": 2.0,
            "tracking_ang_vel": 0.5,     # V7: 0.3 → V9: 0.5（控えめに強化）

            # === Phase-based Reference報酬 ===
            "trajectory_tracking": 5.0,  # V7と同じ
            "phase_consistency": 2.0,    # V7と同じ

            # === 交互歩行報酬 ===
            "alternating_gait": 1.0,
            "foot_swing": 0.5,
            "feet_air_time": 0.8,
            "no_fly": -0.8,
            "single_stance": 0.3,

            # === 滑らかさペナルティ ===
            "action_rate": -0.02,        # V7と同じ
            "dof_vel": -0.0001,
            "dof_acc": -1e-7,

            # === 姿勢・高さ維持 ===
            "orientation": -4.0,         # V7: -3.0 → V9: -4.0（控えめに強化）
            "base_height": -30.0,
            "lin_vel_z": -1.5,
            "ang_vel_xy": -0.05,         # V7と同じ（V8の-0.2は過剰だった）

            # === その他 ===
            "similar_to_default": -0.02,
            "torques": -0.0001,

            # === V9: 新規報酬（1つだけ）===
            "symmetric_gait": 0.5,       # 控えめに追加
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.3, 0.3],
        "lin_vel_y_range": [0, 0],
        "ang_vel_range": [0, 0],
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="biped-walking-v9")
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

    env = BipedEnvV9(
        num_envs=args.num_envs, env_cfg=env_cfg, obs_cfg=obs_cfg, reward_cfg=reward_cfg, command_cfg=command_cfg
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()
