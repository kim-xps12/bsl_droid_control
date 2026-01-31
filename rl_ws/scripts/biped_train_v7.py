"""
BSL-Droid二脚ロボット Genesis訓練スクリプト v7

V6からの修正:
- Phase-based Reference軌道追従: ROS2のbiped_gait_controlから移植した楕円弧軌道
- 観測空間にphase信号を追加（39→43次元）
- trajectory_tracking報酬: つま先位置の参照軌道追従を報酬
- phase_consistency報酬: 足の接地状態が位相と一致しているかを報酬
- 歩容パラメータ（gait_cfg）: step_height, step_length, step_frequency

目標: 明示的な参照軌道に沿った周期的な歩行を実現

Usage:
    cd rl_ws
    uv run python scripts/biped_train_v7.py --max_iterations 1000
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
from envs.biped_env_v7 import BipedEnvV7


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
        # V6修正: URDFの正しいリンク名を使用
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
        # ===== V7: 歩容パラメータ（ROS2 biped_gait_controlと一致）=====
        "gait_cfg": {
            "step_height": 0.04,      # 足の持ち上げ高さ [m]
            "step_length": 0.08,      # 歩幅 [m]
            "step_frequency": 0.8,    # 歩行周波数 [Hz]（ROS2の0.5Hzより少し速め）
            "thigh_length": 0.18,     # 大腿長 [m]（URDFと一致）
            "shank_length": 0.20,     # 下腿長 [m]（URDFと一致）
        },
    }

    # V7: 観測空間を39→43次元に拡張（phase信号4次元追加）
    obs_cfg = {
        "num_obs": 43,
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }

    # V7: Phase-based Reference報酬を追加
    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.40,
        "feet_air_time_target": 0.25,
        "trajectory_tracking_sigma": 0.05,  # V7: 軌道追従の誤差許容（5cm）
        "reward_scales": {
            # === 追従報酬 ===
            "tracking_lin_vel": 2.0,
            "tracking_ang_vel": 0.3,

            # ===== V7: Phase-based Reference報酬（新規・最重要）=====
            "trajectory_tracking": 5.0,    # 参照軌道追従（高い重み）
            "phase_consistency": 2.0,      # 位相と接地状態の一致

            # === 交互歩行報酬（V4から継続）===
            "alternating_gait": 1.0,       # 緩和（trajectory_trackingが主導）
            "foot_swing": 0.5,
            "feet_air_time": 0.8,
            "no_fly": -0.8,
            "single_stance": 0.3,

            # === 滑らかさペナルティ ===
            "action_rate": -0.02,
            "dof_vel": -0.0001,
            "dof_acc": -1e-7,

            # === 姿勢・高さ維持 ===
            "orientation": -3.0,
            "base_height": -30.0,
            "lin_vel_z": -1.5,
            "ang_vel_xy": -0.05,

            # === その他 ===
            "similar_to_default": -0.02,
            "torques": -0.0001,
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
    parser.add_argument("-e", "--exp_name", type=str, default="biped-walking-v7")
    parser.add_argument("-B", "--num_envs", type=int, default=4096)
    parser.add_argument("--max_iterations", type=int, default=1000)
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

    env = BipedEnvV7(
        num_envs=args.num_envs, env_cfg=env_cfg, obs_cfg=obs_cfg, reward_cfg=reward_cfg, command_cfg=command_cfg
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()
