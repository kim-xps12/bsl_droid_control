"""
BSL-Droid二脚ロボット Genesis訓練スクリプト v8

V7からの修正:
- Yaw補正強化: tracking_ang_velを0.3→1.5に増加、直進性を重視
- ロール安定化: orientationペナルティを-3.0→-8.0、ang_vel_xyを-0.05→-0.2に強化
- 歩行周波数調整: step_frequency 0.8→0.6Hzに下げて自然なペースに
- trajectory_tracking係数を5.0→3.0に下げ、速度追従とのバランスを改善

目標: V7の滑らかさを維持しつつ、直進性とロール安定性を改善

Usage:
    cd rl_ws
    uv run python scripts/biped_train_v8.py --max_iterations 500
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
from envs.biped_env_v8 import BipedEnvV8


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
        # ===== V8: 歩容パラメータ（周波数を下げて自然なペースに）=====
        "gait_cfg": {
            "step_height": 0.04,      # 足の持ち上げ高さ [m]
            "step_length": 0.08,      # 歩幅 [m]
            "step_frequency": 0.6,    # 歩行周波数 [Hz]（V7の0.8Hzから下げる）
            "thigh_length": 0.18,     # 大腿長 [m]（URDFと一致）
            "shank_length": 0.20,     # 下腿長 [m]（URDFと一致）
        },
    }

    # V8: 観測空間（V7と同じ43次元）
    obs_cfg = {
        "num_obs": 43,
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }

    # V8: Yaw補正・ロール安定化を強化した報酬設定
    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.40,
        "feet_air_time_target": 0.25,
        "trajectory_tracking_sigma": 0.05,
        "reward_scales": {
            # === 追従報酬（V8: 角速度追従を大幅強化）===
            "tracking_lin_vel": 2.0,
            "tracking_ang_vel": 1.5,       # V7: 0.3 → V8: 1.5 (直進性強化)

            # ===== Phase-based Reference報酬（係数を下げてバランス調整）=====
            "trajectory_tracking": 3.0,    # V7: 5.0 → V8: 3.0
            "phase_consistency": 1.5,      # V7: 2.0 → V8: 1.5

            # === 交互歩行報酬 ===
            "alternating_gait": 1.0,
            "foot_swing": 0.5,
            "feet_air_time": 0.8,
            "no_fly": -0.8,
            "single_stance": 0.3,

            # ===== V8: 新規報酬関数 =====
            "symmetric_gait": 1.0,         # 左右対称な歩行を促進
            "smooth_joint_velocity": -1.0, # ジャーク（急激な速度変化）を抑制
            "heading_alignment": 0.8,      # 進行方向と向きを一致させる

            # === 滑らかさペナルティ ===
            "action_rate": -0.015,         # V7: -0.02 → V8: -0.015 (少し緩和)
            "dof_vel": -0.0001,
            "dof_acc": -1e-7,

            # === 姿勢・高さ維持（V8: ロール安定化強化）===
            "orientation": -8.0,           # V7: -3.0 → V8: -8.0 (大幅強化)
            "base_height": -30.0,
            "lin_vel_z": -1.5,
            "ang_vel_xy": -0.2,            # V7: -0.05 → V8: -0.2 (4倍強化)

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
    parser.add_argument("-e", "--exp_name", type=str, default="biped-walking-v8")
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

    env = BipedEnvV8(
        num_envs=args.num_envs, env_cfg=env_cfg, obs_cfg=obs_cfg, reward_cfg=reward_cfg, command_cfg=command_cfg
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()
