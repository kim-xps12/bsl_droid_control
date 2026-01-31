"""BSL-Droid Simplified 二脚ロボット Genesis訓練スクリプト v5

============================================================
V5: 初期姿勢で膝を深く曲げる + 膝ペナルティ緩和
============================================================

V4の問題:
- 膝ペナルティを強化しても負角度侵入を防げない
- 副作用としてhip_pitch可動域が1/3に縮小 (0.5rad → 0.15rad)
- 原因: 初期膝角度 (0.6rad ≈ 34°) が浅く、歩行中に負角度に入らざるを得ない

V5の変更点 (V4からの差分):
1. 膝初期角度を深くする: 0.6rad → 1.2rad (約69°)
2. 股関節・足首を調整: hip_pitch -0.3→-0.5, ankle -0.3→-0.7
3. 初期高さを下げる: 0.30m → 0.25m
4. 膝ペナルティをV3レベルに戻す: knee_positive -2.0→-0.5
5. knee_min_angleを削除

Usage:
    cd rl_ws
    uv run python biped_walking/train/droid_train_v5.py --max_iterations 500
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
from biped_walking.envs.droid_env import DroidEnv


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
        # BSL-Droid Simplified用デフォルト関節角度
        # V5: 膝を深く曲げた初期姿勢
        "default_joint_angles": {
            "left_hip_yaw_joint": 0.0,
            "left_hip_roll_joint": 0.0,
            "left_hip_pitch_joint": -0.5,    # V4: -0.3 → V5: -0.5（深く前傾）
            "left_knee_pitch_joint": 1.2,    # V4: 0.6 → V5: 1.2（約69°、深く曲げる）
            "left_ankle_pitch_joint": -0.7,  # V4: -0.3 → V5: -0.7（膝と相殺）
            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_pitch_joint": -0.5,   # V4: -0.3 → V5: -0.5
            "right_knee_pitch_joint": 1.2,   # V4: 0.6 → V5: 1.2
            "right_ankle_pitch_joint": -0.7, # V4: -0.3 → V5: -0.7
        },
        # 正しいリンク名
        "feet_names": ["left_foot_link", "right_foot_link"],
        # PD gains
        "kp": 35.0,
        "kd": 2.0,
        "termination_if_roll_greater_than": 30,
        "termination_if_pitch_greater_than": 30,
        # V5: 膝を深く曲げるため初期高さを下げる
        "base_init_pos": [0.0, 0.0, 0.25],   # V4: 0.30 → V5: 0.25
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
    # V5: 膝ペナルティをV3レベルに戻す
    # ============================================================
    #
    # V4では knee_positive=-2.0, knee_min_angle=-1.0 としたが:
    # - 膝の負角度侵入は防げなかった
    # - 副作用としてhip_pitch可動域が1/3に縮小
    #
    # V5では:
    # - knee_positive: -2.0 → -0.5 (V3と同じ)
    # - knee_min_angle: 削除
    # - 代わりに初期姿勢で膝を深く曲げる
    #
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        # V5: 高さ目標も下げる (膝を深く曲げるため)
        "base_height_target": 0.22,
        "feet_air_time_target": 0.25,
        "gait_frequency": 1.5,  # Hz

        "reward_scales": {
            # ========== 主タスク報酬 ==========
            "tracking_lin_vel": 1.5,
            "tracking_ang_vel": 0.5,
            "alive": 0.1,
            "forward_progress": 0.3,

            # ========== 交互歩行報酬 ==========
            "alternating_gait": 1.5,
            "foot_swing": 0.8,
            "feet_air_time": 1.0,
            "single_stance": 0.5,
            "no_fly": -1.0,

            # ========== 動的歩行報酬 (速度ベース) ==========
            "hip_pitch_alternation": 2.0,
            "hip_pitch_velocity": 0.5,
            "contact_alternation": 0.8,

            # ========== 姿勢・安定性ペナルティ ==========
            "orientation": -2.5,
            "base_height": -10.0,
            "base_height_high": -5.0,
            "lin_vel_z": -2.0,
            "ang_vel_xy": -0.05,
            "pitch_penalty": -3.0,
            "roll_penalty": -3.0,

            # ========== V5: 膝ペナルティをV3レベルに戻す ==========
            "knee_positive": -0.5,      # V4: -2.0 → V5: -0.5 (V3と同じ)
            # knee_min_angle: 削除 (V4で追加したが効果なし)

            # ========== 後退ペナルティ ==========
            "backward_velocity": -2.0,

            # ========== 振動抑制ペナルティ ==========
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
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v5")
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
