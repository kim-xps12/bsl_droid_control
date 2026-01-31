"""BSL-Droid Simplified 二脚ロボット Genesis訓練スクリプト v10

============================================================
V10: Phase-based Reference Trajectory（位相同期参照軌道）
============================================================

コンセプト: 「かわいく生き物のように歩く」

先行研究からの知見:
1. Legged Gym (ETH): feet_air_time報酬で周期的足上げ
2. Cassie Biped: no_fly報酬で単脚接地を促進
3. Energy Minimization (Berkeley): エネルギー最小化で自然な歩容創発
4. CPG (Central Pattern Generator): 神経科学的な周期的パターン生成

V10のアプローチ:
- 正弦波の参照軌道をhip_pitchに与え、位相同期で追従
- 左右180°位相差で自然な交互歩行を促進
- 接地タイミングも位相に同期させる
- 滑らかな動作変化を報酬化

新規報酬関数:
1. phase_hip_pitch_tracking: 正弦波参照軌道追従
2. phase_contact_sync: 接地タイミング-位相同期
3. phase_velocity_sync: hip_pitch速度-位相同期
4. smooth_action: 滑らかなアクション変化
5. periodic_foot_lift: 周期的足上げ
6. natural_rhythm: 自然なリズム（ゼロクロス検出）

Usage:
    cd rl_ws
    uv run python biped_walking/train/droid_train_v10.py --max_iterations 500
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

    # V8/V9と同じ初期姿勢を継続
    import math
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
        "termination_if_roll_greater_than": 30,
        "termination_if_pitch_greater_than": 30,
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
    # V10: Phase-based Reference Trajectory 報酬設定
    # ============================================================
    # 「かわいく生き物のように歩く」ための設計:
    # 1. 正弦波参照軌道への追従を主報酬に
    # 2. 交互歩行は参照軌道の180°位相差で自然に創発
    # 3. 滑らかな動作変化を報酬化
    # 4. 従来の報酬は補助的に使用
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.22,
        "feet_air_time_target": 0.25,
        "gait_frequency": 1.2,  # Hz（V9の1.5より少し遅く、かわいらしく）

        # Phase-based trajectory parameters
        "ref_hip_pitch_amplitude": 0.25,  # rad ≈ 14°（ゆったりとした振幅）
        "ref_hip_pitch_offset": 0.0,      # rad
        "phase_tracking_sigma": 0.1,      # 追従報酬のシグマ

        "reward_scales": {
            # ========== Phase-based 主報酬（V10新規） ==========
            "phase_hip_pitch_tracking": 3.0,  # 正弦波参照軌道追従
            "phase_contact_sync": 1.5,        # 接地-位相同期
            "phase_velocity_sync": 1.0,       # 速度-位相同期
            "smooth_action": 1.0,             # 滑らかなアクション
            "periodic_foot_lift": 2.0,        # 周期的足上げ
            "natural_rhythm": 0.5,            # 自然なリズム

            # ========== 主タスク報酬 ==========
            "tracking_lin_vel": 1.5,
            "tracking_ang_vel": 0.5,
            "alive": 0.1,
            "forward_progress": 0.3,

            # ========== 従来の交互歩行報酬（補助的） ==========
            "alternating_gait": 0.5,      # V9: 1.5 → V10: 0.5（phase報酬に移行）
            "foot_swing": 0.3,
            "feet_air_time": 1.0,         # V9: 2.0 → V10: 1.0
            "single_stance": 0.3,
            "no_fly": -0.5,

            # ========== hip_pitch動作報酬（補助的） ==========
            "hip_pitch_alternation": 1.0,  # V9: 4.0 → V10: 1.0（phase報酬に移行）
            "hip_pitch_velocity": 0.3,
            "contact_alternation": 0.5,    # V9: 1.5 → V10: 0.5

            # ========== V9からの同期ペナルティ（維持） ==========
            "hip_pitch_sync_penalty": -1.0,  # V9: -3.0 → V10: -1.0

            # ========== 姿勢・安定性ペナルティ ==========
            "orientation": -2.5,
            "base_height": -10.0,
            "base_height_high": -5.0,
            "base_height_low": -10.0,
            "lin_vel_z": -2.0,
            "ang_vel_xy": -0.05,
            "pitch_penalty": -3.0,
            "roll_penalty": -3.0,

            # ========== Yawドリフト対策 ==========
            "yaw_rate": -1.0,
            "symmetry": -0.5,

            # ========== 膝角度制約 ==========
            "dof_pos_limits": -5.0,
            "knee_negative": -3.0,
            "knee_max_angle": -3.0,

            # ========== 後退ペナルティ ==========
            "backward_velocity": -2.0,

            # ========== 振動抑制ペナルティ ==========
            "action_rate": -0.02,  # V9: -0.03（smooth_action報酬があるので緩和）
            "dof_vel": -1e-3,
            "dof_acc": -5e-7,
            "torques": -5e-5,
            "similar_to_default": -0.01,  # V9: -0.02（phase追従があるので緩和）
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
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v10")
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
