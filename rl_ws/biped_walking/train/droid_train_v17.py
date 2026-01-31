"""BSL-Droid Simplified 二脚ロボット Genesis訓練スクリプト v17

============================================================
V17: V9完全回帰 + roll_penalty
============================================================

V16の評価結果（致命的失敗）:
- X移動距離: 3.234m（最高記録）
- Y移動距離: -0.346m（大幅悪化）
- hip_pitch相関: **+0.697**（同期歩行、最悪の結果）
- 「びっこを引く」ような歩行

V16の失敗原因分析:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
`hip_pitch_sign_change`報酬の論理的欠陥:

  left_sign_change + right_sign_change = reward

  | 状態 | 報酬 |
  |------|------|
  | 両脚静止 | 0 |
  | 片脚のみ変化 | 1 |
  | 両脚同時変化 | 2 (最大) ← これが問題！

同時符号変化が最大報酬となり、同期歩行を誘発した。
設計意図（交互歩行促進）とは完全に逆の結果。
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

============================================================
V17の設計方針: V9完全回帰 + roll_penalty
============================================================

V9がhip_pitch相関-0.516を達成した唯一のバージョンであることを踏まえ、
V9の報酬設計に完全回帰する。V16で失敗したhip_pitch_sign_changeは採用しない。

■ 基本方針
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. V9の報酬設計に完全回帰
2. V16のhip_pitch_sign_changeは削除（論理的欠陥）
3. V15で有効だったroll_penaltyのみ追加

■ V16からの変更
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
| パラメータ | V16 | V17 | 理由 |
|-----------|-----|-----|------|
| base_height_target | 0.24m | 0.22m | V9に回帰 |
| hip_pitch_alternation | 6.0 | 4.0 | V9に回帰 |
| hip_pitch_sync_penalty | -5.0 | -3.0 | V9に回帰 |
| hip_pitch_sign_change | 3.0 | 削除 | 論理的欠陥 |
| hip_pitch_range | 1.0 | 削除 | V9にはなかった |
| yaw_rate | -2.0 | -1.0 | V9に回帰 |
| symmetry | -0.3 | -0.5 | V9と同じ |
| roll_penalty | -5.0 | -5.0 | V15で有効、維持 |

============================================================
期待効果
============================================================
| 指標 | V9 | V15 | V16 | V17期待 |
|------|-----|------|------|---------|
| X移動距離 | 2.95m | 3.05m | 3.23m | > 2.8m |
| Y移動距離 | 0.01m | -0.04m | -0.35m | < 0.1m |
| hip_pitch相関 | -0.516 | -0.242 | +0.697 | < -0.4 |
| Yawドリフト | -4.6° | -18.1° | -19.2° | < 10° |
| Roll mean | 1.3° | -2.9° | -1.6° | < 3° |

Usage:
    cd rl_ws
    uv run python biped_walking/train/droid_train_v17.py --max_iterations 500
"""

import argparse
import math
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

    # V9と同じ初期姿勢
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
    # V17: V9完全回帰 + roll_penalty
    # ============================================================
    # V16のhip_pitch_sign_changeが論理的欠陥により同期歩行を誘発したため、
    # V9の報酬設計に完全回帰する。V15で有効だったroll_penaltyのみ追加。
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        # V9と同じ目標高さ
        "base_height_target": 0.22,
        "feet_air_time_target": 0.25,
        "gait_frequency": 1.5,
        # V11以降の接地判定改善は維持
        "contact_threshold": 0.04,

        "reward_scales": {
            # ========== 主タスク報酬（4項目） ==========
            "tracking_lin_vel": 1.5,
            "tracking_ang_vel": 0.5,
            "alive": 0.1,
            "forward_progress": 0.3,

            # ========== 交互歩行報酬（V9から完全継承） ==========
            "hip_pitch_alternation": 4.0,      # V9成功要因
            "hip_pitch_sync_penalty": -3.0,    # V9成功要因
            "contact_alternation": 1.5,
            "feet_air_time": 2.0,
            "single_stance": 0.5,
            "no_fly": -2.0,
            "hip_pitch_velocity": 0.8,         # V9で使用
            "foot_clearance": 2.0,             # V9で使用
            "alternating_gait": 1.5,           # V9で使用
            "foot_swing": 0.8,                 # V9で使用

            # ★ V16のhip_pitch_sign_changeは削除（論理的欠陥）

            # ========== 姿勢・安定性ペナルティ ==========
            "orientation": -2.5,
            "base_height": -10.0,
            "yaw_rate": -1.0,                  # V9と同じ
            "backward_velocity": -2.0,
            "roll_penalty": -5.0,              # V15で有効（V17で維持）
            "pitch_penalty": -3.0,             # V9で使用
            "symmetry": -0.5,                  # V9と同じ

            # ========== 膝角度制約 ==========
            "dof_pos_limits": -5.0,
            "knee_negative": -3.0,
            "knee_max_angle": -3.0,

            # ========== 振動抑制 ==========
            "action_rate": -0.03,
            "dof_vel": -1e-3,
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
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v17")
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
