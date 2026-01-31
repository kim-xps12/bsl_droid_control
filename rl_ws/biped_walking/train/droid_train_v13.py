"""BSL-Droid Simplified 二脚ロボット Genesis訓練スクリプト v13

============================================================
V13: V9回帰 + 抜本的シンプル化
============================================================

背景分析:
- V9がdroid-walkingで最良の結果を達成（hip_pitch相関-0.516）
- V10以降ではPhase-based報酬が同期を促進する副作用を生んだ
- V12はYawドリフト-47°、斜行-0.85mと大幅悪化

失敗の根本原因:
1. 報酬関数の過剰複雑化（20以上の報酬項目）
2. Phase-based報酬とペナルティベース報酬の競合
3. smooth_action, ground_contact_bonus等が同期歩行を間接促進

V13の設計原則:
1. **V9をベースにする**: 交互歩行を達成した唯一の設計
2. **Phase-based報酬を全削除**: 同期促進の原因を排除
3. **報酬項目を最小限に**: 本質的な項目のみ残す
4. **contact_threshold修正**: 0.025m→0.04m（V11の改善点は維持）

V13の報酬設計（V9ベース、14項目に厳選）:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
■ 主タスク（4項目）
  - tracking_lin_vel: 1.5    # 速度追従
  - tracking_ang_vel: 0.5    # 角速度追従
  - alive: 0.1               # 生存報酬
  - forward_progress: 0.3    # 前進報酬

■ 交互歩行（6項目）- V9の核心
  - hip_pitch_alternation: 4.0    # 左右hip_pitch逆相（V9の成功要因）
  - hip_pitch_sync_penalty: -3.0  # 同期ペナルティ（V9の成功要因）
  - contact_alternation: 1.5      # 接地タイミング交互
  - feet_air_time: 2.0            # 足の滞空時間
  - single_stance: 0.5            # 片足立ち報酬
  - no_fly: -2.0                  # 両足宙浮きペナルティ（V9の-1.0を強化）

■ 姿勢・安定性（4項目）
  - orientation: -2.5       # 姿勢維持
  - base_height: -10.0      # 高さ維持
  - yaw_rate: -1.5          # Yawドリフト対策（V9の-1.0を強化）
  - backward_velocity: -2.0 # 後退ペナルティ
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

削除した報酬（V10-V12で追加したもの）:
- phase_hip_pitch_tracking: 同期促進の原因
- phase_contact_sync: 同期促進の原因
- phase_velocity_sync: 同期促進の原因
- smooth_action: 同期促進の原因
- periodic_foot_lift: 複雑化
- natural_rhythm: 複雑化
- symmetry_hip_roll: V11で追加、効果薄い
- ground_contact_bonus: 両足同時接地を促進
- strict_alternating_contact: V12で追加、効果薄い

期待効果:
- hip_pitch相関: V9相当（<-0.3）
- Yawドリフト: V9相当（<10°/10s）
- 前進距離: V9相当（>2.8m/10s）

Usage:
    cd rl_ws
    uv run python biped_walking/train/droid_train_v13.py --max_iterations 500
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

    # V8/V9と同じ初期姿勢を継続
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
    # V13: V9回帰 + 抜本的シンプル化
    # ============================================================
    # V9の成功要因を維持しつつ、V10-V12で追加した複雑な報酬を全削除
    # 報酬項目数: V12の28項目 → V13の14項目
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.22,  # V9と同じ
        "feet_air_time_target": 0.25,
        "gait_frequency": 1.5,  # Hz（V9と同じ、Phase-basedでは使用しない）

        # V11で導入した接地判定閾値の改善は維持
        "contact_threshold": 0.04,  # 0.025m → 0.04m

        "reward_scales": {
            # ========== 主タスク報酬（4項目） ==========
            "tracking_lin_vel": 1.5,
            "tracking_ang_vel": 0.5,
            "alive": 0.1,
            "forward_progress": 0.3,

            # ========== 交互歩行報酬（6項目）- V9の核心 ==========
            "hip_pitch_alternation": 4.0,      # V9成功要因: 左右hip_pitch逆相
            "hip_pitch_sync_penalty": -3.0,    # V9成功要因: 同期ペナルティ
            "contact_alternation": 1.5,        # V9: 接地タイミング交互
            "feet_air_time": 2.0,              # V9: 足の滞空時間
            "single_stance": 0.5,              # V9: 片足立ち報酬
            "no_fly": -2.0,                    # V9の-1.0を強化

            # ========== 姿勢・安定性ペナルティ（4項目） ==========
            "orientation": -2.5,
            "base_height": -10.0,
            "yaw_rate": -1.5,                  # V9の-1.0を強化（Yawドリフト対策）
            "backward_velocity": -2.0,

            # ========== 膝角度制約（動作には必須） ==========
            "dof_pos_limits": -5.0,
            "knee_negative": -3.0,
            "knee_max_angle": -3.0,

            # ========== 最小限の振動抑制 ==========
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
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v13")
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
