"""BSL-Droid Simplified 二脚ロボット Genesis訓練スクリプト v16

============================================================
V16: 脚役割スイッチングによる「剣道すり足」解消
============================================================

V15の評価結果:
- X移動距離: 3.050m（最高記録！）
- Y移動距離: -0.039m（ほぼ直進）
- Yawドリフト: -18.11°（V14: -27.2°より改善）
- Roll mean: -2.9°（V14: -11.8°より大幅改善）
- 平均速度: 0.298m/s（目標0.3m/sにほぼ到達）

V15の問題点:
1. **hip_pitch相関が弱い**: -0.242（V9: -0.516）
2. **「剣道すり足」パターン**: 前脚/後脚の役割が固定
   - 視覚的に片脚が常に前、もう片脚が常に後ろ
   - 脚の役割（前/後）が入れ替わらない
3. **hip_pitch範囲の左右差**: L=0.469 rad, R=0.378 rad（左脚主導）

============================================================
V16の設計方針: 脚役割スイッチング強制
============================================================

「剣道すり足」の本質は「脚の役割が入れ替わらない」こと。
hip_pitchの符号変化を報酬化して、前脚⇔後脚の切り替えを促進する。

■ 変更1: hip_pitch_sign_change報酬（新規）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  hip_pitch_sign_change: 3.0 (新規)

hip_pitchが正から負、または負から正に変わるタイミングを報酬化。
これにより「常に前脚」「常に後脚」というパターンを崩す。

■ 変更2: hip_pitch_alternation強化
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  V15: 4.0 → V16: 6.0

V9が-0.516を達成した際はこの報酬が効いていた。
さらに強化して交互動作を促進。

■ 変更3: hip_pitch_sync_penalty強化
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  V15: -3.0 → V16: -5.0

同期歩行（両脚が同方向に動く）へのペナルティを強化。

■ 変更4: V9の追加報酬を復活
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  foot_clearance: 2.0（V9から復活）
  hip_pitch_range: 1.0（V9から復活）
  alternating_gait: 1.5（V9から復活）
  foot_swing: 0.8（V9から復活）

V9で使用していた交互歩行関連の報酬を復活させる。

■ 変更5: symmetry復活（弱く）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  symmetry: -0.3 (V9: -0.5, V14: -1.0)

V9で-0.5だったが、V14で-1.0にしたら逆効果だった。
V9より弱い-0.3で試す。

============================================================
期待効果
============================================================
| 指標 | V9 | V15 | V16期待 |
|------|-----|-----|---------|
| X移動距離 | 2.95m | 3.05m | > 2.8m |
| hip_pitch相関 | -0.516 | -0.242 | < -0.4 |
| Yawドリフト | -4.6° | -18.1° | < 15° |
| Roll mean | 1.3° | -2.9° | < 5° |

Usage:
    cd rl_ws
    uv run python biped_walking/train/droid_train_v16.py --max_iterations 500
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

    # V8/V9/V13/V14/V15と同じ初期姿勢を継続
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
    # V16: 脚役割スイッチングによる「剣道すり足」解消
    # ============================================================
    # V15の良い点（直進性、Roll修正）を維持しつつ
    # V9の交互歩行成功要因を復活・強化
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        # V15と同じ目標高さ
        "base_height_target": 0.24,
        "feet_air_time_target": 0.25,
        "gait_frequency": 1.5,
        "contact_threshold": 0.04,

        "reward_scales": {
            # ========== 主タスク報酬（4項目） ==========
            "tracking_lin_vel": 1.5,
            "tracking_ang_vel": 0.5,
            "alive": 0.1,
            "forward_progress": 0.3,

            # ========== 交互歩行報酬（V9から復活 + V16強化） ==========
            # ★ V16変更2: hip_pitch_alternation強化
            "hip_pitch_alternation": 6.0,   # V15: 4.0 → V16: 6.0
            # ★ V16変更3: hip_pitch_sync_penalty強化
            "hip_pitch_sync_penalty": -5.0, # V15: -3.0 → V16: -5.0
            "contact_alternation": 1.5,
            "feet_air_time": 2.0,
            "single_stance": 0.5,
            "no_fly": -2.0,
            "hip_pitch_velocity": 0.8,

            # ★ V16変更1: 脚役割スイッチング報酬（新規）
            "hip_pitch_sign_change": 3.0,   # 新規: hip_pitchの符号変化を報酬化

            # ★ V16変更4: V9から復活
            "foot_clearance": 2.0,          # V9から復活
            "hip_pitch_range": 1.0,         # V9から復活
            "alternating_gait": 1.5,        # V9から復活
            "foot_swing": 0.8,              # V9から復活

            # ========== 姿勢・安定性ペナルティ ==========
            "orientation": -2.5,
            "base_height": -10.0,
            "yaw_rate": -2.0,               # V15から継承
            "backward_velocity": -2.0,
            "roll_penalty": -5.0,           # V15から継承
            "pitch_penalty": -3.0,

            # ★ V16変更5: symmetry復活（弱く）
            "symmetry": -0.3,               # V9: -0.5, V14: -1.0 → V16: -0.3

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
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v16")
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
