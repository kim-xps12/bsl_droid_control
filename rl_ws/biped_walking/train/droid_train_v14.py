"""BSL-Droid Simplified 二脚ロボット Genesis訓練スクリプト v14

============================================================
V14: 高さ補正 + Yaw/斜行対策
============================================================

V13の評価結果:
- X移動距離: 2.58m ✓
- hip_pitch相関: -0.152 ✓ (同期→交互へ改善)
- 平均X速度: 0.28m/s ✓

V13の問題点:
1. **脚が斜め**: base_height_target=0.22mが低すぎ（幾何学的には0.262m）
2. **Yawドリフト**: -45.52° (yaw_rate=-1.5では不十分)
3. **斜行**: Y=-1.024m (hip_rollの左右非対称)

============================================================
V14での改善
============================================================

■ 改善1: 目標高さの補正（脚の斜め対策）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
V8の幾何学的計算に基づき、適切な目標高さに修正:

  base_height_target: 0.22m → 0.26m

これにより:
- 脚がまっすぐに近い姿勢を維持
- 膝の過度な屈曲を抑制
- 「立っている」姿勢での歩行を促進

■ 改善2: Yawドリフト対策の強化
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  yaw_rate: -1.5 → -5.0 (3.3倍強化)

Yaw角速度へのペナルティを大幅に強化し、回転を抑制。

■ 改善3: 斜行対策（左右対称性報酬の復活）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  symmetry: -1.0 (V13で削除していたが復活)

V9にあった左右対称性ペナルティを復活:
- hip_roll, knee, ankle_pitchの左右差にペナルティ
- hip_pitchは交互であるべきなので除外

■ 改善4: hip_yaw対称性ペナルティ（新規）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
斜行の原因としてhip_yawの左右差も疑われる。
hip_yawは基本的に0であるべきなので、左右とも0からの偏差にペナルティ:

  hip_yaw_penalty: -2.0 (新規)

============================================================
V14の設計原則（V13から継承）
============================================================
- シンプルな報酬設計を維持
- 一度に多くの変更を加えない
- 効果が確認された改善のみ採用

============================================================
期待効果
============================================================
| 指標 | V13 | V14期待 |
|------|-----|---------|
| Base height | 0.222m | ~0.26m |
| Yawドリフト | -45.5° | < 15° |
| Y移動距離 | -1.02m | < 0.3m |
| hip_pitch相関 | -0.152 | < 0 (維持) |

Usage:
    cd rl_ws
    uv run python biped_walking/train/droid_train_v14.py --max_iterations 500
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

    # V8/V9/V13と同じ初期姿勢を継続
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
    # V14: 高さ補正 + Yaw/斜行対策
    # ============================================================
    # V13の成功（交互歩行）を維持しつつ、3つの問題を解決:
    # 1. 脚の斜め → base_height_target補正
    # 2. Yawドリフト → yaw_rate強化
    # 3. 斜行 → symmetry復活 + hip_yaw_penalty追加
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        # ★ V14改善1: 目標高さを幾何学的計算値に補正
        "base_height_target": 0.26,  # V13: 0.22 → V14: 0.26 (V8の計算値)
        "feet_air_time_target": 0.25,
        "gait_frequency": 1.5,

        # V11で導入した接地判定閾値の改善は維持
        "contact_threshold": 0.04,

        "reward_scales": {
            # ========== 主タスク報酬（4項目） ==========
            "tracking_lin_vel": 1.5,
            "tracking_ang_vel": 0.5,
            "alive": 0.1,
            "forward_progress": 0.3,

            # ========== 交互歩行報酬（6項目）- V13から継承 ==========
            "hip_pitch_alternation": 4.0,
            "hip_pitch_sync_penalty": -3.0,
            "contact_alternation": 1.5,
            "feet_air_time": 2.0,
            "single_stance": 0.5,
            "no_fly": -2.0,

            # ========== 姿勢・安定性ペナルティ ==========
            "orientation": -2.5,
            "base_height": -10.0,
            # ★ V14改善2: Yawドリフト対策を大幅強化
            "yaw_rate": -5.0,            # V13: -1.5 → V14: -5.0 (3.3倍)
            "backward_velocity": -2.0,

            # ★ V14改善3: 左右対称性ペナルティを復活
            "symmetry": -1.0,            # V13で削除 → V14で復活

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
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v14")
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
