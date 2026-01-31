"""
BSL-Droid二脚ロボット Genesis訓練スクリプト v12

============================================================
V11の評価結果に基づく改良版 - 脚の交差問題への対処
============================================================

V11からの主な変更点:

1. similar_to_defaultを削除
   - default位置に戻りすぎることで脚の振り幅が制限される問題を解消

2. 脚交差報酬を新規追加
   - hip_pitch_opposite_sign: 左右hip_pitchが反対符号になることを報酬
   - hip_pitch_range: hip_pitchが正（前方）領域を使うことを報酬
   - stride_length: 左右hip_pitch差分（ストライド長）を報酬

3. hip_pitch_alternationを強化
   - 1.0 → 1.5

4. 交互歩行報酬の維持
   - V11から継続

問題点（V11）:
- 脚が交差しない: hip_pitchが常に負（後方）領域のみを使用
- 左hip_pitch範囲: [-0.529, 0.000]
- 右hip_pitch範囲: [-0.196, 0.000]
- 正の値（前方への振り出し）が一度も出現しない

目標:
- hip_pitch相関: 0.495 → < -0.3（真の逆位相）
- hip_pitch範囲: 負だけでなく正も使用（脚の交差）
- ストライド長: 現状より大きく

Usage:
    cd rl_ws
    uv run python scripts/biped_train_v12.py --max_iterations 500
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
    # V12: V11評価結果に基づく報酬スケール - 脚交差問題対策
    # ============================================================
    #
    # 変更方針:
    # 1. similar_to_defaultを削除（脚の振り幅制限の原因）
    # 2. 脚交差報酬を新規追加（hip_pitch_opposite_sign, hip_pitch_range, stride_length）
    # 3. hip_pitch_alternationを強化
    #
    # 目標:
    # - hip_pitchが正負両方の領域を使用
    # - 左右hip_pitchが反対符号に（真の交互歩行）
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.40,
        "feet_air_time_target": 0.25,

        "reward_scales": {
            # ========== 主タスク報酬 ==========
            "tracking_lin_vel": 1.5,
            "tracking_ang_vel": 0.5,
            "alive": 0.1,
            "forward_progress": 0.3,

            # ========== 交互歩行報酬（V11維持）==========
            "alternating_gait": 1.5,
            "foot_swing": 0.8,
            "feet_air_time": 1.0,
            "single_stance": 0.5,
            "no_fly": -1.0,

            # ========== V12新規：脚交差報酬 ==========
            "hip_pitch_alternation": 1.5,     # V11: 1.0 → 強化
            "hip_pitch_opposite_sign": 1.5,   # 新規：左右hip_pitchが反対符号
            "hip_pitch_range": 1.0,           # 新規：前方スイングを促進
            "stride_length": 0.8,             # 新規：ストライド長を促進

            # ========== 姿勢・安定性ペナルティ（V11維持）==========
            "orientation": -2.5,
            "base_height": -15.0,
            "lin_vel_z": -2.0,
            "ang_vel_xy": -0.05,
            "pitch_penalty": -3.0,

            # ========== 振動抑制ペナルティ（V11維持）==========
            "action_rate": -0.03,
            "dof_vel": -1e-3,
            "dof_acc": -5e-7,
            "torques": -5e-5,
            # similar_to_default を削除（脚の振り幅制限の原因）
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
    parser.add_argument("-e", "--exp_name", type=str, default="biped-walking-v12")
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
