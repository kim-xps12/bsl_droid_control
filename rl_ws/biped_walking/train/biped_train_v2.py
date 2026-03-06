"""
BSL-Droid二脚ロボット Genesis訓練スクリプト v2

改善版: 滑らかで実機デプロイ向けの動作を学習

主な変更点:
- action_rate ペナルティを10倍強化 (-0.05)
- dof_vel ペナルティ追加（高速振動抑制）
- dof_acc ペナルティ追加（急激な動き抑制）
- action_scale を0.5に拡大（可動域拡大）
- feet_air_time報酬追加（長いストライド促進）

Usage:
    cd rl_ws
    uv run python scripts/biped_train_v2.py

    # ビューア付きで短時間テスト
    uv run python scripts/biped_train_v2.py --max_iterations 10 --num_envs 16

    # 本番訓練（1000イテレーション - より長い訓練で安定）
    uv run python scripts/biped_train_v2.py --max_iterations 1000
"""

from __future__ import annotations

import argparse
import os
import pickle
import shutil
from importlib import metadata
from pathlib import Path
from typing import Any


try:
    try:
        if metadata.version("rsl-rl"):
            raise ImportError
    except metadata.PackageNotFoundError:
        if metadata.version("rsl-rl-lib") != "2.2.4":
            raise ImportError from None
except (metadata.PackageNotFoundError, ImportError) as e:
    raise ImportError("Please uninstall 'rsl_rl' and install 'rsl-rl-lib==2.2.4'.") from e
# envsパッケージへのパスを追加
import sys

import genesis as gs
from rsl_rl.runners import OnPolicyRunner


rl_ws_dir = Path(__file__).parent.parent
sys.path.insert(0, str(rl_ws_dir))
from biped_walking.envs.biped_env import BipedEnv


def get_train_cfg(exp_name: str, max_iterations: int) -> dict[str, Any]:
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


def get_cfgs() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any]]:
    """環境設定を取得"""
    # URDFパスを解決（Genesis assetsからの相対パス）
    script_dir = Path(__file__).parent
    rl_ws_dir = script_dir.parent.parent.parent
    urdf_path = rl_ws_dir / "assets" / "biped_digitigrade.urdf"

    env_cfg = {
        "num_actions": 10,
        "urdf_path": str(urdf_path),
        # 関節名（左脚→右脚）
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
        # 初期関節角度（逆関節に適した立位）
        "default_joint_angles": {
            "left_hip_yaw_joint": 0.0,
            "left_hip_roll_joint": 0.0,
            "left_hip_pitch_joint": 0.0,
            "left_knee_pitch_joint": -0.52,  # 約-30° (前方屈曲)
            "left_ankle_pitch_joint": 0.52,  # 膝と相殺
            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_pitch_joint": 0.0,
            "right_knee_pitch_joint": -0.52,
            "right_ankle_pitch_joint": 0.52,
        },
        # 足のリンク名（feet_air_time用）
        "feet_names": ["left_toe", "right_toe"],
        # PD制御ゲイン（やや低めでソフトな動作）
        "kp": 35.0,
        "kd": 2.0,  # ダンピング増加で振動抑制
        # 終了条件
        "termination_if_roll_greater_than": 30,  # 度
        "termination_if_pitch_greater_than": 30,  # 度
        # 初期姿勢
        "base_init_pos": [0.0, 0.0, 0.45],
        "base_init_quat": [1.0, 0.0, 0.0, 0.0],
        # エピソード設定
        "episode_length_s": 20.0,
        "resampling_time_s": 4.0,
        # ★ action_scale拡大で可動域を広く
        "action_scale": 0.5,
        "simulate_action_latency": True,
        "clip_actions": 10.0,  # クリッピングも調整
    }

    obs_cfg = {
        "num_obs": 39,  # 3 + 3 + 3 + 10 + 10 + 10
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }

    # ★ 報酬設定の大幅改善
    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.40,  # 二脚の目標高さ
        "feet_air_time_target": 0.2,  # 目標滞空時間（秒）
        "reward_scales": {
            # 追従報酬
            "tracking_lin_vel": 1.5,  # 速度追従をより重視
            "tracking_ang_vel": 0.5,
            # ★ 滑らかさペナルティ（大幅強化）
            "action_rate": -0.1,  # 10倍強化: アクション変化を強くペナルティ
            "dof_vel": -0.001,  # 新規: 関節速度ペナルティ（振動抑制）
            "dof_acc": -2.5e-7,  # 新規: 関節加速度ペナルティ
            "smoothness": -0.01,  # 新規: 動作の滑らかさ
            # 姿勢・高さ維持
            "orientation": -5.0,  # 直立維持
            "base_height": -30.0,  # 高さ維持（やや緩和）
            "lin_vel_z": -2.0,  # 上下動抑制
            "ang_vel_xy": -0.05,  # ボディ回転抑制
            # 歩容改善
            "feet_air_time": 1.0,  # 新規: 長いストライド促進
            "no_fly": -0.5,  # 新規: 両足離地ペナルティ
            # その他
            "similar_to_default": -0.05,  # デフォルト姿勢維持（緩和）
            "torques": -0.0001,  # エネルギー効率
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.3, 0.3],  # まずは固定速度で訓練
        "lin_vel_y_range": [0, 0],
        "ang_vel_range": [0, 0],
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="biped-walking-v2")
    parser.add_argument("-B", "--num_envs", type=int, default=4096)
    parser.add_argument("--max_iterations", type=int, default=1000)  # より長い訓練
    args = parser.parse_args()

    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg = get_cfgs()
    train_cfg = get_train_cfg(args.exp_name, args.max_iterations)

    if os.path.exists(log_dir):
        shutil.rmtree(log_dir)
    os.makedirs(log_dir, exist_ok=True)

    with open(f"{log_dir}/cfgs.pkl", "wb") as f:
        pickle.dump(
            [env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg],
            f,
        )

    gs.init(backend=gs.gpu, precision="32", logging_level="warning", seed=train_cfg["seed"], performance_mode=True)

    env = BipedEnv(
        num_envs=args.num_envs, env_cfg=env_cfg, obs_cfg=obs_cfg, reward_cfg=reward_cfg, command_cfg=command_cfg
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()
