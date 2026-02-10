"""BSL-Droid Simplified 二脚ロボット Genesis訓練スクリプト v15

============================================================
V15: Roll傾斜修正 + V9安定性への回帰
============================================================

V14の評価結果:
- X移動距離: 2.43m (V9: 2.95m から悪化)
- Yawドリフト: -27.2° (V9: -4.6° より悪化、V13: -45.5° より改善)
- Y移動距離: -0.40m (V9: 0.01m より悪化、V13: -1.02m より改善)
- hip_pitch相関: -0.388 (V9: -0.516 より悪化)

V14の深刻な問題点:
1. **Roll角が-11.8°**: 左に大きく傾いている（V9: 1.3°, V13: 0.3°）
2. **hip_pitch rangeが小さい**: L=0.368, R=0.395 (V9: L=0.477, R=0.529)
3. **DOF velocity左右非対称**: 右脚が2倍以上速く動いている
   - hip_pitch: L=0.343, R=0.834 rad/s
   - 「片足で支えて反対の片足で飛び跳ね」の原因

V14での変更の影響分析:
- symmetry: -1.0 → 強すぎて学習不安定化の可能性
- base_height_target: 0.26m → 高すぎて不安定化の可能性
- yaw_rate: -5.0 → Yaw抑制の代わりにRoll傾斜が発生

============================================================
V15の設計方針: V9への回帰 + Roll対策
============================================================

V9が最も安定（Roll=1.3°, Yaw=-4.6°, X=2.95m）だったため、
V9に近い設定に戻しつつ、Roll傾斜への直接対策を追加。

■ 変更1: base_height_target中間値
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  V14: 0.26m → V15: 0.24m (V9: 0.22m との中間)

高すぎると不安定、低すぎると脚が斜めになる。
中間値で様子を見る。

■ 変更2: yaw_rate緩和
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  V14: -5.0 → V15: -2.0 (V9: -1.0 より少し強化)

過度なYaw抑制がRoll傾斜を誘発した可能性。
V9に近い値に緩和。

■ 変更3: symmetry削除
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  V14: -1.0 → V15: 削除

V13で削除して問題なかった。V14で復活させたが逆効果だった。
削除してV13と同様の状態に戻す。

■ 変更4: roll_penalty追加（新規）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  roll_penalty: -5.0 (新規)

V14でRoll=-11.8°という深刻な傾斜が発生。
Roll角を直接ペナルティ化してバランスを改善。

■ 変更5: hip_pitch_velocity強化
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  V14: なし → V15: 0.8 (V9と同じ)

V14ではhip_pitch rangeが小さかった（ストライド不足）。
hip_pitch velocityを報酬化してストライド拡大を促進。

============================================================
期待効果
============================================================
| 指標 | V9 | V14 | V15期待 |
|------|-----|-----|---------|
| X移動距離 | 2.95m | 2.43m | > 2.7m |
| Roll mean | 1.3° | -11.8° | < 5° |
| Yawドリフト | -4.6° | -27.2° | < 15° |
| hip_pitch相関 | -0.516 | -0.388 | < -0.4 |

Usage:
    cd rl_ws
    uv run python biped_walking/train/droid_train_v15.py --max_iterations 500
"""

from __future__ import annotations

import argparse
import math
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
import sys

import genesis as gs
from rsl_rl.runners import OnPolicyRunner


# envsパッケージへのパスを追加
rl_ws_dir = Path(__file__).parent.parent
sys.path.insert(0, str(rl_ws_dir))
from biped_walking.envs.droid_env import DroidEnv


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
    script_dir = Path(__file__).parent
    rl_ws_dir = script_dir.parent.parent
    urdf_path = rl_ws_dir / "assets" / "bsl_droid_simplified.urdf"

    # V8/V9/V13/V14と同じ初期姿勢を継続
    hip_pitch_rad = 60 * math.pi / 180  # 1.047 rad
    knee_pitch_rad = -100 * math.pi / 180  # -1.745 rad
    ankle_pitch_rad = 45 * math.pi / 180  # 0.785 rad

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
    # V15: Roll傾斜修正 + V9安定性への回帰
    # ============================================================
    # V14の問題（Roll=-11.8°、ストライド不足）を解決
    # V9の安定性に近づける設定に回帰
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        # ★ V15変更1: 目標高さを中間値に調整
        "base_height_target": 0.24,  # V14: 0.26 → V15: 0.24 (V9: 0.22との中間)
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
            # ========== 交互歩行報酬（7項目）- V9/V13から継承 ==========
            "hip_pitch_alternation": 4.0,
            "hip_pitch_sync_penalty": -3.0,
            "contact_alternation": 1.5,
            "feet_air_time": 2.0,
            "single_stance": 0.5,
            "no_fly": -2.0,
            # ★ V15変更5: hip_pitch_velocity復活（V9と同じ）
            "hip_pitch_velocity": 0.8,  # V14: なし → V15: 0.8
            # ========== 姿勢・安定性ペナルティ ==========
            "orientation": -2.5,
            "base_height": -10.0,
            # ★ V15変更2: yaw_rate緩和
            "yaw_rate": -2.0,  # V14: -5.0 → V15: -2.0
            "backward_velocity": -2.0,
            # ★ V15変更4: roll_penalty追加（Roll傾斜直接対策）
            "roll_penalty": -5.0,  # 新規追加
            "pitch_penalty": -3.0,  # V9から復活
            # ★ V15変更3: symmetry削除（V14から削除）
            # "symmetry": -1.0,          # 削除
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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="droid-walking-v15")
    parser.add_argument("-B", "--num_envs", type=int, default=4096)
    parser.add_argument("--max_iterations", type=int, default=500)
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

    env = DroidEnv(
        num_envs=args.num_envs, env_cfg=env_cfg, obs_cfg=obs_cfg, reward_cfg=reward_cfg, command_cfg=command_cfg
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()
