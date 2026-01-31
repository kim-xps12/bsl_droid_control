"""
BSL-Droid二脚ロボット Genesis訓練スクリプト v10

============================================================
研究調査に基づく報酬設計の根本的見直し版
============================================================

設計原則:
1. V4をベース（交互歩行成功の実績）
2. feet_namesバグ修正（V6から継承）
3. 報酬スケールを研究標準値に近づける
4. Phase-based trajectory trackingを削除（強制周期問題の回避）

報酬スケール設計根拠:
- legged_gym (ANYmal, G1) の標準値を参照
- humanoid-gym の標準値を参照
- "Not Only Rewards But Also Constraints" 論文の知見
- V2/V8の失敗（過剰ペナルティ）を回避

参考値比較:
| 報酬項目 | G1標準 | V4 | V8(失敗) | V10 |
|----------|--------|-----|----------|------|
| tracking_lin_vel | 1.0 | 2.0 | 2.0 | 1.5 |
| tracking_ang_vel | 0.5 | 0.3 | 1.5 | 0.5 |
| orientation | -1.0 | -3.0 | -8.0 | -1.5 |
| ang_vel_xy | -0.05 | -0.05 | -0.2 | -0.05 |
| base_height | -10.0 | -30.0 | -30.0 | -15.0 |
| dof_vel | -1e-3 | -1e-4 | -1e-4 | -5e-4 |
| action_rate | -0.01 | -0.02 | -0.015 | -0.01 |

Usage:
    cd rl_ws
    uv run python scripts/biped_train_v10.py --max_iterations 500
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
        # V10: 正しいリンク名（V6バグ修正を継承）
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
    # V10: 研究に基づく報酬スケール設定
    # ============================================================
    #
    # 設計原則:
    # 1. 主タスク報酬（tracking_lin_vel）を支配的に
    # 2. ペナルティは研究標準値程度に抑制
    # 3. 二脚特有の報酬（alternating_gait等）は適度な重み
    # 4. V8の失敗（orientation=-8.0等）を回避
    #
    # ============================================================

    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.40,
        "feet_air_time_target": 0.2,  # 0.2秒程度の滞空

        "reward_scales": {
            # ========== 主タスク報酬（正の報酬）==========
            # これらが合計で報酬の大部分を占めるべき
            "tracking_lin_vel": 1.5,      # 研究標準1.0、V4は2.0 → 中間値
            "tracking_ang_vel": 0.5,      # 研究標準値（V8の1.5は過剰）
            "alive": 0.1,                 # 生存報酬（小さめ）
            "forward_progress": 0.3,      # 前進インセンティブ（standing still防止）

            # ========== 二脚歩行報酬（V4から継承）==========
            "alternating_gait": 1.0,      # V4は1.5 → やや控えめに
            "foot_swing": 0.5,            # V4は0.8 → やや控えめに
            "feet_air_time": 0.5,         # V4は1.0 → やや控えめに
            "single_stance": 0.3,         # V4は0.5 → やや控えめに
            "no_fly": -0.5,               # V4は-1.0 → 緩和

            # ========== 姿勢・安定性ペナルティ（研究標準値）==========
            "orientation": -1.5,          # G1標準-1.0、V4は-3.0 → 中間
            "base_height": -15.0,         # G1標準-10.0、V4は-30.0 → 中間
            "lin_vel_z": -1.0,            # 研究標準: -2.0 → やや緩和
            "ang_vel_xy": -0.05,          # 研究標準値（V8の-0.2は過剰）

            # ========== 滑らかさ・効率ペナルティ（研究標準値）==========
            "action_rate": -0.01,         # 研究標準値（V4の-0.02は過剰傾向）
            "dof_vel": -5e-4,             # G1標準-1e-3 → やや緩和
            "dof_acc": -2.5e-7,           # 研究標準値
            "torques": -5e-5,             # 小さめ（動きを阻害しない）
            "similar_to_default": -0.02,  # 小さめ（歩行を阻害しない）
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
    parser.add_argument("-e", "--exp_name", type=str, default="biped-walking-v10")
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
