"""
BSL-Droid二脚ロボット Genesis評価スクリプト（統一版）

全バージョン（V1/V2/V3/V4等）のモデルを評価できる汎用スクリプト。
-e オプションで実験名を指定して切り替える。

Usage:
    cd rl_ws

    # V4評価
    uv run python scripts/biped_eval.py -e biped-walking-v4

    # V9評価（最新）
    uv run python scripts/biped_eval.py -e biped-walking-v9

    # 特定チェックポイントを使用
    uv run python scripts/biped_eval.py -e biped-walking-v9 --ckpt 400
"""

import argparse
import os
import pickle
from importlib import metadata

import torch

try:
    try:
        if metadata.version("rsl-rl"):
            raise ImportError
    except metadata.PackageNotFoundError:
        if metadata.version("rsl-rl-lib") != "2.2.4":
            raise ImportError
except (metadata.PackageNotFoundError, ImportError) as e:
    raise ImportError("Please uninstall 'rsl_rl' and install 'rsl-rl-lib==2.2.4'.") from e
from rsl_rl.modules import ActorCritic

import genesis as gs

import sys
import re
import importlib
from pathlib import Path

# envsパッケージへのパスを追加
rl_ws_dir = Path(__file__).parent.parent
sys.path.insert(0, str(rl_ws_dir))


def get_env_class(exp_name: str):
    """実験名に応じて適切な環境クラスを動的に返す
    
    例: "biped-walking-v7" -> envs.biped_env_v7.BipedEnvV7
        "biped-walking" -> envs.biped_env_v2.BipedEnvV2 (デフォルト)
    """
    # バージョン番号を抽出 (例: "biped-walking-v7" -> "v7", "biped-walking" -> None)
    match = re.search(r'-(v\d+)$', exp_name)
    
    if match:
        version = match.group(1)  # "v7"
    else:
        # バージョンなし（V1相当）はV2環境を使用
        version = "v2"
    
    # モジュール名とクラス名を生成
    module_name = f"envs.biped_env_{version}"  # "envs.biped_env_v7"
    class_name = f"BipedEnv{version.upper()}"  # "BipedEnvV7"
    
    # 動的インポート
    module = importlib.import_module(module_name)
    return getattr(module, class_name)


def main():
    parser = argparse.ArgumentParser(description="BSL-Droid Biped Walking Evaluation")
    parser.add_argument("-e", "--exp_name", type=str, default="biped-walking-v4",
                        help="Experiment name (e.g., biped-walking, biped-walking-v2, biped-walking-v3, biped-walking-v4)")
    parser.add_argument("--ckpt", type=int, default=None,
                        help="Checkpoint iteration (default: latest)")
    parser.add_argument("--no-viewer", action="store_true",
                        help="Run without GUI viewer (headless mode)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Evaluation duration in seconds (default: 10.0)")
    args = parser.parse_args()

    gs.init(backend=gs.gpu, precision="32", logging_level="warning", seed=1)

    log_dir = f"logs/{args.exp_name}"
    
    # 設定を読み込み
    with open(f"{log_dir}/cfgs.pkl", "rb") as f:
        env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = pickle.load(f)

    # チェックポイントを決定
    if args.ckpt is None:
        available = [f for f in os.listdir(log_dir) if f.startswith("model_") and f.endswith(".pt")]
        if not available:
            raise FileNotFoundError(f"No checkpoints found in {log_dir}")
        latest = sorted(available, key=lambda x: int(x.split("_")[1].split(".")[0]))[-1]
        ckpt_path = f"{log_dir}/{latest}"
        ckpt_num = int(latest.split("_")[1].split(".")[0])
    else:
        ckpt_path = f"{log_dir}/model_{args.ckpt}.pt"
        ckpt_num = args.ckpt
        if not os.path.exists(ckpt_path):
            raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")

    print(f"=== BSL-Droid Biped Walking Evaluation ===")
    print(f"Experiment: {args.exp_name}")
    print(f"Checkpoint: model_{ckpt_num}.pt")
    print(f"Target velocity: {command_cfg['lin_vel_x_range'][0]} m/s")
    print(f"Action scale: {env_cfg['action_scale']}")
    print()

    # 実験名に応じた環境クラスを取得
    EnvClass = get_env_class(args.exp_name)
    
    # 環境を作成
    env = EnvClass(
        num_envs=1,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        command_cfg=command_cfg,
        show_viewer=not args.no_viewer,
    )

    # ポリシーを読み込み
    policy = ActorCritic(
        num_actor_obs=env.num_obs,
        num_critic_obs=env.num_obs,
        num_actions=env.num_actions,
        actor_hidden_dims=train_cfg["policy"]["actor_hidden_dims"],
        critic_hidden_dims=train_cfg["policy"]["critic_hidden_dims"],
        activation=train_cfg["policy"]["activation"],
    ).to(gs.device)
    
    policy.load_state_dict(torch.load(ckpt_path, map_location=gs.device, weights_only=True)["model_state_dict"])
    policy.eval()
    print(f"Loaded: {ckpt_path}")
    print()
    
    if args.no_viewer:
        print(f"Running headless evaluation for {args.duration} seconds...")
    else:
        print("Running evaluation... Press Ctrl+C to stop.")
    print()

    obs, _ = env.reset()
    step = 0
    max_steps = int(args.duration / env.dt) if args.no_viewer else float('inf')
    
    # 統計収集用
    positions = []
    velocities = []
    orientations = []
    dof_vels = []
    actions_list = []
    dof_pos_list = []
    
    with torch.no_grad():
        while step < max_steps:
            actions = policy.act_inference(obs)
            obs, rew, done, info = env.step(actions)
            
            # データ収集
            pos = env.base_pos[0].cpu().numpy()
            vel = env.base_lin_vel[0].cpu().numpy()
            quat = env.base_quat[0].cpu().numpy()
            dof_pos = env.dof_pos[0].cpu().numpy()
            dof_vel = env.dof_vel[0].cpu().numpy()
            act = actions[0].cpu().numpy()
            
            positions.append(pos.copy())
            velocities.append(vel.copy())
            orientations.append(quat.copy())
            dof_vels.append(dof_vel.copy())
            actions_list.append(act.copy())
            dof_pos_list.append(dof_pos.copy())
            
            # 時間ステップごとの出力（50ステップごと = 1秒ごと）
            if step % 50 == 0:
                dof_vel_rms = torch.sqrt(torch.mean(torch.square(env.dof_vel[0]))).item()
                action_rms = torch.sqrt(torch.mean(torch.square(actions[0]))).item()
                # クォータニオンからオイラー角を計算
                from genesis.utils.geom import quat_to_xyz
                euler = quat_to_xyz(env.base_quat[0:1])[0].cpu().numpy()
                euler_deg = euler * 180 / 3.14159
                
                print(f"t={step*env.dt:5.2f}s | pos=({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:5.3f}) | "
                      f"vel=({vel[0]:5.2f}, {vel[1]:5.2f}) | "
                      f"rpy=({euler_deg[0]:5.1f}, {euler_deg[1]:5.1f}, {euler_deg[2]:5.1f})° | "
                      f"dof_vel_rms={dof_vel_rms:5.2f}")
            
            step += 1
    
    # 統計出力
    import numpy as np
    positions = np.array(positions)
    velocities = np.array(velocities)
    orientations = np.array(orientations)
    dof_vels = np.array(dof_vels)
    actions_list = np.array(actions_list)
    dof_pos_list = np.array(dof_pos_list)
    
    print("\n=== Evaluation Statistics ===")
    print(f"Duration: {args.duration}s ({step} steps)")
    print(f"\nTravel distance:")
    print(f"  X: {positions[-1, 0] - positions[0, 0]:.3f} m")
    print(f"  Y: {positions[-1, 1] - positions[0, 1]:.3f} m")
    print(f"  Total: {np.linalg.norm(positions[-1, :2] - positions[0, :2]):.3f} m")
    print(f"\nAverage velocity:")
    print(f"  X: {np.mean(velocities[:, 0]):.3f} m/s (target: {command_cfg['lin_vel_x_range'][0]:.3f})")
    print(f"  Y: {np.mean(velocities[:, 1]):.3f} m/s")
    print(f"\nBase height:")
    print(f"  Mean: {np.mean(positions[:, 2]):.3f} m")
    print(f"  Std: {np.std(positions[:, 2]):.4f} m")
    print(f"\nDOF velocity RMS: {np.sqrt(np.mean(dof_vels**2)):.3f} rad/s")
    print(f"Action RMS: {np.sqrt(np.mean(actions_list**2)):.3f}")
    print()


if __name__ == "__main__":
    main()
