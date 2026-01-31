#!/usr/bin/env python3
"""
BSL-Droid二脚ロボット Genesis評価スクリプト（ヘッドレス・ログ出力付き）

Usage:
    cd rl_ws
    uv run python scripts/biped_eval_headless.py
    uv run python scripts/biped_eval_headless.py --ckpt 499
"""

import argparse
import pickle
import sys
from pathlib import Path

import torch

# biped_envをインポート
rl_ws_dir = Path(__file__).parent.parent
sys.path.insert(0, str(rl_ws_dir))
from biped_walking.envs.biped_env import BipedEnv

import genesis as gs
from rsl_rl.runners import OnPolicyRunner


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="biped-walking")
    parser.add_argument("--ckpt", type=int, default=499)
    parser.add_argument("--steps", type=int, default=500)
    args = parser.parse_args()

    gs.init(backend=gs.cpu)

    log_dir = f"logs/{args.exp_name}"

    # 設定読み込み
    env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = pickle.load(
        open(f"{log_dir}/cfgs.pkl", "rb")
    )
    reward_cfg["reward_scales"] = {}

    # 環境作成（ヘッドレス）
    env = BipedEnv(
        num_envs=1,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        command_cfg=command_cfg,
        show_viewer=False,
    )

    # ポリシー読み込み
    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)
    resume_path = f"{log_dir}/model_{args.ckpt}.pt"
    print(f"Loading checkpoint: {resume_path}")
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=gs.device)

    print("=== Evaluation Start ===")
    print(f"Command config: {command_cfg}")
    print(f"Steps: {args.steps}")
    print()

    obs, _ = env.reset()
    total_reward = 0

    for step in range(args.steps):
        with torch.no_grad():
            actions = policy(obs)
        obs, rews, dones, infos = env.step(actions)
        total_reward += rews.item()

        if step % 50 == 0:
            base_pos = env.base_pos[0].cpu().numpy()
            base_lin_vel = env.base_lin_vel[0].cpu().numpy()
            print(
                f"Step {step:3d}: "
                f"pos=({base_pos[0]:7.3f}, {base_pos[1]:7.3f}, {base_pos[2]:7.3f}), "
                f"vel=({base_lin_vel[0]:6.3f}, {base_lin_vel[1]:6.3f}, {base_lin_vel[2]:6.3f}), "
                f"rew={rews.item():.4f}"
            )

        if dones.any():
            print(f"Episode ended at step {step}")
            break

    print()
    print(f"=== Total reward: {total_reward:.2f} ===")
    print(f"Final position: x={env.base_pos[0, 0].item():.3f} m")


if __name__ == "__main__":
    main()
