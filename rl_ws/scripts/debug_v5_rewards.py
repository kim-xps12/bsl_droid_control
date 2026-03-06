#!/usr/bin/env python
"""V5報酬関数のデバッグスクリプト"""

from __future__ import annotations

import pickle
import sys
from pathlib import Path

import genesis as gs
import torch


sys.path.insert(0, str(Path(__file__).parent / "genesis_official/examples/locomotion"))
from biped_env_v5 import BipedEnvV5


gs.init(backend=gs.gpu, precision="32", logging_level="warning", seed=1)

# 設定読み込み
with open("logs/biped-walking-v5/cfgs.pkl", "rb") as f:
    env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = pickle.load(f)

# 環境作成
env = BipedEnvV5(
    num_envs=1,
    env_cfg=env_cfg,
    obs_cfg=obs_cfg,
    reward_cfg=reward_cfg,
    command_cfg=command_cfg,
    show_viewer=False,
)

print("=== feet_indices ===")
print(f"feet_indices: {env.feet_indices}")
print(f"feet_names: {env.feet_names}")

print("\n=== Initial state check ===")
obs, _ = env.reset()
print(f"commands: {env.commands[0]}")
print(f"command norm: {torch.norm(env.commands[0, :2]).item():.4f}")

# 10ステップ実行して報酬を確認
for _i in range(10):
    actions = torch.zeros((1, env.num_actions), device=gs.device)
    obs, rew, done, info = env.step(actions)

# 各報酬関数を個別にテスト
print("\n=== Reward function debug ===")
contacts = env._get_foot_contacts()
print(f"contacts: {contacts}")

foot_heights = env._get_foot_heights()
print(f"foot_heights: {foot_heights}")

foot_positions = env._get_foot_positions()
print(f"foot_positions shape: {foot_positions.shape if foot_positions is not None else None}")
print(f"foot_positions: {foot_positions}")

print(f"last_feet_pos: {env.last_feet_pos}")

# 報酬関数を直接呼び出し
print("\n=== Direct reward function calls ===")
print(f"foot_clearance: {env._reward_foot_clearance().item():.6f}")
print(f"stride_length: {env._reward_stride_length().item():.6f}")
print(f"alternating_gait: {env._reward_alternating_gait().item():.6f}")
print(f"single_stance: {env._reward_single_stance().item():.6f}")
print(f"feet_air_time: {env._reward_feet_air_time().item():.6f}")
print(f"foot_swing: {env._reward_foot_swing().item():.6f}")
print(f"no_fly: {env._reward_no_fly().item():.6f}")
