"""
BSL-Droid二脚ロボット Genesis評価スクリプト（統一版）

全バージョン（biped-walking-*, droid-walking-*）のモデルを評価できる汎用スクリプト。
-e オプションで実験名を指定して切り替える。

Usage:
    cd rl_ws

    # biped-walking系の評価
    uv run python biped_walking/biped_eval.py -e biped-walking-v22

    # droid-walking系の評価
    uv run python biped_walking/biped_eval.py -e droid-walking-v1

    # 特定チェックポイントを使用
    uv run python biped_walking/biped_eval.py -e droid-walking-v1 --ckpt 400

    # MP4録画（ヘッドレスモード推奨）
    uv run python biped_walking/biped_eval.py -e droid-walking-v1 --record droid_v1_walking.mp4 --duration 10
"""

import argparse
import importlib
import os
import pickle
import re
import sys
from importlib import metadata
from pathlib import Path

# rsl-rl-libバージョンチェック（rsl_rlインポート前に実行）
try:
    try:
        if metadata.version("rsl-rl"):
            raise ImportError
    except metadata.PackageNotFoundError:
        if metadata.version("rsl-rl-lib") != "2.2.4":
            raise ImportError
except (metadata.PackageNotFoundError, ImportError) as e:
    raise ImportError("Please uninstall 'rsl_rl' and install 'rsl-rl-lib==2.2.4'.") from e

import genesis as gs
import torch
from rsl_rl.modules import ActorCritic

# biped_walkingパッケージへのパスを追加
rl_ws_dir = Path(__file__).parent.parent
sys.path.insert(0, str(rl_ws_dir))


def get_env_class(exp_name: str):
    """実験名に応じて適切な環境クラスを動的に返す

    統一版環境を使用:
    - "biped-walking-*" -> biped_walking.envs.biped_env.BipedEnv
    - "droid-walking-unitree-v{N}" -> biped_walking.envs.droid_env_unitree_v{N}.DroidEnvUnitreeV{N}（動的生成）
    - "droid-walking-v26" -> biped_walking.envs.droid_env_taskspace_e2e_v26.DroidEnvTaskSpaceE2EV26
    - "droid-walking-v25" -> biped_walking.envs.droid_env_taskspace_e2e_v25.DroidEnvTaskSpaceE2EV25
    - "droid-walking-v24" -> biped_walking.envs.droid_env_taskspace_e2e.DroidEnvTaskSpaceE2E
    - "droid-walking-v22", "droid-walking-v23" -> biped_walking.envs.droid_env_taskspace.DroidEnvTaskSpace
    - "droid-walking-*" -> biped_walking.envs.droid_env.DroidEnv
    """
    # droid-walking-unitree-v{N}: Unitree参考実装（exp007）- 全バージョン共通の統合環境を使用
    if re.match(r"droid-walking-unitree-v\d+", exp_name):
        from biped_walking.envs.droid_env_unitree import DroidEnvUnitree

        return DroidEnvUnitree

    # droid-walking-v26: 足先空間改善版E2E環境
    if exp_name.startswith("droid-walking-v26"):
        from biped_walking.envs.droid_env_taskspace_e2e_v26 import DroidEnvTaskSpaceE2EV26
        return DroidEnvTaskSpaceE2EV26

    # droid-walking-v25: 報酬バランス改善版E2E環境
    if exp_name.startswith("droid-walking-v25"):
        from biped_walking.envs.droid_env_taskspace_e2e_v25 import DroidEnvTaskSpaceE2EV25
        return DroidEnvTaskSpaceE2EV25

    # droid-walking-v24: TaskSpace E2E環境を使用
    if exp_name.startswith("droid-walking-v24"):
        from biped_walking.envs.droid_env_taskspace_e2e import DroidEnvTaskSpaceE2E
        return DroidEnvTaskSpaceE2E

    # droid-walking-v22, v23: TaskSpace環境を使用
    if exp_name.startswith("droid-walking-v22") or exp_name.startswith("droid-walking-v23"):
        from biped_walking.envs.droid_env_taskspace import DroidEnvTaskSpace
        return DroidEnvTaskSpace

    # droid-walking系の場合: 統一版を使用
    if exp_name.startswith("droid-walking"):
        from biped_walking.envs.droid_env import DroidEnv
        return DroidEnv

    # biped-walking系の場合: 統一版を使用
    from biped_walking.envs.biped_env import BipedEnv
    return BipedEnv


def get_urdf_path(exp_name: str, rl_ws_dir: Path) -> Path:
    """実験名に応じてURDFパスを返す"""
    if exp_name.startswith("droid-walking"):
        return rl_ws_dir / "assets" / "bsl_droid_simplified.urdf"
    else:
        return rl_ws_dir / "assets" / "biped_digitigrade.urdf"


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
    parser.add_argument("--record", type=str, default=None,
                        help="Record to MP4 file (e.g., --record output.mp4)")
    parser.add_argument("--fps", type=int, default=30,
                        help="FPS for recorded video (default: 30)")
    args = parser.parse_args()

    gs.init(backend=gs.gpu, precision="32", logging_level="warning", seed=1)

    log_dir = f"logs/{args.exp_name}"

    # 設定を読み込み
    with open(f"{log_dir}/cfgs.pkl", "rb") as f:
        cfgs = pickle.load(f)
        # 新形式（dict）と旧形式（tuple）の両方に対応
        if isinstance(cfgs, dict):
            env_cfg = cfgs["env_cfg"]
            obs_cfg = cfgs["obs_cfg"]
            reward_cfg = cfgs["reward_cfg"]
            command_cfg = cfgs["command_cfg"]
            train_cfg = cfgs["train_cfg"]
        else:
            env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = cfgs

    # URDFパスを現在の環境に合わせて上書き（異なるマシンで学習した場合の互換性）
    script_dir = Path(__file__).resolve().parent
    rl_ws_dir = script_dir.parent
    urdf_path = get_urdf_path(args.exp_name, rl_ws_dir)
    env_cfg["urdf_path"] = str(urdf_path)

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

    print("=== BSL-Droid Biped Walking Evaluation ===")
    print(f"Experiment: {args.exp_name}")
    print(f"Checkpoint: model_{ckpt_num}.pt")
    # command_cfgのキー名の互換性対応
    lin_vel_x_key = "lin_vel_x_range" if "lin_vel_x_range" in command_cfg else "lin_vel_x"
    print(f"Target velocity: {command_cfg[lin_vel_x_key][0]} m/s")
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
        show_viewer=not args.no_viewer and not args.record,  # 録画時はviewerオフ
        recording_camera=args.record is not None,  # 録画用カメラを追加
    )

    # 録画用カメラの参照を取得
    recording_cam = env.recording_cam if hasattr(env, 'recording_cam') else None
    if recording_cam is not None:
        recording_cam.start_recording()
        print(f"Recording to: {args.record}")

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

    if args.record:
        print(f"Recording evaluation for {args.duration} seconds...")
    elif args.no_viewer:
        print(f"Running headless evaluation for {args.duration} seconds...")
    else:
        print("Running evaluation... Press Ctrl+C to stop.")
    print()

    obs, _ = env.reset()
    step = 0
    max_steps = int(args.duration / env.dt) if (args.no_viewer or args.record) else float('inf')

    # 統計収集用
    positions = []
    velocities = []
    orientations = []
    euler_angles = []  # RPY角度
    dof_vels = []
    actions_list = []
    dof_pos_list = []
    contact_states = []  # 接地状態 [left, right]

    with torch.no_grad():
        while step < max_steps:
            actions = policy.act_inference(obs)
            obs, rew, done, info = env.step(actions)

            # 録画用カメラのレンダリング（ロボットを追従）
            if recording_cam is not None:
                robot_pos = env.base_pos[0].cpu().numpy()
                # カメラをロボットの斜め後方に配置して追従
                recording_cam.set_pose(
                    pos=(robot_pos[0] - 0.5, robot_pos[1] - 2.0, 1.2),
                    lookat=(robot_pos[0], robot_pos[1], 0.4),
                )
                recording_cam.render()

            # データ収集
            pos = env.base_pos[0].cpu().numpy()
            vel = env.base_lin_vel[0].cpu().numpy()
            quat = env.base_quat[0].cpu().numpy()
            dof_pos = env.dof_pos[0].cpu().numpy()
            dof_vel = env.dof_vel[0].cpu().numpy()
            act = actions[0].cpu().numpy()

            # オイラー角を取得
            from genesis.utils.geom import quat_to_xyz
            euler = quat_to_xyz(env.base_quat[0:1])[0].cpu().numpy()
            euler_deg = euler * 180 / 3.14159

            # 接地状態を取得（環境に_get_foot_contactsがある場合）
            if hasattr(env, '_get_foot_contacts'):
                contacts = env._get_foot_contacts()
                if contacts is not None:
                    contact_state = contacts[0].cpu().numpy()  # [left, right]
                else:
                    contact_state = np.array([True, True])  # デフォルト
            else:
                contact_state = np.array([True, True])

            positions.append(pos.copy())
            velocities.append(vel.copy())
            orientations.append(quat.copy())
            euler_angles.append(euler_deg.copy())
            dof_vels.append(dof_vel.copy())
            actions_list.append(act.copy())
            dof_pos_list.append(dof_pos.copy())
            contact_states.append(contact_state.copy())

            # 時間ステップごとの出力（10ステップごと = 0.2秒ごと）
            if step % 10 == 0:
                dof_vel_rms = torch.sqrt(torch.mean(torch.square(env.dof_vel[0]))).item()
                action_rms = torch.sqrt(torch.mean(torch.square(actions[0]))).item()

                print(f"t={step*env.dt:5.2f}s | pos=({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:5.3f}) | "
                      f"vel=({vel[0]:5.2f}, {vel[1]:5.2f}) | "
                      f"rpy=({euler_deg[0]:5.1f}, {euler_deg[1]:5.1f}, {euler_deg[2]:5.1f})° | "
                      f"dof_vel_rms={dof_vel_rms:5.2f}")

            step += 1

    # 録画を保存
    if recording_cam is not None:
        recording_cam.stop_recording(save_to_filename=args.record, fps=args.fps)
        print(f"\nVideo saved to: {args.record}")

    # 統計出力
    import numpy as np
    positions = np.array(positions)
    velocities = np.array(velocities)
    orientations = np.array(orientations)
    euler_angles = np.array(euler_angles)
    dof_vels = np.array(dof_vels)
    actions_list = np.array(actions_list)
    dof_pos_list = np.array(dof_pos_list)
    contact_states = np.array(contact_states)

    print("\n=== Evaluation Statistics ===")
    print(f"Duration: {args.duration}s ({step} steps)")
    print("\nTravel distance:")
    print(f"  X: {positions[-1, 0] - positions[0, 0]:.3f} m")
    print(f"  Y: {positions[-1, 1] - positions[0, 1]:.3f} m")
    print(f"  Total: {np.linalg.norm(positions[-1, :2] - positions[0, :2]):.3f} m")
    print("\nAverage velocity:")
    # command_cfgのキー名の互換性対応
    lin_vel_x_key = "lin_vel_x_range" if "lin_vel_x_range" in command_cfg else "lin_vel_x"
    print(f"  X: {np.mean(velocities[:, 0]):.3f} m/s (target: {command_cfg[lin_vel_x_key][0]:.3f})")
    print(f"  Y: {np.mean(velocities[:, 1]):.3f} m/s")
    print("\nBase height:")
    print(f"  Mean: {np.mean(positions[:, 2]):.3f} m")
    print(f"  Std: {np.std(positions[:, 2]):.4f} m")

    # 姿勢統計（追加）
    print("\nOrientation (deg):")
    print(f"  Roll:  mean={np.mean(euler_angles[:, 0]):6.2f}, std={np.std(euler_angles[:, 0]):5.2f}")
    print(f"  Pitch: mean={np.mean(euler_angles[:, 1]):6.2f}, std={np.std(euler_angles[:, 1]):5.2f}")
    print(f"  Yaw:   start={euler_angles[0, 2]:6.2f}, end={euler_angles[-1, 2]:6.2f}, drift={euler_angles[-1, 2] - euler_angles[0, 2]:+6.2f}")

    print(f"\nDOF velocity RMS: {np.sqrt(np.mean(dof_vels**2)):.3f} rad/s")
    print(f"Action RMS: {np.sqrt(np.mean(actions_list**2)):.3f}")

    # 左右脚の分離分析
    # BSL-Droid関節インデックス:
    #   L: [hip_yaw(0), hip_roll(1), hip_pitch(2), knee_pitch(3), ankle_pitch(4)]
    #   R: [hip_yaw(5), hip_roll(6), hip_pitch(7), knee_pitch(8), ankle_pitch(9)]
    left_leg_indices = [0, 1, 2, 3, 4]
    right_leg_indices = [5, 6, 7, 8, 9]
    joint_names = ["hip_yaw", "hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch"]

    print("\n=== Joint Movement Analysis ===")
    print("\nDOF position range (rad) [min, max, range]:")
    for i, name in enumerate(joint_names):
        left_min = np.min(dof_pos_list[:, left_leg_indices[i]])
        left_max = np.max(dof_pos_list[:, left_leg_indices[i]])
        right_min = np.min(dof_pos_list[:, right_leg_indices[i]])
        right_max = np.max(dof_pos_list[:, right_leg_indices[i]])
        print(f"  {name:12s}: L=[{left_min:6.3f}, {left_max:6.3f}] ({left_max-left_min:.3f})  "
              f"R=[{right_min:6.3f}, {right_max:6.3f}] ({right_max-right_min:.3f})")

    print("\nDOF velocity std (rad/s):")
    left_vel_std = np.std(dof_vels[:, left_leg_indices], axis=0)
    right_vel_std = np.std(dof_vels[:, right_leg_indices], axis=0)
    for i, name in enumerate(joint_names):
        print(f"  {name:12s}: L={left_vel_std[i]:.3f}  R={right_vel_std[i]:.3f}")

    # 左右の位相差分析（hip_pitchの相関）
    # hip_pitch: index 2 (left), index 7 (right)
    left_hip_pitch = dof_pos_list[:, 2]
    right_hip_pitch = dof_pos_list[:, 7]
    correlation = np.corrcoef(left_hip_pitch, right_hip_pitch)[0, 1]
    print(f"\nLeft-Right hip_pitch correlation: {correlation:.3f}")
    print("  (-1.0 = perfect alternating, +1.0 = perfect synchronized)")

    # 膝角度分析（V5評価用に追加）
    # droid-walking系: インデックス3, 8 (left_knee_pitch, right_knee_pitch)
    # biped-walking系: インデックス2, 7 (left_knee_pitch, right_knee_pitch)
    if args.exp_name.startswith("droid-walking"):
        left_knee_idx, right_knee_idx = 3, 8
    else:
        left_knee_idx, right_knee_idx = 2, 7

    left_knee = dof_pos_list[:, left_knee_idx]
    right_knee = dof_pos_list[:, right_knee_idx]

    print("\n=== Knee Angle Analysis ===")
    print(f"Left knee:  min={np.min(left_knee):6.3f}, max={np.max(left_knee):6.3f}, mean={np.mean(left_knee):6.3f} rad")
    print(f"Right knee: min={np.min(right_knee):6.3f}, max={np.max(right_knee):6.3f}, mean={np.mean(right_knee):6.3f} rad")

    # 膝が負角度に入った割合（droid系のみ意味がある）
    if args.exp_name.startswith("droid-walking"):
        left_negative_ratio = np.mean(left_knee < 0) * 100
        right_negative_ratio = np.mean(right_knee < 0) * 100
        print("\nNegative angle ratio (should be 0% for proper digitigrade):")
        print(f"  Left knee:  {left_negative_ratio:5.1f}%")
        print(f"  Right knee: {right_negative_ratio:5.1f}%")

    # 接地パターン分析
    print("\n=== Contact Pattern Analysis ===")
    both_contact = np.all(contact_states, axis=1)  # 両足接地
    no_contact = np.all(~contact_states, axis=1)   # 両足離地
    single_contact = ~both_contact & ~no_contact   # 片足接地

    total_steps = len(contact_states)
    print(f"Both feet grounded:   {np.sum(both_contact):5d} steps ({np.mean(both_contact)*100:5.1f}%)")
    print(f"Single foot grounded: {np.sum(single_contact):5d} steps ({np.mean(single_contact)*100:5.1f}%)")
    print(f"Both feet airborne:   {np.sum(no_contact):5d} steps ({np.mean(no_contact)*100:5.1f}%)")

    # 歩行品質スコア
    total_dof_range = np.sum([np.max(dof_pos_list[:, i]) - np.min(dof_pos_list[:, i]) for i in range(10)])
    print("\nGait quality indicators:")
    print(f"  Total DOF range sum: {total_dof_range:.3f} rad")
    print("  (Higher = more movement, typically >2.0 for good walking)")
    print()


if __name__ == "__main__":
    main()
