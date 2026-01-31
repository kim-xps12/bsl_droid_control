#!/usr/bin/env python3
"""
Sim2Sim: MuJoCoでGenesisで学習した二脚ロボットポリシーを評価するスクリプト

Genesis -> MuJoCo転送の検証

Usage:
    # 事前準備: URDFをMJCFに変換
    cd rl_ws && uv run python scripts/convert_biped_urdf_to_mjcf.py

    # ビューア付きで実行
    cd rl_ws && uv run mjpython scripts/biped_eval_mujoco.py

    # ビューアなし（数値確認のみ）
    cd rl_ws && uv run python scripts/biped_eval_mujoco.py --no-viewer
"""

import argparse
import pickle
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import torch
import torch.nn as nn


# デフォルトのモデルパス
DEFAULT_MODEL_PATH = "assets/biped_digitigrade.xml"


class ActorMLP(nn.Module):
    """Genesisで学習したActorネットワークの再構築"""

    def __init__(
        self,
        num_obs: int,
        num_actions: int,
        hidden_dims: list[int],
        activation: str = "elu",
    ):
        super().__init__()

        activation_fn = {
            "elu": nn.ELU(),
            "relu": nn.ReLU(),
            "tanh": nn.Tanh(),
            "leaky_relu": nn.LeakyReLU(),
        }[activation.lower()]

        layers = []
        in_dim = num_obs
        for hidden_dim in hidden_dims:
            layers.append(nn.Linear(in_dim, hidden_dim))
            layers.append(activation_fn)
            in_dim = hidden_dim
        layers.append(nn.Linear(in_dim, num_actions))

        self.mlp = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.mlp(x)


def quat_rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    ワールド座標からボディローカル座標へ変換（クォータニオン回転の逆）

    Args:
        q: クォータニオン (w, x, y, z) - MuJoCo形式
        v: 回転するベクトル (3,)

    Returns:
        回転後のベクトル (3,)
    """
    q_w, q_x, q_y, q_z = q[0], q[1], q[2], q[3]
    q_vec = np.array([q_x, q_y, q_z])

    a = v - 2.0 * q_w * np.cross(q_vec, v)
    b = a - 2.0 * np.cross(q_vec, np.cross(q_vec, v))

    return b


def compute_observations(
    data: mujoco.MjData,
    model: mujoco.MjModel,
    commands: np.ndarray,
    last_actions: np.ndarray,
    default_dof_pos: np.ndarray,
    obs_scales: dict,
    joint_indices: list[int],
) -> np.ndarray:
    """
    MuJoCoの状態から観測ベクトルを計算

    観測空間 (39次元):
    - ang_vel (3): ボディローカル角速度
    - projected_gravity (3): ボディローカル重力ベクトル
    - commands (3): 速度コマンド (lin_vel_x, lin_vel_y, ang_vel_yaw)
    - dof_pos (10): 関節位置（デフォルトからの偏差）
    - dof_vel (10): 関節速度
    - actions (10): 前回のアクション
    """
    # ベース姿勢クォータニオン (w, x, y, z)
    base_quat = data.qpos[3:7]

    # ワールド座標での角速度 -> ボディローカル座標
    world_ang_vel = data.qvel[3:6]
    local_ang_vel = quat_rotate_inverse(base_quat, world_ang_vel)

    # 重力ベクトル（ワールド座標: (0, 0, -1)）-> ボディローカル座標
    gravity_world = np.array([0.0, 0.0, -1.0])
    projected_gravity = quat_rotate_inverse(base_quat, gravity_world)

    # 関節位置・速度（joint_indicesでマッピング）
    dof_pos = data.qpos[7:][joint_indices]
    dof_vel = data.qvel[6:][joint_indices]

    # 観測を構築（スケーリング適用）
    obs = np.concatenate([
        local_ang_vel * obs_scales.get("ang_vel", 1.0),
        projected_gravity,
        commands * np.array([
            obs_scales.get("lin_vel", 1.0),
            obs_scales.get("lin_vel", 1.0),
            obs_scales.get("ang_vel", 1.0),
        ]),
        (dof_pos - default_dof_pos) * obs_scales.get("dof_pos", 1.0),
        dof_vel * obs_scales.get("dof_vel", 1.0),
        last_actions,
    ])

    return obs.astype(np.float32)


def get_joint_mapping(genesis_joint_names: list[str], mujoco_model: mujoco.MjModel) -> list[int]:
    """
    Genesis関節順序 -> MuJoCoのqpos/qvelインデックスのマッピングを取得

    二脚ロボットの場合、Genesis/MuJoCoともに同じ順序:
    left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee_pitch, left_ankle_pitch,
    right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee_pitch, right_ankle_pitch

    Returns:
        マッピングリスト: mapping[genesis_idx] = mujoco_dof_idx (0-9)
    """
    mujoco_joint_names = []
    for i in range(mujoco_model.njnt):
        joint_name = mujoco_model.joint(i).name
        if joint_name and joint_name != "root":
            mujoco_joint_names.append(joint_name)

    print(f"  - MuJoCo joint names: {mujoco_joint_names}")

    mapping = []
    for genesis_name in genesis_joint_names:
        mujoco_name = genesis_name
        try:
            mujoco_dof_idx = mujoco_joint_names.index(mujoco_name)
            mapping.append(mujoco_dof_idx)
        except ValueError:
            print(f"Warning: Joint {mujoco_name} not found in MuJoCo model")
            print(f"Available joints: {mujoco_joint_names}")
            raise

    return mapping


def main():
    parser = argparse.ArgumentParser(description="Sim2Sim: MuJoCoで二脚ロボットポリシーを評価")
    parser.add_argument(
        "--checkpoint",
        type=str,
        default="logs/biped-walking/model_100.pt",
        help="学習済みチェックポイントのパス",
    )
    parser.add_argument(
        "--config",
        type=str,
        default="logs/biped-walking/cfgs.pkl",
        help="設定ファイルのパス",
    )
    parser.add_argument(
        "--model",
        type=str,
        default=DEFAULT_MODEL_PATH,
        help="MuJoCoモデル（MJCF）のパス",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="シミュレーション時間 [秒]",
    )
    parser.add_argument(
        "--cmd_vel_x",
        type=float,
        default=0.3,
        help="前進速度コマンド [m/s]",
    )
    parser.add_argument(
        "--cmd_vel_y",
        type=float,
        default=0.0,
        help="横移動速度コマンド [m/s]",
    )
    parser.add_argument(
        "--cmd_ang_vel",
        type=float,
        default=0.0,
        help="回転速度コマンド [rad/s]",
    )
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="ビューアなしで実行",
    )
    args = parser.parse_args()

    # パスの解決
    script_dir = Path(__file__).parent.parent
    checkpoint_path = script_dir / args.checkpoint
    config_path = script_dir / args.config
    model_path = script_dir / args.model

    print(f"=== Sim2Sim: Genesis -> MuJoCo (Biped) ===")
    print(f"Checkpoint: {checkpoint_path}")
    print(f"Config: {config_path}")
    print(f"MuJoCo Model: {model_path}")

    # モデルファイルの存在確認
    if not model_path.exists():
        print(f"\nERROR: Model file not found: {model_path}")
        if args.model == DEFAULT_MODEL_PATH:
            print("\nURDFから変換が必要です。以下を実行してください:")
            print("  cd rl_ws && uv run python scripts/convert_biped_urdf_to_mjcf.py")
        return

    # 設定を読み込み
    print("\n[1/4] Loading configuration...")
    if not config_path.exists():
        print(f"\nERROR: Config file not found: {config_path}")
        print("訓練を先に実行してください:")
        print("  cd rl_ws && uv run python genesis_official/examples/locomotion/biped_train.py")
        return

    with open(config_path, "rb") as f:
        cfgs = pickle.load(f)
    env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = cfgs

    num_actions = env_cfg["num_actions"]
    joint_names = env_cfg["joint_names"]
    kp = env_cfg["kp"]
    kd = env_cfg["kd"]
    default_joint_angles = env_cfg["default_joint_angles"]
    obs_scales = obs_cfg["obs_scales"]
    hidden_dims = train_cfg["policy"]["actor_hidden_dims"]
    activation = train_cfg["policy"]["activation"]

    print(f"  - Num actions: {num_actions}")
    print(f"  - Joint names: {joint_names}")
    print(f"  - PD gains: kp={kp}, kd={kd}")
    print(f"  - Hidden dims: {hidden_dims}")
    print(f"  - Activation: {activation}")

    # ポリシーを構築・読み込み
    print("\n[2/4] Loading policy...")
    if not checkpoint_path.exists():
        print(f"\nERROR: Checkpoint not found: {checkpoint_path}")
        print("訓練を先に実行してください")
        return

    num_obs = obs_cfg["num_obs"]
    policy = ActorMLP(num_obs, num_actions, hidden_dims, activation)

    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)

    state_dict = {}
    for key, value in checkpoint["model_state_dict"].items():
        if key.startswith("actor."):
            new_key = key.replace("actor.", "mlp.")
            state_dict[new_key] = value

    policy.load_state_dict(state_dict)
    policy.eval()
    print(f"  - Loaded {len(state_dict)} parameters")

    # MuJoCoモデルを読み込み
    print("\n[3/4] Loading MuJoCo model...")
    print(f"  - Loading MJCF: {model_path}")
    mj_model = mujoco.MjModel.from_xml_path(str(model_path))
    mj_data = mujoco.MjData(mj_model)

    print(f"  - Num bodies: {mj_model.nbody}")
    print(f"  - Num joints: {mj_model.njnt}")
    print(f"  - Num actuators: {mj_model.nu}")
    print(f"  - Timestep: {mj_model.opt.timestep} s")

    # 関節マッピングを取得
    joint_mapping = get_joint_mapping(joint_names, mj_model)
    print(f"  - Joint mapping (Genesis -> MuJoCo): {joint_mapping}")

    # デフォルト関節角度
    default_dof_pos = np.array([default_joint_angles[name] for name in joint_names])
    print(f"  - Default joint angles: {default_dof_pos}")

    # シミュレーション設定
    print("\n[4/4] Starting simulation...")
    dt_policy = 0.02  # ポリシー周期 (50Hz)
    dt_sim = mj_model.opt.timestep
    decimation = int(dt_policy / dt_sim)

    commands = np.array([args.cmd_vel_x, args.cmd_vel_y, args.cmd_ang_vel])
    print(f"  - Policy dt: {dt_policy} s ({1/dt_policy} Hz)")
    print(f"  - Sim dt: {dt_sim} s")
    print(f"  - Decimation: {decimation}")
    print(f"  - Commands: vel_x={args.cmd_vel_x}, vel_y={args.cmd_vel_y}, ang_vel={args.cmd_ang_vel}")

    # 初期状態をリセット
    mujoco.mj_resetData(mj_model, mj_data)

    # homeキーフレームがあれば適用
    if mj_model.nkey > 0:
        mujoco.mj_resetDataKeyframe(mj_model, mj_data, 0)
        print("  - Applied 'home' keyframe")
    else:
        print("  - Setting initial pose manually")
        mj_data.qpos[2] = 0.45
        for genesis_idx, mujoco_dof_idx in enumerate(joint_mapping):
            mj_data.qpos[7 + mujoco_dof_idx] = default_dof_pos[genesis_idx]
        mujoco.mj_forward(mj_model, mj_data)

    # 初期化
    last_actions = np.zeros(num_actions)
    step_count = 0
    total_steps = int(args.duration / dt_policy)

    def simulation_step():
        nonlocal last_actions, step_count

        obs = compute_observations(
            mj_data, mj_model, commands, last_actions,
            default_dof_pos, obs_scales, joint_mapping
        )

        with torch.no_grad():
            obs_tensor = torch.from_numpy(obs).unsqueeze(0)
            actions = policy(obs_tensor).squeeze(0).numpy()

        action_scale = env_cfg.get("action_scale", 0.25)
        target_pos = actions * action_scale + default_dof_pos

        current_pos = mj_data.qpos[7:][joint_mapping]
        current_vel = mj_data.qvel[6:][joint_mapping]

        torques = kp * (target_pos - current_pos) - kd * current_vel

        for genesis_idx, mujoco_idx in enumerate(joint_mapping):
            mj_data.ctrl[mujoco_idx] = torques[genesis_idx]

        for _ in range(decimation):
            mujoco.mj_step(mj_model, mj_data)

        last_actions = actions.copy()
        step_count += 1

        if step_count % 50 == 0:
            base_pos = mj_data.qpos[:3]
            base_vel = mj_data.qvel[:3]
            print(f"  Step {step_count}/{total_steps}: "
                  f"pos=({base_pos[0]:.2f}, {base_pos[1]:.2f}, {base_pos[2]:.2f}), "
                  f"vel=({base_vel[0]:.2f}, {base_vel[1]:.2f}, {base_vel[2]:.2f})")

        return step_count < total_steps

    # シミュレーション実行
    if args.no_viewer:
        print("\n  Running simulation...")
        start_time = time.time()

        while simulation_step():
            pass

        total_time = time.time() - start_time
        print(f"\n=== Simulation Complete ===")
        print(f"  - Total time: {total_time:.2f} s")
        print(f"  - Total steps: {step_count}")
        print(f"  - Final position: {mj_data.qpos[:3]}")
    else:
        use_opencv_fallback = False

        try:
            print("\n  Launching MuJoCo viewer (close window to exit)...")
            with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
                start_time = time.time()
                while viewer.is_running() and step_count < total_steps:
                    step_start = time.time()

                    simulation_step()
                    viewer.sync()

                    elapsed = time.time() - step_start
                    sleep_time = dt_policy - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)

                total_time = time.time() - start_time
                print(f"\n=== Simulation Complete ===")
                print(f"  - Total time: {total_time:.2f} s")
                print(f"  - Total steps: {step_count}")
                print(f"  - Final position: {mj_data.qpos[:3]}")
        except RuntimeError as e:
            if "mjpython" in str(e).lower():
                use_opencv_fallback = True
            else:
                raise

        if use_opencv_fallback:
            print("\n  Note: MuJoCo viewer requires 'mjpython' on macOS.")
            print("  Falling back to OpenCV viewer...")
            print("  Press 'q' or ESC to quit")

            try:
                import cv2
            except ImportError:
                print("  ERROR: OpenCV not installed. Use --no-viewer instead.")
                return

            renderer = mujoco.Renderer(mj_model, height=480, width=640)
            start_time = time.time()

            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            try:
                cam.trackbodyid = mj_model.body('base').id
            except KeyError:
                cam.trackbodyid = 1
            cam.distance = 2.0
            cam.azimuth = 135
            cam.elevation = -20

            while step_count < total_steps:
                step_start = time.time()

                simulation_step()

                renderer.update_scene(mj_data, camera=cam)
                frame = renderer.render()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imshow("MuJoCo Sim2Sim (Biped)", frame_bgr)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    break

                elapsed = time.time() - step_start
                sleep_time = dt_policy - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

            cv2.destroyAllWindows()
            total_time = time.time() - start_time
            print(f"\n=== Simulation Complete ===")
            print(f"  - Total time: {total_time:.2f} s")
            print(f"  - Total steps: {step_count}")
            print(f"  - Final position: {mj_data.qpos[:3]}")


if __name__ == "__main__":
    main()
