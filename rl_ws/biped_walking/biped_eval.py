"""
BSL-Droid二脚ロボット Genesis評価スクリプト（統一版）

全バージョン（biped-walking-*, droid-walking-*）のモデルを評価できる汎用スクリプト。
-e オプションで実験名を指定して切り替える。
ゲームパッド（Logicool F710等）で速度指令値をリアルタイムに操縦可能。

Usage:
    cd rl_ws

    # 通常の評価
    uv run python biped_walking/biped_eval.py -e droid-walking-narrow-v25

    # ゲームパッド操縦モード
    uv run python biped_walking/biped_eval.py -e droid-walking-narrow-v25 --gamepad

    # ゲームパッド＋横速度・yaw速度の操縦を有効化
    uv run python biped_walking/biped_eval.py -e droid-walking-narrow-v25 --gamepad --gamepad-vel-y 0.2 --gamepad-vel-yaw 1.0

    # コマンド固定評価
    uv run python biped_walking/biped_eval.py -e droid-walking-v1 --command 0.3 0.0 0.0

    # ヘッドレス＋CSV出力
    uv run python biped_walking/biped_eval.py -e droid-walking-v1 --no-viewer --duration 10 --csv

ゲームパッド軸マッピング（F710 DirectInputモード）:
    左スティック上下: 前後速度 (lin_vel_x)
    左スティック左右: 横速度 (lin_vel_y)
    右スティック左右: yaw角速度 (ang_vel_yaw)
"""

from __future__ import annotations

import argparse
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
            raise ImportError from None
except (metadata.PackageNotFoundError, ImportError) as e:
    raise ImportError("Please uninstall 'rsl_rl' and install 'rsl-rl-lib==2.2.4'.") from e

import genesis as gs
import numpy as np
import torch
from rsl_rl.modules import ActorCritic


# biped_walkingパッケージへのパスを追加
rl_ws_dir = Path(__file__).parent.parent
sys.path.insert(0, str(rl_ws_dir))

# macOS 用 SDL ヒント: pygame-ce (SDL 2.32+) の HIDAPI Xbox ドライバで F710 を検出
_MACOS_SDL_HINTS: dict[str, str] = {
    "SDL_JOYSTICK_MFI": "0",
    "SDL_JOYSTICK_HIDAPI": "1",
    "SDL_JOYSTICK_HIDAPI_XBOX": "1",
    "SDL_JOYSTICK_HIDAPI_XBOX_360": "1",
}


def get_env_class(exp_name: str) -> type:
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
    # droid-walking-omni-v{N}: exp009 omni-directional variant (統合環境を使用)
    if re.match(r"droid-walking-omni-v\d+", exp_name):
        from biped_walking.envs.droid_env_unitree import DroidEnvUnitree

        return DroidEnvUnitree

    # droid-walking-narrow-v{N}: exp008 narrow torso variant (統合環境を使用)
    if re.match(r"droid-walking-narrow-v\d+", exp_name):
        from biped_walking.envs.droid_env_unitree import DroidEnvUnitree

        return DroidEnvUnitree

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
    if exp_name.startswith("droid-walking-narrow"):
        return rl_ws_dir / "assets" / "bsl_droid_simplified_v2.urdf"
    if exp_name.startswith("droid-walking"):
        return rl_ws_dir / "assets" / "bsl_droid_simplified.urdf"
    else:
        return rl_ws_dir / "assets" / "biped_digitigrade.urdf"


def _init_gamepad(device_index: int = 0) -> "pygame.joystick.JoystickType":
    """pygameジョイスティックを初期化して返す。

    Args:
        device_index: pygameジョイスティックインデックス

    Returns:
        初期化済みのJoystickオブジェクト

    Raises:
        SystemExit: ジョイスティックが検出されない場合
    """
    import pygame

    # macOS: SDL ヒントを pygame.init() 前に設定
    if sys.platform == "darwin":
        for key, value in _MACOS_SDL_HINTS.items():
            os.environ.setdefault(key, value)

    pygame.init()
    pygame.joystick.init()

    count = pygame.joystick.get_count()
    if count == 0:
        print("ERROR: No gamepad detected. Connect a gamepad and retry.")
        print("  macOS: Set F710 switch to D, hold Logitech button while plugging in receiver.")
        sys.exit(1)

    if device_index >= count:
        print(f"ERROR: Device index {device_index} out of range (detected: {count})")
        for i in range(count):
            joy = pygame.joystick.Joystick(i)
            joy.init()
            print(f"  [{i}] {joy.get_name()} (axes={joy.get_numaxes()}, buttons={joy.get_numbuttons()})")
        sys.exit(1)

    joy = pygame.joystick.Joystick(device_index)
    joy.init()
    print(f"Gamepad: {joy.get_name()} (axes={joy.get_numaxes()}, buttons={joy.get_numbuttons()})")
    return joy


def _read_gamepad_commands(
    joy: "pygame.joystick.JoystickType",
    max_vel_x: float,
    max_vel_y: float,
    max_vel_yaw: float,
    deadzone: float,
) -> tuple[float, float, float]:
    """ゲームパッドのスティック値を速度コマンドに変換する。

    軸マッピング（F710 DirectInputモード）:
        a0: 左スティックX (left=-1, right=+1) → lin_vel_y
        a1: 左スティックY (up=-1, down=+1)   → lin_vel_x
        a2: 右スティックX (left=-1, right=+1) → ang_vel_yaw

    Returns:
        (vel_x, vel_y, vel_yaw) の速度コマンドタプル
    """
    import pygame

    pygame.event.pump()

    def apply_deadzone(value: float) -> float:
        return 0.0 if abs(value) < deadzone else value

    raw_x = apply_deadzone(joy.get_axis(1))  # 左スティック上下
    raw_y = apply_deadzone(joy.get_axis(0))  # 左スティック左右
    raw_yaw = apply_deadzone(joy.get_axis(2)) if joy.get_numaxes() > 2 else 0.0  # 右スティック左右

    # 符号反転: pygame は上=-1, 左=-1 だが、ロボットでは前進=+X, 左移動=+Y, 左旋回=+yaw
    vel_x = -raw_x * max_vel_x
    vel_y = -raw_y * max_vel_y
    vel_yaw = -raw_yaw * max_vel_yaw

    return vel_x, vel_y, vel_yaw


def main() -> None:
    parser = argparse.ArgumentParser(description="BSL-Droid Biped Walking Evaluation")
    parser.add_argument(
        "-e",
        "--exp_name",
        type=str,
        default="biped-walking-v4",
        help="Experiment name (e.g., biped-walking, biped-walking-v2, biped-walking-v3, biped-walking-v4)",
    )
    parser.add_argument("--ckpt", type=int, default=None, help="Checkpoint iteration (default: latest)")
    parser.add_argument("--no-viewer", action="store_true", help="Run without GUI viewer (headless mode)")
    parser.add_argument("--duration", type=float, default=10.0, help="Evaluation duration in seconds (default: 10.0)")
    parser.add_argument(
        "--csv",
        type=str,
        nargs="?",
        const="auto",
        default=None,
        help="Export time-series data to CSV. "
        "Without path: saves to logs/{exp}/eval_{ckpt}.csv. "
        "With path: saves to the specified file.",
    )
    parser.add_argument(
        "--command",
        type=float,
        nargs=3,
        metavar=("VX", "VY", "VYAW"),
        default=None,
        help="Fix velocity command [vx, vy, vyaw]. Overrides command_cfg ranges.",
    )
    # ゲームパッド関連引数
    parser.add_argument("--gamepad", action="store_true", help="Enable gamepad control for velocity commands")
    parser.add_argument("--gamepad-device", type=int, default=0, help="Pygame joystick device index (default: 0)")
    parser.add_argument("--gamepad-deadzone", type=float, default=0.08, help="Stick deadzone threshold (default: 0.08)")
    parser.add_argument(
        "--gamepad-vel-y", type=float, default=0.0, help="Max lateral velocity [m/s] (default: 0.0 = disabled)"
    )
    parser.add_argument(
        "--gamepad-vel-yaw",
        type=float,
        default=0.0,
        help="Max yaw angular velocity [rad/s] (default: 0.0 = disabled)",
    )
    parser.add_argument("--seed", type=int, default=1, help="Genesis random seed (default: 1)")
    args = parser.parse_args()

    if args.gamepad and args.command is not None:
        parser.error("--gamepad and --command cannot be used together")

    gs.init(backend=gs.gpu, precision="32", logging_level="warning", seed=args.seed)

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

    # command_cfgのキー名の互換性対応（一箇所で定義）
    lin_vel_x_key = "lin_vel_x_range" if "lin_vel_x_range" in command_cfg else "lin_vel_x"

    # --commandフラグによるコマンド固定（方向別評価用）
    if args.command is not None:
        vx, vy, vyaw = args.command
        command_cfg["lin_vel_x_range"] = [vx, vx]
        command_cfg["lin_vel_y_range"] = [vy, vy]
        command_cfg["ang_vel_range"] = [vyaw, vyaw]

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
    if args.command is not None:
        print(f"Fixed command: vx={args.command[0]:.3f}, vy={args.command[1]:.3f}, vyaw={args.command[2]:.3f}")
    else:
        print(f"Target velocity: {command_cfg[lin_vel_x_key][0]} m/s")
    print(f"Action scale: {env_cfg['action_scale']}")
    print(f"Seed: {args.seed}")
    print()

    # ゲームパッド初期化
    gamepad_joy = None
    max_vel_x = 0.0
    max_vel_y = 0.0
    max_vel_yaw = 0.0
    if args.gamepad:
        gamepad_joy = _init_gamepad(args.gamepad_device)
        max_vel_x = max(abs(v) for v in command_cfg[lin_vel_x_key])
        max_vel_y = (
            abs(args.gamepad_vel_y)
            if args.gamepad_vel_y != 0.0
            else max(abs(v) for v in command_cfg.get("lin_vel_y_range", [0, 0]))
        )
        max_vel_yaw = (
            abs(args.gamepad_vel_yaw)
            if args.gamepad_vel_yaw != 0.0
            else max(abs(v) for v in command_cfg.get("ang_vel_range", [0, 0]))
        )
        print(f"Gamepad velocity limits: X=\u00b1{max_vel_x:.3f} m/s, Y=\u00b1{max_vel_y:.3f} m/s, Yaw=\u00b1{max_vel_yaw:.3f} rad/s")
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

    if args.gamepad:
        print("Gamepad control active. Use left stick for movement, right stick for rotation.")
    if args.no_viewer:
        print(f"Running headless evaluation for {args.duration} seconds...")
    else:
        print("Running evaluation... Press Ctrl+C to stop.")
    print()

    obs, _ = env.reset()
    step = 0
    max_steps = int(args.duration / env.dt) if args.no_viewer else float("inf")

    # ゲームパッドモード: 初回のコマンドをオーバーライド
    if gamepad_joy is not None:
        vel_x, vel_y, vel_yaw = _read_gamepad_commands(
            gamepad_joy, max_vel_x, max_vel_y, max_vel_yaw, args.gamepad_deadzone
        )
        env.commands[0, 0] = vel_x
        env.commands[0, 1] = vel_y
        env.commands[0, 2] = vel_yaw
        env._update_observation()
        obs = env.obs_buf

    # 統計収集用
    positions = []
    velocities = []
    orientations = []
    euler_angles = []  # RPY角度
    dof_vels = []
    actions_list = []
    dof_pos_list = []
    contact_states = []  # 接地状態 [left, right]
    command_list = []  # コマンド速度 [vx_cmd, vy_cmd, vyaw_cmd]

    with torch.no_grad():
        while step < max_steps:
            actions = policy.act_inference(obs)
            obs, _rew, _done, _info = env.step(actions)

            # ゲームパッドコマンド注入
            if gamepad_joy is not None:
                vel_x, vel_y, vel_yaw = _read_gamepad_commands(
                    gamepad_joy, max_vel_x, max_vel_y, max_vel_yaw, args.gamepad_deadzone
                )
                env.commands[0, 0] = vel_x
                env.commands[0, 1] = vel_y
                env.commands[0, 2] = vel_yaw
                env._update_observation()
                obs = env.obs_buf

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
            if hasattr(env, "_get_foot_contacts"):
                contacts = env._get_foot_contacts()
                contact_state = contacts[0].cpu().numpy() if contacts is not None else np.array([True, True])
            else:
                contact_state = np.array([True, True])

            # コマンド速度を取得
            if hasattr(env, "commands"):
                cmd = env.commands[0].cpu().numpy()
            else:
                cmd = np.zeros(3)

            positions.append(pos.copy())
            velocities.append(vel.copy())
            orientations.append(quat.copy())
            euler_angles.append(euler_deg.copy())
            dof_vels.append(dof_vel.copy())
            actions_list.append(act.copy())
            dof_pos_list.append(dof_pos.copy())
            contact_states.append(contact_state.copy())
            command_list.append(cmd.copy())

            # 時間ステップごとの出力（10ステップごと = 0.2秒ごと）
            if step % 10 == 0:
                dof_vel_rms = torch.sqrt(torch.mean(torch.square(env.dof_vel[0]))).item()

                print(
                    f"t={step * env.dt:5.2f}s | "
                    f"cmd=({cmd[0]:+5.2f}, {cmd[1]:+5.2f}, {cmd[2]:+5.2f}) | "
                    f"pos=({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:5.3f}) | "
                    f"vel=({vel[0]:5.2f}, {vel[1]:5.2f}) | "
                    f"rpy=({euler_deg[0]:5.1f}, {euler_deg[1]:5.1f}, {euler_deg[2]:5.1f})\u00b0 | "
                    f"dof_vel_rms={dof_vel_rms:5.2f}"
                )

            step += 1

    # 統計出力: リストをndarrayに変換
    pos_arr = np.array(positions)
    vel_arr = np.array(velocities)
    _orient_arr = np.array(orientations)
    euler_arr = np.array(euler_angles)
    dof_vel_arr = np.array(dof_vels)
    act_arr = np.array(actions_list)
    dof_pos_arr = np.array(dof_pos_list)
    contact_arr = np.array(contact_states)

    print("\n=== Evaluation Statistics ===")
    print(f"Duration: {args.duration}s ({step} steps)")
    print("\nTravel distance:")
    print(f"  X: {pos_arr[-1, 0] - pos_arr[0, 0]:.3f} m")
    print(f"  Y: {pos_arr[-1, 1] - pos_arr[0, 1]:.3f} m")
    print(f"  Total: {np.linalg.norm(pos_arr[-1, :2] - pos_arr[0, :2]):.3f} m")
    print("\nAverage velocity:")
    print(f"  X: {np.mean(vel_arr[:, 0]):.3f} m/s (target: {command_cfg[lin_vel_x_key][0]:.3f})")
    print(f"  Y: {np.mean(vel_arr[:, 1]):.3f} m/s")
    print("\nBase height:")
    print(f"  Mean: {np.mean(pos_arr[:, 2]):.3f} m")
    print(f"  Std: {np.std(pos_arr[:, 2]):.4f} m")

    # 姿勢統計（追加）
    print("\nOrientation (deg):")
    print(f"  Roll:  mean={np.mean(euler_arr[:, 0]):6.2f}, std={np.std(euler_arr[:, 0]):5.2f}")
    print(f"  Pitch: mean={np.mean(euler_arr[:, 1]):6.2f}, std={np.std(euler_arr[:, 1]):5.2f}")
    print(
        f"  Yaw:   start={euler_arr[0, 2]:6.2f}, end={euler_arr[-1, 2]:6.2f}, drift={euler_arr[-1, 2] - euler_arr[0, 2]:+6.2f}"
    )

    print(f"\nDOF velocity RMS: {np.sqrt(np.mean(dof_vel_arr**2)):.3f} rad/s")
    print(f"Action RMS: {np.sqrt(np.mean(act_arr**2)):.3f}")

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
        left_min = np.min(dof_pos_arr[:, left_leg_indices[i]])
        left_max = np.max(dof_pos_arr[:, left_leg_indices[i]])
        right_min = np.min(dof_pos_arr[:, right_leg_indices[i]])
        right_max = np.max(dof_pos_arr[:, right_leg_indices[i]])
        print(
            f"  {name:12s}: L=[{left_min:6.3f}, {left_max:6.3f}] ({left_max - left_min:.3f})  "
            f"R=[{right_min:6.3f}, {right_max:6.3f}] ({right_max - right_min:.3f})"
        )

    print("\nDOF velocity std (rad/s):")
    left_vel_std = np.std(dof_vel_arr[:, left_leg_indices], axis=0)
    right_vel_std = np.std(dof_vel_arr[:, right_leg_indices], axis=0)
    for i, name in enumerate(joint_names):
        print(f"  {name:12s}: L={left_vel_std[i]:.3f}  R={right_vel_std[i]:.3f}")

    # 左右の位相差分析（hip_pitchの相関）
    # hip_pitch: index 2 (left), index 7 (right)
    left_hip_pitch = dof_pos_arr[:, 2]
    right_hip_pitch = dof_pos_arr[:, 7]
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

    left_knee = dof_pos_arr[:, left_knee_idx]
    right_knee = dof_pos_arr[:, right_knee_idx]

    print("\n=== Knee Angle Analysis ===")
    print(f"Left knee:  min={np.min(left_knee):6.3f}, max={np.max(left_knee):6.3f}, mean={np.mean(left_knee):6.3f} rad")
    print(
        f"Right knee: min={np.min(right_knee):6.3f}, max={np.max(right_knee):6.3f}, mean={np.mean(right_knee):6.3f} rad"
    )

    # 膝が負角度に入った割合（droid系のみ意味がある）
    if args.exp_name.startswith("droid-walking"):
        left_negative_ratio = np.mean(left_knee < 0) * 100
        right_negative_ratio = np.mean(right_knee < 0) * 100
        print("\nNegative angle ratio (should be 0% for proper digitigrade):")
        print(f"  Left knee:  {left_negative_ratio:5.1f}%")
        print(f"  Right knee: {right_negative_ratio:5.1f}%")

    # 接地パターン分析
    print("\n=== Contact Pattern Analysis ===")
    both_contact = np.all(contact_arr, axis=1)  # 両足接地
    no_contact = np.all(~contact_arr, axis=1)  # 両足離地
    single_contact = ~both_contact & ~no_contact  # 片足接地

    print(f"Both feet grounded:   {np.sum(both_contact):5d} steps ({np.mean(both_contact) * 100:5.1f}%)")
    print(f"Single foot grounded: {np.sum(single_contact):5d} steps ({np.mean(single_contact) * 100:5.1f}%)")
    print(f"Both feet airborne:   {np.sum(no_contact):5d} steps ({np.mean(no_contact) * 100:5.1f}%)")

    # 歩行品質スコア
    total_dof_range = np.sum([np.max(dof_pos_arr[:, i]) - np.min(dof_pos_arr[:, i]) for i in range(10)])
    print("\nGait quality indicators:")
    print(f"  Total DOF range sum: {total_dof_range:.3f} rad")
    print("  (Higher = more movement, typically >2.0 for good walking)")
    print()

    # CSV出力
    if args.csv is not None:
        if args.csv == "auto":
            if args.command is not None:
                vx, vy, vyaw = args.command
                dir_tag = f"_cmd_{vx:.2f}_{vy:.2f}_{vyaw:.2f}_s{args.seed}"
                csv_path = f"{log_dir}/eval_{ckpt_num}{dir_tag}.csv"
            else:
                csv_path = f"{log_dir}/eval_{ckpt_num}.csv"
        else:
            csv_path = args.csv

        joint_names_lr = [
            "L_hip_yaw",
            "L_hip_roll",
            "L_hip_pitch",
            "L_knee_pitch",
            "L_ankle_pitch",
            "R_hip_yaw",
            "R_hip_roll",
            "R_hip_pitch",
            "R_knee_pitch",
            "R_ankle_pitch",
        ]

        # タイムスタンプ列
        timestamps = np.arange(len(pos_arr)) * env.dt

        cmd_arr = np.array(command_list)

        # 全データを結合
        data = np.column_stack(
            [
                timestamps,
                pos_arr,  # x, y, z
                vel_arr,  # vx, vy, vz
                euler_arr,  # roll, pitch, yaw (deg)
                dof_pos_arr,  # 10 joints
                dof_vel_arr,  # 10 joints
                act_arr,  # 10 actions
                contact_arr.astype(float),  # left, right
                cmd_arr,  # vx_cmd, vy_cmd, vyaw_cmd
            ]
        )

        # ヘッダ
        header_cols = (
            ["timestamp"]
            + ["base_pos_x", "base_pos_y", "base_pos_z"]
            + ["base_vel_x", "base_vel_y", "base_vel_z"]
            + ["roll_deg", "pitch_deg", "yaw_deg"]
            + [f"dof_pos_{name}" for name in joint_names_lr]
            + [f"dof_vel_{name}" for name in joint_names_lr]
            + [f"action_{i}" for i in range(10)]
            + ["contact_left", "contact_right"]
            + ["cmd_vel_x", "cmd_vel_y", "cmd_vel_yaw"]
        )

        os.makedirs(os.path.dirname(os.path.abspath(csv_path)), exist_ok=True)
        np.savetxt(csv_path, data, delimiter=",", header=",".join(header_cols), comments="", fmt="%.6f")
        print(f"CSV saved to: {csv_path} ({len(data)} rows, {len(header_cols)} columns)")


if __name__ == "__main__":
    main()
