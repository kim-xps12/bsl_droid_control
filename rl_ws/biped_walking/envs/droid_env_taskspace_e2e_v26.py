"""BSL-Droid Simplified 二脚ロボット Genesis環境（V26: 足先空間改善版）

============================================================
V26: 足先空間を維持した改善版
============================================================

V25の失敗原因:
1. hip_roll異常: IK=0だがPDで追従失敗 → 脚が八の字に開く
2. 震え: foot_height_diff等の報酬を高周波振動でexploit
3. Yaw drift: 左右非対称な関節動作

V26の改善点:
1. gait_phase復活 → 周期的歩容報酬で高周波振動を抑制
2. action_rate/dof_acc ペナルティ大幅強化 → 震え対策
3. 周期的歩容報酬 _reward_gait_cycle 追加
4. hip_roll明示的制御を追加（行動空間を6次元に拡張）

観測空間 (39次元): V22/V23と同じ（gait_phase復活）
行動空間 (6次元): 左右足先XZ + 左右hip_roll
"""

from __future__ import annotations

import math
from typing import Any

import genesis as gs
import torch
from genesis.utils.geom import inv_quat, quat_to_xyz, transform_by_quat, transform_quat_by_quat

from biped_walking.envs.droid_kinematics import DroidKinematics


def gs_rand(lower: torch.Tensor, upper: torch.Tensor, batch_shape: tuple[int, ...]) -> torch.Tensor:
    """一様乱数を生成"""
    assert lower.shape == upper.shape
    return (upper - lower) * torch.rand(size=(*batch_shape, *lower.shape), dtype=gs.tc_float, device=gs.device) + lower


class DroidEnvTaskSpaceE2EV26:
    """BSL-Droid Simplified二脚ロボットのGenesis強化学習環境（V26: 足先空間改善版）"""

    def __init__(
        self,
        num_envs: int,
        env_cfg: dict[str, Any],
        obs_cfg: dict[str, Any],
        reward_cfg: dict[str, Any],
        command_cfg: dict[str, Any],
        show_viewer: bool = False,
        recording_camera: bool = False,
    ) -> None:
        self.num_envs: int = num_envs
        self.num_obs: int = obs_cfg["num_obs"]  # 39次元（gait_phase復活）
        self.num_privileged_obs: int | None = None
        self.num_actions: int = env_cfg["num_actions"]  # 6次元（足先4 + hip_roll 2）
        self.num_commands: int = command_cfg["num_commands"]
        self.device = gs.device

        self.simulate_action_latency = True
        self.dt = 0.02  # 50Hz
        self.max_episode_length = math.ceil(env_cfg["episode_length_s"] / self.dt)

        self.env_cfg = env_cfg
        self.obs_cfg = obs_cfg
        self.reward_cfg = reward_cfg
        self.command_cfg = command_cfg

        self.obs_scales = obs_cfg["obs_scales"]
        self.reward_scales = reward_cfg["reward_scales"]

        # 運動学計算器を初期化
        self.kinematics = DroidKinematics(device=gs.device)

        # V26: 歩容パラメータ復活（周期的報酬用）
        self.gait_frequency = env_cfg.get("gait_frequency", 1.5)  # Hz（V25より低め）
        self.swing_height = env_cfg.get("swing_height", 0.03)  # m

        # デフォルト足先位置
        default_pos = self.kinematics.get_default_foot_position(is_left=True)
        self.default_foot_x = default_pos[0].item()
        self.default_foot_y = default_pos[1].item()
        self.default_foot_z = default_pos[2].item()

        # シーン作成
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(
                dt=self.dt,
                substeps=2,
            ),
            rigid_options=gs.options.RigidOptions(
                enable_self_collision=False,
                tolerance=1e-5,
                max_collision_pairs=20,
            ),
            viewer_options=gs.options.ViewerOptions(
                camera_pos=(2.0, 0.0, 1.0),
                camera_lookat=(0.0, 0.0, 0.3),
                camera_fov=40,
                max_FPS=int(1.0 / self.dt),
            ),
            vis_options=gs.options.VisOptions(rendered_envs_idx=[0]),
            show_viewer=show_viewer,
        )

        # 地面を追加
        self.scene.add_entity(
            gs.morphs.URDF(
                file="urdf/plane/plane.urdf",
                fixed=True,
            )
        )

        # ロボットを追加
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file=self.env_cfg["urdf_path"],
                pos=self.env_cfg["base_init_pos"],
                quat=self.env_cfg["base_init_quat"],
            ),
        )

        # 録画用カメラ
        self.recording_cam = None
        if recording_camera:
            self.recording_cam = self.scene.add_camera(
                res=(1280, 720),
                pos=(2.0, 0.0, 1.0),
                lookat=(0.0, 0.0, 0.3),
                fov=40,
                GUI=False,
                spp=256,
            )

        # シーンのビルドとバッチ化
        self.scene.build(n_envs=num_envs)

        # DOF設定
        dofs_idx = self._setup_motor_dofs()
        self.motors_dof_idx = dofs_idx["motor"]
        self.left_leg_dof_idx = dofs_idx["left_leg"]
        self.right_leg_dof_idx = dofs_idx["right_leg"]

        # アクション用インデックス（全10関節）
        self.actions_dof_idx = list(range(10))
        self.num_dofs = 10

        # 足先リンクのインデックス
        self._setup_foot_links()

        # hip_pitch, knee_pitchのインデックス（motor順）
        self.left_hip_pitch_idx = 1
        self.right_hip_pitch_idx = 6
        self.left_knee_pitch_idx = 3
        self.right_knee_pitch_idx = 8
        # hip_rollのインデックス
        self.left_hip_roll_idx = 0
        self.right_hip_roll_idx = 5

        # グローバル重力
        self.global_gravity = torch.tensor([0.0, 0.0, -1.0], dtype=gs.tc_float, device=gs.device)

        # バッファ初期化
        self._init_buffers()

        # 報酬関数の設定
        self._setup_reward_functions()

    def _setup_motor_dofs(self) -> dict[str, list[int]]:
        """モーターDOFの設定"""
        motor_joints = [
            "left_hip_roll_joint",
            "left_hip_pitch_joint",
            "left_hip_yaw_joint",
            "left_knee_pitch_joint",
            "left_ankle_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_yaw_joint",
            "right_knee_pitch_joint",
            "right_ankle_pitch_joint",
        ]

        motor_dof_idx = []
        for joint_name in motor_joints:
            idx = self.robot.get_joint(joint_name).dofs_idx_local[0]
            motor_dof_idx.append(idx)

        # PD制御の設定
        self.robot.set_dofs_kp(
            [self.env_cfg["kp"]] * len(motor_joints),
            motor_dof_idx,
        )
        self.robot.set_dofs_kv(
            [self.env_cfg["kd"]] * len(motor_joints),
            motor_dof_idx,
        )
        self.robot.set_dofs_force_range(
            [-self.env_cfg["torque_limit"]] * len(motor_joints),
            [self.env_cfg["torque_limit"]] * len(motor_joints),
            motor_dof_idx,
        )

        return {
            "motor": motor_dof_idx,
            "left_leg": motor_dof_idx[:5],
            "right_leg": motor_dof_idx[5:],
        }

    def _setup_foot_links(self) -> None:
        """足先リンクの設定"""
        self.feet_indices: list[int] | None
        try:
            left_foot = self.robot.get_link("left_foot_link")
            right_foot = self.robot.get_link("right_foot_link")
            self.feet_indices = [left_foot.idx_local, right_foot.idx_local]
        except Exception:
            self.feet_indices = None

    def _init_buffers(self) -> None:
        """バッファの初期化"""
        # 基本バッファ
        self.base_pos = torch.zeros((self.num_envs, 3), dtype=gs.tc_float, device=gs.device)
        self.base_quat = torch.zeros((self.num_envs, 4), dtype=gs.tc_float, device=gs.device)
        self.base_euler = torch.zeros((self.num_envs, 3), dtype=gs.tc_float, device=gs.device)
        self.base_lin_vel = torch.zeros((self.num_envs, 3), dtype=gs.tc_float, device=gs.device)
        self.base_ang_vel = torch.zeros((self.num_envs, 3), dtype=gs.tc_float, device=gs.device)
        self.projected_gravity = torch.zeros((self.num_envs, 3), dtype=gs.tc_float, device=gs.device)

        self.dof_pos = torch.zeros((self.num_envs, self.num_dofs), dtype=gs.tc_float, device=gs.device)
        self.dof_vel = torch.zeros((self.num_envs, self.num_dofs), dtype=gs.tc_float, device=gs.device)

        # 行動バッファ（6次元: 足先XZ 4 + hip_roll 2）
        self.actions = torch.zeros((self.num_envs, self.num_actions), dtype=gs.tc_float, device=gs.device)
        self.last_actions = torch.zeros_like(self.actions)
        self.last_dof_vel = torch.zeros_like(self.dof_vel)

        # ジョイントコマンド（IKで計算された10次元）
        self.joint_commands = torch.zeros((self.num_envs, self.num_dofs), dtype=gs.tc_float, device=gs.device)

        # 現在の足先位置（FK計算結果）
        self.current_foot_pos = torch.zeros((self.num_envs, 4), dtype=gs.tc_float, device=gs.device)

        # コマンド
        self.commands = torch.zeros((self.num_envs, self.num_commands), dtype=gs.tc_float, device=gs.device)

        # エピソード管理
        self.episode_length_buf = torch.zeros((self.num_envs,), dtype=torch.long, device=gs.device)
        self.rew_buf = torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device)
        self.reset_buf = torch.zeros((self.num_envs,), dtype=torch.bool, device=gs.device)
        self.extras: dict[str, Any] = {
            "observations": {},
            "time_outs": torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device),
        }
        self.obs_buf = torch.zeros((self.num_envs, self.num_obs), dtype=gs.tc_float, device=gs.device)

        # V26: gait_phase復活
        self.gait_phase = torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device)

        # 前進距離追跡用
        self.last_base_pos_x = torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device)

        # 報酬の累積
        self.episode_sums = {
            name: torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device) for name in self.reward_scales
        }

        # 接地状態
        self.feet_air_time: torch.Tensor | None
        self.last_contacts: torch.Tensor | None
        if self.feet_indices is not None:
            self.feet_air_time = torch.zeros((self.num_envs, 2), dtype=gs.tc_float, device=gs.device)
            self.last_contacts = torch.zeros((self.num_envs, 2), dtype=torch.bool, device=gs.device)
        else:
            self.feet_air_time = None
            self.last_contacts = None

        # 初期状態を保存
        self.init_base_pos = torch.tensor(self.env_cfg["base_init_pos"], dtype=gs.tc_float, device=gs.device)
        self.init_base_quat = torch.tensor(self.env_cfg["base_init_quat"], dtype=gs.tc_float, device=gs.device)
        self.inv_base_init_quat = inv_quat(self.init_base_quat)
        self.init_dof_pos = torch.tensor(
            [self.env_cfg["default_joint_angles"][joint.name] for joint in self.robot.joints[1:]],
            dtype=gs.tc_float,
            device=gs.device,
        )
        self.init_qpos = torch.concatenate((self.init_base_pos, self.init_base_quat, self.init_dof_pos))
        self.init_projected_gravity = transform_by_quat(self.global_gravity, self.inv_base_init_quat)
        self.default_dof_pos = self.init_dof_pos.clone()

    def _setup_reward_functions(self) -> None:
        """報酬関数の設定"""
        self.reward_functions = {}
        for name in self.reward_scales:
            method_name = "_reward_" + name
            if hasattr(self, method_name):
                self.reward_functions[name] = getattr(self, method_name)
            else:
                print(f"Warning: Reward function '{method_name}' not found")

    def _compute_joint_commands(self, actions: torch.Tensor) -> None:
        """足先オフセット+hip_rollからジョイントコマンドを計算

        V26変更: hip_rollを行動空間に追加（6次元）

        Args:
            actions: [left_dx, left_dz, right_dx, right_dz, left_hip_roll, right_hip_roll]
        """
        action_scale = self.env_cfg.get("action_scale", 0.05)
        hip_roll_scale = self.env_cfg.get("hip_roll_scale", 0.1)  # 約5.7°

        # 足先位置
        left_x = self.default_foot_x + actions[:, 0] * action_scale
        left_z = self.default_foot_z + actions[:, 1] * action_scale
        right_x = self.default_foot_x + actions[:, 2] * action_scale
        right_z = self.default_foot_z + actions[:, 3] * action_scale

        # hip_roll（行動から直接）
        left_hip_roll = actions[:, 4] * hip_roll_scale
        right_hip_roll = actions[:, 5] * hip_roll_scale

        # 3D足先位置を構築
        batch_size = actions.shape[0]
        left_foot_pos = torch.zeros(batch_size, 3, device=self.device)
        left_foot_pos[:, 0] = left_x
        left_foot_pos[:, 1] = self.default_foot_y
        left_foot_pos[:, 2] = left_z

        right_foot_pos = torch.zeros(batch_size, 3, device=self.device)
        right_foot_pos[:, 0] = right_x
        right_foot_pos[:, 1] = -self.default_foot_y
        right_foot_pos[:, 2] = right_z

        # 逆運動学でジョイント角度を計算（hip_rollは0で計算）
        left_joints = self.kinematics.inverse_kinematics(left_foot_pos, is_left=True)
        right_joints = self.kinematics.inverse_kinematics(right_foot_pos, is_left=False)

        # ジョイントコマンドを構築（motor順序）
        # hip_roll, hip_pitch, hip_yaw, knee_pitch, ankle_pitch
        self.joint_commands[:, 0] = left_hip_roll  # left_hip_roll（行動から）
        self.joint_commands[:, 1] = left_joints[:, 2]  # left_hip_pitch
        self.joint_commands[:, 2] = left_joints[:, 0]  # left_hip_yaw
        self.joint_commands[:, 3] = left_joints[:, 3]  # left_knee_pitch
        self.joint_commands[:, 4] = left_joints[:, 4]  # left_ankle_pitch

        self.joint_commands[:, 5] = right_hip_roll  # right_hip_roll（行動から）
        self.joint_commands[:, 6] = right_joints[:, 2]  # right_hip_pitch
        self.joint_commands[:, 7] = right_joints[:, 0]  # right_hip_yaw
        self.joint_commands[:, 8] = right_joints[:, 3]  # right_knee_pitch
        self.joint_commands[:, 9] = right_joints[:, 4]  # right_ankle_pitch

    def _update_current_foot_pos(self) -> None:
        """現在の足先位置をFKで計算"""
        left_joints = torch.stack(
            [
                self.dof_pos[:, 2],  # left_hip_yaw
                self.dof_pos[:, 0],  # left_hip_roll
                self.dof_pos[:, 1],  # left_hip_pitch
                self.dof_pos[:, 3],  # left_knee_pitch
                self.dof_pos[:, 4],  # left_ankle_pitch
            ],
            dim=1,
        )

        right_joints = torch.stack(
            [
                self.dof_pos[:, 7],  # right_hip_yaw
                self.dof_pos[:, 5],  # right_hip_roll
                self.dof_pos[:, 6],  # right_hip_pitch
                self.dof_pos[:, 8],  # right_knee_pitch
                self.dof_pos[:, 9],  # right_ankle_pitch
            ],
            dim=1,
        )

        left_foot_pos = self.kinematics.forward_kinematics(left_joints, is_left=True)
        right_foot_pos = self.kinematics.forward_kinematics(right_joints, is_left=False)

        self.current_foot_pos[:, 0] = left_foot_pos[:, 0]
        self.current_foot_pos[:, 1] = left_foot_pos[:, 2]
        self.current_foot_pos[:, 2] = right_foot_pos[:, 0]
        self.current_foot_pos[:, 3] = right_foot_pos[:, 2]

    def _get_foot_contacts(self) -> torch.Tensor | None:
        """足の接地状態を取得"""
        if self.feet_indices is None:
            return None
        contact_threshold = self.reward_cfg.get("contact_threshold", 0.025)
        link_pos = self.robot.get_links_pos()
        feet_z = link_pos[:, self.feet_indices, 2]
        return feet_z < contact_threshold

    def reset(self) -> tuple[torch.Tensor, None]:
        """全環境をリセット"""
        self._reset_idx()
        self._update_observation()
        return self.obs_buf, None

    def step(self, actions: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, dict[str, Any]]:
        """環境を1ステップ進める"""
        self.actions = torch.clip(actions, -self.env_cfg["clip_actions"], self.env_cfg["clip_actions"])
        exec_actions = self.last_actions if self.simulate_action_latency else self.actions

        # 足先残差+hip_rollからジョイントコマンドを計算
        self._compute_joint_commands(exec_actions)

        # ジョイントコマンドを適用
        self.robot.control_dofs_position(self.joint_commands[:, self.actions_dof_idx], slice(6, 6 + self.num_dofs))
        self.scene.step()

        # バッファ更新
        self.episode_length_buf += 1
        self.base_pos = self.robot.get_pos()
        self.base_quat = self.robot.get_quat()
        self.base_euler = quat_to_xyz(
            transform_quat_by_quat(self.inv_base_init_quat, self.base_quat), rpy=True, degrees=True
        )
        inv_base_quat = inv_quat(self.base_quat)
        self.base_lin_vel = transform_by_quat(self.robot.get_vel(), inv_base_quat)
        self.base_ang_vel = transform_by_quat(self.robot.get_ang(), inv_base_quat)
        self.projected_gravity = transform_by_quat(self.global_gravity, inv_base_quat)

        self.dof_pos = self.robot.get_dofs_position(self.motors_dof_idx)
        self.dof_vel = self.robot.get_dofs_velocity(self.motors_dof_idx)

        # 現在の足先位置を計算
        self._update_current_foot_pos()

        # V26: gait_phase更新
        self.gait_phase += 2 * math.pi * self.gait_frequency * self.dt
        self.gait_phase = torch.fmod(self.gait_phase, 2 * math.pi)

        # 接地状態の更新
        if self.feet_air_time is not None:
            contacts = self._get_foot_contacts()
            if contacts is not None:
                self.feet_air_time += self.dt
                self.feet_air_time = torch.where(contacts, torch.zeros_like(self.feet_air_time), self.feet_air_time)
                self.last_contacts = contacts

        # 報酬計算
        self.rew_buf.zero_()
        for name, reward_func in self.reward_functions.items():
            rew = reward_func() * self.reward_scales[name]
            self.rew_buf += rew
            self.episode_sums[name] += rew

        # 前進距離追跡の更新
        self.last_base_pos_x.copy_(self.base_pos[:, 0])

        # コマンド再サンプリング
        self._resample_commands(self.episode_length_buf % int(self.env_cfg["resampling_time_s"] / self.dt) == 0)

        # 終了判定
        self.reset_buf = self.episode_length_buf > self.max_episode_length
        self.reset_buf |= torch.abs(self.base_euler[:, 1]) > self.env_cfg["termination_if_pitch_greater_than"]
        self.reset_buf |= torch.abs(self.base_euler[:, 0]) > self.env_cfg["termination_if_roll_greater_than"]

        if "termination_if_height_lower_than" in self.env_cfg:
            self.reset_buf |= self.base_pos[:, 2] < self.env_cfg["termination_if_height_lower_than"]

        self.extras["time_outs"] = (self.episode_length_buf > self.max_episode_length).to(dtype=gs.tc_float)

        self._reset_idx(self.reset_buf)
        self._update_observation()

        self.last_actions.copy_(self.actions)
        self.last_dof_vel.copy_(self.dof_vel)

        self.extras["observations"]["critic"] = self.obs_buf

        return self.obs_buf, self.rew_buf, self.reset_buf, self.extras

    def get_observations(self) -> tuple[torch.Tensor, dict[str, Any]]:
        self.extras["observations"]["critic"] = self.obs_buf
        return self.obs_buf, self.extras

    def get_privileged_observations(self) -> None:
        return None

    def _reset_idx(self, envs_idx: torch.Tensor | None = None) -> None:
        """環境をリセット"""
        self.robot.set_qpos(self.init_qpos, envs_idx=envs_idx, zero_velocity=True, skip_forward=True)

        if envs_idx is None:
            self.base_pos.copy_(self.init_base_pos)
            self.base_quat.copy_(self.init_base_quat)
            self.projected_gravity.copy_(self.init_projected_gravity)
            self.dof_pos.copy_(self.init_dof_pos)
            self.base_lin_vel.zero_()
            self.base_ang_vel.zero_()
            self.dof_vel.zero_()
            self.gait_phase.zero_()
            self.last_base_pos_x.zero_()
            self._resample_commands()
            self.episode_length_buf.zero_()
            self.rew_buf.zero_()
            self.reset_buf.fill_(True)
            self.actions.zero_()
            self.last_actions.zero_()
            for key in self.episode_sums:
                self.episode_sums[key].zero_()
        else:
            torch.where(envs_idx[:, None], self.init_base_pos, self.base_pos, out=self.base_pos)
            torch.where(envs_idx[:, None], self.init_base_quat, self.base_quat, out=self.base_quat)
            torch.where(
                envs_idx[:, None], self.init_projected_gravity, self.projected_gravity, out=self.projected_gravity
            )
            torch.where(envs_idx[:, None], self.init_dof_pos, self.dof_pos, out=self.dof_pos)
            self.base_lin_vel.masked_fill_(envs_idx[:, None], 0.0)
            self.base_ang_vel.masked_fill_(envs_idx[:, None], 0.0)
            self.dof_vel.masked_fill_(envs_idx[:, None], 0.0)
            self.gait_phase.masked_fill_(envs_idx, 0.0)
            self.last_base_pos_x.masked_fill_(envs_idx, 0.0)
            self._resample_commands(envs_idx)
            self.episode_length_buf.masked_fill_(envs_idx, 0)
            self.rew_buf.masked_fill_(envs_idx, 0.0)
            self.reset_buf.masked_fill_(envs_idx, True)
            self.actions.masked_fill_(envs_idx[:, None], 0.0)
            self.last_actions.masked_fill_(envs_idx[:, None], 0.0)
            for key in self.episode_sums:
                self.episode_sums[key].masked_fill_(envs_idx, 0.0)

        self._update_current_foot_pos()
        self._update_observation()

    def _resample_commands(self, envs_idx: torch.Tensor | None = None) -> None:
        """速度コマンドの再サンプリング"""
        lower = torch.tensor(
            [self.command_cfg["lin_vel_x"][0], self.command_cfg["lin_vel_y"][0], self.command_cfg["ang_vel_yaw"][0]],
            dtype=gs.tc_float,
            device=gs.device,
        )
        upper = torch.tensor(
            [self.command_cfg["lin_vel_x"][1], self.command_cfg["lin_vel_y"][1], self.command_cfg["ang_vel_yaw"][1]],
            dtype=gs.tc_float,
            device=gs.device,
        )

        commands = gs_rand(lower, upper, batch_shape=(self.num_envs,))

        if envs_idx is None:
            self.commands.copy_(commands)
        else:
            torch.where(envs_idx[:, None], commands, self.commands, out=self.commands)

    def _update_observation(self) -> None:
        """観測の更新（V26: 39次元、gait_phase復活）"""
        obs_list = [
            self.base_ang_vel * self.obs_scales["ang_vel"],  # 3
            self.projected_gravity,  # 3
            self.commands
            * torch.tensor(
                [[self.obs_scales["lin_vel"], self.obs_scales["lin_vel"], self.obs_scales["ang_vel"]]],
                dtype=gs.tc_float,
                device=gs.device,
            ),  # 3
            (self.dof_pos - self.default_dof_pos) * self.obs_scales["dof_pos"],  # 10
            self.dof_vel * self.obs_scales["dof_vel"],  # 10
            self.actions,  # 6
            torch.stack([torch.sin(self.gait_phase), torch.cos(self.gait_phase)], dim=-1),  # 2
            self.current_foot_pos * 5.0,  # 4
        ]  # 合計: 3+3+3+10+10+6+2+4 = 41次元

        self.obs_buf = torch.cat(obs_list, dim=-1)
        self.obs_buf = torch.clip(self.obs_buf, -self.env_cfg["clip_observations"], self.env_cfg["clip_observations"])

    def set_commands(self, commands: torch.Tensor, envs_idx: torch.Tensor | None = None) -> None:
        """コマンドを設定"""
        if envs_idx is None:
            self.commands.copy_(commands)
        else:
            torch.where(envs_idx[:, None], commands, self.commands, out=self.commands)

    def close(self) -> None:
        pass

    # ==================== 報酬関数 ====================

    def _reward_tracking_lin_vel(self) -> torch.Tensor:
        """線形速度追従報酬"""
        lin_vel_error = torch.sum(torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1)
        return torch.exp(-lin_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_tracking_ang_vel(self) -> torch.Tensor:
        """角速度追従報酬"""
        ang_vel_error = torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_lin_vel_z(self) -> torch.Tensor:
        """Z方向の線形速度ペナルティ"""
        return torch.square(self.base_lin_vel[:, 2])

    def _reward_ang_vel_xy(self) -> torch.Tensor:
        """XY方向の角速度ペナルティ"""
        return torch.sum(torch.square(self.base_ang_vel[:, :2]), dim=1)

    def _reward_orientation(self) -> torch.Tensor:
        """姿勢ペナルティ"""
        return torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1)

    def _reward_base_height(self) -> torch.Tensor:
        """胴体高さ報酬"""
        target_height = self.reward_cfg.get("base_height_target", 0.19)
        height_error = torch.square(self.base_pos[:, 2] - target_height)
        return torch.exp(-height_error / 0.01)

    def _reward_torques(self) -> torch.Tensor:
        """トルクペナルティ"""
        torques = self.robot.get_dofs_force(self.motors_dof_idx)
        return torch.sum(torch.square(torques), dim=1)

    def _reward_dof_vel(self) -> torch.Tensor:
        """関節速度ペナルティ"""
        return torch.sum(torch.square(self.dof_vel), dim=1)

    def _reward_dof_acc(self) -> torch.Tensor:
        """関節加速度ペナルティ"""
        return torch.sum(torch.square((self.dof_vel - self.last_dof_vel) / self.dt), dim=1)

    def _reward_action_rate(self) -> torch.Tensor:
        """アクション変化率ペナルティ"""
        return torch.sum(torch.square(self.actions - self.last_actions), dim=1)

    def _reward_termination(self) -> torch.Tensor:
        """終了ペナルティ"""
        return self.reset_buf.float() * ~self.extras["time_outs"].bool()

    def _reward_alive(self) -> torch.Tensor:
        """生存報酬"""
        return torch.ones((self.num_envs,), dtype=gs.tc_float, device=gs.device)

    def _reward_forward_progress(self) -> torch.Tensor:
        """前進距離報酬"""
        delta_x = self.base_pos[:, 0] - self.last_base_pos_x
        return torch.clamp(delta_x, min=0.0) / self.dt

    def _reward_foot_height_diff(self) -> torch.Tensor:
        """左右足の高さ差報酬（交互歩行誘導）"""
        left_z = self.current_foot_pos[:, 1]
        right_z = self.current_foot_pos[:, 3]
        height_diff = torch.abs(left_z - right_z)
        return height_diff / 0.05

    def _reward_hip_pitch_alternation(self) -> torch.Tensor:
        """股関節ピッチの交互運動報酬"""
        left_hip_pitch = self.dof_pos[:, self.left_hip_pitch_idx]
        right_hip_pitch = self.dof_pos[:, self.right_hip_pitch_idx]
        diff = left_hip_pitch - right_hip_pitch
        return torch.abs(diff) / 1.0

    def _reward_gait_cycle(self) -> torch.Tensor:
        """周期的歩容報酬（V26新規）

        gait_phaseに同期した足の高さを報酬化。
        高周波振動を抑制し、自然な歩行リズムを誘導。
        """
        # 左脚: sin(phase)で上下
        # 右脚: sin(phase + π)で逆位相
        left_target_z = self.default_foot_z + self.swing_height * torch.sin(self.gait_phase)
        right_target_z = self.default_foot_z + self.swing_height * torch.sin(self.gait_phase + math.pi)

        left_z = self.current_foot_pos[:, 1]
        right_z = self.current_foot_pos[:, 3]

        left_error = torch.abs(left_z - left_target_z)
        right_error = torch.abs(right_z - right_target_z)

        return torch.exp(-(left_error + right_error) / 0.02)

    def _reward_hip_roll_penalty(self) -> torch.Tensor:
        """hip_rollが大きくなりすぎることへのペナルティ（V26新規）"""
        left_hip_roll = self.dof_pos[:, self.left_hip_roll_idx]
        right_hip_roll = self.dof_pos[:, self.right_hip_roll_idx]
        # 目標は0付近
        return torch.square(left_hip_roll) + torch.square(right_hip_roll)
