"""
BSL-Droid二脚ロボット Genesis環境 - 統一版

============================================================
全バージョン (V2-V22) の報酬関数を統合した統一環境
============================================================

このファイルは biped_env_v2.py 〜 biped_env_v22.py を統合したものです。
すべての報酬関数を含み、train側の reward_scales で使用する報酬を選択します。
これにより、既存の学習済みモデルとの互換性を維持しつつ、
コードの重複を削減しています。

観測空間 (39次元):
- base_ang_vel (3): ボディローカル角速度
- projected_gravity (3): ボディローカル重力ベクトル
- commands (3): 速度コマンド
- dof_pos (10): 関節位置（デフォルトからの偏差）
- dof_vel (10): 関節速度
- actions (10): 前回のアクション

行動空間 (10次元):
- 各関節の目標位置オフセット（action_scaleでスケーリング）

含まれる報酬関数:
- 基本報酬 (V2-): tracking_lin_vel, tracking_ang_vel, lin_vel_z, ang_vel_xy,
  orientation, base_height, torques, dof_vel, dof_acc, action_rate,
  similar_to_default, feet_air_time, no_fly
- V2追加: joint_symmetry, smoothness
- V5追加: foot_clearance, stride_length (V5), alternating_gait, foot_swing, single_stance
- V8追加: trajectory_tracking, phase_consistency, symmetric_gait,
  smooth_joint_velocity, heading_alignment
- V12追加: hip_pitch_alternation, hip_pitch_opposite_sign, hip_pitch_range,
  stride_length (V12), forward_progress, alive, pitch_penalty
- V13追加: hip_pitch_velocity, contact_alternation, roll_penalty
- V18追加: backward_penalty, yaw_penalty, lateral_velocity_penalty,
  base_height_high
- V20追加: backward_velocity, yaw_rate_penalty
"""

import math

import torch

import genesis as gs
from genesis.utils.geom import quat_to_xyz, transform_by_quat, inv_quat, transform_quat_by_quat


def gs_rand(lower, upper, batch_shape):
    """一様乱数を生成"""
    assert lower.shape == upper.shape
    return (upper - lower) * torch.rand(size=(*batch_shape, *lower.shape), dtype=gs.tc_float, device=gs.device) + lower


class CamberTrajectory:
    """
    楕円弧軌道ジェネレータ（ROS2 biped_gait_controlから移植）

    足先の(x, z)相対軌道を生成:
    - Stance相（0.0-0.5）: 線形接地移動（後→前）
    - Swing相（0.5-1.0）: 半楕円リフト（前→後）
    """

    def __init__(self, step_height=0.04, step_length=0.08, device=None):
        """
        Args:
            step_height: 足の持ち上げ高さ [m]
            step_length: 歩幅 [m]
            device: PyTorchデバイス
        """
        self.step_height = step_height
        self.step_length = step_length
        self.device = device or gs.device

    def generate(self, phase):
        """
        位相から足先の目標位置を計算

        Args:
            phase: torch.Tensor (num_envs,) 範囲 [0.0, 1.0]

        Returns:
            x: torch.Tensor (num_envs,) 前後方向の相対位置
            z: torch.Tensor (num_envs,) 上下方向の相対位置
        """
        half_step = self.step_length / 2

        # Stance相とSwing相を分けて計算
        is_stance = phase < 0.5

        # Stance相（0.0-0.5）: 線形移動（後→前）
        stance_progress = phase / 0.5
        x_stance = half_step * (2 * stance_progress - 1)  # -half_step → +half_step
        z_stance = torch.zeros_like(phase)  # 地面接地

        # Swing相（0.5-1.0）: 楕円軌道（前→後）
        swing_progress = (phase - 0.5) / 0.5
        theta = torch.pi * swing_progress  # 0 → π
        x_swing = half_step * torch.cos(theta)  # +half_step → -half_step
        z_swing = self.step_height * torch.sin(theta)  # 0 → peak → 0

        # 位相に応じて切り替え
        x = torch.where(is_stance, x_stance, x_swing)
        z = torch.where(is_stance, z_stance, z_swing)

        return x, z


class BipedEnv:
    """BSL-Droid二脚ロボットのGenesis強化学習環境（統一版）"""

    def __init__(self, num_envs, env_cfg, obs_cfg, reward_cfg, command_cfg, show_viewer=False, recording_camera=False):
        self.num_envs = num_envs
        self.num_obs = obs_cfg["num_obs"]
        self.num_privileged_obs = None
        self.num_actions = env_cfg["num_actions"]
        self.num_commands = command_cfg["num_commands"]
        self.device = gs.device

        self.simulate_action_latency = True  # 実機では1ステップの遅延がある
        self.dt = 0.02  # 制御周波数 50Hz
        self.max_episode_length = math.ceil(env_cfg["episode_length_s"] / self.dt)

        self.env_cfg = env_cfg
        self.obs_cfg = obs_cfg
        self.reward_cfg = reward_cfg
        self.command_cfg = command_cfg

        self.obs_scales = obs_cfg["obs_scales"]
        self.reward_scales = reward_cfg["reward_scales"]

        # V8: 歩容パラメータ
        self.gait_cfg = env_cfg.get("gait_cfg", {})
        self.step_height = self.gait_cfg.get("step_height", 0.04)
        self.step_length = self.gait_cfg.get("step_length", 0.08)
        self.step_frequency = self.gait_cfg.get("step_frequency", 0.8)  # Hz

        # V8: 参照軌道ジェネレータ
        self.reference_trajectory = CamberTrajectory(
            step_height=self.step_height,
            step_length=self.step_length,
            device=gs.device,
        )

        # V8: 脚のリンク長（URDFと一致させる）
        self.thigh_length = self.gait_cfg.get("thigh_length", 0.18)
        self.shank_length = self.gait_cfg.get("shank_length", 0.20)

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
                camera_pos=(2.0, 0.0, 1.5),
                camera_lookat=(0.0, 0.0, 0.5),
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

        # ロボットを追加（外部URDFを使用）
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file=self.env_cfg["urdf_path"],
                pos=self.env_cfg["base_init_pos"],
                quat=self.env_cfg["base_init_quat"],
            ),
        )

        # 録画用カメラを追加（ビルド前に追加する必要がある）
        self.recording_cam = None
        if recording_camera:
            self.recording_cam = self.scene.add_camera(
                res=(1280, 720),
                pos=(2.0, -1.5, 1.0),  # 斜め横から撮影
                lookat=(0.0, 0.0, 0.4),
                fov=50,
                GUI=False,
            )

        # ビルド
        self.scene.build(n_envs=num_envs)

        # 関節名からインデックスを取得
        self.motors_dof_idx = torch.tensor(
            [self.robot.get_joint(name).dofs_idx_local[0] for name in self.env_cfg["joint_names"]],
            dtype=gs.tc_int,
            device=gs.device,
        )
        self.actions_dof_idx = torch.argsort(self.motors_dof_idx)

        # PD制御パラメータを設定
        self.robot.set_dofs_kp([self.env_cfg["kp"]] * self.num_actions, self.motors_dof_idx)
        self.robot.set_dofs_kv([self.env_cfg["kd"]] * self.num_actions, self.motors_dof_idx)

        # 重力ベクトル
        self.global_gravity = torch.tensor([0.0, 0.0, -1.0], dtype=gs.tc_float, device=gs.device)

        # 初期状態
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

        # 足のリンクインデックス
        self.feet_names = env_cfg.get("feet_names", ["left_foot_link", "right_foot_link"])
        self.feet_indices = []
        for name in self.feet_names:
            try:
                link = self.robot.get_link(name)
                self.feet_indices.append(link.idx_local)
            except Exception:
                pass
        self.feet_indices = torch.tensor(self.feet_indices, dtype=gs.tc_int, device=gs.device) if self.feet_indices else None

        # バッファ初期化
        self.base_lin_vel = torch.empty((self.num_envs, 3), dtype=gs.tc_float, device=gs.device)
        self.base_ang_vel = torch.empty((self.num_envs, 3), dtype=gs.tc_float, device=gs.device)
        self.projected_gravity = torch.empty((self.num_envs, 3), dtype=gs.tc_float, device=gs.device)
        self.obs_buf = torch.empty((self.num_envs, self.num_obs), dtype=gs.tc_float, device=gs.device)
        self.rew_buf = torch.empty((self.num_envs,), dtype=gs.tc_float, device=gs.device)
        self.reset_buf = torch.ones((self.num_envs,), dtype=gs.tc_bool, device=gs.device)
        self.episode_length_buf = torch.empty((self.num_envs,), dtype=gs.tc_int, device=gs.device)
        self.commands = torch.empty((self.num_envs, self.num_commands), dtype=gs.tc_float, device=gs.device)
        self.commands_scale = torch.tensor(
            [self.obs_scales["lin_vel"], self.obs_scales["lin_vel"], self.obs_scales["ang_vel"]],
            device=gs.device,
            dtype=gs.tc_float,
        )
        self.commands_limits = [
            torch.tensor(values, dtype=gs.tc_float, device=gs.device)
            for values in zip(
                self.command_cfg["lin_vel_x_range"],
                self.command_cfg["lin_vel_y_range"],
                self.command_cfg["ang_vel_range"],
            )
        ]
        self.actions = torch.zeros((self.num_envs, self.num_actions), dtype=gs.tc_float, device=gs.device)
        self.last_actions = torch.zeros_like(self.actions)
        self.dof_pos = torch.empty_like(self.actions)
        self.dof_vel = torch.empty_like(self.actions)
        self.last_dof_vel = torch.zeros_like(self.actions)
        self.last_last_dof_vel = torch.zeros_like(self.actions)  # V8: ジャーク計算用
        self.base_pos = torch.empty((self.num_envs, 3), dtype=gs.tc_float, device=gs.device)
        self.base_quat = torch.empty((self.num_envs, 4), dtype=gs.tc_float, device=gs.device)
        self.default_dof_pos = torch.tensor(
            [self.env_cfg["default_joint_angles"][name] for name in self.env_cfg["joint_names"]],
            dtype=gs.tc_float,
            device=gs.device,
        )

        # feet_air_time用バッファ
        if self.feet_indices is not None and len(self.feet_indices) > 0:
            self.feet_air_time = torch.zeros((self.num_envs, len(self.feet_indices)), dtype=gs.tc_float, device=gs.device)
            self.last_contacts = torch.ones((self.num_envs, len(self.feet_indices)), dtype=gs.tc_bool, device=gs.device)
            # V5: 前ステップの足の位置を保存（歩幅計算用）
            self.last_feet_pos = torch.zeros((self.num_envs, len(self.feet_indices), 3), dtype=gs.tc_float, device=gs.device)
        else:
            self.feet_air_time = None
            self.last_contacts = None
            self.last_feet_pos = None

        # V13: hip_pitch位置のトラッキング（交互歩行分析用）
        # joint_names: [L_hip_yaw, L_hip_roll, L_hip_pitch, L_knee, L_ankle,
        #               R_hip_yaw, R_hip_roll, R_hip_pitch, R_knee, R_ankle]
        self.left_hip_pitch_idx = 2   # left_hip_pitch_joint
        self.right_hip_pitch_idx = 7  # right_hip_pitch_joint
        self.last_left_hip_pitch = torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device)
        self.last_right_hip_pitch = torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device)

        # V13: 歩行サイクル用の位相追跡
        self.gait_phase = torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device)
        self.gait_frequency = self.reward_cfg.get("gait_frequency", 1.5)  # Hz

        # V13: 接地タイミング追跡
        self.last_contact_change_time = torch.zeros((self.num_envs, 2), dtype=gs.tc_float, device=gs.device)

        # V13: hip_pitchの動的振幅追跡（移動平均）
        self.hip_pitch_max_recent = torch.zeros((self.num_envs, 2), dtype=gs.tc_float, device=gs.device)
        self.hip_pitch_min_recent = torch.zeros((self.num_envs, 2), dtype=gs.tc_float, device=gs.device)

        # V12: hip_pitchの最大・最小値を追跡（可動範囲の利用度分析）
        self.hip_pitch_max = torch.zeros((self.num_envs, 2), dtype=gs.tc_float, device=gs.device)  # [left, right]
        self.hip_pitch_min = torch.zeros((self.num_envs, 2), dtype=gs.tc_float, device=gs.device)  # [left, right]

        # V21: 高さ目標
        self.base_height_target = self.reward_cfg.get("base_height_target", 0.35)
        # V18: 非対称報酬用パラメータ
        self.height_penalty_high = self.reward_cfg.get("height_penalty_high", 40.0)
        self.height_penalty_low = self.reward_cfg.get("height_penalty_low", 5.0)

        self.extras = dict()
        self.extras["observations"] = dict()

        # 報酬関数の準備
        self.reward_functions, self.episode_sums = dict(), dict()
        for name in self.reward_scales.keys():
            self.reward_scales[name] *= self.dt
            self.reward_functions[name] = getattr(self, "_reward_" + name)
            self.episode_sums[name] = torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device)

    def _resample_commands(self, envs_idx):
        """速度コマンドを再サンプリング"""
        commands = gs_rand(*self.commands_limits, (self.num_envs,))
        if envs_idx is None:
            self.commands.copy_(commands)
        else:
            torch.where(envs_idx[:, None], commands, self.commands, out=self.commands)

    def _get_foot_contacts(self):
        """足の接地状態を取得（簡易版：Z座標ベース）"""
        if self.feet_indices is None:
            return None
        contact_threshold = 0.03
        link_pos = self.robot.get_links_pos()
        feet_z = link_pos[:, self.feet_indices, 2]
        return feet_z < contact_threshold

    def _get_foot_positions(self):
        """足の位置を取得（ワールド座標）- V5用"""
        if self.feet_indices is None:
            return None
        link_pos = self.robot.get_links_pos()  # (n_envs, n_links, 3)
        return link_pos[:, self.feet_indices, :]  # (n_envs, n_feet, 3)

    def _get_foot_heights(self):
        """足の高さを取得（地面からの高さ）- V5用"""
        if self.feet_indices is None:
            return None
        link_pos = self.robot.get_links_pos()  # (n_envs, n_links, 3)
        return link_pos[:, self.feet_indices, 2]  # (n_envs, n_feet)

    def _update_gait_phase(self):
        """歩行位相を更新 - V8用"""
        # 位相を進める: phase += dt * frequency
        phase_increment = self.dt * self.step_frequency
        self.gait_phase = (self.gait_phase + phase_increment) % 1.0

    def _get_foot_position_from_joints(self, leg_joints, is_left=True):
        """
        関節角度からつま先のX-Z相対位置を計算（簡易2リンク順運動学）- V8用

        Args:
            leg_joints: torch.Tensor (num_envs, 5)
                [hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch]
            is_left: 左足かどうか

        Returns:
            foot_xz: torch.Tensor (num_envs, 2) つま先の相対位置 [x, z]
        """
        # 関節角度を取得（hip_pitch, knee_pitch が主に影響）
        hip_pitch = leg_joints[:, 2]   # index 2: hip_pitch
        knee_pitch = leg_joints[:, 3]  # index 3: knee_pitch

        # 順運動学（X-Z平面、股関節からの相対位置）
        # 大腿の先端（膝の位置）
        knee_x = self.thigh_length * torch.sin(hip_pitch)
        knee_z = -self.thigh_length * torch.cos(hip_pitch)

        # 下腿の先端（足首の位置）
        total_pitch = hip_pitch + knee_pitch
        ankle_x = knee_x + self.shank_length * torch.sin(total_pitch)
        ankle_z = knee_z - self.shank_length * torch.cos(total_pitch)

        return torch.stack([ankle_x, ankle_z], dim=1)

    def step(self, actions):
        """環境を1ステップ進める"""
        self.actions = torch.clip(actions, -self.env_cfg["clip_actions"], self.env_cfg["clip_actions"])
        exec_actions = self.last_actions if self.simulate_action_latency else self.actions
        target_dof_pos = exec_actions * self.env_cfg["action_scale"] + self.default_dof_pos

        self.robot.control_dofs_position(target_dof_pos[:, self.actions_dof_idx], slice(6, 6 + self.num_actions))
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

        # hip_pitch位置を保存（交互歩行報酬計算前）
        self.last_left_hip_pitch = self.dof_pos[:, self.left_hip_pitch_idx].clone()
        self.last_right_hip_pitch = self.dof_pos[:, self.right_hip_pitch_idx].clone()

        self.dof_pos = self.robot.get_dofs_position(self.motors_dof_idx)
        self.dof_vel = self.robot.get_dofs_velocity(self.motors_dof_idx)

        # V13: 歩行位相を更新
        self.gait_phase = (self.gait_phase + self.dt * self.gait_frequency * 2 * math.pi) % (2 * math.pi)

        # V13: hip_pitchの動的振幅を更新（指数移動平均）
        alpha = 0.1  # 更新係数
        left_hp = self.dof_pos[:, self.left_hip_pitch_idx]
        right_hp = self.dof_pos[:, self.right_hip_pitch_idx]
        self.hip_pitch_max_recent[:, 0] = torch.maximum(
            self.hip_pitch_max_recent[:, 0] * (1 - alpha) + left_hp * alpha,
            left_hp
        )
        self.hip_pitch_max_recent[:, 1] = torch.maximum(
            self.hip_pitch_max_recent[:, 1] * (1 - alpha) + right_hp * alpha,
            right_hp
        )
        self.hip_pitch_min_recent[:, 0] = torch.minimum(
            self.hip_pitch_min_recent[:, 0] * (1 - alpha) + left_hp * alpha,
            left_hp
        )
        self.hip_pitch_min_recent[:, 1] = torch.minimum(
            self.hip_pitch_min_recent[:, 1] * (1 - alpha) + right_hp * alpha,
            right_hp
        )

        # V12: hip_pitchの最大・最小値を更新
        self.hip_pitch_max[:, 0] = torch.maximum(self.hip_pitch_max[:, 0], left_hp)
        self.hip_pitch_max[:, 1] = torch.maximum(self.hip_pitch_max[:, 1], right_hp)
        self.hip_pitch_min[:, 0] = torch.minimum(self.hip_pitch_min[:, 0], left_hp)
        self.hip_pitch_min[:, 1] = torch.minimum(self.hip_pitch_min[:, 1], right_hp)

        # feet_air_time更新
        if self.feet_air_time is not None:
            contacts = self._get_foot_contacts()
            if contacts is not None:
                first_contact = (self.feet_air_time > 0.0) & contacts & ~self.last_contacts
                self.feet_air_time += self.dt
                self.feet_air_time = torch.where(contacts, torch.zeros_like(self.feet_air_time), self.feet_air_time)

                # V13: 接地状態変化のタイミングを記録
                contact_changed = contacts != self.last_contacts
                current_time = self.episode_length_buf.float() * self.dt
                for i in range(2):
                    self.last_contact_change_time[:, i] = torch.where(
                        contact_changed[:, i],
                        current_time,
                        self.last_contact_change_time[:, i]
                    )

                # V5: 足の位置を更新（歩幅計算用）
                if self.last_feet_pos is not None:
                    current_feet_pos = self._get_foot_positions()
                    if current_feet_pos is not None:
                        # 接地時に前回位置を更新
                        self.last_feet_pos = torch.where(
                            contacts.unsqueeze(-1),  # (n_envs, n_feet, 1)
                            current_feet_pos,
                            self.last_feet_pos
                        )

                self.last_contacts = contacts
                self._first_contact = first_contact
            else:
                self._first_contact = None
        else:
            self._first_contact = None

        # 報酬計算
        self.rew_buf.zero_()
        for name, reward_func in self.reward_functions.items():
            rew = reward_func() * self.reward_scales[name]
            self.rew_buf += rew
            self.episode_sums[name] += rew

        # コマンド再サンプリング
        self._resample_commands(self.episode_length_buf % int(self.env_cfg["resampling_time_s"] / self.dt) == 0)

        # 終了判定
        self.reset_buf = self.episode_length_buf > self.max_episode_length
        self.reset_buf |= torch.abs(self.base_euler[:, 1]) > self.env_cfg["termination_if_pitch_greater_than"]
        self.reset_buf |= torch.abs(self.base_euler[:, 0]) > self.env_cfg["termination_if_roll_greater_than"]

        self.extras["time_outs"] = (self.episode_length_buf > self.max_episode_length).to(dtype=gs.tc_float)

        self._reset_idx(self.reset_buf)
        self._update_observation()

        self.last_actions.copy_(self.actions)
        self.last_last_dof_vel.copy_(self.last_dof_vel)  # V8: ジャーク計算用
        self.last_dof_vel.copy_(self.dof_vel)

        self.extras["observations"]["critic"] = self.obs_buf

        return self.obs_buf, self.rew_buf, self.reset_buf, self.extras

    def get_observations(self):
        self.extras["observations"]["critic"] = self.obs_buf
        return self.obs_buf, self.extras

    def get_privileged_observations(self):
        return None

    def _reset_idx(self, envs_idx=None):
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
            self.actions.zero_()
            self.last_actions.zero_()
            self.last_dof_vel.zero_()
            self.last_last_dof_vel.zero_()
            self.episode_length_buf.zero_()
            self.reset_buf.fill_(True)
            self.last_left_hip_pitch.zero_()
            self.last_right_hip_pitch.zero_()
            self.gait_phase.zero_()
            self.last_contact_change_time.zero_()
            self.hip_pitch_max_recent.zero_()
            self.hip_pitch_min_recent.zero_()
            self.hip_pitch_max.zero_()
            self.hip_pitch_min.zero_()
            if self.feet_air_time is not None:
                self.feet_air_time.zero_()
                self.last_contacts.fill_(True)
            if self.last_feet_pos is not None:
                self.last_feet_pos.zero_()
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
            self.actions.masked_fill_(envs_idx[:, None], 0.0)
            self.last_actions.masked_fill_(envs_idx[:, None], 0.0)
            self.last_dof_vel.masked_fill_(envs_idx[:, None], 0.0)
            self.last_last_dof_vel.masked_fill_(envs_idx[:, None], 0.0)
            self.episode_length_buf.masked_fill_(envs_idx, 0)
            self.reset_buf.masked_fill_(envs_idx, True)
            self.last_left_hip_pitch.masked_fill_(envs_idx, 0.0)
            self.last_right_hip_pitch.masked_fill_(envs_idx, 0.0)
            self.gait_phase.masked_fill_(envs_idx, 0.0)
            self.last_contact_change_time.masked_fill_(envs_idx[:, None], 0.0)
            self.hip_pitch_max_recent.masked_fill_(envs_idx[:, None], 0.0)
            self.hip_pitch_min_recent.masked_fill_(envs_idx[:, None], 0.0)
            self.hip_pitch_max.masked_fill_(envs_idx[:, None], 0.0)
            self.hip_pitch_min.masked_fill_(envs_idx[:, None], 0.0)
            if self.feet_air_time is not None:
                self.feet_air_time.masked_fill_(envs_idx[:, None], 0.0)
                self.last_contacts.masked_fill_(envs_idx[:, None], True)
            if self.last_feet_pos is not None:
                self.last_feet_pos.masked_fill_(envs_idx[:, None, None], 0.0)

        # エピソード統計
        n_envs = envs_idx.sum() if envs_idx is not None else self.num_envs
        self.extras["episode"] = {}
        for key, value in self.episode_sums.items():
            if envs_idx is None:
                mean = value.mean()
                value.zero_()
            else:
                mean = torch.where(n_envs > 0, value[envs_idx].sum() / n_envs, 0.0)
                self.extras["episode"]["rew_" + key] = mean / self.env_cfg["episode_length_s"]
                value.masked_fill_(envs_idx, 0.0)
            self.extras["episode"]["rew_" + key] = mean / self.env_cfg["episode_length_s"]

        self._resample_commands(envs_idx)

    def _update_observation(self):
        """観測ベクトルを更新 (39次元)"""
        self.obs_buf = torch.concatenate(
            (
                self.base_ang_vel * self.obs_scales["ang_vel"],  # 3
                self.projected_gravity,  # 3
                self.commands * self.commands_scale,  # 3
                (self.dof_pos - self.default_dof_pos) * self.obs_scales["dof_pos"],  # 10
                self.dof_vel * self.obs_scales["dof_vel"],  # 10
                self.actions,  # 10
            ),
            dim=-1,
        )

    def reset(self):
        self._reset_idx()
        self._update_observation()
        return self.obs_buf, None

    # ============================================================================
    # 報酬関数 - 基本報酬 (V2-)
    # ============================================================================

    def _reward_tracking_lin_vel(self):
        """線形速度追従報酬（主タスク）"""
        lin_vel_error = torch.sum(torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1)
        return torch.exp(-lin_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_tracking_ang_vel(self):
        """角速度追従報酬"""
        ang_vel_error = torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_lin_vel_z(self):
        """Z方向速度ペナルティ"""
        return torch.square(self.base_lin_vel[:, 2])

    def _reward_ang_vel_xy(self):
        """XY角速度ペナルティ"""
        return torch.sum(torch.square(self.base_ang_vel[:, :2]), dim=1)

    def _reward_orientation(self):
        """姿勢維持ペナルティ"""
        return torch.square(self.projected_gravity[:, 2] + 1.0)

    def _reward_base_height(self):
        """V21: ベース高さペナルティ（対称版）

        V19のバグを修正: error²のみを返す。
        スケールはreward_scalesで制御。
        """
        height = self.base_pos[:, 2]
        error = height - self.base_height_target
        return torch.square(error)

    def _reward_base_height_high(self):
        """V21: 高すぎる場合の追加ペナルティ

        目標より高い場合のみerror²を返す。
        低い場合は0。
        これにより非対称性を実現。
        """
        height = self.base_pos[:, 2]
        error = height - self.base_height_target
        # 高い場合のみペナルティ
        high_error = torch.clamp(error, min=0.0)
        return torch.square(high_error)

    def _reward_base_height_asymmetric(self):
        """V18: 非対称ベース高さペナルティ

        高すぎる場合は強いペナルティ、低すぎる場合は弱いペナルティ。
        これにより、低い姿勢への探索を促進する。

        - 高すぎる（error > 0）: height_penalty_high * error²
        - 低すぎる（error < 0）: height_penalty_low * error²
        """
        height = self.base_pos[:, 2]
        error = height - self.base_height_target

        # 非対称ペナルティ
        penalty = torch.where(
            error > 0,
            self.height_penalty_high * torch.square(error),  # 高い: 強いペナルティ
            self.height_penalty_low * torch.square(error)    # 低い: 弱いペナルティ
        )

        return penalty

    def _reward_torques(self):
        """トルクペナルティ（エネルギー効率）"""
        return torch.sum(torch.square(self.actions), dim=1)

    def _reward_dof_vel(self):
        """関節速度ペナルティ（振動抑制）"""
        return torch.sum(torch.square(self.dof_vel), dim=1)

    def _reward_dof_acc(self):
        """関節加速度ペナルティ（振動抑制・強化）"""
        return torch.sum(torch.square((self.dof_vel - self.last_dof_vel) / self.dt), dim=1)

    def _reward_action_rate(self):
        """アクション変化率ペナルティ（振動抑制・強化）"""
        return torch.sum(torch.square(self.last_actions - self.actions), dim=1)

    def _reward_similar_to_default(self):
        """デフォルト姿勢維持ペナルティ"""
        return torch.sum(torch.abs(self.dof_pos - self.default_dof_pos), dim=1)

    # ============================================================================
    # 報酬関数 - 二脚歩行報酬
    # ============================================================================

    def _reward_feet_air_time(self):
        """足の滞空時間報酬"""
        if self.feet_air_time is None or self._first_contact is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        target_air_time = self.reward_cfg.get("feet_air_time_target", 0.25)
        rew_air_time = torch.sum(
            (self.feet_air_time - target_air_time) * self._first_contact.float(), dim=1
        )
        rew_air_time *= (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return rew_air_time

    def _reward_no_fly(self):
        """両足同時離地ペナルティ"""
        if self.feet_air_time is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        both_feet_in_air = ~contacts.any(dim=1)
        return both_feet_in_air.float()

    def _reward_alternating_gait(self):
        """交互歩行報酬（接地状態ベース）

        左右の足が交互に接地している状態を報酬。
        """
        contacts = self._get_foot_contacts()
        if contacts is None or contacts.shape[1] != 2:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        left_contact = contacts[:, 0]
        right_contact = contacts[:, 1]
        alternating = left_contact ^ right_contact  # XOR: 片方だけ接地
        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return alternating.float() * has_command

    def _reward_foot_swing(self):
        """足のスイング報酬"""
        if self.feet_air_time is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        air_time_reward = torch.clamp(self.feet_air_time, 0.0, 0.3)
        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return torch.sum(air_time_reward, dim=1) * has_command

    def _reward_single_stance(self):
        """片足立ち報酬（legged_gym Cassieスタイル）

        片足だけが接地している状態を報酬。
        """
        contacts = self._get_foot_contacts()
        if contacts is None or contacts.shape[1] != 2:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        single_stance = contacts.sum(dim=1) == 1
        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return single_stance.float() * has_command

    # ============================================================================
    # 報酬関数 - V2追加: 対称性・滑らかさ
    # ============================================================================

    def _reward_joint_symmetry(self):
        """左右対称性報酬（自然な歩行）- V2"""
        # 左脚と右脚の対応する関節速度の振幅が同程度かをチェック
        left_vel = self.dof_vel[:, :5]
        right_vel = self.dof_vel[:, 5:]
        return torch.sum(torch.square(torch.abs(left_vel) - torch.abs(right_vel)), dim=1)

    def _reward_smoothness(self):
        """動作の滑らかさ報酬（2次微分ペナルティ）- V2"""
        return torch.sum(torch.square(self.dof_vel - self.last_dof_vel), dim=1)

    # ============================================================================
    # 報酬関数 - V5追加: 大きな歩幅
    # ============================================================================

    def _reward_foot_clearance(self):
        """足の高さ報酬: 遊脚時に足を高く上げることを促進 - V5"""
        foot_heights = self._get_foot_heights()
        if foot_heights is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # 遊脚（空中）の足の高さを報酬
        # 目標: 10cm以上の高さ
        target_height = 0.10
        clearance = torch.clamp(foot_heights - target_height, min=0.0)

        # 空中にいる足のみカウント
        airborne_feet = ~contacts
        clearance_reward = clearance * airborne_feet.float()

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()

        return torch.sum(clearance_reward, dim=1) * has_command

    def _reward_stride_length(self):
        """ストライド長報酬 - V12版

        左右hip_pitchの差分（ストライド長に相当）を報酬。
        差が大きいほど大きな歩幅で歩いている。
        """
        left_hp = self.dof_pos[:, self.left_hip_pitch_idx]
        right_hp = self.dof_pos[:, self.right_hip_pitch_idx]

        # hip_pitch差分の絶対値（ストライド長に相当）
        stride = torch.abs(left_hp - right_hp)

        # 0.5rad差で報酬1.0
        reward = stride / 0.5

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return torch.clamp(reward, max=1.5) * has_command

    def _reward_stride_length_v5(self):
        """歩幅報酬: 前回接地位置からの距離を報酬 - V5版"""
        if self.last_feet_pos is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        current_feet_pos = self._get_foot_positions()
        if current_feet_pos is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # 前回接地位置からの水平距離
        stride_vec = current_feet_pos[:, :, :2] - self.last_feet_pos[:, :, :2]  # XY平面
        stride_length = torch.norm(stride_vec, dim=2)  # (n_envs, n_feet)

        # 目標歩幅: 20cm以上
        target_stride = 0.20
        stride_reward = torch.clamp(stride_length - target_stride, min=0.0)

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()

        return torch.sum(stride_reward, dim=1) * has_command

    # ============================================================================
    # 報酬関数 - V8追加: Phase-based Reference
    # ============================================================================

    def _reward_trajectory_tracking(self):
        """
        Phase-based Reference軌道追従報酬 - V8

        左右の足先位置が、参照軌道（楕円弧）に沿っているかを評価
        """
        # 左右の関節角度を取得
        left_joints = self.dof_pos[:, 0:5]
        right_joints = self.dof_pos[:, 5:10]

        # 順運動学でつま先位置を計算
        left_foot_xz = self._get_foot_position_from_joints(left_joints, is_left=True)
        right_foot_xz = self._get_foot_position_from_joints(right_joints, is_left=False)

        # 位相から参照軌道上の目標位置を取得
        # 左足: phase、右足: phase + 0.5（180度位相差）
        # gait_phase は 0-2π なので正規化
        left_phase = (self.gait_phase / (2 * math.pi)) % 1.0
        right_phase = (left_phase + 0.5) % 1.0

        left_ref_x, left_ref_z = self.reference_trajectory.generate(left_phase)
        right_ref_x, right_ref_z = self.reference_trajectory.generate(right_phase)

        # 誤差を計算（X-Z平面上のユークリッド距離）
        left_error = torch.sqrt(
            torch.square(left_foot_xz[:, 0] - left_ref_x) +
            torch.square(left_foot_xz[:, 1] - left_ref_z)
        )
        right_error = torch.sqrt(
            torch.square(right_foot_xz[:, 0] - right_ref_x) +
            torch.square(right_foot_xz[:, 1] - right_ref_z)
        )

        # ガウス分布で報酬に変換（σ=0.05m、5cm以内で高報酬）
        sigma = self.reward_cfg.get("trajectory_tracking_sigma", 0.05)
        left_reward = torch.exp(-torch.square(left_error) / (2 * sigma * sigma))
        right_reward = torch.exp(-torch.square(right_error) / (2 * sigma * sigma))

        # 平均報酬（コマンドがある時のみ）
        total_reward = (left_reward + right_reward) / 2
        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()

        return total_reward * has_command

    def _reward_phase_consistency(self):
        """
        位相一貫性報酬 - V8

        足の接地状態が、位相の期待値と一致しているかを評価
        - 位相 0.0-0.5: 左足接地、右足スイング
        - 位相 0.5-1.0: 左足スイング、右足接地
        """
        contacts = self._get_foot_contacts()
        if contacts is None or contacts.shape[1] != 2:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        left_contact = contacts[:, 0].float()
        right_contact = contacts[:, 1].float()

        # 期待される接地状態
        # gait_phase は 0-2π なので正規化
        normalized_phase = (self.gait_phase / (2 * math.pi)) % 1.0
        left_expected = (normalized_phase < 0.5).float()
        right_expected = (normalized_phase >= 0.5).float()

        # 一致度を計算
        left_match = 1.0 - torch.abs(left_contact - left_expected)
        right_match = 1.0 - torch.abs(right_contact - right_expected)

        consistency = (left_match + right_match) / 2
        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()

        return consistency * has_command

    def _reward_symmetric_gait(self):
        """
        左右対称な歩行を促進する報酬 - V8

        位相差を考慮して、左右の脚の動きが対称であることを評価。
        """
        # 関節構成: [hip_yaw, hip_roll, hip_pitch, knee, ankle] x 2
        left_joints = self.dof_pos[:, 0:5]
        right_joints = self.dof_pos[:, 5:10]

        # hip_yaw: 符号反転が理想（左が+なら右は-）
        yaw_symmetry = torch.abs(left_joints[:, 0] + right_joints[:, 0])

        # hip_roll: 符号反転が理想
        roll_symmetry = torch.abs(left_joints[:, 1] + right_joints[:, 1])

        # hip_pitch, knee, ankle: 関節速度の絶対値が同程度かを評価
        left_vel = self.dof_vel[:, 2:5]
        right_vel = self.dof_vel[:, 7:10]
        vel_magnitude_diff = torch.abs(torch.abs(left_vel) - torch.abs(right_vel))
        pitch_knee_ankle_symmetry = torch.sum(vel_magnitude_diff, dim=1)

        # 総合対称性（小さいほど良い）
        asymmetry = yaw_symmetry + roll_symmetry + 0.5 * pitch_knee_ankle_symmetry

        # ガウス分布で報酬に変換
        reward = torch.exp(-asymmetry / 0.5)

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    def _reward_smooth_joint_velocity(self):
        """
        関節速度の急激な変化（ジャーク）を抑制する報酬 - V8
        """
        # ジャーク = (vel - 2*last_vel + last_last_vel) / dt^2
        jerk = (self.dof_vel - 2 * self.last_dof_vel + self.last_last_dof_vel) / (self.dt * self.dt)

        # L2ノルムの二乗
        jerk_penalty = torch.sum(torch.square(jerk), dim=1)

        # スケーリング（値が非常に大きくなるため）
        return jerk_penalty * 1e-8

    def _reward_heading_alignment(self):
        """
        進行方向とロボットの向きを一致させる報酬 - V8
        """
        # 速度が十分にある場合のみ評価
        vel_magnitude = torch.norm(self.base_lin_vel[:, :2], dim=1)
        has_velocity = vel_magnitude > 0.05

        # ボディローカルで前方(X+)に進んでいれば、velのYは0に近いはず
        vel_y_error = torch.abs(self.base_lin_vel[:, 1])

        # Y方向速度が小さいほど高報酬
        reward = torch.exp(-vel_y_error / 0.1)

        # 速度がない場合は報酬なし
        reward = torch.where(has_velocity, reward, torch.zeros_like(reward))

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    # ============================================================================
    # 報酬関数 - V12追加: hip_pitch関連
    # ============================================================================

    def _reward_hip_pitch_alternation(self):
        """左右hip_pitchの逆相運動報酬（V11から継続、強化）

        左右のhip_pitchが逆方向に動くことを報酬。
        これは速度ベースなので、静的な固定姿勢には報酬しない。
        """
        left_hip_vel = self.dof_vel[:, self.left_hip_pitch_idx]
        right_hip_vel = self.dof_vel[:, self.right_hip_pitch_idx]

        # 速度の積が負 = 逆方向に動いている
        opposite_motion = -left_hip_vel * right_hip_vel

        # 正規化（大きな値にならないように）
        reward = torch.clamp(opposite_motion, min=0.0, max=1.0)

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    def _reward_hip_pitch_opposite_sign(self):
        """左右hip_pitchが反対符号になる報酬 - V12

        左右のhip_pitchが反対符号（片方が前、片方が後ろ）になることを報酬
        """
        left_hp = self.dof_pos[:, self.left_hip_pitch_idx]
        right_hp = self.dof_pos[:, self.right_hip_pitch_idx]

        # 積が負 = 反対符号
        product = left_hp * right_hp
        opposite_sign_reward = torch.clamp(-product, min=0.0)  # 負の積を正の報酬に

        # 正規化（振幅0.5rad x 0.5rad = 0.25を最大として）
        reward = torch.clamp(opposite_sign_reward / 0.25, max=1.0)

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    def _reward_hip_pitch_range(self):
        """hip_pitchの可動範囲利用報酬 - V12

        hip_pitchが正（前方）に行くことを報酬。
        前方スイングを明示的に誘導。
        """
        left_hp = self.dof_pos[:, self.left_hip_pitch_idx]
        right_hp = self.dof_pos[:, self.right_hip_pitch_idx]

        # 正の値（前方スイング）を報酬
        left_forward = torch.clamp(left_hp, min=0.0)
        right_forward = torch.clamp(right_hp, min=0.0)

        # 両脚の前方スイング量の合計を報酬
        reward = (left_forward + right_forward) / 0.5  # 0.5radで正規化

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    def _reward_forward_progress(self):
        """前進進捗報酬"""
        forward_vel = self.base_lin_vel[:, 0]
        target_vel = self.commands[:, 0]
        progress = torch.clamp(forward_vel * torch.sign(target_vel), min=0.0)
        return progress

    def _reward_alive(self):
        """生存報酬"""
        return torch.ones(self.num_envs, dtype=gs.tc_float, device=gs.device)

    def _reward_pitch_penalty(self):
        """Pitch角ペナルティ

        前傾姿勢を抑制。
        """
        pitch_rad = self.base_euler[:, 1] * 3.14159 / 180.0  # degからrad
        return torch.square(pitch_rad)

    # ============================================================================
    # 報酬関数 - V13追加: 動的歩行報酬
    # ============================================================================

    def _reward_hip_pitch_velocity(self):
        """hip_pitchの速度報酬 - V13

        hip_pitchが動いていること（速度が0でない）を報酬。
        静的な固定姿勢ではなく、動的な歩行を促進。
        """
        left_hip_vel = torch.abs(self.dof_vel[:, self.left_hip_pitch_idx])
        right_hip_vel = torch.abs(self.dof_vel[:, self.right_hip_pitch_idx])

        # 速度の合計を報酬（動いていることを報酬）
        velocity_sum = left_hip_vel + right_hip_vel

        # 正規化（適度な速度を報酬、過度な速度は抑制）
        reward = torch.clamp(velocity_sum / 2.0, max=1.0)

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    def _reward_contact_alternation(self):
        """接地タイミングの交互性報酬 - V13

        左右の足が交互に接地・離地することを報酬。
        同時に状態が変わる（両脚同期）ことにペナルティ。
        """
        contacts = self._get_foot_contacts()
        if contacts is None or contacts.shape[1] != 2:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # 接地状態の変化を検出
        left_changed = contacts[:, 0] != self.last_contacts[:, 0]
        right_changed = contacts[:, 1] != self.last_contacts[:, 1]

        # 片方だけが変化 = 交互歩行
        one_changed = left_changed ^ right_changed

        # 両方同時に変化 = 両脚同期（ペナルティ対象）
        both_changed = left_changed & right_changed

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()

        # 片方だけ変化したら報酬、両方変化したらペナルティ
        return (one_changed.float() - both_changed.float() * 0.5) * has_command

    def _reward_roll_penalty(self):
        """Roll角ペナルティ - V13

        横傾斜を抑制。
        """
        roll_rad = self.base_euler[:, 0] * 3.14159 / 180.0  # degからrad
        return torch.square(roll_rad)

    # ============================================================================
    # 報酬関数 - V18追加: 前進方向制御
    # ============================================================================

    def _reward_backward_penalty(self):
        """後方移動ペナルティ - V18

        前進コマンドがあるときに後退するとペナルティ
        """
        forward_vel = self.base_lin_vel[:, 0]
        target_vel = self.commands[:, 0]

        # 前進コマンド（target > 0）のときに後退（vel < 0）するとペナルティ
        backward = torch.where(
            target_vel > 0.1,
            torch.clamp(-forward_vel, min=0.0),  # 後退速度をペナルティ
            torch.zeros_like(forward_vel)
        )
        return backward

    def _reward_yaw_penalty(self):
        """Yaw角ペナルティ（方向安定性）- V18

        Yaw角が0から離れるとペナルティ
        """
        yaw_rad = self.base_euler[:, 2] * 3.14159 / 180.0
        return torch.square(yaw_rad)

    def _reward_lateral_velocity_penalty(self):
        """横方向速度ペナルティ - V18

        Y方向の速度にペナルティを与えて直進性を向上
        """
        lateral_vel = torch.abs(self.base_lin_vel[:, 1])
        return lateral_vel

    # ============================================================================
    # 報酬関数 - V20追加: 後退ペナルティ
    # ============================================================================

    def _reward_backward_velocity(self):
        """後退速度ペナルティ - V20/V21

        ボディローカルX軸の負方向（後退）速度にペナルティ。
        """
        backward_vel = torch.clamp(-self.base_lin_vel[:, 0], min=0.0)
        return torch.square(backward_vel)

    def _reward_yaw_rate_penalty(self):
        """Yaw角速度ペナルティ - V20

        旋回を抑制。
        """
        return torch.square(self.base_ang_vel[:, 2])
