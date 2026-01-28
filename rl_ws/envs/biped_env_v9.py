"""
BSL-Droid二脚ロボット Genesis環境 v9

V7からの変更点:
- symmetric_gait報酬の追加（左右対称な動きを促進、Yaw回転抑制）
- V8の失敗を踏まえ、最小限の変更に留める

※V8の教訓:
- ペナルティの過度な強化は避ける
- 新規報酬関数は1つずつ追加して効果を検証

観測空間 (43次元): V7と同一
- base_ang_vel (3): ボディローカル角速度
- projected_gravity (3): ボディローカル重力ベクトル
- commands (3): 速度コマンド
- dof_pos (10): 関節位置（デフォルトからの偏差）
- dof_vel (10): 関節速度
- actions (10): 前回のアクション
- gait_phase (4): 歩行位相 [sin(phase), cos(phase), sin(phase*2), cos(phase*2)]

行動空間 (10次元):
- 各関節の目標位置オフセット（action_scaleでスケーリング）
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


class BipedEnvV9:
    """BSL-Droid二脚ロボットのGenesis強化学習環境 v9（V7 + symmetric_gait）"""

    def __init__(self, num_envs, env_cfg, obs_cfg, reward_cfg, command_cfg, show_viewer=False):
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

        # 歩容パラメータ
        self.gait_cfg = env_cfg.get("gait_cfg", {})
        self.step_height = self.gait_cfg.get("step_height", 0.04)
        self.step_length = self.gait_cfg.get("step_length", 0.08)
        self.step_frequency = self.gait_cfg.get("step_frequency", 0.8)  # Hz

        # 参照軌道ジェネレータ
        self.reference_trajectory = CamberTrajectory(
            step_height=self.step_height,
            step_length=self.step_length,
            device=gs.device,
        )

        # 脚のリンク長（URDFと一致させる）
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

        # ビルド
        self.scene.build(n_envs=num_envs)

        # 関節名からインデックスを取得
        self.motors_dof_idx = torch.tensor(
            [self.robot.get_joint(name).dof_start for name in self.env_cfg["joint_names"]],
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

        # 足のリンクインデックス（feet_air_time用）
        self.feet_names = env_cfg.get("feet_names", ["left_toe", "right_toe"])
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
        else:
            self.feet_air_time = None
            self.last_contacts = None

        # 歩行位相バッファ
        self.gait_phase = torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device)

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

    def _update_gait_phase(self):
        """歩行位相を更新"""
        # 位相を進める: phase += dt * frequency
        phase_increment = self.dt * self.step_frequency
        self.gait_phase = (self.gait_phase + phase_increment) % 1.0

    def _get_foot_position_from_joints(self, leg_joints, is_left=True):
        """
        関節角度からつま先のX-Z相対位置を計算（簡易2リンク順運動学）

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
        self.dof_pos = self.robot.get_dofs_position(self.motors_dof_idx)
        self.dof_vel = self.robot.get_dofs_velocity(self.motors_dof_idx)

        # 歩行位相を更新
        self._update_gait_phase()

        # feet_air_time更新
        if self.feet_air_time is not None:
            contacts = self._get_foot_contacts()
            if contacts is not None:
                first_contact = (self.feet_air_time > 0.0) & contacts & ~self.last_contacts
                self.feet_air_time += self.dt
                self.feet_air_time = torch.where(contacts, torch.zeros_like(self.feet_air_time), self.feet_air_time)
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
            self.episode_length_buf.zero_()
            self.reset_buf.fill_(True)
            self.gait_phase.zero_()  # 位相リセット
            if self.feet_air_time is not None:
                self.feet_air_time.zero_()
                self.last_contacts.fill_(True)
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
            self.episode_length_buf.masked_fill_(envs_idx, 0)
            self.reset_buf.masked_fill_(envs_idx, True)
            self.gait_phase.masked_fill_(envs_idx, 0.0)  # 位相リセット
            if self.feet_air_time is not None:
                self.feet_air_time.masked_fill_(envs_idx[:, None], 0.0)
                self.last_contacts.masked_fill_(envs_idx[:, None], True)

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
        """観測ベクトルを更新 (43次元 = 39 + 4)"""
        # 位相信号を計算
        phase_2pi = 2 * torch.pi * self.gait_phase
        phase_signal = torch.stack([
            torch.sin(phase_2pi),
            torch.cos(phase_2pi),
            torch.sin(phase_2pi * 2),  # 2倍の周波数
            torch.cos(phase_2pi * 2),
        ], dim=1)

        self.obs_buf = torch.concatenate(
            (
                self.base_ang_vel * self.obs_scales["ang_vel"],  # 3
                self.projected_gravity,  # 3
                self.commands * self.commands_scale,  # 3
                (self.dof_pos - self.default_dof_pos) * self.obs_scales["dof_pos"],  # 10
                self.dof_vel * self.obs_scales["dof_vel"],  # 10
                self.actions,  # 10
                phase_signal,  # 4
            ),
            dim=-1,
        )

    def reset(self):
        self._reset_idx()
        self._update_observation()
        return self.obs_buf, None

    # ------------ 報酬関数 ----------------
    def _reward_tracking_lin_vel(self):
        """線形速度追従報酬"""
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

    def _reward_action_rate(self):
        """アクション変化率ペナルティ"""
        return torch.sum(torch.square(self.last_actions - self.actions), dim=1)

    def _reward_similar_to_default(self):
        """デフォルト姿勢維持ペナルティ"""
        return torch.sum(torch.abs(self.dof_pos - self.default_dof_pos), dim=1)

    def _reward_base_height(self):
        """ベース高さ維持ペナルティ"""
        return torch.square(self.base_pos[:, 2] - self.reward_cfg["base_height_target"])

    def _reward_orientation(self):
        """姿勢維持報酬"""
        return torch.square(self.projected_gravity[:, 2] + 1.0)

    def _reward_torques(self):
        """トルクペナルティ"""
        return torch.sum(torch.square(self.actions), dim=1)

    def _reward_dof_vel(self):
        """関節速度ペナルティ"""
        return torch.sum(torch.square(self.dof_vel), dim=1)

    def _reward_dof_acc(self):
        """関節加速度ペナルティ"""
        return torch.sum(torch.square((self.dof_vel - self.last_dof_vel) / self.dt), dim=1)

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
        """交互歩行報酬"""
        contacts = self._get_foot_contacts()
        if contacts is None or contacts.shape[1] != 2:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        left_contact = contacts[:, 0]
        right_contact = contacts[:, 1]
        alternating = left_contact ^ right_contact
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
        """片足立ち報酬"""
        contacts = self._get_foot_contacts()
        if contacts is None or contacts.shape[1] != 2:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        single_stance = contacts.sum(dim=1) == 1
        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return single_stance.float() * has_command

    # ============ Phase-based Reference報酬 ============

    def _reward_trajectory_tracking(self):
        """
        Phase-based Reference軌道追従報酬

        左右の足先位置が、参照軌道（楕円弧）に沿っているかを評価
        """
        # 左右の関節角度を取得
        # dof_pos: [left_5, right_5] = [yaw, roll, hip_pitch, knee_pitch, ankle_pitch] x 2
        left_joints = self.dof_pos[:, 0:5]
        right_joints = self.dof_pos[:, 5:10]

        # 順運動学でつま先位置を計算
        left_foot_xz = self._get_foot_position_from_joints(left_joints, is_left=True)
        right_foot_xz = self._get_foot_position_from_joints(right_joints, is_left=False)

        # 位相から参照軌道上の目標位置を取得
        # 左足: phase、右足: phase + 0.5（180度位相差）
        left_phase = self.gait_phase
        right_phase = (self.gait_phase + 0.5) % 1.0

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
        位相一貫性報酬

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
        # 左足: phase < 0.5 で接地期待
        # 右足: phase >= 0.5 で接地期待
        left_expected = (self.gait_phase < 0.5).float()
        right_expected = (self.gait_phase >= 0.5).float()

        # 一致度を計算
        left_match = 1.0 - torch.abs(left_contact - left_expected)
        right_match = 1.0 - torch.abs(right_contact - right_expected)

        consistency = (left_match + right_match) / 2
        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()

        return consistency * has_command

    # ============ V9: 新規報酬関数（symmetric_gait のみ）============

    def _reward_symmetric_gait(self):
        """
        左右対称歩行報酬（V9新規）

        左右の脚の関節角度が180度位相差で対称になることを促進。
        Yaw回転を抑制する効果が期待される。
        """
        # 左脚と右脚の関節角度（Yaw/Roll以外のPitch系）
        # dof_pos: [L_yaw, L_roll, L_hip_pitch, L_knee_pitch, L_ankle_pitch,
        #           R_yaw, R_roll, R_hip_pitch, R_knee_pitch, R_ankle_pitch]
        left_hip_pitch = self.dof_pos[:, 2]
        left_knee_pitch = self.dof_pos[:, 3]
        left_ankle_pitch = self.dof_pos[:, 4]

        right_hip_pitch = self.dof_pos[:, 7]
        right_knee_pitch = self.dof_pos[:, 8]
        right_ankle_pitch = self.dof_pos[:, 9]

        # 180度位相差での対称性：左の現在 ≈ 右の0.5位相前
        # 簡易版：左右の速度が逆符号（一方が前進、一方が後退）
        left_hip_vel = self.dof_vel[:, 2]
        right_hip_vel = self.dof_vel[:, 7]

        # 左右の速度が逆符号なら対称（符号が異なるほど報酬）
        symmetry_hip = torch.clamp(-left_hip_vel * right_hip_vel, min=0.0)

        # 報酬はtanh でスケーリング（発散防止）
        reward = torch.tanh(symmetry_hip * 0.5)

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()

        return reward * has_command
