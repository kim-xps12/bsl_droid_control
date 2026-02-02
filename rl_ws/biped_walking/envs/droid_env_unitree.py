"""BSL-Droid Simplified 二脚ロボット Genesis環境（Unitree参考版）

============================================================
EXP007: Unitree RL Gym参考実装版（統合環境）
============================================================

このファイルは全バージョン（V1, V2, V3, ...）で共通して使用する統合環境クラスです。
各バージョンの違いは、訓練スクリプト側の reward_scales 設定で制御します。

【使用方法】
- 報酬関数を使用する場合: reward_scales に非ゼロ値を設定
- 報酬関数を無効化する場合: reward_scales に 0 を設定、または項目を省略

【利用可能な報酬関数】
速度追従報酬:
  - tracking_lin_vel: 線速度追従（exp関数）
  - tracking_ang_vel: 角速度追従（exp関数）

安定性ペナルティ:
  - lin_vel_z: Z方向速度ペナルティ
  - ang_vel_xy: XY角速度ペナルティ
  - orientation: 姿勢維持ペナルティ
  - base_height: ベース高さペナルティ

エネルギー効率ペナルティ:
  - torques: トルクペナルティ
  - dof_vel: 関節速度ペナルティ
  - dof_acc: 関節加速度ペナルティ
  - action_rate: アクション変化率ペナルティ

歩行品質報酬:
  - feet_air_time: 滞空時間報酬（Unitree方式）
  - contact: 接地フェーズ報酬（Unitree G1/H1方式）
  - feet_swing_height: 遊脚高さペナルティ（Unitree G1/H1方式）
  - contact_no_vel: 接地時足速度ペナルティ（Unitree H1方式）
  - hip_pos: 股関節位置ペナルティ（開脚抑制）
  - alive: 生存報酬

静止ポリシー対策（V3追加）:
  - single_foot_contact: 片足接地報酬
  - velocity_deficit: 速度未達ペナルティ

対称性・振動抑制（V5追加）:
  - symmetry: 左右脚対称性報酬

交互歩行・足裏制御（V7追加）:
  - alternating_gait: hip_pitch逆位相報酬
  - foot_flat: 足裏水平ペナルティ（ankle_pitch）
  - step_length: 歩幅報酬

交互歩行改善・胴体安定化（V8追加）:
  - hip_pitch_antiphase: hip_pitch速度逆相関報酬
  - ankle_roll: 足首ロール角ペナルティ

対称性・両脚動作強化（V9追加）:
  - hip_pitch_antiphase_v2: hip_pitch速度逆相関報酬（両脚動作必須版）
  - both_legs_active: 両脚動作報酬

足引きずり抑制（V10追加）:
  - feet_stumble: 接地中の足の水平速度ペナルティ

安全性ペナルティ:
  - termination: 終了ペナルティ
  - dof_pos_limits: 関節位置限界ペナルティ

【観測空間】(50次元)
- base_lin_vel (3): ボディローカル線速度
- base_ang_vel (3): ボディローカル角速度
- projected_gravity (3): ボディローカル重力ベクトル
- commands (3): 速度コマンド
- dof_pos - default (10): 関節位置偏差
- dof_vel (10): 関節速度
- actions (10): 前回のアクション
- gait_phase_sin/cos (2): 歩行位相
- leg_phase (2): 左右脚の位相
- feet_pos_z (2): 足先高さ
- contact_state (2): 接地状態

【行動空間】(10次元)
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


class DroidEnvUnitree:
    """BSL-Droid Simplified二脚ロボットのGenesis強化学習環境（Unitree参考版）"""

    def __init__(self, num_envs, env_cfg, obs_cfg, reward_cfg, command_cfg, show_viewer=False, recording_camera=False):
        self.num_envs = num_envs
        self.num_obs = obs_cfg["num_obs"]
        self.num_privileged_obs = None
        self.num_actions = env_cfg["num_actions"]
        self.num_commands = command_cfg["num_commands"]
        self.device = gs.device

        self.simulate_action_latency = True
        self.dt = 0.02  # 制御周波数 50Hz
        self.max_episode_length = math.ceil(env_cfg["episode_length_s"] / self.dt)

        self.env_cfg = env_cfg
        self.obs_cfg = obs_cfg
        self.reward_cfg = reward_cfg
        self.command_cfg = command_cfg

        self.obs_scales = obs_cfg["obs_scales"]
        self.reward_scales = reward_cfg["reward_scales"]

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
                pos=(1.5, -1.0, 0.6),
                lookat=(0.0, 0.0, 0.25),
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

        # PD制御パラメータ
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
        self.base_pos = torch.empty((self.num_envs, 3), dtype=gs.tc_float, device=gs.device)
        self.base_quat = torch.empty((self.num_envs, 4), dtype=gs.tc_float, device=gs.device)
        self.default_dof_pos = torch.tensor(
            [self.env_cfg["default_joint_angles"][name] for name in self.env_cfg["joint_names"]],
            dtype=gs.tc_float,
            device=gs.device,
        )

        # 関節インデックス
        self.left_hip_yaw_idx = 0
        self.left_hip_roll_idx = 1
        self.left_hip_pitch_idx = 2
        self.left_knee_idx = 3
        self.left_ankle_idx = 4
        self.right_hip_yaw_idx = 5
        self.right_hip_roll_idx = 6
        self.right_hip_pitch_idx = 7
        self.right_knee_idx = 8
        self.right_ankle_idx = 9

        # 歩行フェーズ（Unitree方式）
        self.gait_phase = torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device)
        self.gait_frequency = self.reward_cfg.get("gait_frequency", 1.5)  # Hz
        # leg_phase: [0, 1)の周期的位相、左右180°位相差
        self.leg_phase = torch.zeros((self.num_envs, 2), dtype=gs.tc_float, device=gs.device)

        # feet_air_time用バッファ
        if self.feet_indices is not None and len(self.feet_indices) > 0:
            self.feet_air_time = torch.zeros((self.num_envs, len(self.feet_indices)), dtype=gs.tc_float, device=gs.device)
            self.last_contacts = torch.ones((self.num_envs, len(self.feet_indices)), dtype=gs.tc_bool, device=gs.device)
        else:
            self.feet_air_time = None
            self.last_contacts = None

        # 足先位置（FK結果キャッシュ用）
        self.feet_pos = torch.zeros((self.num_envs, 2, 3), dtype=gs.tc_float, device=gs.device)
        self.feet_vel = torch.zeros((self.num_envs, 2, 3), dtype=gs.tc_float, device=gs.device)

        # 接触状態キャッシュ
        self.contact_state = torch.zeros((self.num_envs, 2), dtype=gs.tc_float, device=gs.device)

        # 高さ目標
        self.base_height_target = self.reward_cfg.get("base_height_target", 0.20)
        # Unitree方式: 遊脚の目標高さ
        self.swing_height_target = self.reward_cfg.get("swing_height_target", 0.03)
        # Unitree方式: 速度追従のシグマ
        self.tracking_sigma = self.reward_cfg.get("tracking_sigma", 0.25)
        # 接地判定閾値
        self.contact_threshold = self.reward_cfg.get("contact_threshold", 0.025)
        # feet_air_time報酬のオフセット（V5追加: デフォルト0.5秒はUnitree向け、小型ロボットは0.25秒推奨）
        self.air_time_offset = self.reward_cfg.get("air_time_offset", 0.5)

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
        """足の接地状態を取得（Z座標ベース）"""
        if self.feet_indices is None:
            return None
        link_pos = self.robot.get_links_pos()
        feet_z = link_pos[:, self.feet_indices, 2]
        return feet_z < self.contact_threshold

    def _update_feet_state(self):
        """足先の位置と速度を更新"""
        if self.feet_indices is None:
            return
        link_pos = self.robot.get_links_pos()
        link_vel = self.robot.get_links_vel()
        for i, idx in enumerate(self.feet_indices):
            self.feet_pos[:, i, :] = link_pos[:, idx, :]
            self.feet_vel[:, i, :] = link_vel[:, idx, :]

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

        # 歩行フェーズを更新（Unitree方式: [0, 1)）
        phase_increment = self.dt * self.gait_frequency
        self.gait_phase = (self.gait_phase + phase_increment) % 1.0
        # 左脚: gait_phase、右脚: (gait_phase + 0.5) % 1.0
        self.leg_phase[:, 0] = self.gait_phase
        self.leg_phase[:, 1] = (self.gait_phase + 0.5) % 1.0

        # 足先状態を更新
        self._update_feet_state()

        # 接触状態を更新
        contacts = self._get_foot_contacts()
        if contacts is not None:
            self.contact_state = contacts.float()

        # feet_air_time更新
        if self.feet_air_time is not None and contacts is not None:
            first_contact = (self.feet_air_time > 0.0) & contacts & ~self.last_contacts
            self.feet_air_time += self.dt
            self.feet_air_time = torch.where(contacts, torch.zeros_like(self.feet_air_time), self.feet_air_time)
            self.last_contacts = contacts
            self._first_contact = first_contact
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

        if "termination_if_height_lower_than" in self.env_cfg:
            self.reset_buf |= self.base_pos[:, 2] < self.env_cfg["termination_if_height_lower_than"]

        if self.env_cfg.get("termination_if_knee_positive", False):
            left_knee = self.dof_pos[:, self.left_knee_idx]
            right_knee = self.dof_pos[:, self.right_knee_idx]
            self.reset_buf |= (left_knee > 0.1) | (right_knee > 0.1)

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
            self.gait_phase.zero_()
            self.leg_phase.zero_()
            self.feet_pos.zero_()
            self.feet_vel.zero_()
            self.contact_state.zero_()
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
            self.gait_phase.masked_fill_(envs_idx, 0.0)
            self.leg_phase.masked_fill_(envs_idx[:, None], 0.0)
            self.feet_pos.masked_fill_(envs_idx[:, None, None], 0.0)
            self.feet_vel.masked_fill_(envs_idx[:, None, None], 0.0)
            self.contact_state.masked_fill_(envs_idx[:, None], 0.0)
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
        """観測ベクトルを更新 (50次元)"""
        gait_phase_sin = torch.sin(self.gait_phase * 2 * math.pi).unsqueeze(1)
        gait_phase_cos = torch.cos(self.gait_phase * 2 * math.pi).unsqueeze(1)

        self.obs_buf = torch.concatenate(
            (
                self.base_lin_vel * self.obs_scales["lin_vel"],  # 3
                self.base_ang_vel * self.obs_scales["ang_vel"],  # 3
                self.projected_gravity,  # 3
                self.commands * self.commands_scale,  # 3
                (self.dof_pos - self.default_dof_pos) * self.obs_scales["dof_pos"],  # 10
                self.dof_vel * self.obs_scales["dof_vel"],  # 10
                self.actions,  # 10
                gait_phase_sin,  # 1
                gait_phase_cos,  # 1
                self.leg_phase,  # 2
                self.feet_pos[:, :, 2],  # 2 (足先Z座標)
                self.contact_state,  # 2
            ),
            dim=-1,
        )

    def reset(self):
        self._reset_idx()
        self._update_observation()
        return self.obs_buf, None

    # ============================================================
    # 報酬関数（全バージョン共通）
    # ============================================================

    # ------------ 速度追従報酬（Unitree方式）----------------

    def _reward_tracking_lin_vel(self):
        """線形速度追従報酬（Unitree方式: exp関数）"""
        lin_vel_error = torch.sum(torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1)
        return torch.exp(-lin_vel_error / self.tracking_sigma)

    def _reward_tracking_ang_vel(self):
        """角速度追従報酬（Unitree方式: exp関数）"""
        ang_vel_error = torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / self.tracking_sigma)

    # ------------ 安定性ペナルティ（Unitree方式）----------------

    def _reward_lin_vel_z(self):
        """Z方向速度ペナルティ"""
        return torch.square(self.base_lin_vel[:, 2])

    def _reward_ang_vel_xy(self):
        """XY角速度ペナルティ"""
        return torch.sum(torch.square(self.base_ang_vel[:, :2]), dim=1)

    def _reward_orientation(self):
        """姿勢維持ペナルティ（Unitree方式: projected_gravity）"""
        return torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1)

    def _reward_base_height(self):
        """ベース高さペナルティ（Unitree方式）"""
        height = self.base_pos[:, 2]
        return torch.square(height - self.base_height_target)

    # ------------ エネルギー効率ペナルティ ----------------

    def _reward_torques(self):
        """トルクペナルティ"""
        return torch.sum(torch.square(self.actions), dim=1)

    def _reward_dof_vel(self):
        """関節速度ペナルティ"""
        return torch.sum(torch.square(self.dof_vel), dim=1)

    def _reward_dof_acc(self):
        """関節加速度ペナルティ"""
        return torch.sum(torch.square((self.dof_vel - self.last_dof_vel) / self.dt), dim=1)

    def _reward_action_rate(self):
        """アクション変化率ペナルティ"""
        return torch.sum(torch.square(self.last_actions - self.actions), dim=1)

    # ------------ 歩行品質報酬（Unitree方式）----------------

    def _reward_feet_air_time(self):
        """滞空時間報酬（Unitree方式）

        オフセット値はreward_cfg["air_time_offset"]で設定可能（デフォルト: 0.5秒）。
        小型ロボット（BSL-Droid）では0.25秒程度が適切。
        """
        if self.feet_air_time is None or self._first_contact is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # air_time_offsetは設定可能（V5追加）
        rew_air_time = torch.sum(
            (self.feet_air_time - self.air_time_offset) * self._first_contact.float(), dim=1
        )
        rew_air_time *= (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return rew_air_time

    def _reward_contact(self):
        """接地フェーズ報酬（Unitree G1/H1方式）

        脚の接地状態と歩行フェーズの整合性を報酬化。
        - leg_phase < 0.55: スタンス相（接地期待）
        - leg_phase >= 0.55: スイング相（非接地期待）
        """
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        res = torch.zeros(self.num_envs, dtype=gs.tc_float, device=self.device)
        contact_threshold = 0.55  # スタンス/スイング境界

        for i in range(2):  # 左右の脚
            is_stance = self.leg_phase[:, i] < contact_threshold
            contact = contacts[:, i]
            # XORの否定：期待と一致で報酬
            res += (~(contact ^ is_stance)).float()

        return res

    def _reward_feet_swing_height(self):
        """遊脚高さペナルティ（Unitree G1/H1方式）

        スイング中の足の高さを目標値に近づける。
        """
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # 足先Z座標と目標高さの誤差
        pos_error = torch.square(self.feet_pos[:, :, 2] - self.swing_height_target)
        # 接地していない足のみを対象
        return torch.sum(pos_error * (~contacts).float(), dim=1)

    def _reward_contact_no_vel(self):
        """接地時足速度ペナルティ（Unitree H1方式）

        接地中の足の速度をペナルティ化（足滑り防止）。
        """
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # 水平方向の速度のみ
        vel_xy = self.feet_vel[:, :, :2]
        vel_sq = torch.sum(torch.square(vel_xy), dim=2)  # [num_envs, 2]
        return torch.sum(vel_sq * contacts.float(), dim=1)

    def _reward_hip_pos(self):
        """股関節位置ペナルティ（Unitree G1/H1方式）

        hip_yaw, hip_rollの過度な変位をペナルティ化。
        """
        # BSL-Droid: hip_yaw (0, 5), hip_roll (1, 6)
        hip_angles = torch.cat([
            self.dof_pos[:, self.left_hip_yaw_idx:self.left_hip_yaw_idx+1],
            self.dof_pos[:, self.left_hip_roll_idx:self.left_hip_roll_idx+1],
            self.dof_pos[:, self.right_hip_yaw_idx:self.right_hip_yaw_idx+1],
            self.dof_pos[:, self.right_hip_roll_idx:self.right_hip_roll_idx+1],
        ], dim=1)
        return torch.sum(torch.square(hip_angles), dim=1)

    def _reward_alive(self):
        """生存報酬（Unitree方式）"""
        return torch.ones(self.num_envs, dtype=gs.tc_float, device=gs.device)

    # ------------ 静止ポリシー対策（V3追加）----------------

    def _reward_single_foot_contact(self):
        """片足接地報酬（V3追加）

        移動コマンド時に「片足のみ接地」状態を報酬化。
        これにより「両足で静止」という局所最適を回避する。

        【参考文献】
        Revisiting Reward Design and Evaluation for Robust Humanoid Standing
        and Walking (arXiv 2024): https://arxiv.org/html/2404.19173v1
        """
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # 移動コマンドかどうかを判定（X速度コマンド > 0.05 m/s）
        is_moving_command = self.commands[:, 0].abs() > 0.05

        # 接地状態を取得
        left_contact = contacts[:, 0]
        right_contact = contacts[:, 1]

        # 片足のみ接地 = XOR
        single_contact = left_contact ^ right_contact

        # 移動コマンド時のみ報酬、静止コマンド時は1.0
        reward = torch.where(
            is_moving_command,
            single_contact.float(),
            torch.ones(self.num_envs, dtype=gs.tc_float, device=gs.device)
        )
        return reward

    def _reward_velocity_deficit(self):
        """目標速度未達ペナルティ（V3追加）

        目標速度を下回っている場合にペナルティを付与。
        「動かない」局所最適を回避する。

        【設計意図】
        - 静止ポリシーでは、velocity_deficit = (target - 0)² > 0 となりペナルティ
        - 歩行ポリシーでは、target近傍でdeficit ≈ 0
        - これにより静止が明示的に不利になる
        """
        # 目標X速度 - 実際のX速度（正の値 = 目標未達）
        deficit = torch.clamp(self.commands[:, 0] - self.base_lin_vel[:, 0], min=0)
        return deficit ** 2

    # ------------ 安全性ペナルティ ----------------

    def _reward_termination(self):
        """終了ペナルティ"""
        return self.reset_buf.float() * (self.episode_length_buf < self.max_episode_length).float()

    def _reward_dof_pos_limits(self):
        """関節位置限界ペナルティ

        膝関節が0度（まっすぐ）に近づきすぎることを抑制。
        """
        knee_upper_limit = -0.2  # rad
        left_knee = self.dof_pos[:, self.left_knee_idx]
        right_knee = self.dof_pos[:, self.right_knee_idx]
        out_of_limits = torch.clamp(left_knee - knee_upper_limit, min=0) + \
                        torch.clamp(right_knee - knee_upper_limit, min=0)
        return out_of_limits

    # ------------ 対称性・振動抑制（V5追加）----------------

    def _reward_symmetry(self):
        """左右脚対称性報酬（V5追加）

        hip_roll, knee, ankleの左右差をペナルティ化することで、
        左右非対称な動きを抑制する。

        【設計意図】
        - hip_pitchは交互歩行で逆位相が理想なので除外
        - hip_roll, knee, ankleは同位相（対称）が理想

        【参考文献】
        Leveraging Symmetry in RL-based Legged Locomotion Control (IROS 2024)
        """
        # 左脚: roll(1), knee(3), ankle(4)
        # 右脚: roll(6), knee(8), ankle(9)
        left_roll = self.dof_pos[:, self.left_hip_roll_idx]
        right_roll = self.dof_pos[:, self.right_hip_roll_idx]
        left_knee = self.dof_pos[:, self.left_knee_idx]
        right_knee = self.dof_pos[:, self.right_knee_idx]
        left_ankle = self.dof_pos[:, self.left_ankle_idx]
        right_ankle = self.dof_pos[:, self.right_ankle_idx]

        # 左右差の二乗和
        symmetry_error = (
            torch.square(left_roll - (-right_roll)) +  # rollは符号反転で対称
            torch.square(left_knee - right_knee) +
            torch.square(left_ankle - right_ankle)
        )

        # ガウシアン型報酬（誤差が小さいほど報酬が高い）
        return torch.exp(-symmetry_error / 0.5)

    # ------------ 交互歩行・足裏制御（V7追加）----------------

    def _reward_alternating_gait(self):
        """hip_pitch逆位相報酬（V7追加）

        交互歩行を強化するため、左右のhip_pitchが逆位相であることを報酬化。
        理想的な歩行では、左脚が前に出るとき右脚は後ろにあり、その逆も同様。

        【設計原理】
        - hip_pitch_left + hip_pitch_right ≈ 0（逆位相時）
        - 両脚が同位相（同期）の場合、和が大きくなりペナルティ

        【参考文献】
        MS-PPO: Morphological-Symmetry-Equivariant Policy (arXiv 2024)
        """
        left_hip_pitch = self.dof_pos[:, self.left_hip_pitch_idx] - self.default_dof_pos[self.left_hip_pitch_idx]
        right_hip_pitch = self.dof_pos[:, self.right_hip_pitch_idx] - self.default_dof_pos[self.right_hip_pitch_idx]

        # 逆位相時は和が0に近い
        phase_sum = torch.abs(left_hip_pitch + right_hip_pitch)

        # ガウシアン型報酬（和が0に近いほど報酬が高い）
        return torch.exp(-phase_sum / 0.3)

    def _reward_foot_flat(self):
        """足裏水平報酬（V7追加）

        足首（ankle_pitch）が極端に傾かないように制御。
        爪先立ちや踵立ちを防止し、足裏全体での接地を促進。

        【設計原理】
        - ankle_pitchがデフォルト角度から大きく逸脱するとペナルティ
        - 接地中の足のみを対象とすることで、スイング相での自由度を確保

        【参考文献】
        Barrier-Based Style Rewards (arXiv 2024)
        """
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # ankle_pitchのデフォルトからの偏差
        left_ankle_dev = torch.abs(
            self.dof_pos[:, self.left_ankle_idx] - self.default_dof_pos[self.left_ankle_idx]
        )
        right_ankle_dev = torch.abs(
            self.dof_pos[:, self.right_ankle_idx] - self.default_dof_pos[self.right_ankle_idx]
        )

        # 接地中の足のみを対象（スイング相は除外）
        left_penalty = left_ankle_dev * contacts[:, 0].float()
        right_penalty = right_ankle_dev * contacts[:, 1].float()

        # 二乗誤差
        return torch.square(left_penalty) + torch.square(right_penalty)

    def _reward_step_length(self):
        """歩幅報酬（V7追加）

        前後の足の距離が大きいことを報酬化し、大股歩行を促進。

        【設計原理】
        - 遊脚時に足が前方に大きく振り出されることを報酬
        - 両足が近い位置にある（小刻み歩行）状態を抑制
        """
        # 前後方向（X軸）の足間距離
        left_foot_x = self.feet_pos[:, 0, 0]
        right_foot_x = self.feet_pos[:, 1, 0]
        foot_distance_x = torch.abs(left_foot_x - right_foot_x)

        # 移動コマンド時のみ報酬
        is_moving = self.commands[:, 0].abs() > 0.05
        reward = foot_distance_x * is_moving.float()

        return reward

    # ------------ 交互歩行改善・胴体安定化（V8追加）----------------

    def _reward_hip_pitch_antiphase(self):
        """hip_pitch速度の逆相関報酬（V8追加）

        V7のalternating_gait報酬は「位置の和が0」を報酬化したが、
        これは両脚が同じ位置（両方デフォルト）でも達成可能だった。
        V8では速度の逆相関を報酬化し、動的な交互歩行を誘導する。

        【設計原理】
        - 理想的な交互歩行では、左脚が前に振れるとき右脚は後ろに振れる
        - hip_pitch速度の符号が逆（積が負）であれば報酬
        - 両脚が同期している場合、積が正となりペナルティ相当

        【参考文献】
        exp007_unitree_rl_gym_survey.md セクション7.2.1 Periodic Reward Composition
        """
        left_vel = self.dof_vel[:, self.left_hip_pitch_idx]
        right_vel = self.dof_vel[:, self.right_hip_pitch_idx]

        # 速度の積が負（逆符号）なら報酬、正（同符号）ならペナルティ相当
        # -sign(left_vel * right_vel) は逆相関時に+1、同相関時に-1
        velocity_product = left_vel * right_vel

        # 移動コマンド時のみ適用
        is_moving = self.commands[:, 0].abs() > 0.05

        # 速度の積が負のとき報酬（逆相関）
        # tanh関数でスケーリングして滑らかな報酬に
        reward = -torch.tanh(velocity_product / 0.1)

        return reward * is_moving.float()

    def _reward_ankle_roll(self):
        """足首ロール角ペナルティ（V8追加）

        V7ではfoot_flat報酬がankle_pitch（ピッチ方向）のみを対象としていたが、
        実際にはankle_roll（ロール方向）の傾斜も問題だった。
        V8ではankle_rollの偏差も明示的にペナルティ化する。

        【設計原理】
        - ankle_rollがデフォルト（0度）から大きく逸脱するとペナルティ
        - 足が内側/外側に傾くことを防止し、安定した接地を促進

        【参考文献】
        exp007_report_v7.md 根本原因分析セクション2
        """
        # BSL-Droid Simplifiedにはankle_rollがないため、
        # hip_rollを代替として使用（足の傾きに寄与）
        # 注: ankle_roll関節が追加された場合は該当インデックスに変更
        left_roll = self.dof_pos[:, self.left_hip_roll_idx]
        right_roll = self.dof_pos[:, self.right_hip_roll_idx]

        # ロール角の二乗和
        return torch.square(left_roll) + torch.square(right_roll)

    # ------------ 対称性・両脚動作強化（V9追加）----------------

    def _reward_hip_pitch_antiphase_v2(self):
        """hip_pitch速度の逆相関報酬・修正版（V9追加）

        V8のhip_pitch_antiphaseは「速度の積が負」を報酬化したが、
        片方が静止（速度=0）の場合も報酬0（ペナルティなし）となり、
        「右脚だけ動かす」局所最適に収束した。

        V9では**両脚が共に動いている場合のみ**逆相関報酬を与える。

        【設計原理】
        - 速度の積が負（逆相関）かつ両脚が動いている場合に報酬
        - 片脚静止の場合は報酬0（積極的な動作を促す）
        - 両脚が同期している場合は報酬減少

        【参考文献】
        exp007_report_v8.md 根本原因分析セクション1
        """
        left_vel = self.dof_vel[:, self.left_hip_pitch_idx]
        right_vel = self.dof_vel[:, self.right_hip_pitch_idx]

        # 両脚の速度の大きさ
        left_mag = torch.abs(left_vel)
        right_mag = torch.abs(right_vel)

        # 速度の積
        velocity_product = left_vel * right_vel

        # 両脚が共に動いている閾値（0.1 rad/s以上）
        min_vel_threshold = 0.1
        both_active = (left_mag > min_vel_threshold) & (right_mag > min_vel_threshold)

        # 逆相関時（積が負）に報酬、同相関時（積が正）にペナルティ
        # tanh関数でスケーリング
        antiphase_reward = -torch.tanh(velocity_product / 0.1)

        # 両脚が動いている場合のみ報酬を与える
        reward = antiphase_reward * both_active.float()

        # 移動コマンド時のみ適用
        is_moving = self.commands[:, 0].abs() > 0.05
        return reward * is_moving.float()

    def _reward_both_legs_active(self):
        """両脚動作報酬（V9追加）

        V8では片脚（右脚）だけが動く局所最適に収束した。
        この報酬は両脚が共に動いていることを直接報酬化する。

        【設計原理】
        - 両脚のhip_pitch速度の最小値を報酬化
        - 片方が静止（速度=0）の場合、報酬=0
        - 両脚が動いている場合のみ報酬が発生
        - min(left, right)を使用することで、片脚静止戦略を明示的に抑制

        【参考文献】
        exp007_report_v8.md 次バージョンへの提案・優先度1
        """
        left_vel_mag = torch.abs(self.dof_vel[:, self.left_hip_pitch_idx])
        right_vel_mag = torch.abs(self.dof_vel[:, self.right_hip_pitch_idx])

        # 両脚の速度の最小値を報酬（片方が0なら報酬0）
        min_vel = torch.min(left_vel_mag, right_vel_mag)

        # 速度を適度にスケーリング（大きすぎる値を抑制）
        reward = torch.tanh(min_vel / 0.5)

        # 移動コマンド時のみ適用
        is_moving = self.commands[:, 0].abs() > 0.05
        return reward * is_moving.float()

    # ------------ 足引きずり抑制（V10追加）----------------

    def _reward_feet_stumble(self):
        """足引きずりペナルティ（V10追加）

        接地中の足が水平方向に大きな速度を持つ場合にペナルティを与える。
        足を引きずるような動作を抑制し、クリアな足上げ動作を促進する。

        【設計原理】
        - 接地中の足は地面に対して静止しているべき
        - 水平方向の速度が大きい場合、足を引きずっている可能性がある
        - Unitree RL Gymのfeet_stumbleペナルティを参考に実装

        【参考文献】
        exp007_unitree_rl_gym_survey.md セクション2.5.4
        exp007_report_v9.md 次バージョンへの提案・優先度3
        """
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # 水平方向の足速度（XY平面）
        vel_xy = torch.sqrt(self.feet_vel[:, :, 0] ** 2 + self.feet_vel[:, :, 1] ** 2)

        # 接地中の足のみを対象
        return torch.sum(vel_xy * contacts.float(), dim=1)
