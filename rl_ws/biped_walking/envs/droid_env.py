"""BSL-Droid Simplified 二脚ロボット Genesis環境（統一版）

============================================================
全バージョン統合版 droid_env.py
============================================================

このファイルは droid_env_v{1,2,3,4,5,6}.py を統合した単一ファイルです。
各 droid_train_v{N}.py から DroidEnv をimportし、reward_scales で
使用する報酬関数を選択することで、バージョン間の互換性を維持します。

バージョン履歴:
- V1: biped_env_v22ベース、BSL-Droid Simplified用に調整
- V2: V1と同一（設定違いのみ）
- V3: _reward_knee_positive 追加（膝正角度維持）
- V4: _reward_knee_min_angle 追加（膝最小角度制約）
- V5: 初期姿勢変更（膝を深く曲げる）、V4と同一報酬関数
- V6: _reward_dof_pos_limits 追加（Legged Gym方式ソフトリミット）

設計原則:
- 報酬関数の追加: env側に _reward_xxx を追加、train側で reward_scales に追加
- 報酬関数の無効化: train側の reward_scales から削除
- 報酬関数の削除: **禁止**（互換性維持のため）

ロボット仕様:
- 脚長: 約0.31m (thigh: 0.11m + shank: 0.12m + foot: 0.08m)
- 膝関節: -135度〜0度 (負で後方屈曲、Unitree/ANYmal規約)
- 総質量: 約5.8kg

観測空間 (39次元):
- base_ang_vel (3): ボディローカル角速度
- projected_gravity (3): ボディローカル重力ベクトル
- commands (3): 速度コマンド
- dof_pos (10): 関節位置 (デフォルトからの偏差)
- dof_vel (10): 関節速度
- actions (10): 前回のアクション

行動空間 (10次元):
- 各関節の目標位置オフセット（action_scaleでスケーリング）
"""

from __future__ import annotations

import math
from typing import Any

import genesis as gs
import torch
from genesis.utils.geom import inv_quat, quat_to_xyz, transform_by_quat, transform_quat_by_quat


def gs_rand(lower: torch.Tensor, upper: torch.Tensor, batch_shape: tuple[int, ...]) -> torch.Tensor:
    """一様乱数を生成"""
    assert lower.shape == upper.shape
    return (upper - lower) * torch.rand(size=(*batch_shape, *lower.shape), dtype=gs.tc_float, device=gs.device) + lower


class DroidEnv:
    """BSL-Droid Simplified二脚ロボットのGenesis強化学習環境（統一版）"""

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
        self.num_obs: int = obs_cfg["num_obs"]
        self.num_privileged_obs: int | None = None
        self.num_actions: int = env_cfg["num_actions"]
        self.num_commands: int = command_cfg["num_commands"]
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
                pos=(1.5, -1.0, 0.6),  # 斜め横から撮影（低いロボット用に調整）
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
        _feet_idx_list: list[int] = []
        for name in self.feet_names:
            try:
                link = self.robot.get_link(name)
                _feet_idx_list.append(link.idx_local)
            except Exception:
                pass
        self.feet_indices: torch.Tensor | None = (
            torch.tensor(_feet_idx_list, dtype=gs.tc_int, device=gs.device) if _feet_idx_list else None
        )

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
                strict=False,
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
        self.feet_air_time: torch.Tensor | None
        self.last_contacts: torch.Tensor | None
        if self.feet_indices is not None and len(self.feet_indices) > 0:
            self.feet_air_time = torch.zeros(
                (self.num_envs, len(self.feet_indices)), dtype=gs.tc_float, device=gs.device
            )
            self.last_contacts = torch.ones((self.num_envs, len(self.feet_indices)), dtype=gs.tc_bool, device=gs.device)
        else:
            self.feet_air_time = None
            self.last_contacts = None

        # hip_pitch/hip_roll位置のトラッキング（交互歩行分析用）
        # joint_names: [L_hip_yaw, L_hip_roll, L_hip_pitch, L_knee, L_ankle,
        #               R_hip_yaw, R_hip_roll, R_hip_pitch, R_knee, R_ankle]
        self.left_hip_yaw_idx = 0  # left_hip_yaw_joint
        self.left_hip_roll_idx = 1  # left_hip_roll_joint
        self.left_hip_pitch_idx = 2  # left_hip_pitch_joint
        self.right_hip_yaw_idx = 5  # right_hip_yaw_joint
        self.right_hip_roll_idx = 6  # right_hip_roll_joint
        self.right_hip_pitch_idx = 7  # right_hip_pitch_joint
        self.last_left_hip_pitch = torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device)
        self.last_right_hip_pitch = torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device)

        # 歩行サイクル用の位相追跡
        self.gait_phase = torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device)
        self.gait_frequency = self.reward_cfg.get("gait_frequency", 1.5)  # Hz

        # 接地タイミング追跡
        self.last_contact_change_time = torch.zeros((self.num_envs, 2), dtype=gs.tc_float, device=gs.device)

        # hip_pitchの動的振幅追跡（移動平均）
        self.hip_pitch_max_recent = torch.zeros((self.num_envs, 2), dtype=gs.tc_float, device=gs.device)
        self.hip_pitch_min_recent = torch.zeros((self.num_envs, 2), dtype=gs.tc_float, device=gs.device)

        # 高さ目標（脚が短いため低く設定）
        self.base_height_target = self.reward_cfg.get("base_height_target", 0.25)

        self.extras: dict[str, Any] = {}
        self.extras["observations"] = {}

        # 報酬関数の準備
        self.reward_functions, self.episode_sums = {}, {}
        for name in self.reward_scales:
            self.reward_scales[name] *= self.dt
            self.reward_functions[name] = getattr(self, "_reward_" + name)
            self.episode_sums[name] = torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device)

    def _resample_commands(self, envs_idx: torch.Tensor | None) -> None:
        """速度コマンドを再サンプリング"""
        commands = gs_rand(*self.commands_limits, (self.num_envs,))  # type: ignore[arg-type, call-arg]
        if envs_idx is None:
            self.commands.copy_(commands)
        else:
            torch.where(envs_idx[:, None], commands, self.commands, out=self.commands)

    def _get_foot_contacts(self) -> torch.Tensor | None:
        """足の接地状態を取得（簡易版：Z座標ベース）"""
        if self.feet_indices is None:
            return None
        # V11: contact_thresholdを設定から読み取り（デフォルト0.025m、V11で0.04mに変更）
        contact_threshold = self.reward_cfg.get("contact_threshold", 0.025)
        link_pos = self.robot.get_links_pos()
        feet_z = link_pos[:, self.feet_indices, 2]
        return feet_z < contact_threshold

    def step(self, actions: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, dict[str, Any]]:
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

        # 歩行位相を更新
        self.gait_phase = (self.gait_phase + self.dt * self.gait_frequency * 2 * math.pi) % (2 * math.pi)

        # hip_pitchの動的振幅を更新（指数移動平均）
        alpha = 0.1
        left_hp = self.dof_pos[:, self.left_hip_pitch_idx]
        right_hp = self.dof_pos[:, self.right_hip_pitch_idx]
        self.hip_pitch_max_recent[:, 0] = torch.maximum(
            self.hip_pitch_max_recent[:, 0] * (1 - alpha) + left_hp * alpha, left_hp
        )
        self.hip_pitch_max_recent[:, 1] = torch.maximum(
            self.hip_pitch_max_recent[:, 1] * (1 - alpha) + right_hp * alpha, right_hp
        )
        self.hip_pitch_min_recent[:, 0] = torch.minimum(
            self.hip_pitch_min_recent[:, 0] * (1 - alpha) + left_hp * alpha, left_hp
        )
        self.hip_pitch_min_recent[:, 1] = torch.minimum(
            self.hip_pitch_min_recent[:, 1] * (1 - alpha) + right_hp * alpha, right_hp
        )

        # feet_air_time更新
        if self.feet_air_time is not None:
            assert self.last_contacts is not None
            contacts = self._get_foot_contacts()
            if contacts is not None:
                first_contact = (self.feet_air_time > 0.0) & contacts & ~self.last_contacts
                self.feet_air_time += self.dt
                self.feet_air_time = torch.where(contacts, torch.zeros_like(self.feet_air_time), self.feet_air_time)

                # 接地状態変化のタイミングを記録
                contact_changed = contacts != self.last_contacts
                current_time = self.episode_length_buf.float() * self.dt
                for i in range(2):
                    self.last_contact_change_time[:, i] = torch.where(
                        contact_changed[:, i], current_time, self.last_contact_change_time[:, i]
                    )

                self.last_contacts = contacts
                self._first_contact = first_contact
            else:
                self._first_contact = None  # type: ignore[assignment]
        else:
            self._first_contact = None  # type: ignore[assignment]

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

        # V18: 追加の終了条件
        # 高さ下限（転倒検出）
        if "termination_if_height_lower_than" in self.env_cfg:
            self.reset_buf |= self.base_pos[:, 2] < self.env_cfg["termination_if_height_lower_than"]

        # 膝が正の角度（物理的に不正な状態）
        if self.env_cfg.get("termination_if_knee_positive", False):
            # 膝関節のインデックス: left_knee_pitch=3, right_knee_pitch=8
            left_knee = self.dof_pos[:, 3]
            right_knee = self.dof_pos[:, 8]
            # 膝が正（前方に曲がる）は終了
            self.reset_buf |= (left_knee > 0.1) | (right_knee > 0.1)

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
            self.actions.zero_()
            self.last_actions.zero_()
            self.last_dof_vel.zero_()
            self.episode_length_buf.zero_()
            self.reset_buf.fill_(True)
            self.last_left_hip_pitch.zero_()
            self.last_right_hip_pitch.zero_()
            self.gait_phase.zero_()
            self.last_contact_change_time.zero_()
            self.hip_pitch_max_recent.zero_()
            self.hip_pitch_min_recent.zero_()
            if self.feet_air_time is not None:
                assert self.last_contacts is not None
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
            self.last_left_hip_pitch.masked_fill_(envs_idx, 0.0)
            self.last_right_hip_pitch.masked_fill_(envs_idx, 0.0)
            self.gait_phase.masked_fill_(envs_idx, 0.0)
            self.last_contact_change_time.masked_fill_(envs_idx[:, None], 0.0)
            self.hip_pitch_max_recent.masked_fill_(envs_idx[:, None], 0.0)
            self.hip_pitch_min_recent.masked_fill_(envs_idx[:, None], 0.0)
            if self.feet_air_time is not None:
                assert self.last_contacts is not None
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
                mean = torch.where(n_envs > 0, value[envs_idx].sum() / n_envs, 0.0)  # type: ignore[arg-type]
                self.extras["episode"]["rew_" + key] = mean / self.env_cfg["episode_length_s"]
                value.masked_fill_(envs_idx, 0.0)
            self.extras["episode"]["rew_" + key] = mean / self.env_cfg["episode_length_s"]

        self._resample_commands(envs_idx)

    def _update_observation(self) -> None:
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

    def reset(self) -> tuple[torch.Tensor, None]:
        self._reset_idx()
        self._update_observation()
        return self.obs_buf, None

    # ============================================================
    # 報酬関数（V1〜V6全て含む）
    # ============================================================

    # ------------ 基本報酬関数（V1から存在）----------------

    def _reward_tracking_lin_vel(self) -> torch.Tensor:
        """線形速度追従報酬（主タスク）"""
        lin_vel_error = torch.sum(torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1)
        return torch.exp(-lin_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_tracking_ang_vel(self) -> torch.Tensor:
        """角速度追従報酬"""
        ang_vel_error = torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_lin_vel_z(self) -> torch.Tensor:
        """Z方向速度ペナルティ"""
        return torch.square(self.base_lin_vel[:, 2])

    def _reward_ang_vel_xy(self) -> torch.Tensor:
        """XY角速度ペナルティ"""
        return torch.sum(torch.square(self.base_ang_vel[:, :2]), dim=1)

    def _reward_orientation(self) -> torch.Tensor:
        """姿勢維持ペナルティ"""
        return torch.square(self.projected_gravity[:, 2] + 1.0)

    def _reward_base_height(self) -> torch.Tensor:
        """ベース高さペナルティ"""
        height = self.base_pos[:, 2]
        error = height - self.base_height_target
        return torch.square(error)

    def _reward_base_height_high(self) -> torch.Tensor:
        """高すぎる場合の追加ペナルティ"""
        height = self.base_pos[:, 2]
        error = height - self.base_height_target
        high_error = torch.clamp(error, min=0.0)
        return torch.square(high_error)

    def _reward_backward_velocity(self) -> torch.Tensor:
        """後退速度ペナルティ"""
        backward_vel = torch.clamp(-self.base_lin_vel[:, 0], min=0.0)
        return torch.square(backward_vel)

    def _reward_torques(self) -> torch.Tensor:
        """トルクペナルティ（エネルギー効率）"""
        return torch.sum(torch.square(self.actions), dim=1)

    def _reward_dof_vel(self) -> torch.Tensor:
        """関節速度ペナルティ（振動抑制）"""
        return torch.sum(torch.square(self.dof_vel), dim=1)

    def _reward_dof_acc(self) -> torch.Tensor:
        """関節加速度ペナルティ（振動抑制・強化）"""
        return torch.sum(torch.square((self.dof_vel - self.last_dof_vel) / self.dt), dim=1)

    def _reward_action_rate(self) -> torch.Tensor:
        """アクション変化率ペナルティ（振動抑制・強化）"""
        return torch.sum(torch.square(self.last_actions - self.actions), dim=1)

    def _reward_similar_to_default(self) -> torch.Tensor:
        """デフォルト姿勢維持ペナルティ"""
        return torch.sum(torch.abs(self.dof_pos - self.default_dof_pos), dim=1)

    # ------------ 二脚歩行報酬（V1から存在）----------------

    def _reward_feet_air_time(self) -> torch.Tensor:
        """足の滞空時間報酬"""
        if self.feet_air_time is None or self._first_contact is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        target_air_time = self.reward_cfg.get("feet_air_time_target", 0.25)
        rew_air_time = torch.sum((self.feet_air_time - target_air_time) * self._first_contact.float(), dim=1)
        rew_air_time *= (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return rew_air_time

    def _reward_no_fly(self) -> torch.Tensor:
        """両足同時離地ペナルティ"""
        if self.feet_air_time is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        both_feet_in_air = ~contacts.any(dim=1)
        return both_feet_in_air.float()

    def _reward_alternating_gait(self) -> torch.Tensor:
        """交互歩行報酬（接地状態ベース）"""
        contacts = self._get_foot_contacts()
        if contacts is None or contacts.shape[1] != 2:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        left_contact = contacts[:, 0]
        right_contact = contacts[:, 1]
        alternating = left_contact ^ right_contact  # XOR: 片方だけ接地
        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return alternating.float() * has_command

    def _reward_foot_swing(self) -> torch.Tensor:
        """足のスイング報酬"""
        if self.feet_air_time is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        air_time_reward = torch.clamp(self.feet_air_time, 0.0, 0.3)
        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return torch.sum(air_time_reward, dim=1) * has_command

    def _reward_single_stance(self) -> torch.Tensor:
        """片足立ち報酬"""
        contacts = self._get_foot_contacts()
        if contacts is None or contacts.shape[1] != 2:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        single_stance = contacts.sum(dim=1) == 1
        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return single_stance.float() * has_command

    # ------------ 動的歩行報酬（V1から存在）----------------

    def _reward_hip_pitch_alternation(self) -> torch.Tensor:
        """左右hip_pitchの逆相運動報酬"""
        left_hip_vel = self.dof_vel[:, self.left_hip_pitch_idx]
        right_hip_vel = self.dof_vel[:, self.right_hip_pitch_idx]

        opposite_motion = -left_hip_vel * right_hip_vel
        reward = torch.clamp(opposite_motion, min=0.0, max=1.0)

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    def _reward_hip_pitch_velocity(self) -> torch.Tensor:
        """hip_pitchの速度報酬"""
        left_hip_vel = torch.abs(self.dof_vel[:, self.left_hip_pitch_idx])
        right_hip_vel = torch.abs(self.dof_vel[:, self.right_hip_pitch_idx])

        velocity_sum = left_hip_vel + right_hip_vel
        reward = torch.clamp(velocity_sum / 2.0, max=1.0)

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    def _reward_contact_alternation(self) -> torch.Tensor:
        """接地タイミングの交互性報酬"""
        contacts = self._get_foot_contacts()
        if contacts is None or contacts.shape[1] != 2 or self.last_contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        left_changed = contacts[:, 0] != self.last_contacts[:, 0]
        right_changed = contacts[:, 1] != self.last_contacts[:, 1]

        one_changed = left_changed ^ right_changed
        both_changed = left_changed & right_changed

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()

        return (one_changed.float() - both_changed.float() * 0.5) * has_command

    def _reward_forward_progress(self) -> torch.Tensor:
        """前進進捗報酬"""
        forward_vel = self.base_lin_vel[:, 0]
        target_vel = self.commands[:, 0]
        progress = torch.clamp(forward_vel * torch.sign(target_vel), min=0.0)
        return progress

    def _reward_alive(self) -> torch.Tensor:
        """生存報酬"""
        return torch.ones(self.num_envs, dtype=gs.tc_float, device=gs.device)

    def _reward_pitch_penalty(self) -> torch.Tensor:
        """Pitch角ペナルティ"""
        pitch_rad = self.base_euler[:, 1] * 3.14159 / 180.0
        return torch.square(pitch_rad)

    def _reward_roll_penalty(self) -> torch.Tensor:
        """Roll角ペナルティ"""
        roll_rad = self.base_euler[:, 0] * 3.14159 / 180.0
        return torch.square(roll_rad)

    # ------------ V3追加: 膝負角度維持報酬 ----------------

    def _reward_knee_negative(self) -> torch.Tensor:
        """膝関節が負の角度を維持することを促す報酬

        BSL-Droid Simplifiedの膝関節は負角度で後方屈曲（逆関節）。
        軸はhip_pitch/ankle_pitchと統一（0 1 0）し、リミットで逆関節を表現。
        - リミット: -135°〜0°
        - 0° = 脚まっすぐ
        - 負角度 = 膝が後方に曲がる（逆関節らしい姿勢）

        膝関節インデックス:
        - left_knee_pitch_joint: 3
        - right_knee_pitch_joint: 8
        """
        left_knee = self.dof_pos[:, 3]  # left_knee_pitch_joint
        right_knee = self.dof_pos[:, 8]  # right_knee_pitch_joint

        # 正の角度にペナルティ（負で0、正で|角度|を返す）
        penalty = torch.clamp(left_knee, min=0) + torch.clamp(right_knee, min=0)
        return penalty

    # 後方互換性のためのエイリアス（V3-V6で使用）
    def _reward_knee_positive(self) -> torch.Tensor:
        """後方互換性エイリアス: knee_negativeを呼び出す"""
        return self._reward_knee_negative()

    # ------------ V4追加: 膝最大角度制約 ----------------

    def _reward_knee_min_angle(self) -> torch.Tensor:
        """膝が最大角度 (-0.3 rad) を上回らないようにする

        デフォルト姿勢の膝角度 (-0.6 rad) に対して、
        0に近づきすぎないよう制約。
        -0.3 rad ≈ -17度（軽い屈曲）
        """
        max_knee_angle = -0.3  # 最大値（0に近い側）
        left_knee = self.dof_pos[:, 3]  # left_knee_pitch_joint
        right_knee = self.dof_pos[:, 8]  # right_knee_pitch_joint

        # max_knee_angleを上回ったらペナルティ（0に近づきすぎ）
        penalty = torch.clamp(left_knee - max_knee_angle, min=0) + torch.clamp(right_knee - max_knee_angle, min=0)
        return penalty

    # ------------ V6追加: 関節可動域ソフトリミット ----------------

    def _reward_dof_pos_limits(self) -> torch.Tensor:
        """関節可動域のソフトリミット（Legged Gym方式）

        膝関節が上限（-0.2 rad）を上回った場合に線形ペナルティ。
        これにより膝が0°（まっすぐ）に近づきすぎることを抑制する。

        参考: ETH Zurich Legged Gym
        https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot.py

        膝関節インデックス:
        - left_knee_pitch_joint: 3
        - right_knee_pitch_joint: 8
        """
        # 膝関節の上限を設定（0に近い側）
        knee_upper_limit = -0.2  # rad (約-11度)

        left_knee = self.dof_pos[:, 3]  # left_knee_pitch_joint
        right_knee = self.dof_pos[:, 8]  # right_knee_pitch_joint

        # 上限を上回った分だけペナルティ（0に近づきすぎ）
        out_of_limits = torch.clamp(left_knee - knee_upper_limit, min=0) + torch.clamp(
            right_knee - knee_upper_limit, min=0
        )

        return out_of_limits

    # ------------ V8追加: 膝の過度な屈曲制限 ----------------

    def _reward_knee_max_angle(self) -> torch.Tensor:
        """膝が過度に曲がりすぎることを抑制する報酬

        膝がURDFリミット（-150°）に近づきすぎないようソフトリミットを設定。
        -2 rad（約-115°）程度までは想定動作範囲内。

        膝関節インデックス:
        - left_knee_pitch_joint: 3
        - right_knee_pitch_joint: 8
        """
        # 膝関節の下限を設定（URDFリミット -150° = -2.618 rad に近づきすぎない）
        # -2.4 rad ≈ -137° が下限、これを下回るとペナルティ
        knee_lower_limit = -2.4  # rad (約-137度)

        left_knee = self.dof_pos[:, 3]  # left_knee_pitch_joint
        right_knee = self.dof_pos[:, 8]  # right_knee_pitch_joint

        # 下限を下回った分だけペナルティ（曲がりすぎ）
        out_of_limits = torch.clamp(knee_lower_limit - left_knee, min=0) + torch.clamp(
            knee_lower_limit - right_knee, min=0
        )

        return out_of_limits

    # ------------ V9追加: 交互歩行・歩容改善報酬 ----------------

    def _reward_hip_pitch_sync_penalty(self) -> torch.Tensor:
        """左右hip_pitchの同期動作にペナルティ

        V8では左右脚が同期して動いていた（hip_pitch相関+0.772）。
        同じ方向に同時に動くとペナルティを与え、交互動作を促す。
        """
        left_hip_vel = self.dof_vel[:, self.left_hip_pitch_idx]
        right_hip_vel = self.dof_vel[:, self.right_hip_pitch_idx]

        # 同じ方向に動いている場合（積が正）にペナルティ
        sync_motion = torch.clamp(left_hip_vel * right_hip_vel, min=0.0)

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return sync_motion * has_command

    def _reward_foot_clearance(self) -> torch.Tensor:
        """足のクリアランス（持ち上げ高さ）報酬

        スイング中の足が地面から十分に離れていることを報酬化。
        滑走歩行を防ぎ、明確な足上げを促す。
        """
        if self.feet_indices is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        link_pos = self.robot.get_links_pos()
        feet_z = link_pos[:, self.feet_indices, 2]  # [num_envs, 2]

        # 接地していない足の高さを報酬化
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # 非接地足の高さ（接地足は0として扱う）
        clearance = torch.where(contacts, torch.zeros_like(feet_z), feet_z)

        # 目標クリアランス（0.03m）を超えた分を報酬
        target_clearance = 0.03
        reward = torch.clamp(clearance - target_clearance, min=0.0, max=0.05)

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return torch.sum(reward, dim=1) * has_command

    def _reward_hip_pitch_range(self) -> torch.Tensor:
        """hip_pitchの動作範囲報酬

        ストライドを大きくするため、hip_pitchの振幅を報酬化。
        """
        # 最近の最大・最小値から振幅を計算
        left_range = self.hip_pitch_max_recent[:, 0] - self.hip_pitch_min_recent[:, 0]
        right_range = self.hip_pitch_max_recent[:, 1] - self.hip_pitch_min_recent[:, 1]

        # 目標振幅（0.3 rad ≈ 17°）
        target_range = 0.3
        reward = torch.clamp((left_range + right_range) / 2 - target_range, min=0.0, max=0.3)

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    def _reward_hip_pitch_sign_change(self) -> torch.Tensor:
        """hip_pitchの符号変化報酬（V16新規）

        「剣道すり足」問題の解消を目的とした報酬。
        hip_pitchが正から負、または負から正に変化するタイミングを報酬化。
        これにより「常に前脚」「常に後脚」というパターンを崩し、
        脚の役割（前/後）の入れ替わりを促進する。

        報酬ロジック:
        - 前ステップと現ステップでhip_pitchの符号が変化したら報酬
        - 左右両脚で独立に検出し、合計を報酬とする
        """
        # 現在のhip_pitch（デフォルトからの相対値）
        left_hp = self.dof_pos[:, self.left_hip_pitch_idx] - self.default_dof_pos[self.left_hip_pitch_idx]
        right_hp = self.dof_pos[:, self.right_hip_pitch_idx] - self.default_dof_pos[self.right_hip_pitch_idx]

        # 前ステップのhip_pitch（デフォルトからの相対値）
        last_left_hp = self.last_left_hip_pitch - self.default_dof_pos[self.left_hip_pitch_idx]
        last_right_hp = self.last_right_hip_pitch - self.default_dof_pos[self.right_hip_pitch_idx]

        # 符号変化検出（前ステップと現ステップの積が負 = 符号変化）
        left_sign_change = (last_left_hp * left_hp < 0).float()
        right_sign_change = (last_right_hp * right_hp < 0).float()

        # 両脚の符号変化を合計（0, 1, または2）
        reward = left_sign_change + right_sign_change

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    def _reward_hip_pitch_alternating_sign_change(self) -> torch.Tensor:
        """交互符号変化報酬（V18オプション用）

        V16のhip_pitch_sign_changeはSUM方式で、両脚同時変化が最大報酬となり
        同期歩行を誘発した（論理的欠陥）。

        このXOR方式は、片脚のみ符号変化したときに報酬を与え、
        両脚同時変化は報酬なし（またはペナルティ）とする。

        | 状態 | reward |
        |------|--------|
        | 両脚静止 | 0 |
        | 左脚のみ変化 | 1 (最大) |
        | 右脚のみ変化 | 1 (最大) |
        | 両脚同時変化 | 0 |

        これにより交互歩行（片脚ずつ役割が入れ替わる）が最適となる。
        """
        # 現在のhip_pitch（デフォルトからの相対値）
        left_hp = self.dof_pos[:, self.left_hip_pitch_idx] - self.default_dof_pos[self.left_hip_pitch_idx]
        right_hp = self.dof_pos[:, self.right_hip_pitch_idx] - self.default_dof_pos[self.right_hip_pitch_idx]

        # 前ステップのhip_pitch（デフォルトからの相対値）
        last_left_hp = self.last_left_hip_pitch - self.default_dof_pos[self.left_hip_pitch_idx]
        last_right_hp = self.last_right_hip_pitch - self.default_dof_pos[self.right_hip_pitch_idx]

        # 符号変化検出
        left_sign_change = (last_left_hp * left_hp < 0).float()
        right_sign_change = (last_right_hp * right_hp < 0).float()

        # XOR: 片方だけ変化したときに1、両方同時変化または両方静止は0
        alternating_change = ((left_sign_change + right_sign_change) == 1).float()

        # オプション: 両脚同時変化にペナルティ（コメントアウト、必要に応じて有効化）
        # both_change = (left_sign_change * right_sign_change).float()
        # reward = alternating_change - 0.5 * both_change

        reward = alternating_change

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    def _reward_yaw_rate(self) -> torch.Tensor:
        """Yaw角速度ペナルティ

        V8ではYawが10秒で-15.7°ドリフトした。
        Yaw角速度にペナルティを与えて直進性を改善。
        """
        yaw_rate = self.base_ang_vel[:, 2]  # Z軸角速度
        return torch.square(yaw_rate)

    def _reward_symmetry(self) -> torch.Tensor:
        """左右対称性報酬

        左右の脚の動きが対称（位相差180°）になるよう促す。
        左右の関節角度の差にペナルティを与える（ただしhip_pitchは除く）。
        """
        # hip_roll, knee, ankle_pitchの左右差
        left_roll = self.dof_pos[:, 1]  # left_hip_roll
        right_roll = self.dof_pos[:, 6]  # right_hip_roll
        left_knee = self.dof_pos[:, 3]
        right_knee = self.dof_pos[:, 8]
        left_ankle = self.dof_pos[:, 4]
        right_ankle = self.dof_pos[:, 9]

        # 左右差のペナルティ（hip_pitchは交互であるべきなので除外）
        diff = (
            torch.abs(left_roll - right_roll)
            + torch.abs(left_knee - right_knee) * 0.5
            + torch.abs(left_ankle - right_ankle) * 0.5
        )

        return diff

    def _reward_base_height_low(self) -> torch.Tensor:
        """低すぎる高さへの追加ペナルティ

        base_height_targetより大幅に低い場合に追加ペナルティ。
        しゃがみ込み防止。
        """
        height = self.base_pos[:, 2]
        low_threshold = self.base_height_target - 0.04  # 目標より4cm以上低い
        low_error = torch.clamp(low_threshold - height, min=0.0)
        return torch.square(low_error)

    # ============================================================
    # V10+: Phase-based Reference Trajectory Rewards
    # ============================================================
    # 「かわいく生き物のように歩く」ための位相同期報酬群
    # CPG（Central Pattern Generator）インスパイアの正弦波参照軌道
    # ============================================================

    def _reward_phase_hip_pitch_tracking(self) -> torch.Tensor:
        """位相同期hip_pitch参照軌道追従報酬

        正弦波の参照軌道を生成し、hip_pitchがそれに追従するよう報酬化。
        左右は180°（π）位相差で交互歩行を促進。

        参照軌道:
            left_ref = amplitude * sin(phase)
            right_ref = amplitude * sin(phase + π)  # 180°位相差

        amplitude: hip_pitch振幅（default ~0.3 rad ≈ 17°）
        """
        # 参照軌道のパラメータ
        amplitude = self.reward_cfg.get("ref_hip_pitch_amplitude", 0.3)  # rad
        offset = self.reward_cfg.get("ref_hip_pitch_offset", 0.0)  # rad

        # 左右の参照軌道（180°位相差）
        left_ref = offset + amplitude * torch.sin(self.gait_phase)
        right_ref = offset + amplitude * torch.sin(self.gait_phase + math.pi)

        # 現在のhip_pitch角度（default_dof_posからの偏差）
        left_hp = self.dof_pos[:, self.left_hip_pitch_idx] - self.default_dof_pos[self.left_hip_pitch_idx]
        right_hp = self.dof_pos[:, self.right_hip_pitch_idx] - self.default_dof_pos[self.right_hip_pitch_idx]

        # 追従誤差
        left_error = torch.square(left_hp - left_ref)
        right_error = torch.square(right_hp - right_ref)

        # 指数関数的報酬（追従が良いほど高い）
        tracking_sigma = self.reward_cfg.get("phase_tracking_sigma", 0.1)
        reward = torch.exp(-(left_error + right_error) / tracking_sigma)

        # コマンドがある場合のみ報酬
        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    def _reward_phase_contact_sync(self) -> torch.Tensor:
        """位相-接地同期報酬

        歩行位相と足の接地タイミングが同期しているかを報酬化。
        phase ∈ [0, π): 左足接地期待、右足スイング期待
        phase ∈ [π, 2π): 右足接地期待、左足スイング期待
        """
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        left_contact = contacts[:, 0].float()  # 左足接地
        right_contact = contacts[:, 1].float()  # 右足接地

        # 期待される接地状態（正弦波ベース）
        # sin(phase) > 0 → 左足スイング期待、右足接地期待
        # sin(phase) < 0 → 左足接地期待、右足スイング期待
        phase_sin = torch.sin(self.gait_phase)

        # 期待との一致度
        # phase_sin > 0: 右足接地期待 → right_contactが1なら良い
        # phase_sin < 0: 左足接地期待 → left_contactが1なら良い
        expected_right = (phase_sin > 0).float()
        expected_left = (phase_sin < 0).float()

        # ソフトマッチング（完全一致でなくても部分報酬）
        match_left = 1.0 - torch.abs(left_contact - expected_left)
        match_right = 1.0 - torch.abs(right_contact - expected_right)

        reward = (match_left + match_right) / 2.0

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    def _reward_phase_velocity_sync(self) -> torch.Tensor:
        """位相-hip_pitch速度同期報酬

        hip_pitchの速度が位相の微分（cos）と同期しているかを報酬化。
        これにより滑らかな正弦波運動を促進。

        d/dt[sin(ωt)] = ω*cos(ωt)
        """
        # hip_pitch速度
        left_vel = self.dof_vel[:, self.left_hip_pitch_idx]
        right_vel = self.dof_vel[:, self.right_hip_pitch_idx]

        # 参照速度（正弦波の微分 = 余弦波）
        self.reward_cfg.get("ref_hip_pitch_amplitude", 0.3)
        2 * math.pi * self.gait_frequency  # 角速度

        # 期待される速度の符号
        # left: d/dt[sin(phase)] = cos(phase)
        # right: d/dt[sin(phase + π)] = cos(phase + π) = -cos(phase)
        expected_left_sign = torch.cos(self.gait_phase)
        expected_right_sign = -torch.cos(self.gait_phase)

        # 速度の符号が期待と一致しているか
        left_sign_match = (left_vel * expected_left_sign > 0).float()
        right_sign_match = (right_vel * expected_right_sign > 0).float()

        reward = (left_sign_match + right_sign_match) / 2.0

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    def _reward_smooth_action(self) -> torch.Tensor:
        """滑らかなアクション報酬

        action_rateはペナルティだが、こちらは滑らかな変化を正の報酬として与える。
        「かわいい」動きは急激な変化がなく滑らか。
        """
        # アクション変化量
        action_diff = self.actions - self.last_actions
        action_smoothness = torch.exp(-torch.sum(torch.square(action_diff), dim=1) * 10.0)

        return action_smoothness

    def _reward_periodic_foot_lift(self) -> torch.Tensor:
        """周期的な足の持ち上げ報酬

        歩行位相に同期して足を持ち上げることを報酬化。
        スイング期に足が地面から離れていることを期待。
        """
        if self.feet_indices is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        link_pos = self.robot.get_links_pos()
        feet_z = link_pos[:, self.feet_indices, 2]  # [num_envs, 2]

        # スイング期の識別（sin(phase) > 0 → 左スイング、< 0 → 右スイング）
        phase_sin = torch.sin(self.gait_phase)
        left_swing = (phase_sin > 0).float()
        right_swing = (phase_sin < 0).float()

        # スイング期の足の高さを報酬化
        target_lift = 0.03  # 目標持ち上げ高さ
        left_lift_reward = torch.clamp(feet_z[:, 0] - target_lift, min=0.0, max=0.05) * left_swing
        right_lift_reward = torch.clamp(feet_z[:, 1] - target_lift, min=0.0, max=0.05) * right_swing

        reward = left_lift_reward + right_lift_reward

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    def _reward_natural_rhythm(self) -> torch.Tensor:
        """自然なリズム報酬

        歩行周波数が目標に近いことを報酬化。
        hip_pitchのゼロクロスを検出し、実際の歩行周波数を推定。
        """
        # 前回と今回のhip_pitchの符号変化でゼロクロス検出
        left_hp = self.dof_pos[:, self.left_hip_pitch_idx] - self.default_dof_pos[self.left_hip_pitch_idx]

        # 符号変化検出
        sign_change = (self.last_left_hip_pitch * left_hp < 0).float()

        # ゼロクロス時に報酬（自然な周期運動の証拠）
        reward = sign_change * 0.5

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    # ------------ V11追加: ハイブリッドアプローチ報酬 ----------------

    def _reward_symmetry_hip_roll(self) -> torch.Tensor:
        """左右hip_rollの対称性報酬（V11新規）

        斜行対策。左右のhip_rollは符号反転で対称であるべき。
        例: 左が外転(+)なら右は内転(-)
        差が大きいほどペナルティ。
        """
        left_hr = self.dof_pos[:, self.left_hip_roll_idx]
        right_hr = self.dof_pos[:, self.right_hip_roll_idx]

        # 左右で符号反転が正常（外転・内転が対称）
        # diff = |left + right| → 対称なら0に近い
        diff = torch.abs(left_hr + right_hr)

        # 差が小さいほど報酬大（ガウシアン型）
        reward = torch.exp(-diff / 0.1)

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return reward * has_command

    def _reward_ground_contact_bonus(self) -> torch.Tensor:
        """接地ボーナス報酬（V11新規）

        少なくとも片足が接地している時に正の報酬。
        両足宙浮き（滑走歩行）を抑制し、接地を促進。
        """
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # 少なくとも1つの足が接地していれば報酬
        at_least_one = (contacts.sum(dim=1) >= 1).float()

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return at_least_one * has_command

    # ------------ V12追加: 同期歩行修正用報酬 ----------------

    def _reward_strict_alternating_contact(self) -> torch.Tensor:
        """厳格な交互接地報酬（V12新規）

        接地状態が厳密に交互（片足のみ接地）であることを報酬化。
        両足同時接地や両足宙浮きにはペナルティ。

        期待される状態:
        - 左足のみ接地 → 報酬
        - 右足のみ接地 → 報酬
        - 両足接地 → 0（ペナルティではないが報酬なし）
        - 両足宙浮き → 0

        これにより同期歩行（両足同時接地）を直接抑制。
        """
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        left_contact = contacts[:, 0].float()  # 左足接地
        right_contact = contacts[:, 1].float()  # 右足接地

        # XOR: 片足のみ接地の場合に1
        # left XOR right = |left - right|
        alternating = torch.abs(left_contact - right_contact)

        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return alternating * has_command

    # ------------ V18追加: 関節角速度制限 ----------------

    def _reward_dof_vel_limits(self) -> torch.Tensor:
        """関節角速度のソフトリミット報酬（V18追加）

        【設計原理】
        実機（RobStride RS-02）の最大角速度は ±44 rad/s。
        シミュレーションでこの制限を超える動作を学習すると、実機展開時に
        速度飽和による動作不安定・性能劣化が発生する。

        この報酬はsoft_dof_vel_limit（デフォルト90%）を超えた角速度に
        ペナルティを与えることで、実機パラメータ内での動作を促す。

        【参考文献】
        - ETH Zurich Legged Gym:
          https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot.py
        - RobStride RS-02仕様:
          ros2_ws/src/robstride_hardware/include/robstride_hardware/robstride_driver.hpp
          constexpr double VELOCITY = 44.0; // ±44 rad/s

        【実装】
        - dof_vel_limits: 実機の最大速度（44 rad/s）
        - soft_dof_vel_limit: 制限の何%で報酬を開始するか（0.9 = 90%）
        - ペナルティは制限超過分を0-1の範囲にクリップして合計

        【使用例】
        reward_cfg = {
            "dof_vel_limits": 44.0,      # RobStride RS-02の仕様
            "soft_dof_vel_limit": 0.9,   # 制限の90%（39.6 rad/s）で報酬開始
            "reward_scales": {
                "dof_vel_limits": -0.3,  # ペナルティ係数
            }
        }
        """
        dof_vel_limits = self.reward_cfg.get("dof_vel_limits", 44.0)  # デフォルト: RS-02仕様
        soft_limit_factor = self.reward_cfg.get("soft_dof_vel_limit", 0.9)  # デフォルト: 90%

        # 各関節の速度絶対値が制限の何%を超えているか
        # (|vel| - limit*factor) をクリップして合計
        out_of_limits = (torch.abs(self.dof_vel) - dof_vel_limits * soft_limit_factor).clip(min=0.0, max=1.0)

        return torch.sum(out_of_limits, dim=1)
