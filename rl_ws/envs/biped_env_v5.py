"""
BSL-Droid二脚ロボット Genesis環境 v5

改善点（V4からの変更）:
- 足の高さ報酬（foot_clearance）: 遊脚時に足を高く上げることを促進（大きな歩幅）
- 歩幅報酬（stride_length）: 実際の歩幅が長いことを報酬
- V4の交互歩行機構を維持しつつ、よりダイナミックな動きを実現

観測空間 (39次元):
- base_ang_vel (3): ボディローカル角速度
- projected_gravity (3): ボディローカル重力ベクトル
- commands (3): 速度コマンド
- dof_pos (10): 関節位置（デフォルトからの偏差）
- dof_vel (10): 関節速度
- actions (10): 前回のアクション

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


class BipedEnvV5:
    """BSL-Droid二脚ロボットのGenesis強化学習環境 v5（大きな歩幅版）"""

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
        # 左足先と右足先のリンクを取得
        self.feet_names = env_cfg.get("feet_names", ["left_toe", "right_toe"])
        self.feet_indices = []
        for name in self.feet_names:
            try:
                link = self.robot.get_link(name)
                self.feet_indices.append(link.idx_local)
            except Exception:
                # リンクが見つからない場合はスキップ
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
            # V5: 前ステップの足の位置を保存（歩幅計算用）
            self.last_feet_pos = torch.zeros((self.num_envs, len(self.feet_indices), 3), dtype=gs.tc_float, device=gs.device)
        else:
            self.feet_air_time = None
            self.last_contacts = None
            self.last_feet_pos = None

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
        # 足先リンクのZ座標が閾値以下なら接地と判定
        contact_threshold = 0.03  # 3cm以下で接地判定
        link_pos = self.robot.get_links_pos()  # (n_envs, n_links, 3)
        feet_z = link_pos[:, self.feet_indices, 2]  # (n_envs, n_feet)
        return feet_z < contact_threshold

    def _get_foot_positions(self):
        """足の位置を取得（ワールド座標）"""
        if self.feet_indices is None:
            return None
        link_pos = self.robot.get_links_pos()  # (n_envs, n_links, 3)
        return link_pos[:, self.feet_indices, :]  # (n_envs, n_feet, 3)

    def _get_foot_heights(self):
        """足の高さを取得（地面からの高さ）"""
        if self.feet_indices is None:
            return None
        link_pos = self.robot.get_links_pos()  # (n_envs, n_links, 3)
        return link_pos[:, self.feet_indices, 2]  # (n_envs, n_feet)

    def step(self, actions):
        """環境を1ステップ進める"""
        self.actions = torch.clip(actions, -self.env_cfg["clip_actions"], self.env_cfg["clip_actions"])
        exec_actions = self.last_actions if self.simulate_action_latency else self.actions
        target_dof_pos = exec_actions * self.env_cfg["action_scale"] + self.default_dof_pos

        # 二脚ロボット: DOFスライス (6:16) = 10関節
        # freejoint(6) + 10 joints = 16
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

        # feet_air_time更新とlast_feet_pos更新（V5: 歩幅計算用）
        if self.feet_air_time is not None:
            contacts = self._get_foot_contacts()
            if contacts is not None:
                # 接地したら first_contact を記録
                first_contact = (self.feet_air_time > 0.0) & contacts & ~self.last_contacts
                self.feet_air_time += self.dt
                # 接地したらリセット
                self.feet_air_time = torch.where(contacts, torch.zeros_like(self.feet_air_time), self.feet_air_time)
                
                # V5: 足の位置を更新（歩幅計算用）
                current_feet_pos = self._get_foot_positions()
                if current_feet_pos is not None:
                    # 接地時に前回位置を更新
                    self.last_feet_pos = torch.where(
                        contacts.unsqueeze(-1),  # (n_envs, n_feet, 1)
                        current_feet_pos,
                        self.last_feet_pos
                    )
                
                self.last_contacts = contacts
                self._first_contact = first_contact  # 報酬計算用に保存
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

        # 終了判定（転倒検出）
        self.reset_buf = self.episode_length_buf > self.max_episode_length
        self.reset_buf |= torch.abs(self.base_euler[:, 1]) > self.env_cfg["termination_if_pitch_greater_than"]
        self.reset_buf |= torch.abs(self.base_euler[:, 0]) > self.env_cfg["termination_if_roll_greater_than"]

        # タイムアウト
        self.extras["time_outs"] = (self.episode_length_buf > self.max_episode_length).to(dtype=gs.tc_float)

        # リセット
        self._reset_idx(self.reset_buf)

        # 観測更新
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
            if self.feet_air_time is not None:
                self.feet_air_time.zero_()
                self.last_contacts.fill_(True)
                # V5: last_feet_posもリセット
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
            self.episode_length_buf.masked_fill_(envs_idx, 0)
            self.reset_buf.masked_fill_(envs_idx, True)
            if self.feet_air_time is not None:
                self.feet_air_time.masked_fill_(envs_idx[:, None], 0.0)
                self.last_contacts.masked_fill_(envs_idx[:, None], True)
                # V5: last_feet_posもリセット
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
        """Z方向速度ペナルティ（上下動抑制）"""
        return torch.square(self.base_lin_vel[:, 2])

    def _reward_ang_vel_xy(self):
        """XY角速度ペナルティ（ボディの揺れ抑制）"""
        return torch.sum(torch.square(self.base_ang_vel[:, :2]), dim=1)

    def _reward_action_rate(self):
        """アクション変化率ペナルティ（滑らかな動き）★強化"""
        return torch.sum(torch.square(self.last_actions - self.actions), dim=1)

    def _reward_similar_to_default(self):
        """デフォルト姿勢維持ペナルティ"""
        return torch.sum(torch.abs(self.dof_pos - self.default_dof_pos), dim=1)

    def _reward_base_height(self):
        """ベース高さ維持ペナルティ"""
        return torch.square(self.base_pos[:, 2] - self.reward_cfg["base_height_target"])

    def _reward_orientation(self):
        """姿勢維持報酬（直立を維持）"""
        # projected_gravityのz成分が-1に近いほど直立
        return torch.square(self.projected_gravity[:, 2] + 1.0)

    def _reward_torques(self):
        """トルクペナルティ（エネルギー効率）"""
        # アクションの大きさでペナルティ（PD制御なのでアクションがトルクに比例）
        return torch.sum(torch.square(self.actions), dim=1)

    def _reward_dof_vel(self):
        """関節速度ペナルティ（高速振動抑制）"""
        return torch.sum(torch.square(self.dof_vel), dim=1)

    def _reward_dof_acc(self):
        """関節加速度ペナルティ（急激な動き抑制）"""
        return torch.sum(torch.square((self.dof_vel - self.last_dof_vel) / self.dt), dim=1)

    def _reward_feet_air_time(self):
        """足の滞空時間報酬（長いストライドを促進）"""
        if self.feet_air_time is None or self._first_contact is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)
        
        # 最初の接地時に、滞空時間が目標以上なら報酬
        target_air_time = self.reward_cfg.get("feet_air_time_target", 0.25)
        rew_air_time = torch.sum(
            (self.feet_air_time - target_air_time) * self._first_contact.float(), dim=1
        )
        # コマンドがある時のみ報酬
        rew_air_time *= (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return rew_air_time

    def _reward_no_fly(self):
        """両足同時離地ペナルティ（安定歩行）"""
        if self.feet_air_time is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)
        
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)
        
        # 両足が同時に離地している場合ペナルティ
        both_feet_in_air = ~contacts.any(dim=1)
        return both_feet_in_air.float()

    def _reward_joint_symmetry(self):
        """左右対称性報酬（自然な歩行）"""
        left_vel = self.dof_vel[:, :5]
        right_vel = self.dof_vel[:, 5:]
        return torch.sum(torch.square(torch.abs(left_vel) - torch.abs(right_vel)), dim=1)

    def _reward_smoothness(self):
        """動作の滑らかさ報酬（2次微分ペナルティ）"""
        return torch.sum(torch.square(self.dof_vel - self.last_dof_vel), dim=1)

    # ============ V4: 交互歩行報酬（新規）============
    
    def _reward_alternating_gait(self):
        """交互歩行報酬: 左右の足が交互に接地していることを報酬
        
        理想: 片方の足が接地中、もう片方は空中
        すり足防止のコア報酬
        """
        contacts = self._get_foot_contacts()
        if contacts is None or contacts.shape[1] != 2:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)
        
        left_contact = contacts[:, 0]   # 左足の接地状態
        right_contact = contacts[:, 1]  # 右足の接地状態
        
        # XOR: 片方だけが接地している場合に報酬
        # 両足接地または両足離地はペナルティ（0報酬）
        alternating = left_contact ^ right_contact  # XOR
        
        # コマンドがある時のみ報酬
        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        
        return alternating.float() * has_command

    def _reward_foot_swing(self):
        """足のスイング報酬: 歩行中に足を持ち上げることを報酬
        
        足が地面から離れてスイングしていることを報酬
        すり足（足が常に地面に接触）を防ぐ
        """
        if self.feet_air_time is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)
        
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)
        
        # 足が空中にいる時間を報酬（ただし長すぎるとペナルティ）
        # 0.1〜0.3秒程度が理想的
        air_time_reward = torch.clamp(self.feet_air_time, 0.0, 0.3)
        
        # コマンドがある時のみ報酬
        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        
        return torch.sum(air_time_reward, dim=1) * has_command

    def _reward_single_stance(self):
        """片足立ち報酬: 歩行中に片足で立っている時間を報酬
        
        交互歩行の前提条件: 片足支持期が存在すること
        """
        contacts = self._get_foot_contacts()
        if contacts is None or contacts.shape[1] != 2:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)
        
        # 片足だけが接地している状態
        single_stance = contacts.sum(dim=1) == 1
        
        # コマンドがある時のみ報酬
        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        
        return single_stance.float() * has_command

    # ============ V5: 大きな歩幅報酬（新規）============
    
    def _reward_foot_clearance(self):
        """足の高さ報酬: 遊脚時に足を高く上げることを促進
        
        すり足から大きな歩幅への移行に必要
        """
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
        
        # コマンドがある時のみ報酬
        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        
        return torch.sum(clearance_reward, dim=1) * has_command

    def _reward_stride_length(self):
        """歩幅報酬: 前回接地位置からの距離を報酬
        
        長いストライドを促進
        """
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
        
        # コマンドがある時のみ報酬
        has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        
        return torch.sum(stride_reward, dim=1) * has_command
