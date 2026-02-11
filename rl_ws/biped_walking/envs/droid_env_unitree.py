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

hip_pitch可動域対称性（V17追加）:
  - symmetry_range: hip_pitch可動域の左右対称性報酬

足引きずり抑制（V10追加）:
  - feet_stumble: 接地中の足の水平速度ペナルティ

hip_pitch可動域対称性（V17追加）:
  - symmetry_range: hip_pitch可動域の左右対称性報酬

安全性ペナルティ:
  - termination: 終了ペナルティ
  - dof_pos_limits: 関節位置限界ペナルティ

実機パラメータ適合性（V18追加）:
  - dof_vel_limits: 関節速度ソフトリミットペナルティ（RobStride RS-02対応）

hip_pitch可動域促進（V19追加）:
  - hip_pitch_range: hip_pitch可動域の大きさ報酬（大股歩行促進）

遊脚時足首角度制限（V22a追加）:
  - ankle_pitch_range: 遊脚時のankle_pitch角度制限ペナルティ（つま先突き抑制）

接地相足先位置制約（V12 exp008追加）:
  - stance_foot_lateral_position: 接地足横方向位置ペナルティ（内股解消タスク空間制約）

速度二乗EMA対称性（V12 exp009追加）:
  - symmetry_vel_ema: 左右関節の運動エネルギー均等性報酬（位相不問）

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


class DroidEnvUnitree:
    """BSL-Droid Simplified二脚ロボットのGenesis強化学習環境（Unitree参考版）"""

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

        # Contact Sensor の追加（V20追加）
        # Z座標閾値ベースの接地検出から、物理エンジンの実際の接触判定に移行
        # V21追加: use_contact_sensorオプションでContact Sensorの使用を制御
        # Falseの場合はZ座標閾値ベースの接地検出にフォールバック
        self._feet_names_for_sensor = env_cfg.get("feet_names", ["left_foot_link", "right_foot_link"])
        self.contact_sensors = []
        if env_cfg.get("use_contact_sensor", True):
            for foot_name in self._feet_names_for_sensor:
                try:
                    link = self.robot.get_link(foot_name)
                    sensor = self.scene.add_sensor(
                        gs.sensors.Contact(
                            entity_idx=self.robot.idx,
                            link_idx_local=link.idx_local,
                        )
                    )
                    self.contact_sensors.append(sensor)
                except Exception:
                    pass

        # ビルド
        self.scene.build(n_envs=num_envs)

        # 関節名からインデックスを取得
        self.motors_dof_idx = torch.tensor(
            [self.robot.get_joint(name).dofs_idx_local[0] for name in self.env_cfg["joint_names"]],
            dtype=gs.tc_int,
            device=gs.device,
        )
        self.actions_dof_idx = torch.argsort(self.motors_dof_idx)

        # PD制御パラメータ（関節個別オーバーライド対応）
        # kp: デフォルト値で初期化し、kp_overridesで関節個別の値を上書き
        kp_values = [self.env_cfg["kp"]] * self.num_actions
        kp_overrides: dict[str, float] = self.env_cfg.get("kp_overrides", {})
        if kp_overrides:
            joint_names = self.env_cfg["joint_names"]
            for joint_name, kp_val in kp_overrides.items():
                if joint_name in joint_names:
                    idx = joint_names.index(joint_name)
                    kp_values[idx] = kp_val
        self.robot.set_dofs_kp(kp_values, self.motors_dof_idx)

        # kd: デフォルト値で初期化し、kd_overridesで関節個別の値を上書き（V25追加）
        kd_values = [self.env_cfg["kd"]] * self.num_actions
        kd_overrides: dict[str, float] = self.env_cfg.get("kd_overrides", {})
        if kd_overrides:
            joint_names = self.env_cfg["joint_names"]
            for joint_name, kd_val in kd_overrides.items():
                if joint_name in joint_names:
                    idx = joint_names.index(joint_name)
                    kd_values[idx] = kd_val
        self.robot.set_dofs_kv(kd_values, self.motors_dof_idx)

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

        # 足先位置（FK結果キャッシュ用）
        self.feet_pos = torch.zeros((self.num_envs, 2, 3), dtype=gs.tc_float, device=gs.device)
        self.feet_vel = torch.zeros((self.num_envs, 2, 3), dtype=gs.tc_float, device=gs.device)

        # 接触状態キャッシュ
        self.contact_state = torch.zeros((self.num_envs, 2), dtype=gs.tc_float, device=gs.device)

        # 速度二乗EMA（symmetry_vel_ema報酬用、exp009 V12追加）
        self.vel_sq_ema = torch.zeros((self.num_envs, self.num_actions), dtype=gs.tc_float, device=gs.device)

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
        # contact_no_velの速度次元数（V25追加: 2=XYのみ, 3=XYZ）
        self.contact_no_vel_dims: int = self.reward_cfg.get("contact_no_vel_dims", 2)
        # stance_hip_roll_targetのターゲット角度（exp008 V8追加）
        self.stance_hip_roll_target_angle = self.reward_cfg.get("stance_hip_roll_target_angle", 0.0)
        # stance_foot_lateral_positionの最小距離閾値（exp008 V12追加）
        self.stance_foot_lateral_min_distance = self.reward_cfg.get("stance_foot_lateral_min_distance", 0.050)
        # hip_rollの内向きPDターゲットクランプ値（exp008 V13追加）
        # Noneの場合はクランプなし（既存バージョンとの互換性維持）
        self.hip_roll_inward_limit = self.reward_cfg.get("hip_roll_inward_limit", None)
        # 位相条件付きPDクランプ（exp008 V16追加）
        # Trueの場合、hip_roll PDターゲットクランプを接地相のみに適用
        # Falseの場合、全位相で適用（V13-V15互換）
        self.hip_roll_clamp_stance_only = self.reward_cfg.get("hip_roll_clamp_stance_only", False)
        # 遊脚相の外向きPDターゲットクランプ値（exp008 V18追加）
        # Noneの場合はクランプなし（V17以前との互換性維持）
        # 正の値で指定（左脚の外向き上限、右脚は符号反転で適用）
        self.hip_roll_outward_limit: float | None = self.reward_cfg.get("hip_roll_outward_limit", None)

        # Mirror Augmentation（exp009 V13追加）
        # 半数の環境を永続的にL↔Rミラーし、ポリシーの左右対称性を構造的に保証する。
        # 報酬・物理はミラーされない（実状態で計算）。観測のみミラー、アクションはデミラーして適用。
        self.use_mirror_augmentation: bool = self.env_cfg.get("mirror_augmentation", False)
        if self.use_mirror_augmentation:
            self.mirror_mask = torch.zeros(self.num_envs, dtype=torch.bool, device=gs.device)
            self.mirror_mask[self.num_envs // 2 :] = True
            # DOFミラー: L↔Rスワップ [L0..L4, R5..R9] → [R5..R9, L0..L4]
            self.dof_mirror_idx = torch.tensor([5, 6, 7, 8, 9, 0, 1, 2, 3, 4], device=gs.device)
            # 符号反転: hip_yaw, hip_rollは鏡像で符号反転
            self.dof_mirror_sign = torch.tensor(
                [-1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, 1.0],
                dtype=gs.tc_float,
                device=gs.device,
            )

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
        """足の接地状態を取得（Contact Sensorベース）

        V20変更: Z座標閾値ベース → Genesis Contact Sensor ベース

        【変更理由】
        V19まではZ座標閾値で判定していたが、Base高さの変動で
        誤判定が発生していた（feet_air_time=0, single_foot_contact=0）。
        GenesisのContact Sensorを使用することで、物理エンジンの
        実際の接触判定を利用する。

        Returns:
            torch.Tensor: 接地状態 (n_envs, 2)、True=接地中
        """
        # Contact Sensorが利用可能な場合はそれを使用（V20以降）
        if self.contact_sensors:
            contacts = []
            for sensor in self.contact_sensors:
                contact = sensor.read()  # shape: (n_envs, 1)
                # (n_envs, 1) → (n_envs,) に変換
                if contact.dim() > 1:
                    contact = contact.squeeze(-1)
                contacts.append(contact)
            # (n_envs,) × 2 → (n_envs, 2) にスタック
            result = torch.stack(contacts, dim=-1)
            # Genesis Contact Sensorはintを返すため、明示的にboolに変換
            return result.bool() if result.dtype != torch.bool else result

        # フォールバック: Z座標ベースの旧実装（Contact Sensorがない場合）
        if self.feet_indices is None:
            return None
        link_pos = self.robot.get_links_pos()
        feet_z = link_pos[:, self.feet_indices, 2]
        return feet_z < self.contact_threshold

    def _update_feet_state(self) -> None:
        """足先の位置と速度を更新"""
        if self.feet_indices is None:
            return
        link_pos = self.robot.get_links_pos()
        link_vel = self.robot.get_links_vel()
        for i, idx in enumerate(self.feet_indices):
            self.feet_pos[:, i, :] = link_pos[:, idx, :]
            self.feet_vel[:, i, :] = link_vel[:, idx, :]

    def step(self, actions: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, dict[str, Any]]:
        """環境を1ステップ進める"""
        self.actions = torch.clip(actions, -self.env_cfg["clip_actions"], self.env_cfg["clip_actions"])

        # Mirror Augmentation: ミラー環境のアクションをデミラー（実ロボット座標に変換）
        if self.use_mirror_augmentation:
            m = self.mirror_mask
            self.actions[m] = self.actions[m][:, self.dof_mirror_idx] * self.dof_mirror_sign

        exec_actions = self.last_actions if self.simulate_action_latency else self.actions
        target_dof_pos = exec_actions * self.env_cfg["action_scale"] + self.default_dof_pos

        # hip_rollのPDターゲットクランプ（V13追加, V16拡張: 位相条件付き, V18拡張: 双方向）
        if self.hip_roll_inward_limit is not None or self.hip_roll_outward_limit is not None:
            # 内向きクランプ値（V13-V17）
            if self.hip_roll_inward_limit is not None:
                left_inward_clamped = torch.clamp(
                    target_dof_pos[:, self.left_hip_roll_idx], min=self.hip_roll_inward_limit
                )
                right_inward_clamped = torch.clamp(
                    target_dof_pos[:, self.right_hip_roll_idx], max=-self.hip_roll_inward_limit
                )

            # 外向きクランプ値（V18新規）
            if self.hip_roll_outward_limit is not None:
                left_outward_clamped = torch.clamp(
                    target_dof_pos[:, self.left_hip_roll_idx], max=self.hip_roll_outward_limit
                )
                right_outward_clamped = torch.clamp(
                    target_dof_pos[:, self.right_hip_roll_idx], min=-self.hip_roll_outward_limit
                )

            if self.hip_roll_clamp_stance_only:
                # 位相条件付きクランプ（V16追加, V18拡張: 双方向）
                # contact_stateは前ステップ値（20ms遅延、stance ~500msに対し十分小さい）
                left_stance = self.contact_state[:, 0] > 0.5
                right_stance = self.contact_state[:, 1] > 0.5
                left_swing = ~left_stance
                right_swing = ~right_stance

                # 接地相: 内向きクランプ（V13-V17同様）
                if self.hip_roll_inward_limit is not None:
                    target_dof_pos[:, self.left_hip_roll_idx] = torch.where(
                        left_stance, left_inward_clamped, target_dof_pos[:, self.left_hip_roll_idx]
                    )
                    target_dof_pos[:, self.right_hip_roll_idx] = torch.where(
                        right_stance, right_inward_clamped, target_dof_pos[:, self.right_hip_roll_idx]
                    )

                # 遊脚相: 外向きクランプ（V18新規）
                if self.hip_roll_outward_limit is not None:
                    target_dof_pos[:, self.left_hip_roll_idx] = torch.where(
                        left_swing, left_outward_clamped, target_dof_pos[:, self.left_hip_roll_idx]
                    )
                    target_dof_pos[:, self.right_hip_roll_idx] = torch.where(
                        right_swing, right_outward_clamped, target_dof_pos[:, self.right_hip_roll_idx]
                    )
            else:
                # 全位相クランプ（V13-V15互換）
                if self.hip_roll_inward_limit is not None:
                    target_dof_pos[:, self.left_hip_roll_idx] = left_inward_clamped
                    target_dof_pos[:, self.right_hip_roll_idx] = right_inward_clamped
                if self.hip_roll_outward_limit is not None:
                    target_dof_pos[:, self.left_hip_roll_idx] = left_outward_clamped
                    target_dof_pos[:, self.right_hip_roll_idx] = right_outward_clamped

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
            assert self.last_contacts is not None
            first_contact = (self.feet_air_time > 0.0) & contacts & ~self.last_contacts
            self.feet_air_time += self.dt
            self.feet_air_time = torch.where(contacts, torch.zeros_like(self.feet_air_time), self.feet_air_time)
            self.last_contacts = contacts
            self._first_contact = first_contact
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

        if "termination_if_height_lower_than" in self.env_cfg:
            self.reset_buf |= self.base_pos[:, 2] < self.env_cfg["termination_if_height_lower_than"]

        if self.env_cfg.get("termination_if_knee_positive", False):
            left_knee = self.dof_pos[:, self.left_knee_idx]
            right_knee = self.dof_pos[:, self.right_knee_idx]
            self.reset_buf |= (left_knee > 0.1) | (right_knee > 0.1)

        self.extras["time_outs"] = (self.episode_length_buf > self.max_episode_length).to(dtype=gs.tc_float)

        self._reset_idx(self.reset_buf)
        self._update_observation()

        # Mirror Augmentation: 観測をミラー（ポリシーが左右反転世界を見る）
        if self.use_mirror_augmentation:
            self._mirror_observations()

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
            self.gait_phase.zero_()
            self.leg_phase.zero_()
            self.feet_pos.zero_()
            self.feet_vel.zero_()
            self.contact_state.zero_()
            self.vel_sq_ema.zero_()
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
            self.gait_phase.masked_fill_(envs_idx, 0.0)
            self.leg_phase.masked_fill_(envs_idx[:, None], 0.0)
            self.feet_pos.masked_fill_(envs_idx[:, None, None], 0.0)
            self.feet_vel.masked_fill_(envs_idx[:, None, None], 0.0)
            self.contact_state.masked_fill_(envs_idx[:, None], 0.0)
            self.vel_sq_ema.masked_fill_(envs_idx[:, None], 0.0)
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

    def _mirror_observations(self) -> None:
        """ミラー環境の観測をL↔R反転する（exp009 V13追加）

        観測ベクトル (50次元) の構造:
          [0:3]   base_lin_vel    → vy (idx 1) 符号反転
          [3:6]   base_ang_vel    → roll (idx 3), yaw (idx 5) 符号反転
          [6:9]   projected_gravity → gy (idx 7) 符号反転
          [9:12]  commands        → cmd_vy (idx 10), cmd_vyaw (idx 11) 符号反転
          [12:22] dof_pos         → L↔Rスワップ + yaw/roll符号反転
          [22:32] dof_vel         → 同上
          [32:42] actions         → 同上
          [42:44] gait_phase      → sin/cos符号反転（V14バグ修正: 位相+0.5→符号反転）
          [44:46] leg_phase       → L↔Rスワップ
          [46:48] feet_pos_z      → L↔Rスワップ
          [48:50] contact_state   → L↔Rスワップ
        """
        m = self.mirror_mask
        obs = self.obs_buf

        # base_lin_vel: vy符号反転
        obs[m, 1] *= -1
        # base_ang_vel: roll, yaw符号反転
        obs[m, 3] *= -1
        obs[m, 5] *= -1
        # projected_gravity: gy符号反転
        obs[m, 7] *= -1
        # commands: cmd_vy, cmd_vyaw符号反転
        obs[m, 10] *= -1
        obs[m, 11] *= -1

        # DOF系 (pos, vel, actions): L↔Rスワップ + yaw/roll符号反転
        sign = self.dof_mirror_sign
        idx = self.dof_mirror_idx
        for start in (12, 22, 32):
            block = obs[m, start : start + 10].clone()
            obs[m, start : start + 10] = block[:, idx] * sign

        # gait_phase: sin/cos符号反転（V14バグ修正）
        # gait_phase = sin(L脚位相*2π), cos(L脚位相*2π)
        # ミラー時はR脚位相 = L脚位相+0.5 → sin((φ+0.5)*2π) = -sin(φ*2π), cos同様
        obs[m, 42] *= -1
        obs[m, 43] *= -1
        # leg_phase: L↔Rスワップ
        lp = obs[m, 44:46].clone()
        obs[m, 44] = lp[:, 1]
        obs[m, 45] = lp[:, 0]
        # feet_pos_z: L↔Rスワップ
        fz = obs[m, 46:48].clone()
        obs[m, 46] = fz[:, 1]
        obs[m, 47] = fz[:, 0]
        # contact_state: L↔Rスワップ
        cs = obs[m, 48:50].clone()
        obs[m, 48] = cs[:, 1]
        obs[m, 49] = cs[:, 0]

    def reset(self) -> tuple[torch.Tensor, None]:
        self._reset_idx()
        self._update_observation()
        if self.use_mirror_augmentation:
            self._mirror_observations()
        return self.obs_buf, None

    # ============================================================
    # 報酬関数（全バージョン共通）
    # ============================================================

    # ------------ 速度追従報酬（Unitree方式）----------------

    def _reward_tracking_lin_vel(self) -> torch.Tensor:
        """線形速度追従報酬（Unitree方式: exp関数）"""
        lin_vel_error = torch.sum(torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1)
        return torch.exp(-lin_vel_error / self.tracking_sigma)

    def _reward_tracking_ang_vel(self) -> torch.Tensor:
        """角速度追従報酬（Unitree方式: exp関数）"""
        ang_vel_error = torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / self.tracking_sigma)

    # ------------ 安定性ペナルティ（Unitree方式）----------------

    def _reward_lin_vel_z(self) -> torch.Tensor:
        """Z方向速度ペナルティ"""
        return torch.square(self.base_lin_vel[:, 2])

    def _reward_base_vel_y(self) -> torch.Tensor:
        """Y方向並進速度ペナルティ（横方向揺れ抑制）

        Added in exp008 V15: 横方向並進速度を直接ペナルティ化。
        V14で顕在化した横方向並進揺れ（lateral std 48mm, Y速度std 0.225m/s）を抑制する。
        ang_vel_xyが角速度（回転）を制約するのに対し、本報酬は並進速度を直接制約する。
        """
        return torch.square(self.base_lin_vel[:, 1])

    def _reward_ang_vel_xy(self) -> torch.Tensor:
        """XY角速度ペナルティ"""
        return torch.sum(torch.square(self.base_ang_vel[:, :2]), dim=1)

    def _reward_orientation(self) -> torch.Tensor:
        """姿勢維持ペナルティ（Unitree方式: projected_gravity）"""
        return torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1)

    def _reward_base_height(self) -> torch.Tensor:
        """ベース高さペナルティ（Unitree方式）"""
        height = self.base_pos[:, 2]
        return torch.square(height - self.base_height_target)

    # ------------ エネルギー効率ペナルティ ----------------

    def _reward_torques(self) -> torch.Tensor:
        """トルクペナルティ"""
        return torch.sum(torch.square(self.actions), dim=1)

    def _reward_dof_vel(self) -> torch.Tensor:
        """関節速度ペナルティ"""
        return torch.sum(torch.square(self.dof_vel), dim=1)

    def _reward_dof_acc(self) -> torch.Tensor:
        """関節加速度ペナルティ"""
        return torch.sum(torch.square((self.dof_vel - self.last_dof_vel) / self.dt), dim=1)

    def _reward_action_rate(self) -> torch.Tensor:
        """アクション変化率ペナルティ"""
        return torch.sum(torch.square(self.last_actions - self.actions), dim=1)

    # ------------ 歩行品質報酬（Unitree方式）----------------

    def _reward_feet_air_time(self) -> torch.Tensor:
        """滞空時間報酬（Unitree方式）

        オフセット値はreward_cfg["air_time_offset"]で設定可能（デフォルト: 0.5秒）。
        小型ロボット（BSL-Droid）では0.25秒程度が適切。
        """
        if self.feet_air_time is None or self._first_contact is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # air_time_offsetは設定可能（V5追加）
        rew_air_time = torch.sum((self.feet_air_time - self.air_time_offset) * self._first_contact.float(), dim=1)
        rew_air_time *= (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
        return rew_air_time

    def _reward_contact(self) -> torch.Tensor:
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

    def _reward_feet_swing_height(self) -> torch.Tensor:
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

    def _reward_contact_no_vel(self) -> torch.Tensor:
        """接地時足速度ペナルティ（Unitree H1方式、次元数設定可能）

        接地中の足の速度をペナルティ化（足滑り防止）。

        【次元数設定】reward_cfg["contact_no_vel_dims"]で制御:
        - 2 (デフォルト): XY方向のみ（V23以前互換、足滑り防止）
        - 3: XYZ 3D速度（V24で試行、構造的欠陥によりV25で非推奨）
        """
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # 足速度の次元選択（contact_no_vel_dims: 2=XY, 3=XYZ）
        vel = self.feet_vel[:, :, : self.contact_no_vel_dims]
        vel_sq = torch.sum(torch.square(vel), dim=2)  # [num_envs, 2]
        return torch.sum(vel_sq * contacts.float(), dim=1)

    def _reward_hip_pos(self) -> torch.Tensor:
        """股関節位置ペナルティ（Unitree G1/H1方式）

        hip_yaw, hip_rollの過度な変位をペナルティ化。
        """
        # BSL-Droid: hip_yaw (0, 5), hip_roll (1, 6)
        hip_angles = torch.cat(
            [
                self.dof_pos[:, self.left_hip_yaw_idx : self.left_hip_yaw_idx + 1],
                self.dof_pos[:, self.left_hip_roll_idx : self.left_hip_roll_idx + 1],
                self.dof_pos[:, self.right_hip_yaw_idx : self.right_hip_yaw_idx + 1],
                self.dof_pos[:, self.right_hip_roll_idx : self.right_hip_roll_idx + 1],
            ],
            dim=1,
        )
        return torch.sum(torch.square(hip_angles), dim=1)

    def _reward_alive(self) -> torch.Tensor:
        """生存報酬（Unitree方式）"""
        return torch.ones(self.num_envs, dtype=gs.tc_float, device=gs.device)

    # ------------ 静止ポリシー対策（V3追加）----------------

    def _reward_single_foot_contact(self) -> torch.Tensor:
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

        # 移動コマンドかどうかを判定（XY速度ノルム > 0.1 m/s、多方向対応）
        is_moving_command = torch.norm(self.commands[:, :2], dim=1) > 0.1

        # 接地状態を取得
        left_contact = contacts[:, 0]
        right_contact = contacts[:, 1]

        # 片足のみ接地 = XOR
        single_contact = left_contact ^ right_contact

        # 移動コマンド時のみ報酬、静止コマンド時は1.0
        reward = torch.where(
            is_moving_command, single_contact.float(), torch.ones(self.num_envs, dtype=gs.tc_float, device=gs.device)
        )
        return reward

    def _reward_velocity_deficit(self) -> torch.Tensor:
        """目標速度未達ペナルティ（V3追加、exp009で多方向対応に汎化）

        目標速度を下回っている場合にペナルティを付与。
        「動かない」局所最適を回避する。

        【設計意図】
        - 静止ポリシーでは、velocity_deficit = (target - 0)² > 0 となりペナルティ
        - 歩行ポリシーでは、target近傍でdeficit ≈ 0
        - これにより静止が明示的に不利になる

        【多方向対応（exp009）】
        コマンド方向への射影ベースに変更。XY平面の任意方向コマンドに対応。
        exp008以前（Y=0）の場合は旧実装と同一結果を返す。
        """
        cmd_xy = self.commands[:, :2]
        cmd_mag = torch.norm(cmd_xy, dim=1)
        is_moving = cmd_mag > 0.05
        cmd_dir = cmd_xy / (cmd_mag.unsqueeze(1) + 1e-8)
        vel_along_cmd = torch.sum(self.base_lin_vel[:, :2] * cmd_dir, dim=1)
        deficit = torch.clamp(cmd_mag - vel_along_cmd, min=0)
        return deficit**2 * is_moving.float()

    # ------------ 安全性ペナルティ ----------------

    def _reward_termination(self) -> torch.Tensor:
        """終了ペナルティ"""
        return self.reset_buf.float() * (self.episode_length_buf < self.max_episode_length).float()

    def _reward_dof_pos_limits(self) -> torch.Tensor:
        """関節位置限界ペナルティ

        膝関節が0度（まっすぐ）に近づきすぎることを抑制。
        """
        knee_upper_limit = -0.2  # rad
        left_knee = self.dof_pos[:, self.left_knee_idx]
        right_knee = self.dof_pos[:, self.right_knee_idx]
        out_of_limits = torch.clamp(left_knee - knee_upper_limit, min=0) + torch.clamp(
            right_knee - knee_upper_limit, min=0
        )
        return out_of_limits

    # ------------ 対称性・振動抑制（V5追加）----------------

    def _reward_symmetry(self) -> torch.Tensor:
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
            torch.square(left_roll - (-right_roll))  # rollは符号反転で対称
            + torch.square(left_knee - right_knee)
            + torch.square(left_ankle - right_ankle)
        )

        # ガウシアン型報酬（誤差が小さいほど報酬が高い）
        return torch.exp(-symmetry_error / 0.5)

    def _reward_symmetry_range(self) -> torch.Tensor:
        """hip_pitch可動域の左右対称性報酬（V17追加）

        V16問題: hip_pitch可動域 L=0.524 vs R=0.197 rad（2.7倍の非対称）
        既存の_reward_symmetryはhip_pitchを除外しているため、この問題に対処できない。

        【設計原理】
        - 既存symmetry報酬はhip_pitchを除外（交互歩行で逆位相が理想のため）
        - 本報酬は「振幅」の対称性を評価（位相ではなく可動域の大きさ）
        - 両脚が同程度の可動域で動くことを促進
        - 片脚だけ大きく動く局所最適を回避

        【参考文献】
        - Leveraging Symmetry in RL-based Legged Locomotion Control (IROS 2024)
        - exp007_report_v16.md: 根本原因分析
        """
        left_hip_pitch = self.dof_pos[:, self.left_hip_pitch_idx]
        right_hip_pitch = self.dof_pos[:, self.right_hip_pitch_idx]

        # デフォルトからの偏差（振幅の指標）
        left_deviation = torch.abs(left_hip_pitch - self.default_dof_pos[self.left_hip_pitch_idx])
        right_deviation = torch.abs(right_hip_pitch - self.default_dof_pos[self.right_hip_pitch_idx])

        # 偏差の差をペナルティ（対称なら0）
        deviation_diff = torch.abs(left_deviation - right_deviation)

        # 指数型報酬（対称時に最大1.0）
        sigma = 0.1  # rad
        return torch.exp(-deviation_diff / sigma)

    # ------------ 速度二乗EMA対称性（V12 exp009追加）----------------

    def _reward_symmetry_vel_ema(self) -> torch.Tensor:
        """左右関節の運動エネルギー均等性報酬（V12 exp009追加）

        交互歩行では各瞬間に左右の関節角度は異なるため、瞬時の関節位置対称性
        （_reward_symmetry）は同位相同期（うさぎジャンプ）を誘導するリスクがある。

        本報酬は関節速度の二乗の指数移動平均（EMA）を用いて時間平均運動エネルギーの
        左右均等性を評価する。対称な交互歩行では1歩行周期の時間平均で
        ∫dq_L²dt = ∫dq_R²dt が成立するため、位相に依存しない対称性指標となる。

        【設計原理】
        - 速度二乗 = 運動エネルギーの代理指標（質量は左右同一なので省略可）
        - EMAの時定数 ≈ 1歩行周期（α=0.03, 50Hz制御, 1.5Hz歩容で約33ステップ）
        - 正規化により歩行速度に依存しない相対的対称性を評価
        - hip_pitchを含む全関節ペア（roll, pitch, knee, ankle）を評価対象

        【既存報酬との差異】
        - _reward_symmetry: 瞬時位置対称性 → 同位相同期リスク
        - _reward_symmetry_range: hip_pitch振幅のみ → race to bottom
        - 本報酬: 時間平均運動エネルギー均等性 → 位相不問かつ全関節対応

        【参考文献】
        - exp009_report_v10.md: 非対称性の二層構造分析
        - Leveraging Symmetry in RL-based Legged Locomotion Control (IROS 2024)
        """
        alpha: float = self.reward_cfg.get("symmetry_ema_alpha", 0.03)
        sigma: float = self.reward_cfg.get("symmetry_ema_sigma", 2.0)
        epsilon = 1e-6

        # EMA更新: vel_sq_ema = (1-α) * vel_sq_ema + α * dof_vel²
        vel_sq = self.dof_vel**2
        self.vel_sq_ema.mul_(1.0 - alpha).add_(vel_sq, alpha=alpha)

        # 左右関節ペア: (left_idx, right_idx)
        pairs = [
            (self.left_hip_roll_idx, self.right_hip_roll_idx),
            (self.left_hip_pitch_idx, self.right_hip_pitch_idx),
            (self.left_knee_idx, self.right_knee_idx),
            (self.left_ankle_idx, self.right_ankle_idx),
        ]

        # 正規化された左右差の二乗和
        total_error = torch.zeros(self.num_envs, dtype=self.vel_sq_ema.dtype, device=self.vel_sq_ema.device)
        for left_idx, right_idx in pairs:
            ema_l = self.vel_sq_ema[:, left_idx]
            ema_r = self.vel_sq_ema[:, right_idx]
            # 正規化: (L-R)² / (0.5*(L+R)+ε)² → スケール不変の相対的非対称度
            mean_ema = 0.5 * (ema_l + ema_r) + epsilon
            normalized_diff = (ema_l - ema_r) / mean_ema
            total_error += normalized_diff**2

        return torch.exp(-total_error / sigma)

    # ------------ 交互歩行・足裏制御（V7追加）----------------

    def _reward_alternating_gait(self) -> torch.Tensor:
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

    def _reward_foot_flat(self) -> torch.Tensor:
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
        left_ankle_dev = torch.abs(self.dof_pos[:, self.left_ankle_idx] - self.default_dof_pos[self.left_ankle_idx])
        right_ankle_dev = torch.abs(self.dof_pos[:, self.right_ankle_idx] - self.default_dof_pos[self.right_ankle_idx])

        # 接地中の足のみを対象（スイング相は除外）
        left_penalty = left_ankle_dev * contacts[:, 0].float()
        right_penalty = right_ankle_dev * contacts[:, 1].float()

        # 二乗誤差
        return torch.square(left_penalty) + torch.square(right_penalty)

    def _reward_step_length(self) -> torch.Tensor:
        """歩幅報酬（V7追加、exp009で多方向対応に汎化）

        コマンド方向への足間距離を報酬化し、大股歩行を促進。

        【設計原理】
        - 遊脚時に足がコマンド方向に大きく振り出されることを報酬
        - 両足が近い位置にある（小刻み歩行）状態を抑制

        【多方向対応（exp009）】
        X軸固定からコマンド方向への射影に変更。横歩き・後退でも機能。
        exp008以前（Y=0）の場合は旧実装と同一結果を返す。
        """
        cmd_xy = self.commands[:, :2]
        cmd_mag = torch.norm(cmd_xy, dim=1)
        is_moving = cmd_mag > 0.05
        cmd_dir = cmd_xy / (cmd_mag.unsqueeze(1) + 1e-8)

        foot_delta_x = self.feet_pos[:, 0, 0] - self.feet_pos[:, 1, 0]
        foot_delta_y = self.feet_pos[:, 0, 1] - self.feet_pos[:, 1, 1]
        foot_dist_along_cmd = torch.abs(foot_delta_x * cmd_dir[:, 0] + foot_delta_y * cmd_dir[:, 1])

        return foot_dist_along_cmd * is_moving.float()

    # ------------ 交互歩行改善・胴体安定化（V8追加）----------------

    def _reward_hip_pitch_antiphase(self) -> torch.Tensor:
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

        # 移動コマンド時のみ適用（XY速度ノルム > 0.1 m/s、多方向対応）
        is_moving = torch.norm(self.commands[:, :2], dim=1) > 0.1

        # 速度の積が負のとき報酬（逆相関）
        # tanh関数でスケーリングして滑らかな報酬に
        reward = -torch.tanh(velocity_product / 0.1)

        return reward * is_moving.float()

    def _reward_hip_yaw_pos(self) -> torch.Tensor:
        """hip_yaw位置ペナルティ（exp008 V3追加）

        hip_posのcompound penalty（hip_yaw²+hip_roll²）を分離し、
        hip_yaw²のみをペナルティ化する。

        【設計原理】
        - hip_posはhip_yawとhip_rollを同時にペナルティ化していたが、
          narrow URDFではhip_rollの必要自由度が増加するため構造的矛盾が発生
        - hip_yaw制約はYawドリフト抑制に不可欠（V2で実証）
        - hip_roll制御はankle_rollに委任し、独立制御を実現

        【参考文献】
        exp008_report_v2.md: compound penalty構造の問題を壊滅的に実証
        """
        left_yaw = self.dof_pos[:, self.left_hip_yaw_idx]
        right_yaw = self.dof_pos[:, self.right_hip_yaw_idx]

        return torch.square(left_yaw) + torch.square(right_yaw)

    def _reward_ankle_roll(self) -> torch.Tensor:
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

    def _reward_stance_hip_roll_target(self) -> torch.Tensor:
        """接地相限定hip_rollペナルティ（exp008 V4追加, V8拡張: ターゲット角度対応）

        接地脚のみに(hip_roll - target)²ペナルティを適用する。

        【設計原理】
        - V4: hip_roll²で内股offset ±15°が持続 → 0°ターゲットでは矯正力不足
        - V8: 明示的な外向きターゲット角度を設定し、(hip_roll - target)²で直接制約
        - ターゲット角度はreward_cfg["stance_hip_roll_target_angle"]で設定（rad）
        - L脚: +target（外向き）, R脚: -target（外向き）の符号規則
        - target=0の場合はV4と同一動作（後方互換）

        【参考文献】
        exp008_report_v7.md: 代替案B（ターゲット角度設定）
        """
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        left_roll = self.dof_pos[:, self.left_hip_roll_idx]
        right_roll = self.dof_pos[:, self.right_hip_roll_idx]

        # ターゲット角度: L=+target(外向き), R=-target(外向き)
        target = self.stance_hip_roll_target_angle
        penalty = (
            torch.square(left_roll - target) * contacts[:, 0].float()
            + torch.square(right_roll + target) * contacts[:, 1].float()
        )

        return penalty

    def _reward_stance_foot_lateral_position(self) -> torch.Tensor:
        """接地足横方向位置ペナルティ（exp008 V12追加）

        接地足が胴体中心に近すぎる場合にペナルティを適用する。
        関節角度（hip_roll）ではなく足位置を直接制約することで、
        FK鎖全体（hip_roll, hip_pitch, knee等の組合せ）を活用した
        内股解消の新経路を提供する。

        【設計原理】
        - V4-V11: hip_roll角度ペナルティでは飽和域（99.8%）で壁に当たり突破不可能
        - V12: 足の横方向位置を直接ペナルティ化（タスク空間ペナルティ）
        - 片側制約: 最小距離以下のみペナルティ（それ以上は完全に自由）
        - 接地相のみ適用（遊脚は自由に動ける）
        - 距離を閾値で正規化し、(1 - dist/threshold)²形式で0~1の範囲にスケーリング

        【ペナルティ計算】
        - 各足の横方向距離: |foot_y - base_y|（ワールドフレーム）
        - 閾値: stance_foot_lateral_min_distance (default: 0.050m)
        - 正規化不足量: max(1 - lateral_distance / min_distance, 0)
        - ペナルティ: 正規化不足量²（0~1の範囲、閾値で0、胴体直下で1）

        【stance_hip_roll_targetとの違い】
        - stance_hip_roll_target: hip_roll²（間接指標、1関節のみ制約）
        - stance_foot_lateral_position: 足横位置（直接指標、全関節で達成可能）

        【参考文献】
        exp008_report_v11.md: パラダイムシフト提案（関節角度→足位置）
        """
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # 各足の横方向距離（胴体中心からの距離）
        left_lateral_dist = torch.abs(self.feet_pos[:, 0, 1] - self.base_pos[:, 1])
        right_lateral_dist = torch.abs(self.feet_pos[:, 1, 1] - self.base_pos[:, 1])

        # 正規化された片側制約: (1 - dist/threshold)² で0~1の範囲
        min_dist = self.stance_foot_lateral_min_distance
        left_deficit_norm = torch.clamp(1.0 - left_lateral_dist / min_dist, min=0.0)
        right_deficit_norm = torch.clamp(1.0 - right_lateral_dist / min_dist, min=0.0)

        # 接地脚のみペナルティ適用
        return (
            torch.square(left_deficit_norm) * contacts[:, 0].float()
            + torch.square(right_deficit_norm) * contacts[:, 1].float()
        )

    # ------------ 対称性・両脚動作強化（V9追加）----------------

    def _reward_hip_pitch_antiphase_v2(self) -> torch.Tensor:
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

        # 移動コマンド時のみ適用（XY速度ノルム > 0.1 m/s、多方向対応）
        is_moving = torch.norm(self.commands[:, :2], dim=1) > 0.1
        return reward * is_moving.float()

    def _reward_both_legs_active(self) -> torch.Tensor:
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

        # 移動コマンド時のみ適用（XY速度ノルム > 0.1 m/s、多方向対応）
        is_moving = torch.norm(self.commands[:, :2], dim=1) > 0.1
        return reward * is_moving.float()

    # ------------ 足引きずり抑制（V10追加）----------------

    def _reward_feet_stumble(self) -> torch.Tensor:
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

    # ------------ 関節速度制限（V18追加）----------------

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

    # ------------ hip_pitch可動域促進（V19追加）----------------

    def _reward_hip_pitch_range(self) -> torch.Tensor:
        """hip_pitch可動域の大きさを報酬（V19追加）

        【設計意図】
        - V18課題: hip_pitch可動域が小さい（0.368-0.432 rad）
        - 両脚のhip_pitchがデフォルトから大きく動くことを報酬化
        - symmetry_rangeと異なり、「動いている」ことを前提とした報酬

        【V17 symmetry_rangeとの違い】
        - symmetry_range: 左右の偏差の差を評価（静止でも最大報酬）
        - hip_pitch_range: 偏差の大きさ自体を報酬（動かないと報酬0）

        【参考文献】
        - exp007_report_v17.md: 静止局所解問題の分析
        - exp007_report_v18.md: ストライド不足の分析
        """
        left_hip = self.dof_pos[:, self.left_hip_pitch_idx]
        right_hip = self.dof_pos[:, self.right_hip_pitch_idx]

        # デフォルトからの偏差（可動域の大きさ）
        left_range = torch.abs(left_hip - self.default_dof_pos[self.left_hip_pitch_idx])
        right_range = torch.abs(right_hip - self.default_dof_pos[self.right_hip_pitch_idx])

        # 両脚の平均可動域を報酬（動かないと0、大きく動くと大きな報酬）
        avg_range = (left_range + right_range) / 2

        # 目標可動域を0.3 radとし、それ以上で最大報酬
        target_range = 0.3
        return torch.clamp(avg_range / target_range, max=1.0)

    # ------------ 遊脚時足首角度制限（V22a追加）----------------

    def _reward_ankle_pitch_range(self) -> torch.Tensor:
        """遊脚時のankle_pitch角度制限ペナルティ（V22a追加）

        【設計原理】
        V20課題: つま先を地面に突く動作が発生
        原因: ankle_pitchの可動域が大きい（L: 0.484 rad, R: 0.369 rad）
        対策: 遊脚（非接地）時のankle_pitch角度を制限

        つま先が下を向く（ankle_pitchが正方向に大きくなる）動きを抑制することで、
        足を下ろす際の姿勢を改善する。

        【V7 foot_flatとの違い】
        - foot_flat: 接地中の足首傾きをペナルティ（足裏を水平に保つ）
        - ankle_pitch_range: 遊脚時の足首傾きをペナルティ（つま先が下を向く動きを抑制）

        【参考文献】
        - exp007_report_v20.md: 次バージョンへの提案A
        - exp007_unitree_rl_gym_survey.md Section 7.5.1 Barrier-Based Style Rewards
        """
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        ankle_pitch_limit = self.reward_cfg.get("ankle_pitch_limit", 0.3)  # rad

        # ankle_pitchのデフォルトからの偏差
        left_ankle_dev = torch.abs(self.dof_pos[:, self.left_ankle_idx] - self.default_dof_pos[self.left_ankle_idx])
        right_ankle_dev = torch.abs(self.dof_pos[:, self.right_ankle_idx] - self.default_dof_pos[self.right_ankle_idx])

        # 許容範囲を超えた分をペナルティ
        left_excess = torch.clamp(left_ankle_dev - ankle_pitch_limit, min=0)
        right_excess = torch.clamp(right_ankle_dev - ankle_pitch_limit, min=0)

        # 遊脚（非接地）時のみペナルティ
        left_penalty = torch.square(left_excess) * (~contacts[:, 0]).float()
        right_penalty = torch.square(right_excess) * (~contacts[:, 1]).float()

        return left_penalty + right_penalty

    # ------------ 遊脚直交速度ペナルティ（V24追加）----------------

    def _reward_swing_foot_lateral_velocity(self) -> torch.Tensor:
        """遊脚の速度指令直交成分ペナルティ（V24追加）

        【設計原理】
        V23課題: 遊脚が体の内側を通る軌道（内股）になっている
        原因: hip rollが脱力気味で、エネルギー最小化のために内側を通る軌道が選択される
        対策: 遊脚（非接地）時の速度指令と直交する速度成分をペナルティ化

        速度指令ベクトル cmd = (cmd_x, cmd_y) に対して、
        遊脚速度 vel = (vel_x, vel_y) の直交成分をペナルティ化する。

        数学的には:
          cmd_dir = cmd / |cmd|  (指令方向の単位ベクトル)
          vel_parallel = vel · cmd_dir  (指令方向成分)
          |vel_perp|² = |vel|² - vel_parallel²  (直交成分の二乗)

        これにより、前進コマンド時はY軸成分が、横移動コマンド時はX軸成分が、
        斜め移動時は指令方向と直交する成分がペナルティ化される。
        前後左右自由自在に歩けるポリシーを学習できる。

        【contact_no_velとの違い】
        - contact_no_vel: 接地中の足速度をペナルティ（足滑り防止）
        - swing_foot_lateral_velocity: 遊脚時の指令直交速度をペナルティ（内股防止）

        【参考文献】
        - exp007_report_v23.md: 内股軌道の課題分析
        - ユーザー提案: 遊脚速度成分の方向制御
        """
        contacts = self._get_foot_contacts()
        if contacts is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # 速度指令ベクトル (cmd_x, cmd_y)
        cmd_x = self.commands[:, 0]  # shape: (num_envs,)
        cmd_y = self.commands[:, 1]  # shape: (num_envs,)

        # 指令ベクトルの大きさ
        cmd_norm = torch.sqrt(cmd_x**2 + cmd_y**2 + 1e-8)  # ゼロ除算防止

        # 移動コマンドかどうか（静止時はペナルティなし）
        is_moving = cmd_norm > 0.05

        # 指令方向の単位ベクトル
        cmd_dir_x = cmd_x / cmd_norm  # shape: (num_envs,)
        cmd_dir_y = cmd_y / cmd_norm  # shape: (num_envs,)

        # 遊脚速度ベクトル (vel_x, vel_y) - 左右の足
        vel_x = self.feet_vel[:, :, 0]  # shape: (num_envs, 2)
        vel_y = self.feet_vel[:, :, 1]  # shape: (num_envs, 2)

        # 速度ベクトルの大きさの二乗
        vel_sq = vel_x**2 + vel_y**2  # shape: (num_envs, 2)

        # 指令方向成分（内積）: vel · cmd_dir
        # cmd_dir を (num_envs, 1) に拡張して計算
        vel_parallel = vel_x * cmd_dir_x.unsqueeze(1) + vel_y * cmd_dir_y.unsqueeze(1)

        # 直交成分の二乗: |vel_perp|² = |vel|² - vel_parallel²
        vel_perp_sq = vel_sq - vel_parallel**2
        # 数値誤差で負になる可能性があるのでclamp
        vel_perp_sq = torch.clamp(vel_perp_sq, min=0)

        # 遊脚（非接地）時のみペナルティ
        swing_mask = (~contacts).float()
        penalty = torch.sum(vel_perp_sq * swing_mask, dim=1)

        # 移動コマンド時のみ適用（静止コマンド時は報酬0）
        return penalty * is_moving.float()

    def _reward_foot_lateral_velocity(self) -> torch.Tensor:
        """足先の速度指令直交成分ペナルティ - 全相適用（exp008 V7追加）

        【設計原理】
        _reward_swing_foot_lateral_velocityの拡張版。
        遊脚のみでなく全フェーズ（遊脚+接地脚）で
        速度指令と直交する速度成分をペナルティ化する。

        【設計意図】
        exp008 V6課題: 内股着地が接地相での横方向速度成分にも起因する可能性
        → 遊脚マスクを除去し、全相でvel_perp²をペナルティ化

        【swing_foot_lateral_velocityとの違い】
        - swing_foot_lateral_velocity: 遊脚時のみ適用（swing_mask使用）
        - foot_lateral_velocity: 全フェーズで適用（マスクなし）

        【参考文献】
        - exp008_report_v6.md: 次バージョンへの提案（推奨案）
        """
        # 速度指令ベクトル (cmd_x, cmd_y)
        cmd_x = self.commands[:, 0]
        cmd_y = self.commands[:, 1]

        cmd_norm = torch.sqrt(cmd_x**2 + cmd_y**2 + 1e-8)
        is_moving = cmd_norm > 0.05

        cmd_dir_x = cmd_x / cmd_norm
        cmd_dir_y = cmd_y / cmd_norm

        vel_x = self.feet_vel[:, :, 0]
        vel_y = self.feet_vel[:, :, 1]

        vel_sq = vel_x**2 + vel_y**2
        vel_parallel = vel_x * cmd_dir_x.unsqueeze(1) + vel_y * cmd_dir_y.unsqueeze(1)

        vel_perp_sq = vel_sq - vel_parallel**2
        vel_perp_sq = torch.clamp(vel_perp_sq, min=0)

        # swing_maskなし: 全フェーズで適用
        penalty = torch.sum(vel_perp_sq, dim=1)

        return penalty * is_moving.float()

    # ------------ スイング期間報酬（V29追加）----------------

    def _reward_swing_duration(self) -> torch.Tensor:
        """スイング期間中の累積報酬（V29追加）

        【設計原理】
        V28課題: feet_air_time報酬のfirst_contact構造がタップダンスを促進している
        原因: first_contactは接地開始時にのみTrueになり、同じ足で複数回接地すると
              複数回報酬計算が行われる。短い滞空でも「接地」すれば報酬が発生するため、
              「足を上げてすぐ下ろす」動作が学習される。

        対策: 「空中にいる間ずっと報酬」方式を導入
        - 遊脚が空中にいる時間がair_time_offsetを超えた場合に報酬
        - 毎ステップ報酬を与えることで、長い滞空を促進
        - first_contactによる「接地直後の一回だけ報酬」ではないため、
          タップダンス（短い滞空→接地→短い滞空→接地）のメリットがなくなる

        【feet_air_timeとの違い】
        - feet_air_time: 接地時に滞空時間に応じた報酬を一度だけ付与（first_contact構造）
        - swing_duration: 空中にいる間ずっと報酬を付与（毎ステップ）

        【参考文献】
        - exp007_report_v28.md: first_contact構造の問題分析
        - exp007_unitree_rl_gym_survey.md: 足上げ高さ報酬の設計
        """
        if self.feet_air_time is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # 移動コマンドがある場合のみ
        is_moving = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()

        # 空中にいる時間がair_time_offsetを超えた場合に報酬
        # clampで負の値を0にして、air_time_offset未満の滞空では報酬なし
        reward_per_foot = torch.clamp(self.feet_air_time - self.air_time_offset, min=0.0)

        # 両足の報酬を合計
        reward = torch.sum(reward_per_foot, dim=1) * is_moving

        return reward

    # ------------ スイング接地ペナルティ（V30追加）----------------

    def _reward_swing_contact_penalty(self) -> torch.Tensor:
        """スイング位相中の接地ペナルティ（V30追加）

        【設計原理】
        V29課題: swing_duration報酬が完全に機能しなかった（報酬値0.0000）
        原因: air_time_offset=0.25秒がBSL-Droidでは達成困難
        V29結果: 片足接地率が87.8%→75.8%に悪化（タップダンス悪化）

        対策: スイング位相中の接地を直接ペナルティ化
        - swing_durationと併用し、「空中維持」と「接地抑制」の両面から誘導
        - スイング位相（leg_phase >= 0.55）中に接地した場合にペナルティ

        【参考文献】
        - exp007_report_v29.md: 提案B（swing_contact_penalty追加）
        """
        if self.contact_state is None:
            return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        # 移動コマンドがある場合のみ
        is_moving = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()

        penalty = torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)

        for i in range(2):  # 両足
            # スイング位相の判定（位相が0.55以上）
            is_swing_phase = self.leg_phase[:, i] >= 0.55
            # 接地状態の判定
            is_contact = self.contact_state[:, i] > 0.5
            # スイング位相中に接地 = ペナルティ
            penalty += (is_swing_phase & is_contact).float()

        return penalty * is_moving
