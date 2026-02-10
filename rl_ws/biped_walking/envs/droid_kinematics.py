"""BSL-Droid Simplified の運動学計算モジュール

順運動学（FK）と逆運動学（IK）を提供する。
パラメータはbsl_droid_simplified.urdf.xacroから抽出。

座標系定義:
- X軸: 前方（正）
- Y軸: 左方（正）
- Z軸: 上方（正）

脚構造（5 DOF）:
  hip_yaw_joint   (Z軸回転)
    └─ hip_roll_joint  (X軸回転)
         └─ hip_pitch_joint (Y軸回転)
              └─ knee_pitch_joint (Y軸回転、逆関節：負で後方屈曲)
                   └─ ankle_pitch_joint (Y軸回転)
"""

from __future__ import annotations

import math

import torch


class DroidKinematics:
    """BSL-Droid Simplified の運動学計算クラス"""

    def __init__(self, device: str = "cpu") -> None:
        """
        Args:
            device: 計算デバイス ('cpu' or 'cuda')
        """
        self.device: str = device

        # URDFから抽出したパラメータ [m]
        self.hip_offset_y = 0.10  # 胴体中心から股関節ヨー軸まで
        self.hip_yaw_length = 0.025  # ヨー軸リンク長（下向き）
        self.hip_roll_length = 0.03  # ロール軸リンク長（外側へ）
        self.thigh_length = 0.11  # 大腿部長さ
        self.shank_length = 0.12  # 下腿部長さ
        self.foot_height = 0.035  # 足部高さ
        self.ankle_offset_x = 0.02  # 足首から足先中心へのオフセット

        # 脚の到達範囲
        self.max_reach = self.thigh_length + self.shank_length - 0.005  # 0.225m
        self.min_reach = abs(self.thigh_length - self.shank_length) + 0.005  # 0.015m

        # 関節限界 [rad]
        self.joint_limits = {
            "hip_yaw": (-0.524, 0.524),  # ±30°
            "hip_roll": (-0.436, 0.436),  # ±25°
            "hip_pitch": (-2.094, 1.571),  # -120° ~ +90°
            "knee_pitch": (-2.618, 0.0),  # -150° ~ 0°（逆関節）
            "ankle_pitch": (-1.571, 1.571),  # ±90°
        }

        # デフォルト関節角度（立位姿勢）[rad]
        self.default_joint_angles = {
            "hip_yaw": 0.0,
            "hip_roll": 0.0,
            "hip_pitch": 1.047,  # 60°
            "knee_pitch": -1.745,  # -100°
            "ankle_pitch": 0.785,  # 45°
        }

    def forward_kinematics(self, joint_angles: torch.Tensor, is_left: bool = True) -> torch.Tensor:
        """
        順運動学: 関節角度 → 足先位置（hip_pitch起点からの相対位置）

        Args:
            joint_angles: torch.Tensor (5,) または (batch, 5)
                [hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch]
            is_left: 左脚かどうか

        Returns:
            foot_pos: torch.Tensor (3,) または (batch, 3)
                足先位置 [x, y, z]（hip_pitch起点、Z上向き正）
        """
        # バッチ処理対応
        if joint_angles.dim() == 1:
            joint_angles = joint_angles.unsqueeze(0)
            squeeze_output = True
        else:
            squeeze_output = False

        hip_yaw = joint_angles[:, 0]
        hip_roll = joint_angles[:, 1]
        hip_pitch = joint_angles[:, 2]
        knee_pitch = joint_angles[:, 3]
        joint_angles[:, 4]

        # XZ平面での計算（サジタル面）
        # 膝位置（大腿先端）
        knee_x = self.thigh_length * torch.sin(hip_pitch)
        knee_z = -self.thigh_length * torch.cos(hip_pitch)  # 下向き負

        # 足首位置（下腿先端）
        cumulative_pitch = hip_pitch + knee_pitch
        ankle_x = knee_x + self.shank_length * torch.sin(cumulative_pitch)
        ankle_z = knee_z - self.shank_length * torch.cos(cumulative_pitch)

        # 足先位置（足首から足裏まで）
        foot_x = ankle_x + self.ankle_offset_x
        foot_z = ankle_z - self.foot_height

        # hip_roll による Y-Z平面での回転
        # hip_roll_linkはhip_roll関節からY方向に伸びており、
        # hip_roll回転によりY-Z平面で回転する
        side = 1.0 if is_left else -1.0
        foot_y = side * self.hip_roll_length * torch.cos(hip_roll)
        # Z方向へのオフセット（hip_rollが0でない場合の影響）
        foot_z = foot_z - side * self.hip_roll_length * torch.sin(hip_roll)

        # hip_yaw による回転（XY平面）
        foot_x_rotated = foot_x * torch.cos(hip_yaw) - foot_y * torch.sin(hip_yaw)
        foot_y_rotated = foot_x * torch.sin(hip_yaw) + foot_y * torch.cos(hip_yaw)

        foot_pos = torch.stack([foot_x_rotated, foot_y_rotated, foot_z], dim=1)

        if squeeze_output:
            foot_pos = foot_pos.squeeze(0)

        return foot_pos

    def inverse_kinematics(self, foot_pos: torch.Tensor, is_left: bool = True) -> torch.Tensor:
        """
        逆運動学: 足先位置 → 関節角度（3D対応）

        FKの逆変換:
        1. hip_yaw回転の逆変換でローカル座標を求める
        2. XZ平面で2リンクIKを解く

        Args:
            foot_pos: torch.Tensor (3,) または (batch, 3)
                足先位置 [x, y, z]（hip_pitch起点、Z上向き正）
            is_left: 左脚かどうか

        Returns:
            joint_angles: torch.Tensor (5,) または (batch, 5)
                [hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch]
        """
        # バッチ処理対応
        if foot_pos.dim() == 1:
            foot_pos = foot_pos.unsqueeze(0)
            squeeze_output = True
        else:
            squeeze_output = False

        batch_size = foot_pos.shape[0]
        device = foot_pos.device

        p_x = foot_pos[:, 0]
        p_y = foot_pos[:, 1]
        p_z = foot_pos[:, 2]

        # hip_roll_length: hip_pitch関節からの横方向オフセット
        side = 1.0 if is_left else -1.0
        default_y = side * self.hip_roll_length  # デフォルトのY位置（hip_roll=0時）

        # hip_yawの計算
        # FKでは: foot_x_rotated = foot_x * cos(yaw) - foot_y * sin(yaw)
        #         foot_y_rotated = foot_x * sin(yaw) + foot_y * cos(yaw)
        # ここで foot_y = default_y (hip_roll=0の場合)
        #
        # IKでは yaw を逆算: yaw = atan2(p_y - foot_x*sin(yaw), p_x)...複雑
        #
        # 近似アプローチ: yaw角が小さい場合、foot_x ≈ p_x
        # したがって: yaw ≈ atan2(p_y - default_y, p_x) は不正確
        #
        # 正確な解法: atan2(p_y, p_x) = atan2(foot_y_rotated, foot_x_rotated)
        # これは回転後の角度を与える
        # 元のfoot_x, foot_yに戻すには逆回転が必要
        #
        # 幾何学的関係:
        # p_x = foot_x * cos(yaw) - default_y * sin(yaw)
        # p_y = foot_x * sin(yaw) + default_y * cos(yaw)
        #
        # 解くと:
        # foot_x = p_x * cos(yaw) + p_y * sin(yaw)  (逆回転)
        # default_y = -p_x * sin(yaw) + p_y * cos(yaw)
        #
        # default_yは既知なので、yawを求める:
        # p_y * cos(yaw) - p_x * sin(yaw) = default_y
        # sqrt(p_x^2 + p_y^2) * sin(atan2(p_y, p_x) - yaw) = default_y
        #
        # 別アプローチ: p_x^2 + p_y^2 = foot_x^2 + default_y^2
        # foot_x = sqrt(p_x^2 + p_y^2 - default_y^2)

        r_sq = p_x**2 + p_y**2
        default_y_sq = default_y**2

        # foot_x^2 = r^2 - default_y^2 (負になる場合はクリップ)
        foot_x_sq = torch.clamp(r_sq - default_y_sq, min=0.0001)
        foot_x = torch.sqrt(foot_x_sq)

        # yawの計算: atan2で回転角を求める
        # p_y = foot_x * sin(yaw) + default_y * cos(yaw)
        # p_x = foot_x * cos(yaw) - default_y * sin(yaw)
        #
        # これは foot_x, default_y の回転なので:
        # yaw = atan2(p_y, p_x) - atan2(default_y, foot_x)
        hip_yaw = torch.atan2(p_y, p_x) - torch.atan2(torch.full_like(p_x, default_y), foot_x)

        # 足首位置に変換（足先から逆算）
        ankle_x = foot_x - self.ankle_offset_x
        ankle_z = p_z + self.foot_height  # Z上向き正のまま

        # 2リンク平面IK（XZ平面）
        d_sq = ankle_x**2 + ankle_z**2
        d = torch.sqrt(d_sq)

        L1 = self.thigh_length
        L2 = self.shank_length

        # 到達可能性クリップ
        d = torch.clamp(d, self.min_reach, self.max_reach)
        d_sq = d**2

        # 膝角度（余弦定理）
        cos_knee_inner = (L1**2 + L2**2 - d_sq) / (2 * L1 * L2)
        cos_knee_inner = torch.clamp(cos_knee_inner, -1.0, 1.0)
        knee_inner = torch.acos(cos_knee_inner)

        # 逆関節: 膝角度は常に負（後方屈曲）
        knee_pitch = -(math.pi - knee_inner)

        # 股関節ピッチ角度
        sin_beta = L2 * torch.sin(knee_inner) / d
        sin_beta = torch.clamp(sin_beta, -1.0, 1.0)
        beta = torch.asin(sin_beta)

        alpha = torch.atan2(ankle_x, -ankle_z)
        hip_pitch = alpha + beta

        # 足首角度（足裏を地面に平行に維持）
        ankle_pitch = -(hip_pitch + knee_pitch)

        # hip_roll: 現状は0固定（歩行時は体幹で姿勢を維持）
        hip_roll = torch.zeros(batch_size, device=device)

        # 関節限界でクリップ
        hip_yaw = torch.clamp(hip_yaw, *self.joint_limits["hip_yaw"])
        hip_roll = torch.clamp(hip_roll, *self.joint_limits["hip_roll"])
        hip_pitch = torch.clamp(hip_pitch, *self.joint_limits["hip_pitch"])
        knee_pitch = torch.clamp(knee_pitch, *self.joint_limits["knee_pitch"])
        ankle_pitch = torch.clamp(ankle_pitch, *self.joint_limits["ankle_pitch"])

        joint_angles = torch.stack([hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch], dim=1)

        if squeeze_output:
            joint_angles = joint_angles.squeeze(0)

        return joint_angles

    def inverse_kinematics_xz(self, foot_xz: torch.Tensor, is_left: bool = True) -> torch.Tensor:
        """
        簡略化IK: 足先XZ位置のみから関節角度を計算

        Args:
            foot_xz: torch.Tensor (2,) または (batch, 2)
                足先位置 [x, z]（hip_pitch起点、Z上向き正）
            is_left: 左脚かどうか

        Returns:
            joint_angles: torch.Tensor (5,) または (batch, 5)
        """
        # バッチ処理対応
        if foot_xz.dim() == 1:
            foot_xz = foot_xz.unsqueeze(0)
            squeeze_output = True
        else:
            squeeze_output = False

        batch_size = foot_xz.shape[0]
        device = foot_xz.device

        # Y座標は0として3D位置を構築
        foot_pos = torch.zeros(batch_size, 3, device=device)
        foot_pos[:, 0] = foot_xz[:, 0]  # x
        foot_pos[:, 2] = foot_xz[:, 1]  # z

        joint_angles = self.inverse_kinematics(foot_pos, is_left)

        if squeeze_output:
            joint_angles = joint_angles.squeeze(0)

        return joint_angles

    def get_default_foot_position(self, is_left: bool = True) -> torch.Tensor:
        """
        デフォルト姿勢での足先位置を取得

        Returns:
            foot_pos: torch.Tensor (3,)
        """
        default_angles = torch.tensor(
            [
                self.default_joint_angles["hip_yaw"],
                self.default_joint_angles["hip_roll"],
                self.default_joint_angles["hip_pitch"],
                self.default_joint_angles["knee_pitch"],
                self.default_joint_angles["ankle_pitch"],
            ],
            device=self.device,
        )

        return self.forward_kinematics(default_angles, is_left)

    def get_workspace_bounds(self) -> dict[str, float]:
        """
        足先の作業空間の境界を取得（近似）

        Returns:
            bounds: dict with 'x_min', 'x_max', 'z_min', 'z_max'
        """
        # 簡易的な境界（実際の可動範囲より保守的）
        return {
            "x_min": -0.15,  # 後方
            "x_max": 0.15,  # 前方
            "z_min": -0.25,  # 下方（伸展時）
            "z_max": -0.08,  # 上方（屈曲時）
        }


def test_fk_ik_consistency():
    """FK-IK往復検証"""
    kin = DroidKinematics()

    # テストケース: 様々な関節角度
    test_angles_list = [
        [0, 0, 0.5, -1.0, 0.5],  # 中間姿勢
        [0, 0, 1.0, -1.5, 0.5],  # 前傾
        [0, 0, 0.3, -0.8, 0.5],  # 後傾
        [0, 0, 1.047, -1.745, 0.785],  # デフォルト姿勢
    ]

    print("=== FK-IK Consistency Test ===")
    for i, angles in enumerate(test_angles_list):
        angles_tensor = torch.tensor(angles, dtype=torch.float32)
        foot_pos = kin.forward_kinematics(angles_tensor)
        recovered_angles = kin.inverse_kinematics(foot_pos)
        recovered_pos = kin.forward_kinematics(recovered_angles)

        pos_error = torch.norm(foot_pos - recovered_pos).item()
        print(f"Test {i + 1}: pos_error = {pos_error * 1000:.3f} mm")
        print(f"  Original angles: {angles}")
        print(f"  Foot position:   [{foot_pos[0]:.4f}, {foot_pos[1]:.4f}, {foot_pos[2]:.4f}]")
        print(f"  Recovered:       [{recovered_pos[0]:.4f}, {recovered_pos[1]:.4f}, {recovered_pos[2]:.4f}]")

    # デフォルト姿勢の足先位置
    print("\n=== Default Foot Position ===")
    left_pos = kin.get_default_foot_position(is_left=True)
    right_pos = kin.get_default_foot_position(is_left=False)
    print(f"Left foot:  [{left_pos[0]:.4f}, {left_pos[1]:.4f}, {left_pos[2]:.4f}]")
    print(f"Right foot: [{right_pos[0]:.4f}, {right_pos[1]:.4f}, {right_pos[2]:.4f}]")


if __name__ == "__main__":
    test_fk_ik_consistency()
