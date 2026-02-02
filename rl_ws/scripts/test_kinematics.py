"""DroidKinematics FK/IK 整合性検証スクリプト

droid_env_taskspace.pyがDroidKinematicsを正しく使用できることを検証する。
3D IK（hip_yaw対応）の検証を含む。
"""

import torch
import sys
sys.path.insert(0, '.')

from biped_walking.envs.droid_kinematics import DroidKinematics

def main():
    kin = DroidKinematics(device="cpu")

    print('=== DroidKinematics FK/IK 整合性検証 ===')
    print(f'Parameters: thigh={kin.thigh_length}m, shank={kin.shank_length}m')
    print(f'            foot_h={kin.foot_height}m, ankle_offset_x={kin.ankle_offset_x}m')
    print(f'Reach range: [{kin.min_reach:.4f}, {kin.max_reach:.4f}]m')
    print()

    # デフォルト姿勢の確認
    print('Test 1: デフォルト足先位置')
    left_pos = kin.get_default_foot_position(is_left=True)
    right_pos = kin.get_default_foot_position(is_left=False)
    print(f'  Left foot:  x={left_pos[0]:.4f}m, y={left_pos[1]:.4f}m, z={left_pos[2]:.4f}m')
    print(f'  Right foot: x={right_pos[0]:.4f}m, y={right_pos[1]:.4f}m, z={right_pos[2]:.4f}m')
    print()

    # FK→IK→FK ラウンドトリップ検証（hip_yaw=0, hip_roll=0）
    print('Test 2: FK→IK→FK ラウンドトリップ検証（hip_yaw=0, hip_roll=0）')
    test_angles_list = [
        ('default pose', [0, 0, 1.047, -1.745, 0.785]),
        ('forward lean', [0, 0, 0.8, -1.2, 0.4]),
        ('backward lean', [0, 0, 1.3, -2.0, 0.7]),
        ('small hip angles', [0, 0, 1.0, -1.5, 0.5]),
    ]

    all_passed = True
    for name, angles in test_angles_list:
        angles_tensor = torch.tensor(angles, dtype=torch.float32)
        
        # FK
        foot_pos = kin.forward_kinematics(angles_tensor, is_left=True)
        # IK
        recovered_angles = kin.inverse_kinematics(foot_pos, is_left=True)
        # FK again
        recovered_pos = kin.forward_kinematics(recovered_angles, is_left=True)
        
        error = torch.norm(foot_pos - recovered_pos).item() * 1000
        # 1mm以内なら許容（3D IKは近似のためやや緩め）
        status = 'PASS' if error < 1.0 else 'FAIL'
        if error >= 1.0:
            all_passed = False
        print(f'  {name}: error={error:.3f}mm [{status}]')
    print()

    # バッチ処理の検証
    print('Test 3: バッチ処理の検証')
    batch_size = 4
    # hip_yaw=0, hip_roll=0のケースのみ（IKの制約）
    batch_angles = torch.tensor([
        [0, 0, 1.047, -1.745, 0.785],
        [0, 0, 0.8, -1.2, 0.4],
        [0, 0, 1.3, -2.0, 0.7],
        [0, 0, 1.0, -1.5, 0.5],
    ], dtype=torch.float32)

    # バッチFK
    batch_foot_pos = kin.forward_kinematics(batch_angles, is_left=True)
    # バッチIK
    batch_recovered = kin.inverse_kinematics(batch_foot_pos, is_left=True)
    # バッチFK again
    batch_recovered_pos = kin.forward_kinematics(batch_recovered, is_left=True)

    batch_errors = torch.norm(batch_foot_pos - batch_recovered_pos, dim=1) * 1000
    for i, err in enumerate(batch_errors):
        status = 'PASS' if err.item() < 1.0 else 'FAIL'
        if err.item() >= 1.0:
            all_passed = False
        print(f'  Batch[{i}]: error={err.item():.3f}mm [{status}]')
    print()

    # 3D IK検証: 足先Y座標の変化に対するhip_yaw
    print('Test 4: 3D IK検証（hip_yawによるY座標対応）')
    # 注: hip_yawのみでY座標を変更する場合、到達可能範囲に制約がある
    # 左脚のdefault_y = 0.03m なので、y < 0 への到達は不可能
    # 到達可能な範囲内でテストする
    default_left = kin.get_default_foot_position(is_left=True)
    test_offsets = [
        ('y+0.02m (lateral in, reachable)', [0, 0.02, 0]),
        ('x+0.03m, y+0.01m (oblique forward)', [0.03, 0.01, 0]),
        ('x+0.05m (forward only)', [0.05, 0, 0]),
        ('x-0.02m (backward only)', [-0.02, 0, 0]),
    ]
    
    for name, offset in test_offsets:
        target_pos = default_left + torch.tensor(offset, dtype=torch.float32)
        # IK
        joints = kin.inverse_kinematics(target_pos, is_left=True)
        # FK
        actual_pos = kin.forward_kinematics(joints, is_left=True)
        
        error = torch.norm(target_pos - actual_pos).item() * 1000
        hip_yaw_deg = joints[0].item() * 180 / 3.14159
        status = 'PASS' if error < 5.0 else 'FAIL'  # 5mm許容（近似IK）
        if error >= 5.0:
            all_passed = False
        print(f'  {name}: error={error:.1f}mm, hip_yaw={hip_yaw_deg:.1f}° [{status}]')
    print()

    # 左右対称性の検証
    print('Test 5: 左右対称性の検証')
    test_pos = torch.tensor([0.05, 0.03, -0.18], dtype=torch.float32)
    left_joints = kin.inverse_kinematics(test_pos, is_left=True)
    
    # 右脚はY座標を反転
    test_pos_right = test_pos.clone()
    test_pos_right[1] = -test_pos_right[1]
    right_joints = kin.inverse_kinematics(test_pos_right, is_left=False)

    # hip_yawとhip_rollは符号反転、他は同じであるべき
    print(f'  Left joints:  yaw={left_joints[0]:.4f}, roll={left_joints[1]:.4f}, pitch={left_joints[2]:.4f}')
    print(f'  Right joints: yaw={right_joints[0]:.4f}, roll={right_joints[1]:.4f}, pitch={right_joints[2]:.4f}')
    
    # hip_pitch, knee_pitch, ankle_pitchは同じ
    pitch_match = torch.allclose(left_joints[2:], right_joints[2:], atol=1e-4)
    status = 'PASS' if pitch_match else 'FAIL'
    if not pitch_match:
        all_passed = False
    print(f'  Pitch angles match: [{status}]')
    print()

    if all_passed:
        print('=== 全テスト PASSED ===')
    else:
        print('=== 一部テスト FAILED ===')


if __name__ == '__main__':
    main()
