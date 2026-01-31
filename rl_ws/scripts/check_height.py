"""ロボットの脚の寸法と可動域から達成可能な胴体高さを計算"""
import math

# URDFから読み取ったリンク長
thigh_length = 0.18  # 太腿リンク長
shank_length = 0.20  # 脛リンク長
foot_height = 0.03   # 足部厚さ

# base_linkからhip_pitch関節までのオフセット
hip_yaw_offset = 0.03  # hip_yaw_linkの長さ
hip_roll_offset = 0.02  # hip_roll_linkの長さ
base_to_hip_pitch = hip_yaw_offset + hip_roll_offset  # = 0.05m

# 関節可動域（rad）
hip_pitch_min = -1.5707963267948966  # -90度
hip_pitch_max = 1.5707963267948966   # +90度
knee_pitch_min = -2.0943951023931953  # -120度
knee_pitch_max = 0.0                  # 0度
ankle_pitch_min = -0.7853981633974483  # -45度
ankle_pitch_max = 2.356194490192345    # +135度

# トルク制限
hip_pitch_torque = 100  # Nm
knee_pitch_torque = 100  # Nm
ankle_pitch_torque = 50  # Nm

print("=== ロボット脚の寸法 ===")
print(f"太腿長: {thigh_length}m")
print(f"脛長: {shank_length}m")
print(f"脚全長（直立時）: {thigh_length + shank_length}m = {thigh_length + shank_length:.2f}m")
print(f"base_linkからhip_pitch関節まで: {base_to_hip_pitch}m")
print(f"足部厚さ: {foot_height}m")
print()

print("=== 関節可動域 ===")
print(f"hip_pitch: [{hip_pitch_min*180/math.pi:.1f} deg, {hip_pitch_max*180/math.pi:.1f} deg]")
print(f"knee_pitch: [{knee_pitch_min*180/math.pi:.1f} deg, {knee_pitch_max*180/math.pi:.1f} deg]")
print(f"ankle_pitch: [{ankle_pitch_min*180/math.pi:.1f} deg, {ankle_pitch_max*180/math.pi:.1f} deg]")
print()

print("=== トルク制限 ===")
print(f"hip_pitch: {hip_pitch_torque} Nm")
print(f"knee_pitch: {knee_pitch_torque} Nm")
print(f"ankle_pitch: {ankle_pitch_torque} Nm")
print()

def calc_height(hip_pitch, knee_pitch):
    """胴体中心の高さを計算"""
    # hip_pitchが0のとき脚は真下
    # knee_pitchが負のとき膝が前に曲がる（逆関節）
    h = base_to_hip_pitch
    h += thigh_length * math.cos(hip_pitch)
    # 膝関節後の脛の向き = hip_pitch + knee_pitch
    h += shank_length * math.cos(hip_pitch + knee_pitch)
    h += foot_height / 2  # 足底までの距離
    return h

print("=== 各姿勢での胴体高さ ===")
print(f"直立（hip=0 deg, knee=0 deg）: {calc_height(0, 0):.3f}m")
print(f"軽く曲げ（hip=0 deg, knee=-30 deg）: {calc_height(0, -0.52):.3f}m")
print(f"深く曲げ（hip=0 deg, knee=-60 deg）: {calc_height(0, -1.05):.3f}m")
print(f"最大屈曲（hip=0 deg, knee=-120 deg）: {calc_height(0, -2.09):.3f}m")
print()

print("=== 目標高さ達成に必要な姿勢 ===")
for target in [0.45, 0.40, 0.35, 0.30, 0.25, 0.22, 0.20, 0.15]:
    print(f"\n目標高さ {target}m:")
    found = False
    for hip_deg in range(-90, 91, 5):
        for knee_deg in range(-120, 1, 5):
            hip = hip_deg * math.pi / 180
            knee = knee_deg * math.pi / 180
            h = calc_height(hip, knee)
            if abs(h - target) < 0.015:
                print(f"  hip_pitch={hip_deg} deg, knee_pitch={knee_deg} deg -> height={h:.3f}m")
                found = True
                break
        if found:
            break
    if not found:
        # 最小高さを探索
        min_h = float('inf')
        best_hip, best_knee = 0, 0
        for hip_deg in range(-90, 91, 5):
            for knee_deg in range(-120, 1, 5):
                hip = hip_deg * math.pi / 180
                knee = knee_deg * math.pi / 180
                h = calc_height(hip, knee)
                if h < min_h:
                    min_h = h
                    best_hip, best_knee = hip_deg, knee_deg
        print(f"  達成不可。最小高さ: {min_h:.3f}m (hip={best_hip} deg, knee={best_knee} deg)")

print("\n=== 結論 ===")
min_height = calc_height(0, knee_pitch_min)
print(f"可動域内での最小胴体高さ: {min_height:.3f}m")
print(f"（hip_pitch=0 deg, knee_pitch=-120 deg のとき）")
