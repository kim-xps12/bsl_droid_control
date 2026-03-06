#!/usr/bin/env python3
"""関節軸の方向をGenesis環境で検証するスクリプト"""

from __future__ import annotations

import genesis as gs


gs.init(backend=gs.cpu)

scene = gs.Scene(show_viewer=False)
scene.add_entity(gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True))
robot = scene.add_entity(
    gs.morphs.URDF(
        file="assets/bsl_droid_simplified.urdf",
        pos=[0, 0, 0.5],
        fixed=True,  # 胴体を固定
    )
)
scene.build(n_envs=1)

# 関節情報を表示
print("=== Genesis Joint Info ===")
for _i, joint in enumerate(robot.joints):
    if joint.n_dofs > 0:
        dof = joint.dofs[0]
        print(f"{joint.name}: dof_start={joint.dof_start}, limit=[{dof.limit[0]:.3f}, {dof.limit[1]:.3f}]")

# 初期状態
qpos = robot.get_qpos()
print("\n=== Initial state ===")

# 左膝の動作確認
knee_idx = robot.get_joint("left_knee_pitch_joint").dof_start
thigh = robot.get_link("left_thigh_link")
shank = robot.get_link("left_shank_link")

# 膝 = 0度の状態
qpos[0, 7 + knee_idx] = 0.0
robot.set_qpos(qpos)
scene.step()
thigh_pos_0 = thigh.get_pos()[0].clone()
shank_pos_0 = shank.get_pos()[0].clone()
print("\nKnee = 0.0 rad:")
print(f"  Thigh: X={thigh_pos_0[0]:.4f}, Z={thigh_pos_0[2]:.4f}")
print(f"  Shank: X={shank_pos_0[0]:.4f}, Z={shank_pos_0[2]:.4f}")
print(f"  Shank dX from thigh: {shank_pos_0[0] - thigh_pos_0[0]:.4f}")

# 膝 = +1.0 radの状態
qpos[0, 7 + knee_idx] = 1.0
robot.set_qpos(qpos)
scene.step()
thigh_pos_p = thigh.get_pos()[0].clone()
shank_pos_p = shank.get_pos()[0].clone()
print("\nKnee = +1.0 rad (正角度):")
print(f"  Thigh: X={thigh_pos_p[0]:.4f}, Z={thigh_pos_p[2]:.4f}")
print(f"  Shank: X={shank_pos_p[0]:.4f}, Z={shank_pos_p[2]:.4f}")
print(f"  Shank dX from thigh: {shank_pos_p[0] - thigh_pos_p[0]:.4f}")

# 動きの解釈
dx_change = (shank_pos_p[0] - thigh_pos_p[0]) - (shank_pos_0[0] - thigh_pos_0[0])
print("\n=== Analysis ===")
print(f"Shank X movement when knee goes from 0 to +1.0 rad: {dx_change:.4f}")
if dx_change > 0:
    print("  → 正角度で下腿が前方（+X）へ移動 = 膝が前に曲がる（非逆関節動作）")
else:
    print("  → 正角度で下腿が後方（-X）へ移動 = 膝が後ろに曲がる（逆関節動作）")

# Hip pitchの確認
print("\n=== Hip Pitch Check ===")
hip_pitch_idx = robot.get_joint("left_hip_pitch_joint").dof_start
qpos[0, 7 + knee_idx] = 0.0  # 膝をリセット
qpos[0, 7 + hip_pitch_idx] = 0.0
robot.set_qpos(qpos)
scene.step()
thigh_pos_0 = thigh.get_pos()[0].clone()
print(f"Hip pitch = 0.0 rad: Thigh X={thigh_pos_0[0]:.4f}, Z={thigh_pos_0[2]:.4f}")

qpos[0, 7 + hip_pitch_idx] = -0.5
robot.set_qpos(qpos)
scene.step()
thigh_pos_neg = thigh.get_pos()[0].clone()
print(f"Hip pitch = -0.5 rad: Thigh X={thigh_pos_neg[0]:.4f}, Z={thigh_pos_neg[2]:.4f}")
dx = thigh_pos_neg[0] - thigh_pos_0[0]
print(f"  → Hip pitch -0.5 で大腿 dX={dx:.4f}")
if dx > 0:
    print("  → 負角度で脚が前方へ振り出される")
else:
    print("  → 負角度で脚が後方へ振り出される")
