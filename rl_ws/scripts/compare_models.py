#!/usr/bin/env python3
"""パラメータ比較スクリプト"""
import mujoco

# 両モデルを読み込み
genesis_model = mujoco.MjModel.from_xml_path('assets/go2_genesis.xml')
menagerie_model = mujoco.MjModel.from_xml_path('mujoco_menagerie/unitree_go2/scene.xml')

print('=== 修正後の物理パラメータ比較 ===')
print()

# 関節パラメータを比較
print('--- 関節パラメータ (damping, armature, frictionloss) ---')
for model, name in [(genesis_model, 'Genesis'), (menagerie_model, 'Menagerie')]:
    print(f'{name}:')
    for i in range(model.njnt):
        jnt = model.joint(i)
        if jnt.name and 'hip' in jnt.name:
            dof_id = jnt.dofadr[0]
            print(f'  {jnt.name}: damping={model.dof_damping[dof_id]:.4f}, armature={model.dof_armature[dof_id]:.6f}, frictionloss={model.dof_frictionloss[dof_id]:.4f}')
            break

print()
print('--- 足の接触パラメータ ---')
for model, name in [(genesis_model, 'Genesis'), (menagerie_model, 'Menagerie')]:
    print(f'{name}:')
    for i in range(model.ngeom):
        geom = model.geom(i)
        if geom.name == 'FL':
            print(f'  {geom.name}: friction={geom.friction}, condim={geom.condim}')
            break
    else:
        print('  (FL geom not found)')

print()
print('--- keyframe ---')
print(f'Genesis keyframes: {genesis_model.nkey}')
print(f'Menagerie keyframes: {menagerie_model.nkey}')
