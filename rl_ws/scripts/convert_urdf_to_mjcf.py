#!/usr/bin/env python3
"""
Genesis URDF を MuJoCo MJCF に変換するスクリプト

URDFをMuJoCoで使用可能なMJCF形式に変換し、
MuJoCo Menagerieと等価な物理特性を持つモデルを生成する。

問題点と対策:
- MuJoCoはDAEメッシュを非サポート → ビジュアルメッシュなしで変換
- URDFにはdamping/armature/frictionlossがない → XML後処理で追加
- 足の接触パラメータが不足 → 明示的に設定

変換処理:
1. URDFをMuJoCoでコンパイル・MJCFエクスポート
2. 階層構造を修正（freejoint追加、baseボディでラップ）
3. 関節パラメータを補完（damping, armature, frictionloss）
4. 足の接触パラメータを追加
5. アクチュエータ・keyframe追加

Usage:
    cd rl_ws
    uv run python scripts/convert_urdf_to_mjcf.py

    # 出力先を指定
    uv run python scripts/convert_urdf_to_mjcf.py --output assets/go2_genesis.xml

    # 中間XMLも保存（デバッグ用）
    uv run python scripts/convert_urdf_to_mjcf.py --save-intermediate
"""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


# Genesis URDF関連の定数
GENESIS_URDF_RELATIVE_PATH = "genesis_official/genesis/assets/urdf/go2/urdf/go2.urdf"

# MuJoCo Menagerieと同等の関節パラメータ
# 参照: mujoco_menagerie/unitree_go2/go2.xml
JOINT_PARAMS = {
    "damping": "2",
    "armature": "0.01",
    "frictionloss": "0.2",
}

# 足の接触パラメータ（MuJoCo Menagerieから）
FOOT_CONTACT_PARAMS = {
    "size": "0.022",
    "friction": "0.8 0.02 0.01",
    "solimp": "0.015 1 0.022",
    "condim": "6",
    "priority": "1",
}

# 初期高さ（MuJoCo Menagerieと同値）
BASE_INIT_HEIGHT = 0.445

# Go2ベースボディの慣性パラメータ（MuJoCo Menagerieから）
BASE_INERTIAL = {
    "pos": "0.021112 0 -0.005366",
    "quat": "-0.000543471 0.713435 -0.00173769 0.700719",
    "mass": "6.921",
    "diaginertia": "0.107027 0.0980771 0.0244531",
}

# アクチュエータのトルク制限（MuJoCo Menagerieと同値）
ACTUATOR_CTRL_RANGE = {
    "hip": 23.7,
    "thigh": 23.7,
    "calf": 45.43,
}

# 関節順序（FL, FR, RL, RR）- MuJoCoの標準順序
ACTUATED_JOINTS = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]

# homeキーフレーム（MuJoCo Menagerieと同値）
# qpos: [x, y, z, qw, qx, qy, qz, FL_hip, FL_thigh, FL_calf, FR_*, RL_*, RR_*]
HOME_QPOS = "0 0 0.27 1 0 0 0 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8"
HOME_CTRL = "0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8"


def add_joint_params_recursive(element: ET.Element, params: dict) -> int:
    """
    再帰的にjoint要素を探し、パラメータを追加

    Args:
        element: 探索対象のXML要素
        params: 追加するパラメータ

    Returns:
        更新したjoint数
    """
    count = 0

    # 直接の子要素を探索
    for child in element:
        if child.tag == 'joint':
            joint_name = child.get('name', '')
            # freejoint (root) は除外
            if joint_name and joint_name != 'root' and child.get('type') != 'free':
                for key, value in params.items():
                    child.set(key, value)
                count += 1
        # 再帰的に子要素を探索
        count += add_joint_params_recursive(child, params)

    return count


def find_and_update_foot_geoms(element: ET.Element, body_name: str = "") -> int:
    """
    脚の末端（calf）にある足のgeomを探して接触パラメータを更新

    Args:
        element: 探索対象のXML要素
        body_name: 現在のボディ名

    Returns:
        更新したgeom数
    """
    count = 0

    for child in element:
        if child.tag == 'body':
            child_body_name = child.get('name', '')
            count += find_and_update_foot_geoms(child, child_body_name)
        elif child.tag == 'geom':
            # calfボディ内の球形geom（足）を探す
            # 足のgeomを判定: calfボディ内で、サイズが0.022程度の球
            if 'calf' in body_name.lower():
                # 足先の球形geom（pos="-0.002 0 -0.213"付近）
                geom_pos = child.get('pos', '')
                if geom_pos and '-0.213' in geom_pos:
                    # 足の接触パラメータを設定
                    leg_prefix = body_name.split('_')[0]  # FL, FR, RL, RR
                    child.set('name', f'{leg_prefix}')
                    child.set('size', FOOT_CONTACT_PARAMS['size'])
                    child.set('friction', FOOT_CONTACT_PARAMS['friction'])
                    child.set('solimp', FOOT_CONTACT_PARAMS['solimp'])
                    child.set('condim', FOOT_CONTACT_PARAMS['condim'])
                    child.set('priority', FOOT_CONTACT_PARAMS['priority'])
                    count += 1

    return count


def convert_urdf_to_mjcf(
    urdf_path: Path,
    output_path: Path,
    save_intermediate: bool = False,
) -> None:
    """
    Genesis URDFをMuJoCo MJCFに変換

    Args:
        urdf_path: 入力URDFファイルのパス
        output_path: 出力MJCFファイルのパス
        save_intermediate: 中間ファイルを保存するか
    """
    print(f"=== URDF to MJCF Conversion ===")
    print(f"Input:  {urdf_path}")
    print(f"Output: {output_path}")

    # Step 1: URDFを読み込み、MuJoCoでコンパイル
    print("\n[1/5] Loading URDF and compiling with MuJoCo...")
    temp_model = mujoco.MjModel.from_xml_path(str(urdf_path))
    print(f"  - Original bodies: {temp_model.nbody}")
    print(f"  - Original joints: {temp_model.njnt}")

    # Step 2: MuJoCoが変換したMJCFをエクスポート
    print("\n[2/5] Exporting MuJoCo's internal MJCF representation...")
    intermediate_path = output_path.parent / f"{output_path.stem}_intermediate.xml"
    mujoco.mj_saveLastXML(str(intermediate_path), temp_model)
    print(f"  - Intermediate XML: {intermediate_path}")

    # Step 3: XMLを解析して構造を修正
    print("\n[3/5] Modifying XML structure...")
    tree = ET.parse(intermediate_path)
    root = tree.getroot()

    # モデル名を設定
    root.set('model', 'go2_genesis')

    # オプションを追加/更新（MuJoCo Menagerieと同値）
    option = root.find('option')
    if option is None:
        option = ET.SubElement(root, 'option')
    option.set('cone', 'elliptic')
    option.set('impratio', '100')

    worldbody = root.find('worldbody')
    if worldbody is None:
        raise ValueError("worldbody not found in converted MJCF")

    # worldbody直下の要素を収集
    direct_geoms = list(worldbody.findall('geom'))
    direct_bodies = list(worldbody.findall('body'))

    print(f"  - Direct geoms in worldbody: {len(direct_geoms)}")
    print(f"  - Direct bodies in worldbody: {len(direct_bodies)}")

    # 新しいbaseボディを作成
    base_body = ET.Element('body')
    base_body.set('name', 'base')
    base_body.set('pos', f'0 0 {BASE_INIT_HEIGHT}')

    # freejointを追加
    freejoint = ET.SubElement(base_body, 'freejoint')
    freejoint.set('name', 'root')
    print("  - Added freejoint 'root'")

    # 慣性情報を追加（MuJoCo Menagerieと同値）
    inertial = ET.SubElement(base_body, 'inertial')
    inertial.set('pos', BASE_INERTIAL["pos"])
    inertial.set('quat', BASE_INERTIAL["quat"])
    inertial.set('mass', BASE_INERTIAL["mass"])
    inertial.set('diaginertia', BASE_INERTIAL["diaginertia"])
    print(f"  - Added base inertial (mass={BASE_INERTIAL['mass']})")

    # worldbody直下のgeomをbaseボディに移動
    for geom in direct_geoms:
        worldbody.remove(geom)
        base_body.append(geom)

    # worldbody直下のbodyをbaseボディに移動
    for body in direct_bodies:
        worldbody.remove(body)
        base_body.append(body)

    print(f"  - Moved {len(direct_geoms)} geoms and {len(direct_bodies)} bodies to base")

    # 地面を追加（MuJoCo Menagerieと同様のスタイル）
    ground = ET.SubElement(worldbody, 'geom')
    ground.set('name', 'ground_plane')
    ground.set('type', 'plane')
    ground.set('size', '0 0 0.05')
    ground.set('rgba', '0.2 0.3 0.4 1.0')

    # 光源を追加
    light = ET.SubElement(worldbody, 'light')
    light.set('pos', '0 0 3')
    light.set('dir', '0 0 -1')
    light.set('directional', 'true')

    # baseボディをworldbodyに追加
    worldbody.append(base_body)
    print("  - Added ground plane and light")

    # Step 4: 関節パラメータを補完
    print("\n[4/5] Adding joint parameters (damping, armature, frictionloss)...")
    joint_count = 0
    for joint in root.iter('joint'):
        joint_name = joint.get('name', '')
        if joint_name and joint_name != 'root':
            joint.set('damping', JOINT_PARAMS['damping'])
            joint.set('armature', JOINT_PARAMS['armature'])
            joint.set('frictionloss', JOINT_PARAMS['frictionloss'])
            joint_count += 1
    print(f"  - Updated {joint_count} joints with physics parameters")

    # アクチュエータセクションを追加
    print("\n  Adding actuators:")
    actuator = root.find('actuator')
    if actuator is None:
        actuator = ET.SubElement(root, 'actuator')

    for joint_name in ACTUATED_JOINTS:
        motor = ET.SubElement(actuator, 'motor')
        motor.set('name', joint_name.replace('_joint', ''))
        motor.set('joint', joint_name)
        motor.set('gear', '1')

        # トルク制限の設定
        if "calf" in joint_name:
            ctrl_limit = ACTUATOR_CTRL_RANGE["calf"]
        elif "thigh" in joint_name:
            ctrl_limit = ACTUATOR_CTRL_RANGE["thigh"]
        else:
            ctrl_limit = ACTUATOR_CTRL_RANGE["hip"]
        motor.set('ctrlrange', f'{-ctrl_limit} {ctrl_limit}')
        print(f"    - {joint_name} (torque limit: ±{ctrl_limit} Nm)")

    # keyframeセクションを追加
    print("\n  Adding home keyframe...")
    keyframe = root.find('keyframe')
    if keyframe is None:
        keyframe = ET.SubElement(root, 'keyframe')
    key = ET.SubElement(keyframe, 'key')
    key.set('name', 'home')
    key.set('qpos', HOME_QPOS)
    key.set('ctrl', HOME_CTRL)
    print(f"    - home qpos: {HOME_QPOS[:50]}...")

    # Step 5: 最終MJCFを保存
    print(f"\n[5/5] Saving final MJCF...")

    # XMLを整形して保存
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding='unicode', xml_declaration=True)
    print(f"  - Saved to: {output_path}")

    # 検証：再読み込みしてコンパイル
    print("\n[Verification] Reloading and compiling...")
    final_model = mujoco.MjModel.from_xml_path(str(output_path))
    print(f"  - Bodies: {final_model.nbody}")
    print(f"  - Joints: {final_model.njnt} (including freejoint)")
    print(f"  - Actuators: {final_model.nu}")
    print(f"  - Keyframes: {final_model.nkey}")

    # 関節名を表示
    joint_names = []
    for i in range(final_model.njnt):
        name = final_model.joint(i).name
        if name and name != "root":
            joint_names.append(name)
    print(f"  - Joint names: {joint_names}")

    # 関節パラメータを検証
    print("\n  Joint parameters verification:")
    for i in range(final_model.njnt):
        jnt = final_model.joint(i)
        if jnt.name and jnt.name != "root":
            dof_id = jnt.dofadr[0]
            damping = final_model.dof_damping[dof_id]
            armature = final_model.dof_armature[dof_id]
            frictionloss = final_model.dof_frictionloss[dof_id]
            print(f"    - {jnt.name}: damping={damping}, armature={armature}, frictionloss={frictionloss}")
            break  # 1つだけ表示

    # 中間ファイルの処理
    if save_intermediate:
        print(f"\n  Intermediate file preserved: {intermediate_path}")
    else:
        intermediate_path.unlink()
        print(f"\n  Intermediate file removed")

    print("\n=== Conversion Complete ===")


def main():
    parser = argparse.ArgumentParser(
        description="Genesis URDF を MuJoCo MJCF に変換"
    )
    parser.add_argument(
        "--urdf",
        type=str,
        default=GENESIS_URDF_RELATIVE_PATH,
        help="入力URDFファイルのパス",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="assets/go2_genesis.xml",
        help="出力MJCFファイルのパス",
    )
    parser.add_argument(
        "--save-intermediate",
        action="store_true",
        help="中間XMLファイルを保存（デバッグ用）",
    )
    args = parser.parse_args()

    # パスの解決
    script_dir = Path(__file__).parent.parent
    urdf_path = script_dir / args.urdf
    output_path = script_dir / args.output

    # 出力ディレクトリを作成
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # 変換実行
    convert_urdf_to_mjcf(urdf_path, output_path, args.save_intermediate)


if __name__ == "__main__":
    main()
