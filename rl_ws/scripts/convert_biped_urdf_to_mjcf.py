#!/usr/bin/env python3
"""
BSL-Droid二脚ロボット URDF を MuJoCo MJCF に変換するスクリプト

URDFをMuJoCoで使用可能なMJCF形式に変換し、
Sim2Sim評価用のモデルを生成する。

変換処理:
1. URDFをMuJoCoでコンパイル・MJCFエクスポート
2. 階層構造を修正（freejoint追加、baseボディでラップ）
3. 関節パラメータを補完（damping, armature, frictionloss）
4. 足の接触パラメータを追加
5. アクチュエータ・keyframe追加
6. IMUサイトを追加

Usage:
    cd rl_ws
    uv run python scripts/convert_biped_urdf_to_mjcf.py

    # 中間XMLも保存（デバッグ用）
    uv run python scripts/convert_biped_urdf_to_mjcf.py --save-intermediate
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


# 二脚ロボット URDF パス
BIPED_URDF_PATH = "assets/biped_digitigrade.urdf"

# 関節パラメータ（物理特性）
JOINT_PARAMS = {
    "damping": "2",
    "armature": "0.01",
    "frictionloss": "0.2",
}

# 足の接触パラメータ
FOOT_CONTACT_PARAMS = {
    "friction": "0.8 0.02 0.01",
    "solimp": "0.015 1 0.03",
    "condim": "6",
    "priority": "1",
}

# 初期高さ（脚長に基づく: thigh=0.18 + shank=0.20 + foot_height=0.03 + margin）
# 膝を軽く曲げた状態での高さ
BASE_INIT_HEIGHT = 0.45

# ベースボディの慣性パラメータ（URDFから）
BASE_INERTIAL = {
    "pos": "0 0 0",
    "mass": "3.0",
    "diaginertia": "0.05 0.05 0.05",
}

# アクチュエータのトルク制限
ACTUATOR_CTRL_RANGE = {
    "hip_yaw": 50.0,
    "hip_roll": 50.0,
    "hip_pitch": 100.0,
    "knee_pitch": 100.0,
    "ankle_pitch": 50.0,
}

# 関節順序（左脚→右脚）
ACTUATED_JOINTS = [
    "left_hip_yaw_joint",
    "left_hip_roll_joint",
    "left_hip_pitch_joint",
    "left_knee_pitch_joint",
    "left_ankle_pitch_joint",
    "right_hip_yaw_joint",
    "right_hip_roll_joint",
    "right_hip_pitch_joint",
    "right_knee_pitch_joint",
    "right_ankle_pitch_joint",
]

# 初期姿勢（逆関節に適した立位）
# 膝を軽く曲げ、足首で相殺
DEFAULT_JOINT_ANGLES = {
    "left_hip_yaw_joint": 0.0,
    "left_hip_roll_joint": 0.0,
    "left_hip_pitch_joint": 0.0,
    "left_knee_pitch_joint": -0.52,  # 約-30° (前方屈曲)
    "left_ankle_pitch_joint": 0.52,  # 膝と相殺して足裏を水平に
    "right_hip_yaw_joint": 0.0,
    "right_hip_roll_joint": 0.0,
    "right_hip_pitch_joint": 0.0,
    "right_knee_pitch_joint": -0.52,
    "right_ankle_pitch_joint": 0.52,
}


def convert_urdf_to_mjcf(
    urdf_path: Path,
    output_path: Path,
    save_intermediate: bool = False,
) -> None:
    """
    二脚ロボット URDFをMuJoCo MJCFに変換

    Args:
        urdf_path: 入力URDFファイルのパス
        output_path: 出力MJCFファイルのパス
        save_intermediate: 中間ファイルを保存するか
    """
    print("=== Biped URDF to MJCF Conversion ===")
    print(f"Input:  {urdf_path}")
    print(f"Output: {output_path}")

    # Step 1: URDFを読み込み、MuJoCoでコンパイル
    print("\n[1/6] Loading URDF and compiling with MuJoCo...")
    temp_model = mujoco.MjModel.from_xml_path(str(urdf_path))
    print(f"  - Original bodies: {temp_model.nbody}")
    print(f"  - Original joints: {temp_model.njnt}")

    # Step 2: MuJoCoが変換したMJCFをエクスポート
    print("\n[2/6] Exporting MuJoCo's internal MJCF representation...")
    intermediate_path = output_path.parent / f"{output_path.stem}_intermediate.xml"
    mujoco.mj_saveLastXML(str(intermediate_path), temp_model)
    print(f"  - Intermediate XML: {intermediate_path}")

    # Step 3: XMLを解析して構造を修正
    print("\n[3/6] Modifying XML structure...")
    tree = ET.parse(intermediate_path)
    root = tree.getroot()

    # モデル名を設定
    root.set("model", "biped_digitigrade")

    # コンパイラ設定を追加
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("angle", "radian")
    compiler.set("autolimits", "true")

    # オプションを追加/更新
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("cone", "elliptic")
    option.set("impratio", "100")

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("worldbody not found in converted MJCF")

    # worldbody直下の要素を収集
    direct_geoms = list(worldbody.findall("geom"))
    direct_bodies = list(worldbody.findall("body"))

    print(f"  - Direct geoms in worldbody: {len(direct_geoms)}")
    print(f"  - Direct bodies in worldbody: {len(direct_bodies)}")

    # 新しいbaseボディを作成
    base_body = ET.Element("body")
    base_body.set("name", "base")
    base_body.set("pos", f"0 0 {BASE_INIT_HEIGHT}")

    # freejointを追加
    freejoint = ET.SubElement(base_body, "freejoint")
    freejoint.set("name", "root")
    print("  - Added freejoint 'root'")

    # 慣性情報を追加
    inertial = ET.SubElement(base_body, "inertial")
    inertial.set("pos", BASE_INERTIAL["pos"])
    inertial.set("mass", BASE_INERTIAL["mass"])
    inertial.set("diaginertia", BASE_INERTIAL["diaginertia"])
    print(f"  - Added base inertial (mass={BASE_INERTIAL['mass']})")

    # IMUサイトを追加（胴体上面）
    imu_site = ET.SubElement(base_body, "site")
    imu_site.set("name", "imu")
    imu_site.set("pos", "0 0 0.06")
    imu_site.set("size", "0.01")
    print("  - Added IMU site")

    # worldbody直下のgeomをbaseボディに移動
    for geom in direct_geoms:
        worldbody.remove(geom)
        base_body.append(geom)

    # worldbody直下のbodyをbaseボディに移動
    for body in direct_bodies:
        worldbody.remove(body)
        base_body.append(body)

    print(f"  - Moved {len(direct_geoms)} geoms and {len(direct_bodies)} bodies to base")

    # 地面を追加
    ground = ET.SubElement(worldbody, "geom")
    ground.set("name", "ground_plane")
    ground.set("type", "plane")
    ground.set("size", "0 0 0.05")
    ground.set("rgba", "0.2 0.3 0.4 1.0")

    # 光源を追加
    light = ET.SubElement(worldbody, "light")
    light.set("pos", "0 0 3")
    light.set("dir", "0 0 -1")
    light.set("directional", "true")

    # baseボディをworldbodyに追加
    worldbody.append(base_body)
    print("  - Added ground plane and light")

    # Step 4: 関節パラメータを補完
    print("\n[4/6] Adding joint parameters (damping, armature, frictionloss)...")
    joint_count = 0
    for joint in root.iter("joint"):
        joint_name = joint.get("name", "")
        if joint_name and joint_name != "root":
            joint.set("damping", JOINT_PARAMS["damping"])
            joint.set("armature", JOINT_PARAMS["armature"])
            joint.set("frictionloss", JOINT_PARAMS["frictionloss"])
            joint_count += 1
    print(f"  - Updated {joint_count} joints with physics parameters")

    # Step 5: 足のgeomに接触パラメータを追加
    print("\n[5/6] Adding foot contact parameters...")
    foot_count = 0
    for body in root.iter("body"):
        body_name = body.get("name", "")
        if "foot" in body_name.lower():
            for geom in body.findall("geom"):
                # 足先のbox geomに接触パラメータを設定
                geom.set("name", body_name.replace("_link", ""))
                geom.set("friction", FOOT_CONTACT_PARAMS["friction"])
                geom.set("solimp", FOOT_CONTACT_PARAMS["solimp"])
                geom.set("condim", FOOT_CONTACT_PARAMS["condim"])
                geom.set("priority", FOOT_CONTACT_PARAMS["priority"])
                foot_count += 1
    print(f"  - Updated {foot_count} foot geoms with contact parameters")

    # アクチュエータセクションを追加
    print("\n  Adding actuators:")
    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")

    for joint_name in ACTUATED_JOINTS:
        motor = ET.SubElement(actuator, "motor")
        motor.set("name", joint_name.replace("_joint", ""))
        motor.set("joint", joint_name)
        motor.set("gear", "1")

        # トルク制限の設定
        if "ankle" in joint_name:
            ctrl_limit = ACTUATOR_CTRL_RANGE["ankle_pitch"]
        elif "knee" in joint_name:
            ctrl_limit = ACTUATOR_CTRL_RANGE["knee_pitch"]
        elif "hip_pitch" in joint_name:
            ctrl_limit = ACTUATOR_CTRL_RANGE["hip_pitch"]
        elif "hip_roll" in joint_name:
            ctrl_limit = ACTUATOR_CTRL_RANGE["hip_roll"]
        else:
            ctrl_limit = ACTUATOR_CTRL_RANGE["hip_yaw"]
        motor.set("ctrlrange", f"{-ctrl_limit} {ctrl_limit}")
        print(f"    - {joint_name} (torque limit: ±{ctrl_limit} Nm)")

    # Step 6: keyframeセクションを追加
    print("\n[6/6] Adding home keyframe...")
    keyframe = root.find("keyframe")
    if keyframe is None:
        keyframe = ET.SubElement(root, "keyframe")

    # qpos: [x, y, z, qw, qx, qy, qz, joint1, joint2, ...]
    qpos_values = [0, 0, BASE_INIT_HEIGHT, 1, 0, 0, 0]  # base position + quaternion
    ctrl_values = []
    for joint_name in ACTUATED_JOINTS:
        angle = DEFAULT_JOINT_ANGLES[joint_name]
        qpos_values.append(angle)
        ctrl_values.append(angle)

    qpos_str = " ".join(str(v) for v in qpos_values)
    ctrl_str = " ".join(str(v) for v in ctrl_values)

    key = ET.SubElement(keyframe, "key")
    key.set("name", "home")
    key.set("qpos", qpos_str)
    key.set("ctrl", ctrl_str)
    print(f"    - home qpos: {qpos_str[:60]}...")

    # センサーセクションを追加（IMU）
    print("\n  Adding sensors (IMU)...")
    sensor = root.find("sensor")
    if sensor is None:
        sensor = ET.SubElement(root, "sensor")

    # ジャイロスコープ
    gyro = ET.SubElement(sensor, "gyro")
    gyro.set("site", "imu")
    gyro.set("name", "gyro")

    # 加速度計
    accelerometer = ET.SubElement(sensor, "accelerometer")
    accelerometer.set("site", "imu")
    accelerometer.set("name", "accelerometer")

    # 姿勢（クォータニオン）
    framequat = ET.SubElement(sensor, "framequat")
    framequat.set("objtype", "site")
    framequat.set("objname", "imu")
    framequat.set("name", "orientation")

    print("    - gyro, accelerometer, orientation")

    # 最終MJCFを保存
    print("\n  Saving final MJCF...")
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="unicode", xml_declaration=True)
    print(f"  - Saved to: {output_path}")

    # 検証：再読み込みしてコンパイル
    print("\n[Verification] Reloading and compiling...")
    final_model = mujoco.MjModel.from_xml_path(str(output_path))
    print(f"  - Bodies: {final_model.nbody}")
    print(f"  - Joints: {final_model.njnt} (including freejoint)")
    print(f"  - Actuators: {final_model.nu}")
    print(f"  - Sensors: {final_model.nsensor}")
    print(f"  - Keyframes: {final_model.nkey}")

    # 関節名を表示
    joint_names = []
    for i in range(final_model.njnt):
        name = final_model.joint(i).name
        if name and name != "root":
            joint_names.append(name)
    print(f"  - Joint names: {joint_names}")

    # 中間ファイルの処理
    if save_intermediate:
        print(f"\n  Intermediate file preserved: {intermediate_path}")
    else:
        intermediate_path.unlink()
        print("\n  Intermediate file removed")

    print("\n=== Conversion Complete ===")


def main() -> None:
    parser = argparse.ArgumentParser(description="BSL-Droid二脚ロボット URDF を MuJoCo MJCF に変換")
    parser.add_argument(
        "--urdf",
        type=str,
        default=BIPED_URDF_PATH,
        help="入力URDFファイルのパス",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="assets/biped_digitigrade.xml",
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
