# Biped Digitigrade Robot Description

逆関節（鳥脚型）二脚ロボットのURDF記述パッケージです。

## 特徴

- **逆関節構造**: 膝が前方に曲がる鳥脚型
- **10自由度**: 片脚5関節 × 2脚
- **関節構成**: 腰ヨー, 股関節ロール, 股関節ピッチ, 膝ピッチ, 足首ピッチ
- **RViz2対応**: スライダーGUIで全関節を操作可能

## ロボット仕様

### リンク長（`ref/example_gait.py`と一致）

| パラメータ | 長さ [m] | 説明 |
|-----------|---------|------|
| hip_width | 0.16 | 左右の股関節間距離 |
| hip_yaw_length | 0.03 | ヨー軸リンク長 |
| hip_roll_length | 0.02 | ロール軸リンク長 |
| thigh_length | 0.18 | 大腿部長さ |
| shank_length | 0.20 | 下腿部長さ |
| foot_length | 0.10 | 足部長さ |

### 関節命名規則

#### 左脚（j1x）
- `left_hip_yaw_joint` (j11): 腰ヨー軸 ±30°
- `left_hip_roll_joint` (j12): 股関節ロール軸 ±20°
- `left_hip_pitch_joint` (j13): 股関節ピッチ軸 ±90°
- `left_knee_pitch_joint` (j14): 膝ピッチ軸 -120°~0° （負で前方に曲がる）
- `left_ankle_pitch_joint` (j15): 足首ピッチ軸 -45°~135°

#### 右脚（j2x）
- `right_hip_yaw_joint` (j21): 腰ヨー軸 ±30°
- `right_hip_roll_joint` (j22): 股関節ロール軸 ±20°
- `right_hip_pitch_joint` (j23): 股関節ピッチ軸 ±90°
- `right_knee_pitch_joint` (j24): 膝ピッチ軸 -120°~0°
- `right_ankle_pitch_joint` (j25): 足首ピッチ軸 -45°~135°

## ビルド

```bash
cd ~/Projects/bsl_droid_ros2/ros2_ws
colcon build --packages-select biped_description
source install/setup.bash
```

## 使用方法

### RViz2可視化 + GUIスライダー

```bash
ros2 launch biped_description display.launch.py
```

起動すると：
- **RViz2**: ロボットモデルとTFツリーを表示
- **joint_state_publisher_gui**: 各関節をスライダーで操作するGUI

### GUIなしモード

```bash
ros2 launch biped_description display.launch.py gui:=false
```

### 関節角度の確認

```bash
# joint_statesトピックを確認
ros2 topic echo /joint_states

# TFツリーを確認
ros2 run tf2_tools view_frames
```

## ファイル構成

```
biped_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── urdf/
│   └── biped_digitigrade.urdf.xacro  # ロボットモデル定義
├── launch/
│   └── display.launch.py             # 可視化起動ファイル
└── rviz/
    └── biped_display.rviz            # RViz2設定
```

## トピック一覧

| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/robot_description` | `std_msgs/String` | URDF文字列 |
| `/joint_states` | `sensor_msgs/JointState` | 関節状態 |
| `/tf` | `tf2_msgs/TFMessage` | 座標変換 |
| `/tf_static` | `tf2_msgs/TFMessage` | 静的座標変換 |

## 次のステップ

1. **実機接続**: `robstride_hardware`と連携してCAN通信で実機制御
2. **歩容生成**: `ref/example_gait.py`の逆運動学を`gait_generator`ノードに実装
3. **状態推定**: IMU + エンコーダで`state_estimator`ノード実装
4. **安全監視**: `safety_monitor`ノードで転倒検知・緊急停止

## 参考

- [ros2_control_manual.md](../../ref/ros2_control_manual.md)
- [robstride_ros2_control_guide.md](../../ref/robstride_ros2_control_guide.md)
- [example_gait.py](../../ref/example_gait.py)
