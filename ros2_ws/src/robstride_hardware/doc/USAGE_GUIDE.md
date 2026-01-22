# RobStride Hardware 使い方ガイド

このガイドでは、RobStride Hardware Interfaceの実践的な使い方を、セットアップから高度な使い方まで段階的に説明します。

## 目次

- [初期セットアップ](#初期セットアップ)
- [基本的な使い方](#基本的な使い方)
- [設定のカスタマイズ](#設定のカスタマイズ)
- [モーション制御の例](#モーション制御の例)
- [デバッグとトラブルシューティング](#デバッグとトラブルシューティング)
- [高度な使い方](#高度な使い方)

---

## 初期セットアップ

### 1. ハードウェアの接続

#### 必要な機材

- RobStrideモーター（RS-01, RS-02, RS-03のいずれか）
- CAN-USBアダプター（例: PEAK PCAN-USB, Kvaser Leaf Light）
- 電源（モーターの仕様に応じて、例: 24V/5A以上）
- CANケーブル（CAN_H, CAN_L, GND）

#### 配線

```
PC (USB) ──USB Cable──> CAN-USB Adapter
                              │
                        CAN_H, CAN_L, GND
                              │
                              ▼
                        RobStride Motor
                              │
                        Power Supply (24V)
```

**注意事項:**
- CANバスの両端に120Ωの終端抵抗を配置
- CANケーブルはツイストペアを使用（ノイズ対策）
- GNDは必ず接続（COMONグラウンド）

### 2. CANインターフェースの設定

#### CANインターフェースの確認

```bash
# CANデバイスの確認
ip link show

# 例:
# can0: <NOARP,ECHO> mtu 16 qdisc noop state DOWN mode DEFAULT group default qlen 10
#     link/can
```

#### CANインターフェースの起動

```bash
# ビットレート1Mbpsで起動
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 確認
ip -details link show can0
```

**出力例:**
```
can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can
    can state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 0
          bitrate 1000000 sample-point 0.875
          tq 62 prop-seg 6 phase-seg1 7 phase-seg2 2 sjw 1
          gs_usb: tseg1 1..16 tseg2 1..8 sjw 1..4 brp 1..1024 brp-inc 1
          clock 48000000
          re-started bus-errors arbit-lost error-warn error-pass bus-off
          0          0          0          0          0          0
```

#### 自動起動設定（オプション）

`/etc/network/interfaces`に追加:

```bash
auto can0
iface can0 can static
    bitrate 1000000
    up ip link set $IFACE up
    down ip link set $IFACE down
```

### 3. パッケージのビルド

```bash
cd ~/ros2_ws
pixi run colcon build --packages-select robstride_hardware --symlink-install
source install/setup.bash
```

**ビルド確認:**
```bash
pixi run ros2 pkg list | grep robstride
# 出力: robstride_hardware
```

---

## 基本的な使い方

### ステップ1: モーターIDの確認

モーターのCAN IDを確認します。モーターのディップスイッチまたは設定ツールで確認してください。

**例: Motor ID = 127**

### ステップ2: URDFファイルの編集

`ros2_ws/src/robstride_hardware/urdf/robstride_system.urdf.xacro`を編集:

```xml
<hardware>
  <plugin>robstride_hardware/RobStrideHardware</plugin>
  <param name="can_interface">can0</param>  <!-- 使用するCANインターフェース -->
  <param name="motor_id">127</param>         <!-- モーターのID -->
  <param name="kp">1.5</param>               <!-- 位置ゲイン -->
  <param name="kd">0.01</param>              <!-- 速度ゲイン -->
</hardware>
```

### ステップ3: システムの起動

```bash
pixi run ros2 launch robstride_hardware bringup.launch.py
```

**正常起動時の出力:**
```
[INFO] [controller_manager]: Loading controller 'joint_state_broadcaster'
[INFO] [controller_manager]: Loaded controller 'joint_state_broadcaster'
[INFO] [RobStrideHardware]: Initialized: can=can0, motor_id=127, kp=1.5, kd=0.0
[INFO] [RobStrideHardware]: Connected to CAN interface: can0
[INFO] [RobStrideHardware]: Motor 127 activated in MIT mode
[INFO] [RobStrideHardware]: State reader thread started (100Hz) for motor 127
```

### ステップ4: モーターの状態確認

別のターミナルで:

```bash
# リアルタイムの状態確認
pixi run ros2 topic echo /robstride/joint_states
```

**出力例:**
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: ''
name:
- joint1
position:
- 0.0234
velocity:
- 0.0012
effort:
- 0.045
```

### ステップ5: 位置コマンドの送信

```bash
# 0.5 radに移動
pixi run ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5]"
```

**モーターの動作:**
- ゆっくりと目標位置（0.5 rad）に移動
- Kp, Kdの値によって応答速度が変わる

---

## 設定のカスタマイズ

### Kp, Kdゲインの調整

モーターの応答特性を調整します。

#### Kpゲインの影響

| Kp値 | 特性 | 用途 |
|------|------|------|
| 0.1-1.0 | 柔らかい、遅い応答 | 安全第一、人間との協働 |
| 1.0-10.0 | 中程度の応答 | 一般的な位置制御 |
| 10.0-50.0 | 硬い、速い応答 | 高精度位置決め |
| 50.0-500.0 | 非常に硬い | 固定（事実上剛体） |

#### Kdゲインの影響

| Kd値 | 特性 | 用途 |
|------|------|------|
| 0.001-0.01 | 弱いダンピング | 素早い応答、振動リスク |
| 0.01-0.1 | 中程度のダンピング | 一般的な用途 |
| 0.1-1.0 | 強いダンピング | 振動抑制、安定性重視 |
| 1.0-5.0 | 非常に強いダンピング | オーバーダンプ（遅い応答） |

#### 調整手順

1. **低いKpから開始**: `kp=1.0, kd=0.01`
2. **Kpを徐々に上げる**: 目標応答速度に達するまで
3. **振動が発生したらKdを上げる**: 振動が収まるまで
4. **微調整**: 実際の動作で最適化

**例: 柔らかい制御**
```xml
<param name="kp">1.5</param>
<param name="kd">0.01</param>
```

**例: 硬い制御**
```xml
<param name="kp">30.0</param>
<param name="kd">1.0</param>
```

### Controller Managerの周波数調整

`config/controllers.yaml`で調整:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 200  # 100, 200, 500, 1000等
```

**推奨値:**
- **100Hz**: 低速な動作、CPUリソース節約
- **200Hz**: 標準（デフォルト）
- **500Hz**: 高速な動作、リアルタイムカーネル推奨
- **1000Hz**: 非常に高速、RTカーネル必須

---

## モーション制御の例

### 例1: 正弦波動作

付属のスクリプトを使用:

```bash
# 正弦波動作（振幅1.0 rad, 周波数0.5 Hz）
pixi run ros2 run robstride_hardware sinusoidal_motion_publisher.py
```

**カスタマイズ:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class SinusoidalMotion(Node):
    def __init__(self):
        super().__init__('sinusoidal_motion')
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz
        self.t = 0.0

    def timer_callback(self):
        msg = Float64MultiArray()
        amplitude = 1.0  # 振幅 [rad]
        frequency = 0.5  # 周波数 [Hz]
        msg.data = [amplitude * math.sin(2 * math.pi * frequency * self.t)]
        self.publisher.publish(msg)
        self.t += 0.01

def main():
    rclpy.init()
    node = SinusoidalMotion()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### 例2: ステップ応答テスト

```bash
# 位置0 → 1 rad にステップ
pixi run ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0]"
sleep 2
pixi run ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0]"
```

**応答のプロット:**
```bash
# rostopicのプロット（rqt_plot使用）
pixi run ros2 run rqt_plot rqt_plot /robstride/joint_states/position[0]
```

### 例3: トラジェクトリ追従

カスタムコントローラーを使用:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.t = 0.0
        self.duration = 5.0  # トラジェクトリの長さ [s]

    def timer_callback(self):
        if self.t > self.duration:
            return

        # 5次多項式トラジェクトリ (0 → 1 rad, 5秒)
        s = self.t / self.duration
        position = 10*s**3 - 15*s**4 + 6*s**5  # 0→1に正規化

        msg = Float64MultiArray()
        msg.data = [position * 1.0]  # 目標: 1 rad
        self.publisher.publish(msg)
        self.t += 0.01

def main():
    rclpy.init()
    node = TrajectoryFollower()
    rclpy.spin(node)
```

---

## デバッグとトラブルシューティング

### CANバスのモニタリング

#### candump - CANトラフィックの確認

```bash
# 全てのCANフレームを表示
candump can0

# タイムスタンプ付き
candump -t A can0

# フィルタリング（Motor ID 127のみ）
candump can0,7F:7FF
```

**出力例:**
```
can0  013FFF7F  [8] 85 0B 7F FF 0F 5C 33 33  # コマンド
can0  027F007F  [8] 85 12 7F FE 00 12 00 00  # ステータス
```

#### cansend - 手動コマンド送信

```bash
# Motor 127をEnable
cansend can0 037F#

# Motor 127をDisable
cansend can0 047F#
```

### ログの確認

#### ROS 2ログレベルの変更

```bash
pixi run ros2 launch robstride_hardware bringup.launch.py --log-level debug
```

#### State Readerの統計情報

1秒ごとにログ出力される統計情報を確認:

```
[INFO] [RobStrideHardware]: [StateReader] pos: 0.123 rad, vel: 0.045 rad/s, torque: 0.012, valid: 98/100 (98.0%)
```

- **valid率が低い（<50%）**: CAN通信に問題あり
- **valid率が高い（>90%）**: 正常動作

### よくある問題と解決策

#### 問題1: モーターが動かない

**症状:**
```
[ERROR] [RobStrideHardware]: Failed to enable motor 127
```

**原因と対策:**
1. モーターの電源が入っていない → 電源を確認
2. CAN IDが間違っている → モーターのIDを確認
3. CANバスが起動していない → `ip link set can0 up`
4. 配線が間違っている → CAN_H/CAN_Lを確認

#### 問題2: 通信エラーが多発

**症状:**
```
[INFO] [RobStrideHardware]: valid: 10/100 (10.0%)
```

**原因と対策:**
1. 終端抵抗がない → 120Ωを両端に配置
2. ケーブルが長すぎる → <5mに短縮
3. ノイズ → シールドケーブル使用、GND接続
4. ビットレートが間違っている → 1Mbps確認

#### 問題3: モーターが振動する

**症状:**
- モーターが目標位置付近で振動
- 異音が発生

**原因と対策:**
1. Kpが高すぎる → Kpを下げる（例: 30 → 10）
2. Kdが低すぎる → Kdを上げる（例: 0.01 → 0.1）
3. 機械的共振 → 周波数を変更、ダンピング材追加

---

## 高度な使い方

### カスタムコントローラーの作成

#### Joint Trajectory Controllerの使用

`config/controllers.yaml`に追加:

```yaml
joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint1
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.5
```

Launch fileで起動:

```python
joint_trajectory_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_trajectory_controller'],
)
```

#### MoveIt2との統合

```bash
# MoveIt2設定生成
pixi run ros2 run moveit_setup_assistant moveit_setup_assistant

# URDF: robstride_system.urdf.xacro を選択
# Controller: joint_trajectory_controller を設定
```

### マルチモーターシステム

複数のモーターを制御する場合:

**URDF例:**
```xml
<ros2_control name="robstride_system" type="system">
  <hardware>
    <plugin>robstride_hardware/RobStrideHardware</plugin>
    <param name="can_interface">can0</param>
  </hardware>

  <joint name="joint1">
    <param name="motor_id">127</param>
    <param name="kp">1.5</param>
    <param name="kd">0.01</param>
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <joint name="joint2">
    <param name="motor_id">126</param>
    <param name="kp">2.0</param>
    <param name="kd">0.05</param>
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

**注意:** 現在の実装は1モーターのみ対応。マルチモーター対応には改修が必要。

### RVizでの可視化

```bash
# RVizを起動
rviz2

# 以下を追加:
# - RobotModel (robot_description トピック)
# - TF
# - JointState (joint_states トピック)
```

**launchファイルに追加:**
```python
rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', PathJoinSubstitution([
        FindPackageShare('robstride_hardware'),
        'rviz',
        'robstride.rviz'
    ])],
)
```

### パフォーマンスモニタリング

#### CPU使用率の確認

```bash
# Controller Managerのプロセス
top -p $(pgrep -f ros2_control_node)
```

#### リアルタイムトレース

```bash
# tracecmd でトレース
sudo trace-cmd record -e sched -e irq
sudo trace-cmd report
```

---

## まとめ

このガイドでは、RobStride Hardware Interfaceの基本的な使い方から高度な設定まで説明しました。

**次のステップ:**
1. 実際のロボットシステムに統合
2. MoveIt2でのモーションプランニング
3. カスタムコントローラーの開発
4. センサーフュージョン（力覚センサー等）

詳細な技術情報は以下を参照してください:
- [API Reference](API.md)
- [Technical Details](TECHNICAL_DETAILS.md)
- [Architecture Diagram](architecture.drawio)
