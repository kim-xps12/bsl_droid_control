# RobStride ROS 2 Control 実装ガイド

RobStride RS02モーターをros2_controlフレームワークで制御するためのHardware Interfaceプラグイン実装。

## システムアーキテクチャ

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS 2 Application Layer                   │
│  (Controller Nodes, MoveIt2, Nav2, etc.)                    │
└───────────────────────────┬─────────────────────────────────┘
                            │ ROS 2 Topics/Services
┌───────────────────────────▼─────────────────────────────────┐
│              Controller Manager (200Hz RT Loop)              │
│  - Forward Command Controller                                │
│  - Joint State Broadcaster                                   │
│  - (その他のコントローラー)                                  │
└───────────────────────────┬─────────────────────────────────┘
                            │ Hardware Interface API
┌───────────────────────────▼─────────────────────────────────┐
│         RobStrideHardware (SystemInterface)                  │
│  - read(): モーター状態読み取り                             │
│  - write(): コマンド送信                                     │
│  - Lifecycle管理 (configure/activate/deactivate)            │
└───────────────────────────┬─────────────────────────────────┘
                            │ Driver API
┌───────────────────────────▼─────────────────────────────────┐
│              RobStrideDriver (ROS非依存)                     │
│  - CAN通信管理 (SocketCAN)                                  │
│  - MITプロトコル実装                                         │
│  - enable/disable/set_mode                                   │
│  - send_command/read_state                                   │
└───────────────────────────┬─────────────────────────────────┘
                            │ SocketCAN (Linux)
┌───────────────────────────▼─────────────────────────────────┐
│                    CAN Hardware (can1)                       │
│  - USB CAN Adapter (gs_usb)                                  │
│  - Bitrate: 1 Mbps                                           │
└───────────────────────────┬─────────────────────────────────┘
                            │ CAN Bus
┌───────────────────────────▼─────────────────────────────────┐
│              RobStride RS02 Motor (ID: 127)                  │
│  - MIT Control Mode                                          │
│  - Position/Velocity/Torque control                          │
└─────────────────────────────────────────────────────────────┘
```

## 実装コンポーネント

### 1. RobStrideDriver (ドライバークラス)

**パス:** `ros2_ws/src/robstride_hardware/`
- `include/robstride_hardware/robstride_driver.hpp`
- `src/robstride_driver.cpp`

**役割:** ROS 2非依存のCAN通信・プロトコル層

**主要機能:**
```cpp
class RobStrideDriver {
  // 接続管理
  bool connect(const std::string& interface);
  void disconnect();
  
  // モーター制御
  bool enable(int motor_id);
  bool disable(int motor_id);
  bool set_mode(int motor_id, ControlMode mode);
  
  // リアルタイム通信
  bool send_command(int motor_id, const MitCommand& cmd);
  MotorState read_state(int motor_id);
};
```

**特徴:**
- **Non-blocking I/O**: `O_NONBLOCK`フラグでソケットを完全非同期化
  - ✅ **メリット**: リアルタイムループをブロックしない、決定論的な実行時間（~10μs）
  - ⚠️ **デメリット**: データ未到着時も即座に返る（valid=0）、ポーリングによる若干のCPUコスト
  - **なぜ適切か**: Controller Managerが200Hzで定期的にread()を呼ぶため、タイムアウト待機は不要。非同期化により制御周期の安定性が向上
- **バッファクリア**: 最大20フレームを読み取り、最新データのみ使用
  - 制御ループの遅延を最小化（古いデータを捨てて最新値のみ使用）
- **MITモード対応**: Position, Velocity, Kp, Kd, Torque feedforwardをサポート

### 2. RobStrideHardware (Hardware Interface)

**パス:** `ros2_ws/src/robstride_hardware/`
- `include/robstride_hardware/robstride_hardware.hpp`
- `src/robstride_hardware.cpp`

**役割:** ros2_control標準インターフェース実装

**ライフサイクル:**
```cpp
on_init()      // URDF読み込み、パラメータ初期化
  ↓
on_configure() // CAN接続
  ↓
on_activate()  // モーター有効化、MIT mode設定
  ↓
[read()/write() @ 200Hz] // リアルタイム制御ループ
  ↓
on_deactivate() // セーフティコマンド送信、モーター無効化
  ↓
on_cleanup()   // CAN切断
```

**State/Command Interfaces:**
| Interface | Type | Description |
|-----------|------|-------------|
| `position` | State | モーター位置 (rad) |
| `velocity` | State | モーター速度 (rad/s) |
| `effort` | State | モータートルク (Nm) |
| `position` | Command | 目標位置 (rad) |

## 制御ループ詳細

### read() メソッド (200Hz)

```cpp
hardware_interface::return_type read(Time, Duration) {
  // 1. 開始時刻記録
  auto start = steady_clock::now();
  
  // 2. モーター状態読み取り (non-blocking, ~10μs)
  auto state = driver_.read_state(motor_id_);
  if (state.valid) {
    hw_positions_[0] = state.position;
    hw_velocities_[0] = state.velocity;
    hw_efforts_[0] = state.torque;
  }
  
  // 3. 統計収集 (1秒ごとにログ出力)
  // - 実行時間: ~10μs
  // - 周期: 4999μs (200.0Hz)
  
  return OK;
}
```

### write() メソッド (200Hz)

```cpp
hardware_interface::return_type write(Time, Duration) {
  // 1. 開始時刻記録
  auto start = steady_clock::now();
  
  // 2. MITコマンド構築
  MitCommand cmd;
  cmd.position = hw_commands_position_[0];  // Controller から
  cmd.velocity = 0.0;
  cmd.kp = 30.0;  // URDF設定値
  cmd.kd = 1.0;
  cmd.torque_ff = 0.0;
  
  // 3. CAN送信 (non-blocking, ~17μs)
  driver_.send_command(motor_id_, cmd);
  
  // 4. 統計収集
  // - 実行時間: ~17μs
  // - 周期: 5000μs (200.0Hz)
  
  return OK;
}
```

### パフォーマンス指標

| 項目 | READ | WRITE | 合計 |
|------|------|-------|------|
| **実行時間** | 9-12 μs | 16-19 μs | ~27 μs |
| **周期** | 4999 μs | 5000 μs | - |
| **実周波数** | **200.0 Hz** | **200.0 Hz** | - |
| **CPU使用率** | 0.2% | 0.34% | **0.54%** |
| **余裕率** | - | - | **99.46%** |

## 設定ファイル

### URDF (Robot Description)

**パス:** `ros2_ws/src/robstride_hardware/urdf/robstride_system.urdf.xacro`

```xml
<ros2_control name="robstride_system" type="system">
  <hardware>
    <plugin>robstride_hardware/RobStrideHardware</plugin>
    <param name="can_interface">can1</param>
    <param name="motor_id">127</param>
    <param name="kp">30.0</param>
    <param name="kd">1.0</param>
  </hardware>
  
  <joint name="joint1">
    <command_interface name="position">
      <param name="min">-3.14</param>
      <param name="max">3.14</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

### Controller Configuration

**パス:** `ros2_ws/src/robstride_hardware/config/controllers.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 200  # Hz
    
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    
    forward_position_controller:
      type: forward_command_controller/ForwardCommandController

forward_position_controller:
  ros__parameters:
    joints:
      - joint1
    interface_name: position
```

### Launch File

**パス:** `ros2_ws/src/robstride_hardware/launch/test_control.launch.py`

```python
def generate_launch_description():
    # URDF読み込み
    robot_description = Command(['xacro ', urdf_file])
    
    # Controller Manager起動
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file
        ]
    )
    
    # Robot State Publisher起動
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    return LaunchDescription([
        controller_manager,
        robot_state_publisher
    ])
```

## 実行方法

### 1. CAN準備

```bash
# CAN1インターフェースの設定 (gs_usb)
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

# 状態確認
ip -d link show can1
# → can state ERROR-ACTIVE であること
```

### 2. ビルド

```bash
cd ~/bsl_droid_ros2/ros2_ws
pixi run colcon build --packages-select robstride_hardware
source install/setup.bash
```

### 3. 起動

```bash
# Controller Manager + Hardware Interface起動
pixi run ros2 launch robstride_hardware test_control.launch.py
```

**期待される出力:**
```
[RobStrideHardware]: Initialized: can=can1, motor_id=127, kp=30.0, kd=1.0
[RobStrideHardware]: Connected to CAN interface: can1
[RobStrideHardware]: Activating...
[RobStrideHardware]: Motor 127 activated in MIT mode

# 1秒ごとに統計が出力される
[RobStrideHardware]: [READ] exec: 10.4 μs, period: 4999.3 μs, freq: 200.0 Hz | pos: 0.000, vel: 0.000, valid: 0
[RobStrideHardware]: [WRITE] exec: 18.6 μs, period: 5000.6 μs, freq: 200.0 Hz | cmd_pos: 0.000
```

```bash
pixi run ros2 launch robstride_hardware test_sinusoidal_motion.launch.py
```

### 4. コマンド送信

別ターミナルで目標位置を送信:

```bash
# Forward Position Controllerをロード（launchファイル起動時は自動ロードされる）
pixi run ros2 control load_controller forward_position_controller --set-state active

# 位置コマンド送信 (1.7 rad)
# ⚠️ 重要: -1 オプション（1回送信）または -r 指定（周波数設定）が必要
# デフォルトは1Hzなので、応答性が非常に悪い

# 推奨方法1: 1回送信（コントローラーが値を保持）
pixi run ros2 topic pub -1 /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.7]"

# 推奨方法2: 高頻度で送信（200Hz）
pixi run ros2 topic pub -r 200 /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.7]"

# ⚠️ 非推奨: デフォルト（1Hz送信 = 遅い応答）
# pixi run ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.7]"
```

**なぜ応答が遅いのか？**
- `ros2 topic pub`はデフォルトで **1Hz** でメッセージを送信
- Forward Command Controllerは最後に受信した値を保持するが、新しい値が来るまで更新されない
- 制御ループは200Hzで回っているが、**目標値の更新が1Hzなので、1秒かけて徐々に目標値が変わる**
- `-1`オプションで1回送信すれば、すぐに目標値に到達する（Kp/Kdゲインに依存）

### 5. 状態確認

```bash
# Joint States確認
pixi run ros2 topic echo /joint_states

# Controller Manager統計
pixi run ros2 topic echo /controller_manager/statistics/full --once
```

### 6. 停止

```bash
# Ctrl+C でlaunchファイルを終了
# モーターは自動的にdeactivate (トルクOFF) される
```

## パラメータ調整

### 位置制御ゲイン

**URDF内で調整:**
```xml
<param name="kp">30.0</param>  <!-- 位置ゲイン: 高いほど応答速い/振動しやすい -->
<param name="kd">1.0</param>   <!-- 微分ゲイン: 高いほど減衰が強い -->
```

**推奨値 (RS02):**
- Kp: 20-50 (低負荷) / 50-100 (高負荷)
- Kd: 0.5-2.0

**応答速度の調整:**
- 応答が遅い → Kpを上げる（例: 30→50）
- 振動する → Kdを上げる（例: 1.0→2.0）
- オーバーシュート → Kdを上げる、Kpを下げる

### 制御周期

**Controller Manager設定:**
```yaml
update_rate: 200  # Hz (推奨: 100-500Hz)
```

**注意:** 1000Hz以上にすると処理が間に合わない可能性あり

## トラブルシューティング

### 1. "BUS-OFF" エラー

**症状:** `ip -d link show can1` で `state BUS-OFF`

**原因:**
- モーターの電源が入っていない
- CANケーブル接続不良
- ボーレート不一致

**対処:**
```bash
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

### 2. "valid: 0" (モーター応答なし)

**症状:** ログに `valid: 0` と表示

**原因:**
- モーターがDISABLE状態
- モーターIDが違う
- MITモードに入っていない

**対処:**
```bash
# 手動でENABLE送信
cansend can1 03FF007F#

# 応答確認
candump can1
```

### 3. "Overrun detected" 警告

**症状:** Controller Managerが周期を逃す

**原因:**
- 制御周期が速すぎる (500Hz以上)
- リアルタイム権限がない

**対処:**
```bash
# update_rateを下げる (200Hz推奨)
# または、リアルタイム権限を付与:
sudo setcap cap_sys_nice+ep /path/to/ros2_control_node
```

### 4. モーターが動かない

**確認項目:**
1. CAN状態: `ip -d link show can1`
2. モーター電源: LEDは点灯しているか
3. モーターID: URDFの`motor_id`は正しいか
4. コントローラーロード: `ros2 control list_controllers`

### 5. 応答が遅い（目標位置到達に時間がかかる）

**症状:** `ros2 topic pub`でコマンドを送っても、モーターがゆっくりしか動かない

**原因と対処:**

1. **`ros2 topic pub`のデフォルト1Hz送信** ← **最も多い原因**
   ```bash
   # ✅ 解決策: -1 オプションで1回送信
   pixi run ros2 topic pub -1 /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.7]"
   ```

2. **Kpゲインが低すぎる**
   - URDFの`kp`を30→50に上げる
   - リビルドと再起動が必要

3. **モーター応答が遅い（valid: 0が多い）**
   - `candump can1`で応答フレームを確認
   - モーターのファームウェア設定を確認

## 参考資料

- **ROS 2 Control公式**: https://control.ros.org/
- **RobStride公式**: https://github.com/Seeed-Projects/RobStride_Control
- **本プロジェクトのドキュメント**:
  - [ros2_control マニュアル](ros2_control_manual.md)
  - [アーキテクチャ図](arch_ros2_control.md)
  - [実装リファレンス](robstride_hardware_impl.md)

## 制限事項

1. **現在の実装はMITモードのみ対応**
   - Position/Velocity/Torqueモードは未実装
2. **単一モーター制御**
   - 複数モーターは未対応（URDFで複数joint定義すれば可能）
3. **状態フレームの取得が不安定**
   - `valid: 0` が多い場合、プロトコル実装を要確認
4. **リアルタイムスケジューリング未対応**
   - 厳密なRT制御が必要な場合、RT_PREEMPTカーネルが必要

## 設計判断の補足

### O_NONBLOCK vs タイムアウト方式

**現在の実装**: `O_NONBLOCK` (非同期I/O)

**代替案との比較:**

| 方式 | 実行時間 | 周期安定性 | CPU使用率 | 実装複雑度 |
|------|----------|-----------|-----------|-----------|
| **O_NONBLOCK** | ~10μs | ◎ 安定 | 0.2% | ● 低 |
| タイムアウト1ms | 1000-1010μs | △ 不安定 | 20% | ● 低 |
| select/poll | 可変 | ○ 中 | 0.3% | ●● 中 |
| 非同期I/O (aio) | ~10μs | ◎ 安定 | 0.2% | ●●● 高 |

**O_NONBLOCKを選んだ理由:**
1. **制御周期が固定** (200Hz): Controller Managerが5msごとに確実に呼び出すため、ブロッキングは不要
2. **決定論的**: 常に~10μsで返るため、リアルタイム性が高い
3. **シンプル**: `fcntl(F_SETFL, O_NONBLOCK)` のみで実装可能

**注意点:**
- モーターが応答を返すタイミングと制御周期は非同期
- データ未到着時は前回値を保持（`valid=0`の場合）
- 高頻度で状態更新が必要な場合は、モーター側の送信頻度を上げる必要あり

## 開発メモ

### バージョン
- ROS 2: Jazzy
- ros2_control: 4.x
- Platform: Jetson Orin Nano (Ubuntu 22.04)
- CAN Adapter: gs_usb (canable2)

### 性能測定結果 (2026-01-22)
- READ: 9-12 μs, 200.0 Hz
- WRITE: 16-19 μs, 200.0 Hz
- CPU余裕率: 99.46%

### 今後の改善案
1. 状態フレーム取得の安定化
2. 複数モーター対応
3. Velocity/Torqueモード実装
4. エラーハンドリング強化
5. リアルタイムスケジューリング対応
