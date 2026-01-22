# RobStride Hardware - 技術詳細資料

本ドキュメントでは、RobStride Hardware Interfaceの内部実装、プロトコル仕様、パフォーマンス最適化について詳細に解説します。

## 目次

- [システムアーキテクチャ](#システムアーキテクチャ)
- [CANプロトコル仕様](#canプロトコル仕様)
- [リアルタイム性能](#リアルタイム性能)
- [スレッド設計](#スレッド設計)
- [データエンコーディング](#データエンコーディング)
- [エラーハンドリング](#エラーハンドリング)
- [パフォーマンス最適化](#パフォーマンス最適化)

---

## システムアーキテクチャ

### 設計思想

RobStride Hardware Interfaceは、以下の設計原則に基づいて実装されています:

1. **リアルタイム性の確保**: Controller Managerの200Hz RTループに影響を与えない
2. **スレッド分離**: CAN受信を専用スレッドで処理し、RTループと完全分離
3. **Lock-freeデータ共有**: `std::atomic`を使用してロックなしで状態を共有
4. **モジュール性**: RobStrideDriverはROS 2に依存せず、スタンドアロンで動作可能

### レイヤー構成

```
┌─────────────────────────────────────────────────┐
│  Application Layer (ROS 2)                      │
│  - Controllers, Planners, User nodes            │
└──────────────────┬──────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────┐
│  ros2_control Framework                         │
│  - Controller Manager (200Hz RT Loop)           │
│  - Hardware Interface Plugin System             │
└──────────────────┬──────────────────────────────┘
                   │ read() / write()
                   ▼
┌─────────────────────────────────────────────────┐
│  RobStrideHardware                              │
│  - ros2_control SystemInterface implementation  │
│  - Lifecycle management                         │
│  - RT Loop: write() → send_command()            │
│  - RT Loop: read() → atomic load                │
│  - State Reader Thread: CAN RX → atomic store   │
└──────────────────┬──────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────┐
│  RobStrideDriver                                │
│  - ROS-independent CAN driver                   │
│  - MIT protocol encode/decode                   │
│  - SocketCAN interface                          │
└──────────────────┬──────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────┐
│  Linux SocketCAN                                │
│  - Kernel-level CAN interface                   │
└──────────────────┬──────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────┐
│  CAN Hardware (CAN-USB adapter, etc.)           │
└──────────────────┬──────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────┐
│  RobStride Motor                                │
└─────────────────────────────────────────────────┘
```

---

## CANプロトコル仕様

### Extended CAN IDフォーマット

RobStrideモーターは29-bit Extended CAN IDを使用します。

#### 送信フレーム（Host → Motor）

```
Bits [28:24] (5-bit):  Communication Type
Bits [23:8]  (16-bit): Type-specific data
Bits [7:0]   (8-bit):  Motor ID (1-127)
```

#### 受信フレーム（Motor → Host）

```
Bits [28:24] (5-bit):  Communication Type
Bits [23:16] (8-bit):  Status/Error flags
Bits [15:8]  (8-bit):  Motor ID
Bits [7:0]   (8-bit):  Host ID
```

### Communication Types

| Type | Value | Name | Direction | Description |
|------|-------|------|-----------|-------------|
| 0 | 0x00 | GET_DEVICE_ID | Host→Motor | デバイスID取得 |
| 1 | 0x01 | OPERATION_CONTROL | Host→Motor | MIT制御コマンド |
| 2 | 0x02 | OPERATION_STATUS | Motor→Host | 状態フィードバック |
| 3 | 0x03 | ENABLE | Host→Motor | モーター有効化 |
| 4 | 0x04 | DISABLE | Host→Motor | モーター無効化 |
| 6 | 0x06 | SET_ZERO | Host→Motor | 現在位置をゼロに設定 |
| 18 | 0x12 | WRITE_PARAMETER | Host→Motor | パラメータ書き込み |

### MIT Control Protocol (Type 1)

#### コマンドフレーム（Host → Motor）

**CAN ID構成:**
```
Bits [28:24] = 0x01 (OPERATION_CONTROL)
Bits [23:8]  = torque_ff_u16 (フィードフォワードトルク)
Bits [7:0]   = motor_id
```

**データ (8 bytes, big-endian):**
```
Byte 0-1: Target Position (uint16_t, big-endian)
Byte 2-3: Target Velocity (uint16_t, big-endian)
Byte 4-5: Kp Gain (uint16_t, big-endian)
Byte 6-7: Kd Gain (uint16_t, big-endian)
```

**例:**
```
Motor ID: 127 (0x7F)
Position: 0.5 rad
Velocity: 0.0 rad/s
Kp: 30.0 Nm/rad
Kd: 1.0 Nm/(rad/s)
Torque FF: 0.0 Nm

CAN ID (hex): 0x01 3FFF 7F
           = (0x01 << 24) | (0x3FFF << 8) | 0x7F
           = 0x013FFF7F

Data (hex): [pos_h] [pos_l] [vel_h] [vel_l] [kp_h] [kp_l] [kd_h] [kd_l]
```

#### ステータスフレーム（Motor → Host）

**CAN ID構成:**
```
Bits [28:24] = 0x02 (OPERATION_STATUS)
Bits [23:16] = Status flags
Bits [15:8]  = motor_id
Bits [7:0]   = host_id
```

**データ (8 bytes, big-endian):**
```
Byte 0-1: Current Position (uint16_t)
Byte 2-3: Current Velocity (uint16_t)
Byte 4-5: Current Torque (uint16_t)
Byte 6-7: Reserved
```

### Parameter Write Protocol (Type 18)

パラメータ書き込みに使用。

**CAN ID構成:**
```
Bits [28:24] = 0x12 (WRITE_PARAMETER)
Bits [23:8]  = 0xFFFF (unused)
Bits [7:0]   = motor_id
```

**データ (8 bytes, little-endian):**
```
Byte 0-1: Parameter ID (uint16_t, little-endian)
Byte 2-3: Reserved (0x00)
Byte 4-7: Parameter Value (float or int8, little-endian)
```

**パラメータID一覧:**

| Parameter | ID | Type | Range | Description |
|-----------|------|------|-------|-------------|
| MODE | 0x7005 | int8_t | 0-3 | 制御モード（0=MIT, 1=Position, 2=Velocity, 3=Torque） |
| VELOCITY_LIMIT | 0x7017 | float | 0-44 | 速度リミット [rad/s] |
| TORQUE_LIMIT | 0x700B | float | 0-17 | トルクリミット [Nm] |

**例: MITモード設定**
```
Motor ID: 127
Parameter ID: 0x7005 (MODE)
Value: 0 (MIT mode)

CAN ID: 0x12FFFF7F
Data: [0x05, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
       ^^^^^ param_id (little-endian)
                                 ^^^^ value (int8)
```

---

## データエンコーディング

### スケーリングファクター（RS-02仕様）

RobStrideモーターは物理値をuint16にエンコードして送受信します。

| 物理量 | 記号 | 範囲 | スケールファクター | エンコード方式 |
|-------|------|------|-------------------|---------------|
| Position | θ | ±4π rad | `P_MAX = 4π` | Signed, centered at 0x7FFF |
| Velocity | ω | ±44 rad/s | `V_MAX = 44` | Signed, centered at 0x7FFF |
| Torque | τ | ±17 Nm | `T_MAX = 17` | Signed, centered at 0x7FFF |
| Kp | - | 0-500 Nm/rad | `KP_MAX = 500` | Unsigned, 0 = 0x0000 |
| Kd | - | 0-5 Nm/(rad/s) | `KD_MAX = 5` | Unsigned, 0 = 0x0000 |

### エンコーディング式

#### Signed Values (Position, Velocity, Torque)

物理値 → uint16:
```cpp
double normalized = (value / SCALE_MAX) + 1.0;  // -1~+1 → 0~2
uint16_t encoded = static_cast<uint16_t>(normalized * 0x7FFF);  // 0~2 → 0~0xFFFE
```

uint16 → 物理値:
```cpp
double normalized = static_cast<double>(encoded) / 0x7FFF;  // 0~0xFFFE → 0~2
double value = (normalized - 1.0) * SCALE_MAX;  // 0~2 → -SCALE_MAX~+SCALE_MAX
```

**例: Position = 0.5 rad**
```
P_MAX = 4π ≈ 12.566
normalized = (0.5 / 12.566) + 1.0 ≈ 1.0398
encoded = 1.0398 * 32767 ≈ 34067 (0x850B)
```

#### Unsigned Values (Kp, Kd)

物理値 → uint16:
```cpp
double normalized = value / SCALE_MAX;  // 0~SCALE_MAX → 0~1
uint16_t encoded = static_cast<uint16_t>(normalized * 0xFFFF);  // 0~1 → 0~0xFFFF
```

uint16 → 物理値:
```cpp
double normalized = static_cast<double>(encoded) / 0xFFFF;  // 0~0xFFFF → 0~1
double value = normalized * SCALE_MAX;  // 0~1 → 0~SCALE_MAX
```

**例: Kp = 30.0 Nm/rad**
```
KP_MAX = 500
normalized = 30.0 / 500 = 0.06
encoded = 0.06 * 65535 ≈ 3932 (0x0F5C)
```

### バイトオーダー

- **MIT Control/Status フレーム**: Big-endian（ネットワークバイトオーダー）
- **Parameter Write フレーム**: Little-endian（x86ネイティブ）

**Big-endian packing:**
```cpp
void pack_u16_be(uint8_t* buf, uint16_t val) {
    buf[0] = (val >> 8) & 0xFF;  // 上位バイト
    buf[1] = val & 0xFF;         // 下位バイト
}
```

**Little-endian packing:**
```cpp
void pack_u16_le(uint8_t* buf, uint16_t val) {
    buf[0] = val & 0xFF;         // 下位バイト
    buf[1] = (val >> 8) & 0xFF;  // 上位バイト
}
```

---

## リアルタイム性能

### RTループのタイミング

Controller Managerは200Hzの固定周期でRTループを実行します。

```
Period: 5ms (200Hz)
Deadline: 5ms以内に read() + write() 完了必須
```

#### タイミング内訳

| 処理 | 実行時間 | 説明 |
|------|----------|------|
| `read()` | < 1μs | atomic変数からのコピーのみ |
| Controller計算 | ~100-500μs | 制御則の計算（コントローラー依存） |
| `write()` | ~50-100μs | CANフレーム送信 |
| **合計** | ~200-600μs | 5msの deadline に対して余裕あり |

### 非RTスレッド（State Reader）のタイミング

State Reader Threadは100Hzで動作し、RTループとは完全に独立しています。

```
Period: 10ms (100Hz)
Priority: 通常優先度（非RT）
```

#### タイミング内訳

| 処理 | 実行時間 | 説明 |
|------|----------|------|
| CAN受信（最大20フレーム） | ~500-1000μs | 非ブロッキング読み取り |
| デコード処理 | ~50μs | uint16 → double変換 |
| atomic store | < 1μs | Lock-free書き込み |
| ROS 2パブリッシュ | ~100-200μs | /robstride/joint_states |
| **合計** | ~700-1300μs | 10msの周期に対して余裕あり |

### レイテンシ分析

#### コマンド→アクチュエーション遅延

```
User Command
    ↓
Controller (1 cycle = 5ms)
    ↓
write() (~100μs)
    ↓
CAN Bus (1Mbps, ~100μs for 8-byte frame)
    ↓
Motor Processing (~500μs)
    ↓
Actuation

Total: ~5.7ms
```

#### センサー→フィードバック遅延

```
Motor State Change
    ↓
CAN Bus (~100μs)
    ↓
State Reader (最大10ms待機)
    ↓
atomic store (~1μs)
    ↓
read() (次のRT cycle, 最大5ms待機)
    ↓
Controller

Total: ~15.1ms (worst case)
```

---

## スレッド設計

### スレッド構成

RobStrideHardwareは2つのスレッドで動作します:

1. **RT Loop Thread** (Controller Manager管理)
2. **State Reader Thread** (RobStrideHardware管理)

### RT Loop Thread

**特性:**
- 周波数: 200Hz (5ms周期)
- 優先度: リアルタイム優先度（設定による）
- スケジューリング: SCHED_FIFO または SCHED_RR
- 処理: `read()` → Controller計算 → `write()`

**設計原則:**
- **ブロッキング操作禁止**: mutexロック、条件変数待機、ブロッキングI/O禁止
- **メモリ確保禁止**: 動的メモリ確保（new, malloc）禁止
- **高速処理**: 各関数は数百μs以内に完了すること

**read()の実装:**
```cpp
hardware_interface::return_type RobStrideHardware::read(...) {
    // 完全に非ブロッキング（atomic変数からコピー）
    hw_positions_[0] = latest_position_.load(std::memory_order_relaxed);
    hw_velocities_[0] = latest_velocity_.load(std::memory_order_relaxed);
    hw_efforts_[0] = latest_torque_.load(std::memory_order_relaxed);
    return hardware_interface::return_type::OK;
}
```

**write()の実装:**
```cpp
hardware_interface::return_type RobStrideHardware::write(...) {
    // CANフレーム送信（非ブロッキングソケット）
    MitCommand cmd;
    cmd.position = hw_commands_position_[0];
    cmd.kp = kp_;
    cmd.kd = kd_;
    driver_.send_command(motor_id_, cmd);  // ~50-100μs
    return hardware_interface::return_type::OK;
}
```

### State Reader Thread

**特性:**
- 周波数: 100Hz (10ms周期)
- 優先度: 通常優先度（非RT）
- スケジューリング: SCHED_OTHER
- 処理: CAN受信 → デコード → atomic store → ROS 2パブリッシュ

**設計原則:**
- **RTループと独立**: RTループに一切影響を与えない
- **Lock-free通信**: atomic変数でRTループと通信
- **ベストエフォート**: 受信失敗してもエラーにしない

**実装:**
```cpp
void RobStrideHardware::state_reader_loop() {
    const auto period = std::chrono::milliseconds(10);  // 100Hz
    auto next_time = std::chrono::steady_clock::now();

    while (state_reader_running_.load(std::memory_order_acquire)) {
        // CAN受信（非ブロッキング）
        auto state = driver_.read_state(motor_id_);

        if (state.valid) {
            // atomic変数に格納（lock-free）
            latest_position_.store(state.position, std::memory_order_relaxed);
            latest_velocity_.store(state.velocity, std::memory_order_relaxed);
            latest_torque_.store(state.torque, std::memory_order_relaxed);

            // ROS 2パブリッシュ（RTループに影響なし）
            msg.position[0] = state.position;
            joint_state_pub_->publish(msg);
        }

        // 固定周期でスリープ
        next_time += period;
        std::this_thread::sleep_until(next_time);
    }
}
```

### Lock-freeデータ共有

**atomic変数の使用:**
```cpp
// ヘッダーファイル
std::atomic<double> latest_position_{0.0};
std::atomic<double> latest_velocity_{0.0};
std::atomic<double> latest_torque_{0.0};
```

**メモリオーダー:**
- **relaxed**: 順序保証不要の場合（位置・速度・トルクは独立）
- **acquire/release**: スレッド起動・停止のフラグ管理

```cpp
// State Reader Thread (書き込み)
latest_position_.store(new_value, std::memory_order_relaxed);

// RT Loop (読み込み)
double pos = latest_position_.load(std::memory_order_relaxed);
```

---

## エラーハンドリング

### CANエラーの検出

#### 送信エラー

```cpp
bool RobStrideDriver::send_frame(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    ssize_t nbytes = ::write(can_socket_, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        // 送信失敗（バッファフル、インターフェースダウン等）
        return false;
    }
    return true;
}
```

**対処:**
- `write()`では失敗をログ出力せず、単にfalseを返す
- 上位レイヤー（RobStrideHardware）でエラーハンドリング

#### 受信エラー

```cpp
MotorState RobStrideDriver::read_state(int motor_id) {
    MotorState state;  // valid = false

    for (int attempts = 0; attempts < 20; ++attempts) {
        if (!read_frame(&frame)) {
            break;  // データなし
        }

        // フレームチェック & デコード
        if (/* 正しいフレーム */) {
            state.valid = true;
            // デコード処理
            return state;
        }
    }

    return state;  // valid = false
}
```

**対処:**
- State Reader Threadでは、`valid == false`でもエラーにしない
- 統計情報をログ出力（1秒ごと）

### モーターエラー

#### Status Flagsの解析

受信フレームのbits [23:16]にエラーフラグが含まれます（詳細は非公開）。

**一般的なエラー:**
- 過電流
- 過熱
- 位置リミット超過
- 通信タイムアウト

**対処:**
現在の実装ではStatus Flagsを無視していますが、将来的には以下の対応を検討:
```cpp
uint8_t status_flags = (raw_id >> 16) & 0xFF;
if (status_flags & ERROR_OVERCURRENT) {
    // 過電流エラー処理
}
```

---

## パフォーマンス最適化

### 最適化手法

#### 1. 非ブロッキングソケット

CANソケットを非ブロッキングに設定し、読み取り時の待機を回避:

```cpp
int flags = fcntl(can_socket_, F_GETFL, 0);
fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
```

#### 2. メモリ事前確保

RT loop内でのメモリ確保を回避:

```cpp
// on_init()で確保
hw_positions_.resize(info_.joints.size(), 0.0);
hw_velocities_.resize(info_.joints.size(), 0.0);
hw_commands_position_.resize(info_.joints.size(), 0.0);
```

State Reader Threadでも事前確保:

```cpp
// state_reader_loop()の開始時
sensor_msgs::msg::JointState msg;
msg.name.push_back(joint_name_);
msg.position.resize(1);
msg.velocity.resize(1);
msg.effort.resize(1);

while (...) {
    // msg再利用（メモリ確保なし）
    msg.position[0] = state.position;
    joint_state_pub_->publish(msg);
}
```

#### 3. インライン関数

エンコード・デコード関数をインライン化:

```cpp
namespace {
inline void pack_u16_be(uint8_t* buf, uint16_t val) {
    buf[0] = (val >> 8) & 0xFF;
    buf[1] = val & 0xFF;
}
}
```

#### 4. スレッド分離

CAN受信を専用スレッドに分離し、RTループへの影響を排除:

```
RT Loop (200Hz)          State Reader (100Hz)
     │                          │
  write() ──CAN TX──>          │
     │                          │
  read() <──atomic──────CAN RX──
     │                          │
```

#### 5. バッファリング

State Readerは最大20フレームまで一度に読み取り、最新の状態のみ使用:

```cpp
for (int attempts = 0; attempts < 20; ++attempts) {
    if (!read_frame(&frame)) break;
    // 最後に見つかった有効フレームを採用
}
```

### ベンチマーク結果（参考値）

測定環境: Intel Core i5, Ubuntu 22.04, Kernel 5.15, CAN-USB adapter

| 処理 | 平均時間 | 最大時間 | 備考 |
|------|----------|----------|------|
| `read()` | 0.5μs | 2μs | atomic load only |
| `write()` | 80μs | 150μs | CAN TX |
| State Reader (1 cycle) | 800μs | 1500μs | CAN RX + decode + publish |
| Controller Manager (1 cycle) | 500μs | 1200μs | read + control + write |

**結論:**
- RTループは5ms deadline に対して十分余裕あり
- State Readerは10ms周期に対して余裕あり

---

## セキュリティ考慮事項

### CANバスセキュリティ

CANバスは認証・暗号化機能を持たないため、以下のリスクがあります:

1. **不正コマンド送信**: 他のホストからモーターを制御可能
2. **盗聴**: CANバス上の全トラフィックが可視
3. **DoS攻撃**: 大量のフレームでバスを飽和

**対策:**
- 物理的にCANバスを保護（外部アクセス禁止）
- ホストIDによる送信元識別（十分ではない）
- CAN-FDやCAN-FD SECなどのセキュア版CANの検討

### ソフトウェアセキュリティ

1. **権限管理**: CANソケットは`CAP_NET_RAW`権限が必要
2. **入力検証**: URDFパラメータの範囲チェック
3. **リソース制限**: スレッド数、メモリ使用量の制限

---

## 今後の改善案

1. **マルチモーター対応**: 複数モーターを1つのHardware Interfaceで管理
2. **エラーリカバリ**: 通信エラー時の自動再接続
3. **CAN-FD対応**: より高速な通信プロトコル
4. **Diagnostics統合**: ROS 2 Diagnosticsでモーター状態を監視
5. **パラメータ動的変更**: Kp, Kdをランタイムで変更可能に
6. **トルクセンサーフィードバック**: より正確な力制御

---

## 参考文献

- [SocketCAN Documentation](https://www.kernel.org/doc/Documentation/networking/can.txt)
- [ROS 2 Control Documentation](https://control.ros.org/)
- [RobStride Motor Datasheet](https://www.robstride.com/) (要NDA)
- [Real-Time Linux](https://wiki.linuxfoundation.org/realtime/start)

---

このドキュメントは、RobStride Hardware Interfaceの内部動作を理解し、パフォーマンス調整やトラブルシューティングを行うためのリファレンスとして使用してください。
