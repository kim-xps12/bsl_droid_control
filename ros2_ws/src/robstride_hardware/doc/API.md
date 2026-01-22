# RobStride Hardware API Reference

このドキュメントでは、`robstride_hardware`パッケージの詳細なAPI仕様を説明します。

## 目次

- [RobStrideDriver クラス](#robstridedriver-クラス)
- [RobStrideHardware クラス](#robstridehardware-クラス)
- [データ構造](#データ構造)
- [プロトコル定数](#プロトコル定数)

---

## RobStrideDriver クラス

**ファイル**: `include/robstride_hardware/robstride_driver.hpp`

RobStrideモーターとの低レベルCAN通信を担当するドライバークラス。ROS 2に依存せず、スタンドアロンで使用可能。

### 概要

```cpp
namespace robstride_driver {
    class RobStrideDriver {
    public:
        RobStrideDriver() = default;
        ~RobStrideDriver();

        // 接続管理
        bool connect(const std::string& interface);
        void disconnect();
        bool is_connected() const;

        // モーター制御
        bool enable(int motor_id);
        bool disable(int motor_id);
        bool set_mode(int motor_id, ControlMode mode);

        // パラメータ設定
        bool set_velocity_limit(int motor_id, float limit);
        bool set_torque_limit(int motor_id, float limit);

        // 通信
        bool send_command(int motor_id, const MitCommand& cmd);
        MotorState read_state(int motor_id);

        // ホストID設定
        void set_host_id(int host_id);
    };
}
```

### メソッド詳細

#### `bool connect(const std::string& interface)`

CANインターフェースに接続します。

**パラメータ**:
- `interface`: CANインターフェース名（例: `"can0"`, `"can1"`）

**戻り値**:
- `true`: 接続成功
- `false`: 接続失敗（インターフェースが存在しない、権限がない等）

**使用例**:
```cpp
robstride_driver::RobStrideDriver driver;
if (!driver.connect("can0")) {
    std::cerr << "Failed to connect to CAN interface" << std::endl;
    return -1;
}
```

**注意事項**:
- CANインターフェースは事前に起動しておく必要があります
- `sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up`

---

#### `void disconnect()`

CANインターフェースから切断します。

**使用例**:
```cpp
driver.disconnect();
```

**注意事項**:
- デストラクタでも自動的に呼ばれます
- モーターは自動的に無効化されないため、事前に`disable()`を呼ぶことを推奨

---

#### `bool is_connected() const`

CANインターフェースへの接続状態を確認します。

**戻り値**:
- `true`: 接続中
- `false`: 未接続

---

#### `bool enable(int motor_id)`

モーターを有効化します（通電状態）。

**パラメータ**:
- `motor_id`: モーターのCAN ID（1-127）

**戻り値**:
- `true`: コマンド送信成功
- `false`: 送信失敗

**CANフレーム**:
```
CAN ID: 0x03FFFF7F (例: motor_id=127, host_id=0xFF)
DLC: 0
```

**使用例**:
```cpp
if (!driver.enable(127)) {
    std::cerr << "Failed to enable motor" << std::endl;
}
```

**注意事項**:
- モーターが有効化されるまで約100ms程度かかる場合があります
- 有効化後、すぐにコマンドを送信すると無視される可能性があります

---

#### `bool disable(int motor_id)`

モーターを無効化します（脱力状態）。

**パラメータ**:
- `motor_id`: モーターのCAN ID（1-127）

**戻り値**:
- `true`: コマンド送信成功
- `false`: 送信失敗

**CANフレーム**:
```
CAN ID: 0x04FFFF7F (例: motor_id=127, host_id=0xFF)
DLC: 0
```

**使用例**:
```cpp
driver.disable(127);
```

---

#### `bool set_mode(int motor_id, ControlMode mode)`

モーターの制御モードを設定します。

**パラメータ**:
- `motor_id`: モーターのCAN ID
- `mode`: 制御モード（`ControlMode::MIT`, `POSITION`, `VELOCITY`, `TORQUE`）

**戻り値**:
- `true`: コマンド送信成功
- `false`: 送信失敗

**制御モード**:
```cpp
enum class ControlMode : int8_t {
    MIT = 0,        // MIT制御（位置・速度・トルク統合）
    POSITION = 1,   // 位置制御のみ
    VELOCITY = 2,   // 速度制御のみ
    TORQUE = 3      // トルク制御のみ
};
```

**使用例**:
```cpp
driver.set_mode(127, robstride_driver::ControlMode::MIT);
```

---

#### `bool set_velocity_limit(int motor_id, float limit)`

速度リミットを設定します。

**パラメータ**:
- `motor_id`: モーターのCAN ID
- `limit`: 速度リミット [rad/s]（0-44 rad/s for RS-02）

**戻り値**:
- `true`: 送信成功
- `false`: 送信失敗

---

#### `bool set_torque_limit(int motor_id, float limit)`

トルクリミットを設定します。

**パラメータ**:
- `motor_id`: モーターのCAN ID
- `limit`: トルクリミット [Nm]（0-17 Nm for RS-02）

**戻り値**:
- `true`: 送信成功
- `false`: 送信失敗

---

#### `bool send_command(int motor_id, const MitCommand& cmd)`

MITコントロールコマンドを送信します。

**パラメータ**:
- `motor_id`: モーターのCAN ID
- `cmd`: MITコマンド構造体

**戻り値**:
- `true`: 送信成功
- `false`: 送信失敗

**使用例**:
```cpp
robstride_driver::MitCommand cmd;
cmd.position = 0.5;      // 目標位置 [rad]
cmd.velocity = 0.0;      // 目標速度 [rad/s]
cmd.kp = 30.0;           // 位置ゲイン [Nm/rad]
cmd.kd = 1.0;            // 速度ゲイン [Nm/(rad/s)]
cmd.torque_ff = 0.0;     // フィードフォワードトルク [Nm]

driver.send_command(127, cmd);
```

**内部処理**:
1. 各値をスケーリングファクターでクランプ
2. uint16にエンコード（big-endian）
3. CANフレームに格納して送信

---

#### `MotorState read_state(int motor_id)`

モーターの現在状態を読み取ります。

**パラメータ**:
- `motor_id`: モーターのCAN ID

**戻り値**:
- `MotorState` 構造体（`valid`フラグをチェックすること）

**使用例**:
```cpp
auto state = driver.read_state(127);
if (state.valid) {
    std::cout << "Position: " << state.position << " rad" << std::endl;
    std::cout << "Velocity: " << state.velocity << " rad/s" << std::endl;
    std::cout << "Torque: " << state.torque << " Nm" << std::endl;
} else {
    std::cout << "No valid state received" << std::endl;
}
```

**内部処理**:
1. CANバッファから最大20フレームまで読み取り
2. `OPERATION_STATUS`タイプで、かつ該当motor_idのフレームを抽出
3. 最新の状態をデコードして返す

**注意事項**:
- 非ブロッキング読み取り（データがなければすぐに返る）
- モーターはコマンド受信後に状態を返すため、`send_command()`の後に呼ぶことを推奨

---

#### `void set_host_id(int host_id)`

ホストIDを設定します（デフォルト: 0xFF）。

**パラメータ**:
- `host_id`: ホストID（0-255）

**用途**:
- 複数のホストが同じCANバスに接続されている場合に識別するため
- 通常は変更不要

---

## RobStrideHardware クラス

**ファイル**: `include/robstride_hardware/robstride_hardware.hpp`

ros2_controlフレームワーク用のハードウェアインターフェース実装。

### 概要

```cpp
namespace robstride_hardware {
    class RobStrideHardware : public hardware_interface::SystemInterface {
    public:
        // ライフサイクルコールバック
        CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

        // インターフェースのエクスポート
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // リアルタイムループ
        hardware_interface::return_type read(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
    };
}
```

### ライフサイクル

RobStrideHardwareはros2_controlのライフサイクルに従います。

```
    on_init()
       ↓
  on_configure()
       ↓
  on_activate() ← RT Loop開始
       ↓
  (read/write)
       ↓
 on_deactivate() ← RT Loop停止
       ↓
  on_cleanup()
```

### メソッド詳細

#### `CallbackReturn on_init(const hardware_interface::HardwareInfo & info)`

初期化処理。URDFからパラメータを読み込みます。

**処理内容**:
1. URDFパラメータの読み込み（`can_interface`, `motor_id`, `kp`, `kd`）
2. 状態・コマンド用ベクターの初期化
3. ジョイント名の保存

**URDFパラメータ**:
- `can_interface`: CANインターフェース名（デフォルト: `"can0"`）
- `motor_id`: モーターID（デフォルト: `11`）
- `kp`: 位置ゲイン（デフォルト: `30.0`）
- `kd`: 速度ゲイン（デフォルト: `1.0`）

---

#### `CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)`

ハードウェアの設定処理。

**処理内容**:
1. ROS 2ノードとパブリッシャーの作成（`/robstride/joint_states`）
2. CANインターフェースへの接続

**失敗条件**:
- CANインターフェースへの接続失敗

---

#### `CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)`

ハードウェアのアクティブ化処理。

**処理内容**:
1. モーターの有効化（`enable()`）
2. MITモードの設定
3. コマンドの初期化（現在位置）
4. State Readerスレッドの起動（100Hz）

**失敗条件**:
- モーターの有効化失敗
- モードの設定失敗

---

#### `CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)`

ハードウェアの非アクティブ化処理。

**処理内容**:
1. State Readerスレッドの停止
2. 安全な状態への移行（Kp=0, Kd=0, torque=0）
3. モーターの無効化（`disable()`）

---

#### `CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state)`

クリーンアップ処理。

**処理内容**:
1. CANインターフェースからの切断

---

#### `std::vector<hardware_interface::StateInterface> export_state_interfaces()`

状態インターフェースをエクスポートします。

**エクスポートされるインターフェース**:
- `<joint_name>/position` (double): 位置 [rad]
- `<joint_name>/velocity` (double): 速度 [rad/s]
- `<joint_name>/effort` (double): トルク [Nm]

---

#### `std::vector<hardware_interface::CommandInterface> export_command_interfaces()`

コマンドインターフェースをエクスポートします。

**エクスポートされるインターフェース**:
- `<joint_name>/position` (double): 目標位置 [rad]

---

#### `hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period)`

状態を読み取ります（200Hzで呼ばれる）。

**処理内容**:
1. atomic変数から最新の状態をコピー
2. `hw_positions_`, `hw_velocities_`, `hw_efforts_`を更新

**レイテンシ**: 約1μs以下（CAN通信なし）

**リアルタイム性**: 完全にリアルタイムセーフ

---

#### `hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period)`

コマンドを送信します（200Hzで呼ばれる）。

**処理内容**:
1. `hw_commands_position_`から目標位置を取得
2. MITコマンドを構築（kp, kdを設定）
3. `driver_.send_command()`でCAN送信

**レイテンシ**: 約50-100μs

**リアルタイム性**: 送信のみのため比較的安全（ブロックする可能性は低い）

---

### プライベートメソッド

#### `void state_reader_loop()`

状態読み取りスレッドのメインループ（100Hzで動作）。

**処理内容**:
1. `driver_.read_state()`でCAN受信
2. 受信成功時: atomic変数を更新
3. `/robstride/joint_states`にパブリッシュ
4. 1秒ごとに統計情報をログ出力

**スレッド周期**: 10ms（100Hz）

**リアルタイム性**: RTループとは完全に独立（影響なし）

---

## データ構造

### MotorState

モーターの現在状態を表す構造体。

```cpp
struct MotorState {
    double position = 0.0;   // 位置 [rad]
    double velocity = 0.0;   // 速度 [rad/s]
    double torque = 0.0;     // トルク [Nm]
    bool valid = false;      // データの有効性
};
```

**使用例**:
```cpp
auto state = driver.read_state(127);
if (state.valid) {
    // 有効なデータ
    std::cout << "Position: " << state.position << std::endl;
}
```

---

### MitCommand

MIT制御コマンドを表す構造体。

```cpp
struct MitCommand {
    double position = 0.0;    // 目標位置 [rad]
    double velocity = 0.0;    // 目標速度 [rad/s]
    double kp = 30.0;         // 位置ゲイン [Nm/rad]
    double kd = 1.0;          // 速度ゲイン [Nm/(rad/s)]
    double torque_ff = 0.0;   // フィードフォワードトルク [Nm]
};
```

**使用例**:
```cpp
MitCommand cmd;
cmd.position = 1.57;   // π/2 rad
cmd.velocity = 0.0;
cmd.kp = 50.0;         // 硬めの制御
cmd.kd = 2.0;
cmd.torque_ff = 0.5;   // 重力補償等

driver.send_command(127, cmd);
```

---

### ControlMode

制御モードの列挙型。

```cpp
enum class ControlMode : int8_t {
    MIT = 0,        // MIT統合制御
    POSITION = 1,   // 位置制御
    VELOCITY = 2,   // 速度制御
    TORQUE = 3      // トルク制御
};
```

---

## プロトコル定数

### Communication Types

```cpp
namespace protocol::comm_type {
    constexpr uint32_t GET_DEVICE_ID = 0;
    constexpr uint32_t OPERATION_CONTROL = 1;
    constexpr uint32_t OPERATION_STATUS = 2;
    constexpr uint32_t ENABLE = 3;
    constexpr uint32_t DISABLE = 4;
    constexpr uint32_t SET_ZERO = 6;
    constexpr uint32_t WRITE_PARAMETER = 18;
}
```

### Parameter IDs

```cpp
namespace protocol::param_id {
    constexpr uint16_t MODE = 0x7005;
    constexpr uint16_t VELOCITY_LIMIT = 0x7017;
    constexpr uint16_t TORQUE_LIMIT = 0x700B;
}
```

### Scale Factors (RS-02)

```cpp
namespace scale {
    constexpr double POSITION = 4.0 * 3.14159265358979323846;  // ±4π rad
    constexpr double VELOCITY = 44.0;   // ±44 rad/s
    constexpr double TORQUE = 17.0;     // ±17 Nm
    constexpr double KP = 500.0;        // 0-500 Nm/rad
    constexpr double KD = 5.0;          // 0-5 Nm/(rad/s)
}
```

---

## エンコーディング・デコーディング

### 位置のエンコード

```cpp
// 物理値 → uint16
double pos_clamped = std::clamp(position, -scale::POSITION, scale::POSITION);
uint16_t pos_u16 = static_cast<uint16_t>(
    ((pos_clamped / scale::POSITION) + 1.0) * 0x7FFF
);
```

### 位置のデコード

```cpp
// uint16 → 物理値
uint16_t pos_u16 = unpack_u16_be(&data[0]);
double position = ((static_cast<double>(pos_u16) / 0x7FFF) - 1.0) * scale::POSITION;
```

---

## 使用例

### スタンドアロンでのドライバー使用

```cpp
#include "robstride_hardware/robstride_driver.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    robstride_driver::RobStrideDriver driver;

    // 接続
    if (!driver.connect("can0")) {
        std::cerr << "Failed to connect" << std::endl;
        return -1;
    }

    // 有効化
    driver.enable(127);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // MITモード設定
    driver.set_mode(127, robstride_driver::ControlMode::MIT);

    // 制御ループ
    for (int i = 0; i < 1000; ++i) {
        // コマンド送信
        robstride_driver::MitCommand cmd;
        cmd.position = 0.5 * std::sin(2.0 * M_PI * i / 1000.0);  // 正弦波
        cmd.velocity = 0.0;
        cmd.kp = 30.0;
        cmd.kd = 1.0;
        driver.send_command(127, cmd);

        // 状態読み取り
        auto state = driver.read_state(127);
        if (state.valid) {
            std::cout << "Pos: " << state.position
                      << " Vel: " << state.velocity
                      << " Torque: " << state.torque << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 無効化
    driver.disable(127);
    driver.disconnect();

    return 0;
}
```

---

## まとめ

このAPIを使用することで、以下が可能になります:

1. **低レベル制御**: `RobStrideDriver`で直接CANコマンドを送信
2. **ROS 2統合**: `RobStrideHardware`でros2_controlフレームワークと統合
3. **リアルタイム制御**: 200Hzの高速制御ループ
4. **柔軟な設定**: URDF/YAMLでパラメータをカスタマイズ

詳細な実装例は、`src/`ディレクトリのソースコードを参照してください。
