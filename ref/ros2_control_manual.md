# ros2_control 完全マニュアル

## 目次

1. [概要](#1-概要)
2. [アーキテクチャ](#2-アーキテクチャ)
3. [コンポーネント詳細](#3-コンポーネント詳細)
4. [インターフェース](#4-インターフェース)
5. [ライフサイクル](#5-ライフサイクル)
6. [CLIツール](#6-cliツール)
7. [設定ファイル](#7-設定ファイル)
8. [実行フロー](#8-実行フロー)

---

## 1. 概要

ros2_control は ROS 2 におけるリアルタイムロボット制御のための**標準フレームワーク**。

### なぜ ros2_control を使うのか？

| 自前実装 | ros2_control |
|----------|--------------|
| 制御ループを自分で書く | Controller Manager が管理 |
| タイミング管理自前 | 200Hz RT ループ提供済み |
| コントローラー自作 | 標準コントローラー利用可能 |
| MoveIt2連携困難 | 標準対応 |

### 設計思想

**「ユーザーはハードウェア通信だけ書く」**

```
┌──────────────────────────────────────────────────────┐
│                  ros2_control_node                    │
│  (ROS 2 が提供 - あなたは作らない)                    │
│                                                       │
│  ┌─────────────────────────────────────────────────┐ │
│  │            Controller Manager                    │ │
│  │  ・RT制御ループ (200Hz)                         │ │
│  │  ・コントローラーの管理                          │ │
│  │  ・ハードウェアのロード                          │ │
│  └─────────────────────────────────────────────────┘ │
│                        ↓                             │
│  ┌─────────────────────────────────────────────────┐ │
│  │            Resource Manager                      │ │
│  │  ・State/Command Interface の管理                │ │
│  │  ・複数ハードウェアの統合                        │ │
│  └─────────────────────────────────────────────────┘ │
│                        ↓                             │
│  ┌─────────────────────────────────────────────────┐ │
│  │            Hardware Interface                    │ │
│  │  (あなたが実装する)                              │ │
│  │  ・read(): センサー値読み取り                    │ │
│  │  ・write(): コマンド送信                         │ │
│  └─────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────┘
```

---

## 2. アーキテクチャ

### 2.1 全体像

```
                     ROS 2 Topics/Services
                            ↑
┌───────────────────────────┼───────────────────────────┐
│                Controller Manager                      │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐   │
│  │ Controller1 │  │ Controller2 │  │ Controller3 │   │
│  │ (JointState │  │ (Trajectory │  │ (Forward    │   │
│  │  Broadcaster)│  │  Controller)│  │  Command)   │   │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘   │
│         │                │                │          │
│         └────────────────┼────────────────┘          │
│                          ↓                           │
│  ┌───────────────────────────────────────────────┐   │
│  │              Resource Manager                  │   │
│  │  ┌─────────────────┐  ┌─────────────────┐     │   │
│  │  │ State Interface │  │Command Interface│     │   │
│  │  │ - position      │  │ - position      │     │   │
│  │  │ - velocity      │  │ - velocity      │     │   │
│  │  │ - effort        │  │ - effort        │     │   │
│  │  └────────┬────────┘  └────────┬────────┘     │   │
│  └───────────┼────────────────────┼──────────────┘   │
│              └──────────┬─────────┘                   │
│                         ↓                            │
│  ┌───────────────────────────────────────────────┐   │
│  │           Hardware Interface (Plugin)          │   │
│  │  ┌─────────┐                    ┌─────────┐   │   │
│  │  │ read()  │ ← 200Hz呼び出し → │ write() │   │   │
│  │  └────┬────┘                    └────┬────┘   │   │
│  └───────┼──────────────────────────────┼────────┘   │
└──────────┼──────────────────────────────┼────────────┘
           ↓                              ↓
    ┌──────────────────────────────────────────┐
    │            実際のハードウェア              │
    │     (CAN, EtherCAT, GPIO, etc.)          │
    └──────────────────────────────────────────┘
```

### 2.2 制御ループ（RT スレッド）

```
┌─────────────────────────────────────────────────────┐
│  Controller Manager の RT ループ (200Hz = 5ms周期)   │
│                                                     │
│  while (running) {                                  │
│      sleep_until(next_cycle);  // 絶対時間で待機    │
│                                                     │
│      // 1. ハードウェアから状態読み取り             │
│      for (hw : hardware_interfaces) {               │
│          hw.read(time, period);                     │
│      }                                              │
│                                                     │
│      // 2. コントローラーの更新                     │
│      for (ctrl : active_controllers) {              │
│          ctrl.update(time, period);                 │
│      }                                              │
│                                                     │
│      // 3. ハードウェアへコマンド送信               │
│      for (hw : hardware_interfaces) {               │
│          hw.write(time, period);                    │
│      }                                              │
│                                                     │
│      next_cycle += period;                          │
│  }                                                  │
└─────────────────────────────────────────────────────┘
```

---

## 3. コンポーネント詳細

### 3.1 Controller Manager

**役割**: ros2_control の中核。全体を統括する。

| 機能 | 説明 |
|------|------|
| RT ループ管理 | 200Hz で read → update → write を実行 |
| Hardware ロード | URDF から Hardware Interface プラグインを動的ロード |
| Controller 管理 | コントローラーのロード/有効化/無効化 |
| サービス提供 | CLI ツールが使う ROS 2 サービスを提供 |

**提供するサービス**:
```
/controller_manager/list_controllers
/controller_manager/list_hardware_interfaces
/controller_manager/load_controller
/controller_manager/configure_controller
/controller_manager/switch_controller
...
```

### 3.2 Resource Manager

**役割**: State/Command Interface を一元管理。

```
┌──────────────────────────────────────────────────┐
│               Resource Manager                    │
│                                                   │
│  State Interfaces (読み取り専用):                 │
│  ┌─────────────────────────────────────────────┐ │
│  │ joint1/position  → 0.0    (double*)         │ │
│  │ joint1/velocity  → 0.0    (double*)         │ │
│  │ joint1/effort    → 0.0    (double*)         │ │
│  │ joint2/position  → 0.0    (double*)         │ │
│  │ ...                                         │ │
│  └─────────────────────────────────────────────┘ │
│                                                   │
│  Command Interfaces (書き込み可能):               │
│  ┌─────────────────────────────────────────────┐ │
│  │ joint1/position  → 0.0    [unclaimed]       │ │
│  │ joint1/velocity  → 0.0    [unclaimed]       │ │
│  │ joint2/position  → 0.0    [claimed by ctrl] │ │
│  │ ...                                         │ │
│  └─────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────┘
```

**重要な概念: Claim (排他制御)**

Command Interface は**同時に1つのコントローラーしか使えない**。

```
# joint1/position が forward_position_controller に claimed されている場合
joint_trajectory_controller を active にしようとすると失敗
```

### 3.3 Hardware Interface (Plugin)

**役割**: 実際のハードウェアとの通信を担当。**ユーザーが実装する唯一の部分**。

```cpp
class RobStrideHardware : public hardware_interface::SystemInterface
{
public:
  // 初期化 (URDF パラメータ読み込み)
  CallbackReturn on_init(const HardwareInfo & info) override;
  
  // State Interface のエクスポート
  // "この Hardware Interface は何を読み取れるか"
  std::vector<StateInterface> export_state_interfaces() override;
  
  // Command Interface のエクスポート
  // "この Hardware Interface は何を制御できるか"
  std::vector<CommandInterface> export_command_interfaces() override;
  
  // 200Hz で呼ばれる: ハードウェアから状態読み取り
  return_type read(const Time & time, const Duration & period) override;
  
  // 200Hz で呼ばれる: ハードウェアへコマンド送信
  return_type write(const Time & time, const Duration & period) override;

private:
  // 状態バッファ (read() で更新)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  
  // コマンドバッファ (write() で使用)
  std::vector<double> hw_commands_position_;
};
```

**Hardware Interface の3つのタイプ**:

| タイプ | 用途 | 例 |
|--------|------|-----|
| `SystemInterface` | 複数 joint をまとめて制御 | ロボットアーム全体 |
| `ActuatorInterface` | 単一アクチュエータ | 独立したモーター |
| `SensorInterface` | センサーのみ (読み取り専用) | IMU, Force/Torque センサー |

### 3.4 Controllers

**役割**: 制御アルゴリズムを実装。State Interface を読み、Command Interface に書く。

**標準コントローラー一覧**:

| コントローラー | 機能 | 用途 |
|---------------|------|------|
| `joint_state_broadcaster` | joint state を `/joint_states` に publish | 可視化、他ノード連携 |
| `forward_command_controller` | コマンドをそのまま転送 | 単純な位置/速度制御 |
| `joint_trajectory_controller` | 軌道追従制御 | スムーズな動作 |
| `diff_drive_controller` | 差動駆動 | 移動ロボット |
| `tricycle_controller` | 三輪駆動 | 三輪車型ロボット |

**joint_state_broadcaster の特殊性**:

```
これは「Broadcaster」であり「Controller」ではない。
Command Interface を使わない（claim しない）。
State Interface を読んで ROS 2 topic に publish するだけ。

      State Interface           ROS 2 Topic
  ┌────────────────────┐     ┌─────────────────┐
  │ joint1/position: 0.5 │ → │ /joint_states   │
  │ joint1/velocity: 0.1 │ → │   position: 0.5 │
  │ joint1/effort:   0.0 │ → │   velocity: 0.1 │
  └────────────────────┘     └─────────────────┘
```

**forward_command_controller**:

```
入力 topic の値を Command Interface にそのまま書く。

      ROS 2 Topic              Command Interface
  ┌─────────────────────┐     ┌────────────────────┐
  │ /forward_position   │     │ joint1/position    │
  │   data: [1.57]      │ → → │   value: 1.57      │
  └─────────────────────┘     └────────────────────┘
```

---

## 4. インターフェース

### 4.1 State Interface

**ハードウェアから読み取れる値**を定義。

```cpp
// export_state_interfaces() で作成
StateInterface(
  "joint1",           // joint名
  HW_IF_POSITION,     // インターフェース名 ("position")
  &hw_positions_[0]   // 値へのポインタ
);
```

| 標準インターフェース名 | 定数 | 意味 |
|----------------------|------|------|
| `position` | `HW_IF_POSITION` | 位置 [rad] |
| `velocity` | `HW_IF_VELOCITY` | 速度 [rad/s] |
| `effort` | `HW_IF_EFFORT` | トルク [Nm] |
| `acceleration` | `HW_IF_ACCELERATION` | 加速度 [rad/s²] |

### 4.2 Command Interface

**ハードウェアに送信できるコマンド**を定義。

```cpp
// export_command_interfaces() で作成
CommandInterface(
  "joint1",                  // joint名
  HW_IF_POSITION,            // インターフェース名
  &hw_commands_position_[0]  // 値へのポインタ
);
```

**排他制御 (Claim)**:

```
                 Controller A              Controller B
                      │                         │
                      ↓ claim                   ↓ claim 失敗!
              ┌───────────────┐
              │ joint1/position │
              │   [claimed]     │
              └───────────────┘
```

### 4.3 URDF での定義

```xml
<ros2_control name="robstride_system" type="system">
  <hardware>
    <!-- どのプラグインを使うか -->
    <plugin>robstride_hardware/RobStrideHardware</plugin>
    <!-- Hardware Interface に渡すパラメータ -->
    <param name="can_interface">can0</param>
    <param name="motor_id">11</param>
  </hardware>
  
  <joint name="joint1">
    <!-- このjointで使える Command Interface -->
    <command_interface name="position">
      <param name="min">-3.14</param>
      <param name="max">3.14</param>
    </command_interface>
    
    <!-- このjointで使える State Interface -->
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

---

## 5. ライフサイクル

### 5.1 Hardware Interface のライフサイクル

```
    ┌──────────────┐
    │  unconfigured │ ← 初期状態
    └───────┬──────┘
            │ on_init() → SUCCESS
            ↓
    ┌──────────────┐
    │   inactive    │ ← 設定済み、未稼働
    └───────┬──────┘
            │ on_activate()
            ↓
    ┌──────────────┐
    │    active     │ ← read()/write() が 200Hz で呼ばれる
    └───────┬──────┘
            │ on_deactivate()
            ↓
    ┌──────────────┐
    │   inactive    │
    └──────────────┘
```

### 5.2 Controller のライフサイクル

```
    ┌──────────────┐
    │  unconfigured │ ← load_controller 直後
    └───────┬──────┘
            │ configure (set_controller_state inactive)
            ↓
    ┌──────────────┐
    │   inactive    │ ← 設定済み、Command Interface 未 claim
    └───────┬──────┘
            │ activate (set_controller_state active)
            │ → Command Interface を claim
            ↓
    ┌──────────────┐
    │    active     │ ← update() が 200Hz で呼ばれる
    └───────┬──────┘
            │ deactivate
            │ → Command Interface を release
            ↓
    ┌──────────────┐
    │   inactive    │
    └──────────────┘
```

---

## 6. CLI ツール

### 6.1 ros2 control コマンド一覧

```bash
ros2 control <subcommand>
```

| サブコマンド | 説明 |
|-------------|------|
| `list_hardware_interfaces` | State/Command Interface の一覧 |
| `list_hardware_components` | Hardware Interface プラグインの一覧 |
| `list_controllers` | ロードされたコントローラー一覧 |
| `load_controller` | コントローラーをロード |
| `set_controller_state` | コントローラーの状態遷移 |
| `unload_controller` | コントローラーをアンロード |

### 6.2 実行例と出力の読み方

#### list_hardware_components

```bash
$ ros2 control list_hardware_components

Hardware Component 1
        name: robstride_system          # URDF の ros2_control name
        type: system                     # system / actuator / sensor
        plugin name: robstride_hardware/RobStrideHardware
        state: id=3 label=active         # ライフサイクル状態
        read/write rate: 200 Hz          # 制御周波数
        is_async: False                  # 非同期モードかどうか
        command interfaces
                joint1/position [available] [unclaimed]
                                ↑          ↑
                                │          └─ どのコントローラーにも使われていない
                                └─ 利用可能
```

#### list_hardware_interfaces

```bash
$ ros2 control list_hardware_interfaces

command interfaces
        joint1/position [available] [unclaimed]
state interfaces
        joint1/effort [available]
        joint1/position [available]
        joint1/velocity [available]
```

#### list_controllers

```bash
$ ros2 control list_controllers

joint_state_broadcaster    joint_state_broadcaster/JointStateBroadcaster    active
forward_position_controller forward_command_controller/ForwardCommandController inactive
                           ↑                                                ↑
                           │                                                └─ ライフサイクル状態
                           └─ コントローラーのタイプ
```

#### load_controller

```bash
$ ros2 control load_controller joint_state_broadcaster

Successfully loaded controller joint_state_broadcaster
```

**何が起きるか**:
1. Controller Manager が controllers.yaml からコントローラー設定を読む
2. プラグインローダーが該当タイプのコントローラーをロード
3. コントローラーは `unconfigured` 状態になる

#### set_controller_state

```bash
# unconfigured → inactive (configure)
$ ros2 control set_controller_state joint_state_broadcaster inactive

# inactive → active (activate)
$ ros2 control set_controller_state joint_state_broadcaster active
```

**状態遷移のルール**:
```
unconfigured → inactive → active
                  ↑          │
                  └──────────┘
              (deactivate で戻れる)
```

---

## 7. 設定ファイル

### 7.1 controllers.yaml

```yaml
# Controller Manager の設定
controller_manager:
  ros__parameters:
    update_rate: 200  # 制御周波数 [Hz]
    
    # コントローラーの登録
    # <コントローラー名>:
    #   type: <パッケージ名>/<クラス名>
    
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    
    forward_position_controller:
      type: forward_command_controller/ForwardCommandController

# 各コントローラーの個別設定
forward_position_controller:
  ros__parameters:
    joints:              # 制御対象の joint
      - joint1
    interface_name: position  # 使用する command interface
```

### 7.2 URDF (ros2_control タグ)

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  
  <!-- 通常の URDF (リンク、ジョイント) -->
  <link name="base_link">...</link>
  <joint name="joint1" type="revolute">...</joint>
  
  <!-- ros2_control 設定 -->
  <ros2_control name="my_system" type="system">
    <hardware>
      <plugin>my_package/MyHardwareInterface</plugin>
      <param name="param1">value1</param>
    </hardware>
    
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  
</robot>
```

---

## 8. 実行フロー

### 8.1 起動時のシーケンス

```
1. ros2 launch で ros2_control_node 起動
   │
   ↓
2. Controller Manager が /robot_description を subscribe
   │
   ↓
3. robot_state_publisher が URDF を publish
   │
   ↓
4. Controller Manager が URDF を受信・パース
   │
   ├─ <ros2_control> タグを探す
   ├─ <plugin> を pluginlib でロード
   └─ <param> を Hardware Interface に渡す
   │
   ↓
5. Hardware Interface の on_init() 呼び出し
   │
   ↓
6. Hardware Interface を activate
   │
   ├─ export_state_interfaces()
   └─ export_command_interfaces()
   │
   ↓
7. RT ループ開始 (200Hz)
   │
   ↓
8. spawner でコントローラーをロード・有効化
```

### 8.2 制御ループ (1サイクル = 5ms)

```
┌─────────────────────────────────────────────────────────┐
│  T=0ms                                                  │
│  ┌─────────────────────────────────────────────────┐   │
│  │ read()                                           │   │
│  │   └─ CAN から位置/速度/トルク読み取り            │   │
│  │   └─ hw_positions_[0] = received_position;       │   │
│  └─────────────────────────────────────────────────┘   │
│  T=0.5ms                                                │
│  ┌─────────────────────────────────────────────────┐   │
│  │ Controller.update()                              │   │
│  │   └─ State Interface から現在値読み取り          │   │
│  │   └─ 制御計算                                    │   │
│  │   └─ Command Interface に目標値書き込み          │   │
│  └─────────────────────────────────────────────────┘   │
│  T=1ms                                                  │
│  ┌─────────────────────────────────────────────────┐   │
│  │ write()                                          │   │
│  │   └─ cmd = hw_commands_position_[0];             │   │
│  │   └─ CAN でモーターにコマンド送信                 │   │
│  └─────────────────────────────────────────────────┘   │
│  T=1.5ms                                                │
│  ┌─────────────────────────────────────────────────┐   │
│  │ sleep_until(next_cycle)                          │   │
│  │   └─ 残り 3.5ms スリープ                         │   │
│  └─────────────────────────────────────────────────┘   │
│  T=5ms → 次のサイクル                                   │
└─────────────────────────────────────────────────────────┘
```

### 8.3 データフローの具体例

```
[joint_state_broadcaster の場合]

  Hardware Interface          Resource Manager           Controller            ROS 2
  ┌──────────────┐           ┌──────────────┐        ┌──────────────┐      ┌──────────┐
  │ read()       │           │              │        │              │      │          │
  │  ↓           │           │              │        │              │      │          │
  │ hw_positions_│ ────────→ │ joint1/      │ ─────→ │ update()     │ ───→ │/joint_   │
  │ [0] = 1.57   │  StateInt │ position:1.57│        │  publish()   │      │ states   │
  └──────────────┘           └──────────────┘        └──────────────┘      └──────────┘


[forward_command_controller の場合]

  ROS 2                  Controller              Resource Manager       Hardware Interface
  ┌──────────────┐      ┌──────────────┐        ┌──────────────┐       ┌──────────────┐
  │/forward_     │      │              │        │              │       │              │
  │position_     │ ───→ │ update()     │ ─────→ │ joint1/      │ ────→ │ write()      │
  │controller/   │      │  set_cmd()   │ CmdInt │ position:2.0 │       │  ↓           │
  │commands      │      │              │        │              │       │ send_to_CAN()│
  │ data: [2.0]  │      │              │        │              │       │              │
  └──────────────┘      └──────────────┘        └──────────────┘       └──────────────┘
```

---

## 付録: よくある質問

### Q: なぜ State Interface と Command Interface を分けるのか？

**A**: 安全性と柔軟性のため。

- State Interface は読み取り専用 → 複数コントローラーが同時に読める
- Command Interface は排他制御 → 複数コントローラーが同時に書くと衝突

### Q: read() と write() は別スレッドで呼ばれる？

**A**: いいえ、同じ RT スレッドで順番に呼ばれます。

```
同一スレッド: read() → Controller.update() → write()
```

### Q: Controller を active にしないと何が起きる？

**A**: `update()` が呼ばれない。Command Interface も claim されない。

### Q: Hardware Interface の read()/write() が 5ms 以上かかったら？

**A**: オーバーランとして検出・ログ出力されます。制御品質が劣化。

### Q: FIFO RT scheduling の警告は問題？

**A**: 通常ユーザー権限では SCHED_FIFO を設定できないため出る。
実際の RT 性能には影響するが、テスト目的なら問題なし。

本番環境では以下で解決:
```bash
sudo setcap cap_sys_nice+ep $(which ros2_control_node)
```

---

## 参考リンク

- [ros2_control Documentation](https://control.ros.org/)
- [ros2_controllers](https://github.com/ros-controls/ros2_controllers)
- [Writing a Hardware Interface](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_interface.html)
