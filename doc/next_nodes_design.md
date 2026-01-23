# 二脚ロボット制御システム：次期ノード設計

## 1. はじめに

本文書は、強化学習による歩容生成を最終目標とした二脚ロボット制御システムにおいて、既存の`robstride_hardware`ノード（リアルタイムハードウェア制御層）の次に実装すべきノード群の設計を定義する。

### 1.1 現状の実装

- **robstride_hardware**: ros2_control Hardware Interfaceとして実装
  - 200Hz制御ループでCAN通信を介してRobStride QDDアクチュエータを制御
  - SCHED_FIFO優先度50でリアルタイム動作
  - `read()` → `update()` → `write()` サイクル

### 1.2 目標

強化学習ポリシーによる二脚歩行制御を実現するため、以下の機能を段階的に実装する：

1. **状態推定**: センサフュージョンによるロボット状態の高精度推定
2. **安全監視**: リアルタイム安全性チェックと緊急停止機構
3. **歩容生成**: 強化学習ポリシーによる目標関節軌道生成
4. **高レベル制御**: ナビゲーション・タスク管理

---

## 2. システムアーキテクチャ

### 2.1 制御層の階層構造

```
┌─────────────────────────────────────────────┐
│   High-Level Control Layer (10-50 Hz)      │
│   - Navigation, Task Management             │
│   - Path Planning                           │
└─────────────────────────────────────────────┘
              ↓ Velocity/Direction Commands
┌─────────────────────────────────────────────┐
│   Gait Generation Layer (50-100 Hz)        │
│   - RL Policy Node                          │
│   - Gait Pattern Generator                  │
└─────────────────────────────────────────────┘
              ↓ Target Joint Positions/Velocities
┌─────────────────────────────────────────────┐
│   State Estimation Layer (200 Hz)          │
│   - IMU Integration                         │
│   - Sensor Fusion (EKF/UKF)                │
│   - Contact Detection                       │
└─────────────────────────────────────────────┘
        ↓ Robot State           ↓ Safety Signals
┌──────────────────────┐  ┌────────────────────┐
│  Safety Monitor      │  │ RT Control Layer   │
│  (200 Hz)            │→│ (200 Hz)           │
│  - Fall Detection    │  │ robstride_hardware │
│  - Limit Checks      │  │ (ros2_control)     │
└──────────────────────┘  └────────────────────┘
                                    ↓ CAN Commands
                          ┌──────────────────────┐
                          │  RobStride Motors    │
                          └──────────────────────┘
```

### 2.2 データフロー

- **センサ入力**: IMU (200-1000Hz), Joint encoders (200Hz), Force sensors (200Hz)
- **状態推定**: センサフュージョン → ロボット状態 (position, orientation, velocity, contact)
- **安全監視**: 状態チェック → 安全フラグ/緊急停止信号
- **歩容生成**: ロボット状態 + 高レベルコマンド → 目標関節軌道
- **制御**: 目標軌道 → モータコマンド

---

## 3. 優先実装ノード

### 3.1 Phase 1: 基盤ノード（必須）

#### 3.1.1 State Estimator Node

**目的**: センサフュージョンによるロボット状態の高精度推定

**実装内容**:
- IMUデータ統合（加速度、角速度、姿勢）
- 関節エンコーダデータ統合
- 接地検出（力センサまたは推定ベース）
- EKF/UKF/ComplementaryFilterによる状態推定
- ベース座標系の位置・姿勢・速度の推定

**周波数**: 200 Hz（ハードウェアループと同期）

**入力**:
- `/imu/data` (sensor_msgs/Imu)
- `/joint_states` (sensor_msgs/JointState)
- `/force_sensors` (geometry_msgs/WrenchStamped) ※オプション

**出力**:
- `/robot_state` (custom_msgs/RobotState)
  - base_position (x, y, z)
  - base_orientation (quaternion)
  - base_linear_velocity
  - base_angular_velocity
  - joint_positions, joint_velocities
  - contact_states (left_foot, right_foot)
  - timestamp

**技術選択**:
- **robot_localization** (EKF/UKF): ROS標準パッケージ、設定ベース
- または **カスタム実装**: Complementary Filter + 運動学ベース推定

**言語**: C++（リアルタイム性）

---

#### 3.1.2 Safety Monitor Node

**目的**: リアルタイム安全性監視と緊急停止機構

**実装内容**:
- 姿勢異常検出（過度な傾き、転倒予測）
- 関節角度・速度・トルクリミットチェック
- 温度・電流監視（モータ保護）
- ハードウェアエラー検出
- 緊急停止トリガー

**周波数**: 200 Hz

**入力**:
- `/robot_state` (custom_msgs/RobotState)
- `/joint_states` (sensor_msgs/JointState)
- `/motor_diagnostics` (diagnostic_msgs/DiagnosticArray)

**出力**:
- `/safety_status` (custom_msgs/SafetyStatus)
  - is_safe (bool)
  - warning_flags (bitfield)
  - error_message (string)
- `/emergency_stop` (std_msgs/Bool) → ros2_control emergency stop

**安全条件例**:
```
- Roll/Pitch angle > 45deg → WARNING
- Roll/Pitch angle > 60deg → EMERGENCY_STOP
- Joint velocity > limit * 1.2 → WARNING
- Motor temperature > 80°C → WARNING
- Motor temperature > 90°C → EMERGENCY_STOP
- Base height < 0.1m → EMERGENCY_STOP (fallen)
```

**言語**: C++（リアルタイム性）

---

### 3.2 Phase 2: 歩容生成ノード

#### 3.2.1 RL Policy Node

**目的**: 強化学習ポリシーによる目標関節軌道生成

**実装内容**:
- PyTorch/TensorFlow推論エンジン統合
- 学習済みポリシーのロード
- 観測ベクトルの構築（ロボット状態 → RL observation）
- アクション（目標関節位置/速度）の出力
- Sim-to-Real転移のための補正機能

**周波数**: 50-100 Hz

**入力**:
- `/robot_state` (custom_msgs/RobotState)
- `/velocity_command` (geometry_msgs/Twist) - 高レベルコマンド

**出力**:
- `/gait_target` (custom_msgs/JointTrajectoryPoint)
  - target_joint_positions
  - target_joint_velocities
  - target_joint_efforts (optional)

**観測ベクトル例**:
```python
obs = [
    base_orientation (quaternion or euler),  # 4 or 3
    base_angular_velocity,                   # 3
    projected_gravity,                       # 3
    command_velocity (x, y, yaw),           # 3
    joint_positions,                         # n_joints
    joint_velocities,                        # n_joints
    previous_action,                         # n_joints
]
```

**技術選択**:
- **libtorch (C++)**: 最高性能、デプロイに最適
- または **Python + rclpy**: プロトタイピングに最適、推論速度は劣る

**言語**: C++ (libtorch) または Python (推奨: Phase 2初期はPython、Phase 3でC++化)

---

#### 3.2.2 Gait Pattern Generator Node (代替/フォールバック)

**目的**: CPGまたは軌道ベースの歩容生成（RLポリシーのフォールバック）

**実装内容**:
- Central Pattern Generator (CPG) ベース歩容
- または事前定義された歩行軌道のプレイバック
- 速度指令に応じたパラメータ調整

**周波数**: 50-100 Hz

**入力/出力**: RL Policy Nodeと同じインターフェース

**用途**:
- RL学習前の動作確認
- RLポリシーのフェールセーフ
- シンプルな歩行タスク

**言語**: C++

---

### 3.3 Phase 3: 高レベル制御ノード（将来）

#### 3.3.1 Navigation Stack Integration

- Nav2統合
- 障害物回避
- パス追従

#### 3.3.2 Task Manager Node

- 行動遷移（立つ → 歩く → 止まる → しゃがむ）
- ミッションプランニング

---

## 4. カスタムメッセージ定義

### 4.1 RobotState.msg

```
std_msgs/Header header

# Base state (in world frame)
geometry_msgs/Point base_position
geometry_msgs/Quaternion base_orientation
geometry_msgs/Vector3 base_linear_velocity
geometry_msgs/Vector3 base_angular_velocity

# Joint state
string[] joint_names
float64[] joint_positions
float64[] joint_velocities
float64[] joint_efforts

# Contact state
bool left_foot_contact
bool right_foot_contact

# Timestamps
builtin_interfaces/Time imu_stamp
builtin_interfaces/Time joint_stamp
```

### 4.2 SafetyStatus.msg

```
std_msgs/Header header

bool is_safe
uint32 warning_flags
uint32 error_flags
string message

# Flag definitions (bitfield)
# WARNING_ROLL_PITCH = 0x01
# WARNING_JOINT_VELOCITY = 0x02
# WARNING_MOTOR_TEMP = 0x04
# ERROR_FALL_DETECTED = 0x10
# ERROR_JOINT_LIMIT = 0x20
# ERROR_MOTOR_OVERHEAT = 0x40
```

---

## 5. 実装戦略

### 5.1 推奨実装順序

1. **State Estimator Node** (Week 1-2)
   - robot_localizationパッケージ設定から開始
   - IMU + Joint Statesの統合
   - RVizで可視化して検証

2. **Safety Monitor Node** (Week 3)
   - 基本的な姿勢チェック実装
   - 緊急停止機能のテスト
   - シミュレータで検証後、実機テスト

3. **Gait Pattern Generator Node** (Week 4-5)
   - シンプルなCPGベース歩容
   - 静止立位 → その場足踏み → 前進歩行
   - パラメータチューニング

4. **RL Policy Node** (Week 6-)
   - まずシミュレータで学習（Isaac Gym/MuJoCo）
   - Pythonノードで推論実装
   - Sim-to-Real転移と実機評価
   - 性能が問題ならC++ (libtorch)へ移植

### 5.2 開発環境

**シミュレーション**:
- Gazebo Classic/Ignition: ROS 2統合、物理演算
- Isaac Gym: 大規模並列RL学習
- MuJoCo: 高速・高精度シミュレーション

**テスト**:
- 各ノード単体テスト（gtest/pytest）
- 統合テスト（launch_testing）
- ハードウェア-in-the-loop テスト

### 5.3 依存パッケージ

**Phase 1**:
- `robot_localization`: EKF/UKFベース状態推定
- `realtime_tools`: リアルタイムデータ構造
- `diagnostic_updater`: 診断情報の統一管理

**Phase 2**:
- `libtorch` (C++) または `torch` (Python): RL推論
- `control_toolbox`: PID、フィルタ等
- `angles`: 角度演算ユーティリティ

**Phase 3**:
- `nav2`: ナビゲーションスタック
- `moveit2`: マニピュレーション（上半身制御時）

---

## 6. パフォーマンス要件

### 6.1 レイテンシ

| Node | Frequency | Max Latency | Thread Priority |
|------|-----------|-------------|-----------------|
| robstride_hardware | 200 Hz | 1 ms | SCHED_FIFO 50 |
| state_estimator | 200 Hz | 1 ms | SCHED_FIFO 40 |
| safety_monitor | 200 Hz | 1 ms | SCHED_FIFO 45 |
| rl_policy | 50-100 Hz | 10 ms | SCHED_OTHER |
| gait_generator | 50-100 Hz | 10 ms | SCHED_OTHER |

### 6.2 CPU アフィニティ

Jetson Orin (6コア) の例:
- Core 0: OS + 非リアルタイムプロセス
- Core 1: robstride_hardware (200 Hz)
- Core 2: state_estimator + safety_monitor (200 Hz)
- Core 3: rl_policy / gait_generator (50-100 Hz)
- Core 4-5: ROS通信、ロギング、可視化

---

## 7. ロギングとデバッグ

### 7.1 記録データ

**MCAP (ROS 2 bag)**:
- すべてのトピック（`/robot_state`, `/joint_states`, `/imu/data`, etc.）
- 診断情報（`/diagnostics`）
- 制御パフォーマンスメトリクス

### 7.2 可視化

**RViz**:
- ロボットモデル（URDF + `robot_state_publisher`）
- TF tree（ベース座標系、足先座標系）
- IMU方向、接地状態の表示

**Plotjuggler**:
- 関節角度・速度・トルクのリアルタイムプロット
- RL policy actionの可視化
- 安全フラグのモニタリング

---

## 8. シミュレーションとの統合

### 8.1 Sim-to-Real共通インターフェース

- シミュレータと実機で同じトピック名・メッセージ型を使用
- `use_sim_time` パラメータで時刻管理を切り替え
- 同じlaunchファイルで `sim:=true/false` 引数で切り替え

### 8.2 ドメインランダマイゼーション

RL学習時にシミュレータで適用:
- 質量・慣性の変動
- 摩擦係数の変動
- モータダイナミクスの遅延・ノイズ
- センサノイズ

---

## 9. まとめ

### 9.1 最優先実装ノード（Phase 1）

1. **State Estimator Node** - ロボット状態の正確な推定
2. **Safety Monitor Node** - 安全な動作保証

これらがないと、歩容生成の前提となる「現在の状態」と「安全性」が確保できない。

### 9.2 次のステップ（Phase 2）

3. **Gait Pattern Generator Node** - シンプルな歩容でシステム検証
4. **RL Policy Node** - 強化学習ポリシーの実装

### 9.3 長期目標（Phase 3）

5. **Navigation Integration** - 自律移動
6. **Task Manager** - 複雑な行動計画

---

## 参考文献

1. **MIT Cheetah**: [https://github.com/mit-biomimetics/Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software)
2. **Unitree SDK**: [https://github.com/unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros)
3. **Isaac Gym Legged Robots**: [https://github.com/leggedrobotics/legged_gym](https://github.com/leggedrobotics/legged_gym)
4. **robot_localization**: [http://docs.ros.org/en/noetic/api/robot_localization/html/index.html](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
5. **ROS 2 Control**: [https://control.ros.org/](https://control.ros.org/)
