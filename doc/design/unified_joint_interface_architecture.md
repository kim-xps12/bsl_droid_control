# 統一関節インターフェースアーキテクチャ設計

## 1. 概要

本文書は「**関節角度を適切なフォーマットでpublishすれば、シミュレータでも実機でも動作する**」という統一インターフェースの設計を定義する。

### 1.1 目標

- **MacBook上で歩容設計・物理シミュレーションを実行**
- **コマンド送信先の切り替えのみでJetson経由のモータ実機制御に移行**
- シミュレータと実機で**同一の関節コマンドフォーマット**を使用

### 1.2 現状の課題

| 環境 | 制御方式 | 課題 |
|------|----------|------|
| MacBook + RViz | `/joint_states` 直接publish | 可視化のみ、物理シミュレーションなし |
| MacBook + Gazebo | `/joint_commands` → Gazebo plugin | ros2_controlなし（RoboStack制約） |
| Jetson + 実機 | ros2_control + Hardware Interface | MacBookから分離 |

**問題**: 各環境でコマンドトピック・フォーマットが異なり、切り替えが煩雑。

---

## 2. 統一インターフェース設計

### 2.1 設計原則

1. **単一のコマンドトピック**: 全環境で同一トピック名・メッセージ型を使用
2. **抽象化レイヤー**: 歩容生成ノードは送信先を意識しない
3. **切り替えはLaunch引数で完結**: `target:=sim|hardware`
4. **互換性維持**: 既存の`/joint_states`ベースの可視化を維持

### 2.2 統一トピック仕様

```
/joint_commands (sensor_msgs/JointState)
├── header.stamp          # タイムスタンプ
├── name[]                # 関節名（URDF準拠）
├── position[]            # 目標関節角度 [rad]
├── velocity[]            # 目標関節速度 [rad/s]（オプション）
└── effort[]              # 目標トルク [Nm]（オプション）
```

**メッセージ型選択理由**:
- `sensor_msgs/JointState`: 標準的、ツールサポート豊富
- `trajectory_msgs/JointTrajectory`: 軌道ベース制御にも対応可能だが、単一時点制御には冗長
- **決定**: Phase1は`JointState`、将来的に軌道制御が必要なら`JointTrajectory`へ拡張

### 2.3 アーキテクチャ層

```
┌────────────────────────────────────────────────────────────┐
│                   Gait Generation Layer                    │
│              (biped_gait_control / RL Policy)              │
│                                                            │
│   Output: /joint_commands (sensor_msgs/JointState)        │
└─────────────────────────┬──────────────────────────────────┘
                          │
                          ▼
┌────────────────────────────────────────────────────────────┐
│              Joint Command Router (新規)                   │
│                                                            │
│   target=sim:      → Simulator Bridge                     │
│   target=hardware: → Hardware Interface (via ros2_control)│
│   target=viz_only: → /joint_states直接（可視化のみ）        │
└──┬─────────────────────┬────────────────────────┬─────────┘
   │                     │                        │
   ▼                     ▼                        ▼
┌──────────────┐  ┌───────────────────┐  ┌────────────────────┐
│  Simulator   │  │   ros2_control    │  │  RViz2 直接可視化   │
│  (Gazebo/    │  │  (Jetson Only)    │  │   (MacBook)        │
│   MuJoCo)    │  │                   │  │                    │
│              │  │ forward_position  │  │  /joint_states     │
│  /gazebo/... │  │ _controller       │  │  → robot_state_    │
│              │  │        ↓          │  │     publisher      │
└──────────────┘  │  robstride_       │  │        ↓           │
                  │  hardware         │  │     /tf            │
                  │        ↓          │  └────────────────────┘
                  │  CAN → Motors     │
                  └───────────────────┘
```

---

## 3. 実行環境別構成

### 3.1 環境A: MacBook + 可視化のみ（開発・デバッグ）

**用途**: 歩容パターンの視覚的確認、パラメータチューニング

```
┌─────────────────────────────────────────────────────┐
│                     MacBook                         │
│                                                     │
│  ┌─────────────────┐      ┌───────────────────┐   │
│  │  gait_pattern_  │      │  robot_state_     │   │
│  │  generator      │─────▶│  publisher        │   │
│  │                 │      │                   │   │
│  │ /joint_commands │      │ URDF → /tf        │   │
│  │  (50Hz)         │      └─────────┬─────────┘   │
│  └─────────────────┘                │             │
│           │                         ▼             │
│           │              ┌───────────────────┐   │
│           └─────────────▶│     RViz2         │   │
│           (remap to      │                   │   │
│            /joint_states)│  Robot Model      │   │
│                          │  TF Tree          │   │
│                          └───────────────────┘   │
└─────────────────────────────────────────────────────┘
```

**Launch設定**:
```bash
pixi run ros2 launch biped_gait_control gait_visualization.launch.py target:=viz_only
```

**特徴**:
- 物理なし、純粋な可視化
- 歩容アルゴリズムの視覚的検証に最適
- 最小構成で即座に起動

### 3.2 環境B: MacBook + 物理シミュレータ（机上実験）

**用途**: 物理的な妥当性検証、歩行安定性テスト

```
┌─────────────────────────────────────────────────────┐
│                     MacBook                         │
│                                                     │
│  ┌─────────────────┐                               │
│  │  gait_pattern_  │                               │
│  │  generator      │                               │
│  │                 │                               │
│  │ /joint_commands │                               │
│  │  (50Hz)         │                               │
│  └────────┬────────┘                               │
│           │                                         │
│           ▼                                         │
│  ┌─────────────────────────────────────────────┐   │
│  │         Simulator Bridge Node               │   │
│  │                                             │   │
│  │  /joint_commands  →  Gazebo/MuJoCo API     │   │
│  │                                             │   │
│  │  Gazebo/MuJoCo    →  /joint_states         │   │
│  │  (sensor feedback)    (200Hz)              │   │
│  └─────────────────────────────────────────────┘   │
│           │                                         │
│           ▼                                         │
│  ┌─────────────────┐      ┌───────────────────┐   │
│  │  Gazebo /       │      │     RViz2         │   │
│  │  MuJoCo         │─────▶│                   │   │
│  │                 │      │  Robot Model      │   │
│  │  Physics Sim    │      │  TF Tree          │   │
│  └─────────────────┘      └───────────────────┘   │
└─────────────────────────────────────────────────────┘
```

**Launch設定**:
```bash
pixi run ros2 launch biped_gait_control gait_simulation.launch.py simulator:=gazebo
# または
pixi run ros2 launch biped_gait_control gait_simulation.launch.py simulator:=mujoco
```

**シミュレータ選択**:

| シミュレータ | 長所 | 短所 | 推奨用途 |
|-------------|------|------|----------|
| **Gazebo** | ROS 2統合が成熟、センサプラグイン豊富 | macOSサポート限定的 | センサ統合テスト |
| **MuJoCo** | 高速、接触シミュレーション精度高、macOS対応 | ROS連携は自作必要 | 歩行安定性検証 |
| **PyBullet** | Python親和性、軽量 | 精度劣る | プロトタイピング |

**推奨**: Phase1は**MuJoCo**（macOS対応、高精度接触、RL学習との親和性）

### 3.3 環境C: Jetson + 実機（本番運用）

**用途**: 実際のロボット制御

```
┌──────────────────────────────────┐  ┌──────────────────────────────────┐
│            MacBook               │  │      Jetson Orin Nano Super      │
│                                  │  │                                  │
│  ┌─────────────────┐            │  │  ┌─────────────────────────────┐│
│  │  gait_pattern_  │            │  │  │       ros2_control_node     ││
│  │  generator      │     ROS 2  │  │  │                             ││
│  │                 │──────DDS──▶│  │  │  ┌─────────────────────┐   ││
│  │ /joint_commands │     WiFi   │  │  │  │ forward_position_   │   ││
│  │  (50Hz)         │     or     │  │  │  │ controller          │   ││
│  └─────────────────┘   Ethernet │  │  │  │                     │   ││
│                                  │  │  │  │ /joint_commands    │   ││
│  ┌─────────────────┐            │  │  │  │      ↓              │   ││
│  │     RViz2       │◀───────────│  │  │  │ command_interface  │   ││
│  │                 │            │  │  │  └─────────┬───────────┘   ││
│  │  Robot Model    │  /joint_   │  │  │            │               ││
│  │  TF Tree        │  states    │  │  │  ┌─────────▼───────────┐   ││
│  └─────────────────┘  (200Hz)   │  │  │  │ robstride_hardware  │   ││
│                                  │  │  │  │                     │   ││
│  ┌─────────────────┐            │  │  │  │ CAN → RobStride    │   ││
│  │  Plotjuggler    │◀───────────│  │  │  │        RS02         │   ││
│  │                 │ diagnostics│  │  │  └─────────────────────┘   ││
│  └─────────────────┘            │  │  └─────────────────────────────┘│
└──────────────────────────────────┘  └──────────────────────────────────┘
```

**Launch設定**:

Jetson側:
```bash
# ros2_control + Hardware Interface起動
pixi run ros2 launch robstride_hardware bringup.launch.py
```

MacBook側:
```bash
# 歩容生成 + 可視化（実機向け）
pixi run ros2 launch biped_gait_control gait_control.launch.py target:=hardware

# RViz可視化のみ（Jetsonからの/joint_statesを購読）
pixi run ros2 launch biped_description display_rviz_only.launch.py
```

---

## 4. 統一トピック・インターフェース詳細

### 4.1 トピック一覧

| トピック | 型 | 方向 | 周波数 | 説明 |
|---------|-----|------|--------|------|
| `/joint_commands` | `sensor_msgs/JointState` | Gait→Router | 50Hz | 目標関節角度 |
| `/joint_states` | `sensor_msgs/JointState` | Router→RViz | 50-200Hz | 実際の関節状態 |
| `/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Gait→Router | 50Hz | 軌道ベース（将来拡張） |

### 4.2 関節名規則（URDF準拠）

```python
# 左脚
LEFT_JOINTS = [
    'left_hip_yaw_joint',
    'left_hip_roll_joint',
    'left_hip_pitch_joint',
    'left_knee_pitch_joint',
    'left_ankle_pitch_joint',
]

# 右脚
RIGHT_JOINTS = [
    'right_hip_yaw_joint',
    'right_hip_roll_joint',
    'right_hip_pitch_joint',
    'right_knee_pitch_joint',
    'right_ankle_pitch_joint',
]

# 全関節（順序固定）
ALL_JOINTS = LEFT_JOINTS + RIGHT_JOINTS  # 計10関節
```

### 4.3 角度単位・範囲

| 関節 | 単位 | 最小 | 最大 | 備考 |
|------|------|------|------|------|
| hip_yaw | rad | -π/4 | π/4 | 旋回 |
| hip_roll | rad | -π/6 | π/6 | 左右傾斜 |
| hip_pitch | rad | -π/2 | π/3 | 前後屈曲 |
| knee_pitch | rad | -2π/3 | 0 | 逆関節（負の角度で屈曲） |
| ankle_pitch | rad | -π/3 | π/3 | 足首屈曲 |

**重要**: 角度は全て**ラジアン**で統一。歩容生成側での変換を明示的に行う。

### 4.4 QoS設定

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# コマンド用QoS（信頼性重視）
COMMAND_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=1  # 最新値のみ重要
)

# 状態フィードバック用QoS（低遅延重視）
STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)
```

---

## 5. Joint Command Router 設計

### 5.1 概要

`joint_command_router`は、歩容生成ノードからの`/joint_commands`を受け取り、設定されたターゲットに応じて適切なインターフェースにルーティングするノード。

### 5.2 動作モード

| モード | 動作 | 使用環境 |
|--------|------|----------|
| `viz_only` | `/joint_states`へリマップ（パススルー） | MacBook可視化のみ |
| `gazebo` | Gazebo `gz::msgs::JointPositionCmd`へ変換 | MacBook + Gazebo |
| `mujoco` | MuJoCo Python APIコール | MacBook + MuJoCo |
| `hardware` | `/forward_position_controller/commands`へ転送 | Jetson + 実機 |

### 5.3 実装方針

**Phase 1（即座に実装）**:
- `viz_only`モード: `/joint_commands` → `/joint_states` のリマップ
- 既存の`gait_pattern_generator`を`/joint_commands`にpublishするよう変更

**Phase 2（シミュレータ統合時）**:
- `mujoco`モード: MuJoCo Python APIブリッジ
- `gazebo`モード: Gazebo ros2_controlプラグイン統合

**Phase 3（実機統合時）**:
- `hardware`モード: `/forward_position_controller/commands`への転送
- Jetson側ros2_controlとの接続検証

---

## 6. 実装上の考慮事項

### 6.1 タイムスタンプ同期

- シミュレータ使用時は`use_sim_time:=true`
- 実機使用時はシステムクロック
- 全ノードで`use_sim_time`パラメータを一貫して設定

### 6.2 安全機構

**ソフトウェアリミット**:
```python
def validate_joint_commands(positions: List[float]) -> bool:
    """関節角度がリミット内かチェック"""
    limits = get_joint_limits()  # URDFから取得
    for i, pos in enumerate(positions):
        if not limits[i].lower <= pos <= limits[i].upper:
            return False
    return True
```

**緊急停止トピック**:
```
/emergency_stop (std_msgs/Bool)
- True: 全ノード即座に現在位置保持
- False: 通常運転再開
```

### 6.3 ログ・診断

- `/diagnostics`トピックで各ノードの状態をpublish
- 制御ループ遅延の監視
- 通信品質の監視（特にDDS越え通信時）

---

## 7. 今後の拡張

### 7.1 Phase 1 → Phase 2 拡張

| 機能 | Phase 1 | Phase 2 |
|------|---------|---------|
| シミュレータ | なし（可視化のみ） | MuJoCo/Gazebo |
| センサフィードバック | なし | シミュレータからの仮想センサ |
| 接地判定 | なし | 接触力センサ |

### 7.2 Phase 2 → Phase 3 拡張

| 機能 | Phase 2 | Phase 3 |
|------|---------|---------|
| アクチュエータ | シミュレータ | RobStride RS02 |
| フィードバック | シミュレータ状態 | 実エンコーダ |
| 安全監視 | ソフトウェアのみ | ハードウェアE-Stop連携 |

### 7.3 軌道制御への拡張

現在の単一時点制御から、軌道ベース制御への拡張パス：

```python
# 現在（JointState）
position = [...]  # 単一の目標角度

# 将来（JointTrajectory）
trajectory.points = [
    JointTrajectoryPoint(positions=[...], time_from_start=0.0s),
    JointTrajectoryPoint(positions=[...], time_from_start=0.02s),
    JointTrajectoryPoint(positions=[...], time_from_start=0.04s),
]
```

---

## 8. まとめ

### 8.1 統一インターフェースの利点

1. **開発効率**: MacBookで設計→Jetsonで実行の移行がシームレス
2. **再現性**: 同一コマンドでシミュレータと実機を駆動
3. **デバッグ容易性**: 問題切り分けが明確（歩容生成 vs 制御 vs ハードウェア）
4. **拡張性**: 新しいシミュレータ・ハードウェアの追加が容易

### 8.2 次のアクション

1. **即座に**: `gait_pattern_generator`を`/joint_commands`にpublishするよう変更
2. **Phase 1**: `joint_command_router`（viz_onlyモード）実装
3. **Phase 2**: MuJoCoブリッジ実装、物理シミュレーション検証
4. **Phase 3**: Jetson連携、実機制御検証

---

## 参考資料

- [distributed_architecture.md](./distributed_architecture.md) - 分散システム全体設計
- [next_nodes_design.md](../next_nodes_design.md) - ノード設計詳細
- [exp001_slider_control_rs02.md](../experiments/exp001_slider_control_rs02.md) - 単関節実機制御実験
