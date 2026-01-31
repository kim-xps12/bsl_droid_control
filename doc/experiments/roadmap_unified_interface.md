# 統一関節インターフェース実現ロードマップ

## 概要

本文書は「**関節角度をpublishすれば実機が適切に動作する**」という状態を実現するための、段階的な実験・実装手順を定義する。

### 最終目標

```
MacBookで歩容設計 → 物理シミュレータで検証 → 送信先切り替え → Jetson経由で実機制御
```

### 前提条件

- `biped_gait_control`パッケージで50Hz関節角度生成が動作済み
- `robstride_hardware`パッケージでros2_control Hardware Interfaceが実装済み（未統合）
- Jetson Orin Nano SuperとMacBookの分散構成が確立済み

---

## フェーズ構成

| Phase | 目標 | 期間目安 | 成果物 |
|-------|------|----------|--------|
| **Phase 0** | 既存実装の整理・トピック統一 | 3日 | `/joint_commands`トピック導入 |
| **Phase 1** | 可視化環境の完成 | 2日 | `viz_only`モード動作確認 |
| **Phase 2** | 物理シミュレータ統合 | 1週間 | MuJoCoブリッジ実装 |
| **Phase 3** | 実機制御統合 | 1週間 | Jetsonとの分散制御動作 |
| **Phase 4** | システム統合・安定化 | 1週間 | E2E動作検証完了 |

---

## Phase 0: トピック統一（準備）

### 0.1 目標

現在の`gait_pattern_generator`を修正し、統一トピック`/joint_commands`を導入する。

### 0.2 実装タスク

#### タスク 0.2.1: 出力トピック変更

**変更対象**: `biped_gait_control/biped_gait_control/gait_pattern_generator.py`

```python
# Before
self.joint_state_pub = self.create_publisher(JointState, '/joint_states', qos_profile)

# After  
self.joint_command_pub = self.create_publisher(JointState, '/joint_commands', qos_profile)
```

#### タスク 0.2.2: Launchファイル更新

**変更対象**: `biped_gait_control/launch/gait_visualization.launch.py`

```python
# target引数を追加
DeclareLaunchArgument('target', default_value='viz_only',
    description='Target: viz_only, mujoco, gazebo, hardware')

# targetに応じてリマップ設定
# viz_only: /joint_commands → /joint_states
# hardware: /joint_commands → /forward_position_controller/commands
```

#### タスク 0.2.3: 後方互換性の維持

既存の`display_rviz_only.launch.py`等は変更せず、`/joint_states`を購読する形を維持。

### 0.3 検証項目

- [ ] `pixi run ros2 topic echo /joint_commands` で50Hzの関節角度が流れる
- [ ] `pixi run ros2 topic echo /joint_states` でリマップされた角度が流れる（viz_onlyモード）
- [ ] RVizでロボットモデルが歩行アニメーションする

### 0.4 完了条件

```bash
# このコマンドでRViz可視化が動作する
pixi run ros2 launch biped_gait_control gait_visualization.launch.py target:=viz_only
```

---

## Phase 1: 可視化環境の完成

### 1.1 目標

`viz_only`モードでのMacBook単体動作を完全に確立する。

### 1.2 実装タスク

#### タスク 1.2.1: Joint Command Routerノード作成

**新規ファイル**: `biped_gait_control/biped_gait_control/joint_command_router.py`

```python
class JointCommandRouter(Node):
    """
    /joint_commands を購読し、targetに応じて適切な出力先にルーティング
    
    Modes:
    - viz_only: /joint_states にそのまま転送
    - mujoco: MuJoCo APIへ転送（Phase 2で実装）
    - hardware: /forward_position_controller/commands へ転送（Phase 3で実装）
    """
```

#### タスク 1.2.2: 安全リミットチェック追加

```python
def validate_and_clamp(self, positions: List[float]) -> List[float]:
    """URDF定義のリミット内に収める"""
    clamped = []
    for i, pos in enumerate(positions):
        clamped.append(max(self.limits[i].lower, 
                         min(pos, self.limits[i].upper)))
    return clamped
```

#### タスク 1.2.3: 診断情報パブリッシュ

```python
# /diagnostics トピックでルーター状態をパブリッシュ
# - 現在のモード
# - 受信周波数
# - リミット違反回数
```

### 1.3 検証項目

- [ ] ルーターノードが`/joint_commands`を正しく購読
- [ ] リミット外の角度がクランプされる
- [ ] `/diagnostics`で状態が確認できる

### 1.4 完了条件

```bash
# 可視化が安定動作
pixi run ros2 launch biped_gait_control gait_visualization.launch.py target:=viz_only

# 診断情報が確認できる
pixi run ros2 topic echo /diagnostics
```

---

## Phase 2: 物理シミュレータ統合

### 2.1 目標

MuJoCoシミュレータと統合し、物理的に妥当な歩行シミュレーションを実現する。

### 2.2 なぜMuJoCoか

| 観点 | MuJoCo | Gazebo | PyBullet |
|------|--------|--------|----------|
| macOS対応 | ✅ 公式サポート | ⚠️ 限定的 | ✅ |
| 接触シミュレーション | ✅ 高精度 | ○ | △ |
| RL学習との親和性 | ✅ Isaac Gym互換 | △ | ○ |
| 速度 | ✅ 高速 | ○ | ○ |
| ROS 2統合 | △ 自作必要 | ✅ 成熟 | △ |

**決定**: Phase 2ではMuJoCoを採用。ROS 2統合は自作ブリッジで対応。

### 2.3 実装タスク

#### タスク 2.3.1: MuJoCo環境セットアップ

**新規ファイル**: `ros2_ws/pixi.toml`（依存関係追加）

```toml
[dependencies]
mujoco = ">=3.0"
mujoco-python-viewer = "*"  # オプション
```

#### タスク 2.3.2: URDFからMJCF変換

MuJoCoは直接URDFを読めるが、最適化のためMJCF変換を検討：

```bash
# MuJoCo付属ツールでの変換
pixi run python -m mujoco.mjcf <urdf_file>
```

または、URDFのまま読み込み：

```python
import mujoco
model = mujoco.MjModel.from_xml_path("biped_digitigrade.urdf")
```

#### タスク 2.3.3: MuJoCoブリッジノード作成

**新規ファイル**: `biped_gait_control/biped_gait_control/mujoco_bridge.py`

```python
class MuJoCoBridge(Node):
    """
    MuJoCoシミュレータとROS 2を接続するブリッジノード
    
    Subscriptions:
        /joint_commands (sensor_msgs/JointState): 目標関節角度
    
    Publications:
        /joint_states (sensor_msgs/JointState): シミュレータからのフィードバック
        /imu/data (sensor_msgs/Imu): 仮想IMUデータ
        /contact_states (std_msgs/Bool[]): 接地状態
    """
    
    def __init__(self):
        # MuJoCoモデル読み込み
        self.model = mujoco.MjModel.from_xml_path(self.urdf_path)
        self.data = mujoco.MjData(self.model)
        
        # シミュレーションステップを別スレッドで実行
        self.sim_thread = threading.Thread(target=self._simulation_loop)
        
    def _simulation_loop(self):
        """200Hz シミュレーションループ"""
        while rclpy.ok():
            # 目標角度をPD制御で追従
            self._apply_joint_commands()
            
            # シミュレーションステップ
            mujoco.mj_step(self.model, self.data)
            
            # フィードバックパブリッシュ
            self._publish_joint_states()
            self._publish_imu()
            self._publish_contacts()
```

#### タスク 2.3.4: シミュレーション可視化

MuJoCo組み込みビューワーまたはRViz連携：

```python
# Option A: MuJoCo Viewer
import mujoco.viewer
mujoco.viewer.launch(self.model, self.data)

# Option B: RViz（/joint_statesをrobot_state_publisherへ）
# → 既存のRViz設定を流用可能
```

### 2.4 実験: EXP-002 MuJoCoシミュレーション検証

**実験ID**: EXP-002

**目的**: MuJoCoシミュレータで歩容パターンを実行し、物理的妥当性を検証

**手順**:
1. MuJoCoブリッジノード起動
2. 歩容生成ノード起動（target:=mujoco）
3. 歩行安定性を観察
4. 接地判定・転倒判定のチューニング

**成功基準**:
- [ ] シミュレータ内でロボットが転倒せず10秒以上歩行
- [ ] RVizでシミュレータ状態がリアルタイム可視化される
- [ ] `/joint_states`が200Hzでパブリッシュされる

### 2.5 完了条件

```bash
# MuJoCoシミュレーションが動作
pixi run ros2 launch biped_gait_control gait_simulation.launch.py target:=mujoco

# 別ターミナルでRViz可視化
pixi run ros2 launch biped_description display_rviz_only.launch.py
```

---

## Phase 3: 実機制御統合

### 3.1 目標

Jetson上のros2_controlと統合し、MacBookからの関節コマンドで実機モータを制御する。

### 3.2 前提: EXP-001の完了

[EXP-001: RS02実機スライダー制御実験](./exp001_slider_control_rs02.md)が完了していること。

### 3.3 実装タスク

#### タスク 3.3.1: 分散構成でのトピック接続確認

```bash
# 【MacBook】
export ROS_DOMAIN_ID=42

# 【Jetson】
export ROS_DOMAIN_ID=42

# 【MacBook】トピック確認
pixi run ros2 topic list  # Jetson側のトピックが見えるか
```

#### タスク 3.3.2: コントローラ設定更新

**変更対象**: `robstride_hardware/config/controllers.yaml`

```yaml
forward_position_controller:
  ros__parameters:
    joints:
      - left_hip_yaw_joint
      - left_hip_roll_joint
      - left_hip_pitch_joint
      - left_knee_pitch_joint
      - left_ankle_pitch_joint
      - right_hip_yaw_joint
      - right_hip_roll_joint
      - right_hip_pitch_joint
      - right_knee_pitch_joint
      - right_ankle_pitch_joint
    interface_name: position
```

#### タスク 3.3.3: ルーターのhardwareモード実装

**変更対象**: `biped_gait_control/biped_gait_control/joint_command_router.py`

```python
def _route_to_hardware(self, msg: JointState):
    """ros2_controlのforward_position_controllerへ転送"""
    cmd = Float64MultiArray()
    cmd.data = list(msg.position)
    self.hardware_pub.publish(cmd)
```

#### タスク 3.3.4: 安全機構の強化

**重要**: 実機制御前に以下を実装

```python
class SafetyMonitor:
    """
    実機制御時の安全監視
    
    チェック項目:
    - 関節速度制限
    - 関節加速度制限
    - 連続動作時間制限
    - 緊急停止トリガー
    """
    
    def check_velocity_limit(self, current: List[float], 
                             previous: List[float], dt: float) -> bool:
        velocities = [(c - p) / dt for c, p in zip(current, previous)]
        return all(abs(v) < self.max_velocity for v in velocities)
```

### 3.4 実験: EXP-003 分散環境での歩容実行

**実験ID**: EXP-003

**目的**: MacBookで歩容生成し、Jetson経由で実機モータを制御

**構成**:
```
MacBook                          Jetson
┌──────────────────┐            ┌──────────────────┐
│ gait_pattern_    │            │ ros2_control_node│
│ generator        │───DDS────▶│                  │
│                  │            │ forward_position │
│ joint_command_   │            │ _controller      │
│ router           │            │        ↓         │
│ (hardware mode)  │            │ robstride_       │
│                  │◀───DDS────│ hardware         │
│ RViz2            │/joint_     │        ↓         │
│                  │states      │ CAN → RS02      │
└──────────────────┘            └──────────────────┘
```

**手順**:

1. **【Jetson】ros2_control起動**
   ```bash
   cd ~/Projects/bsl_droid_ros2/ros2_ws
   export ROS_DOMAIN_ID=42
   pixi run ros2 launch robstride_hardware bringup.launch.py
   ```

2. **【MacBook】歩容生成起動（hardwareモード）**
   ```bash
   cd ~/Projects/bsl_droid_ros2/ros2_ws
   export ROS_DOMAIN_ID=42
   pixi run ros2 launch biped_gait_control gait_control.launch.py target:=hardware
   ```

3. **【MacBook】可視化起動**
   ```bash
   pixi run ros2 launch biped_description display_rviz_only.launch.py
   ```

**成功基準**:
- [ ] MacBookからの`/joint_commands`がJetsonに到達
- [ ] モータが歩容パターンに追従して動作
- [ ] RVizで実機状態がリアルタイム可視化される
- [ ] 緊急停止が正常に機能する

### 3.5 完了条件

```bash
# 【Jetson】実機制御
pixi run ros2 launch robstride_hardware bringup.launch.py

# 【MacBook】歩容生成
pixi run ros2 launch biped_gait_control gait_control.launch.py target:=hardware

# 【MacBook】可視化
pixi run ros2 launch biped_description display_rviz_only.launch.py
```

---

## Phase 4: システム統合・安定化

### 4.1 目標

全フェーズの成果を統合し、安定したE2E動作を実現する。

### 4.2 タスク一覧

#### タスク 4.2.1: モード切り替えの自動検出

```python
def auto_detect_target() -> str:
    """
    環境に応じて自動的にtargetを選択
    
    - Jetsonノードが見える → hardware
    - MuJoCoノードが見える → mujoco  
    - それ以外 → viz_only
    """
```

#### タスク 4.2.2: 状態遷移管理

```python
class SystemStateManager:
    """
    システム状態の管理
    
    States:
    - IDLE: 待機
    - INITIALIZING: 初期化中
    - READY: 準備完了
    - WALKING: 歩行中
    - EMERGENCY_STOP: 緊急停止
    """
```

#### タスク 4.2.3: ロギング・データ収集

```bash
# 全トピックをMCAPで記録
pixi run ros2 bag record -a -o experiment_data
```

#### タスク 4.2.4: ドキュメント更新

- README.mdにクイックスタート追加
- 各実験結果の記録
- トラブルシューティングガイド

### 4.3 最終検証

**検証シナリオ**:

1. **可視化のみモード（MacBook単体）**
   ```bash
   pixi run ros2 launch biped_gait_control gait_visualization.launch.py target:=viz_only
   ```
   - [ ] RVizで歩行アニメーションが表示される

2. **シミュレータモード（MacBook + MuJoCo）**
   ```bash
   pixi run ros2 launch biped_gait_control gait_simulation.launch.py target:=mujoco
   ```
   - [ ] MuJoCoシミュレータで物理的に歩行する
   - [ ] RVizでシミュレータ状態が可視化される

3. **実機モード（MacBook + Jetson）**
   ```bash
   # Jetson
   pixi run ros2 launch robstride_hardware bringup.launch.py
   
   # MacBook
   pixi run ros2 launch biped_gait_control gait_control.launch.py target:=hardware
   ```
   - [ ] 実機モータが歩容パターンに追従する
   - [ ] 緊急停止が機能する

### 4.4 完了条件

**ゴール達成の定義**:

> 「`/joint_commands`トピックに関節角度をpublishすれば、Launch引数`target`を切り替えるだけで、可視化のみ・物理シミュレータ・実機モータのいずれでも適切に動作する」

---

## 実験記録テンプレート

各実験を実施する際は、以下のテンプレートで記録を作成する。

```markdown
# EXP-XXX: [実験タイトル]

## 実験概要

| 項目 | 内容 |
|------|------|
| 実験ID | EXP-XXX |
| 実施日 | YYYY-MM-DD |
| 目的 | ... |
| 結果 | 成功 / 失敗 / 部分成功 |

## 環境

- MacBook: macOS XX.X, pixi X.X.X
- Jetson: Ubuntu 22.04, pixi X.X.X (該当する場合)
- ROS 2: Jazzy

## 手順

1. ...
2. ...

## 結果

### 成功した点
- ...

### 問題・課題
- ...

## 次のアクション
- ...
```

---

## 参考資料

- [unified_joint_interface_architecture.md](../design/unified_joint_interface_architecture.md) - アーキテクチャ設計
- [distributed_architecture.md](../design/distributed_architecture.md) - 分散システム設計
- [exp001_slider_control_rs02.md](./exp001_slider_control_rs02.md) - 単関節制御実験
- [next_nodes_design.md](../next_nodes_design.md) - ノード設計詳細
