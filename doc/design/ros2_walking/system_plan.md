# BSL-Droid ROS 2 歩行制御システム構築計画

## Context

BSL-Droid（逆関節10自由度二脚ロボット）をLogitech F710rゲームパッドで操縦するROS 2システムを構築する。RL学習済みポリシー（droid-walking-omni系、50次元観測・10次元行動）が速度指令から歩容を生成する。シミュレーションで検証後、実機（Jetson Orin Nano Super）に展開する段階的アプローチをとる。

実装済み資産:
- `ros2_ws/src/biped_description/` — URDF/RViz可視化（実装済み）
- `ros2_ws/src/robstride_hardware/` — ros2_control HW Interface（実装済み、Jetson専用）
- `rl_ws/` — Genesis+PPOによるRL学習環境、F710ゲームパッド操縦（実装済み）

---

## セッション分割ガイド

この計画は6ステップで構成され、1セッションで全て完遂することはできない。以下のようにセッションを分割して実行する。

### セッション一覧

| セッション | 対応Step | 内容 | 推定規模 | 前提 |
|---|---|---|---|---|
| **Session A** | Step 1 + Step 2 | 要件定義ドキュメント + アーキテクチャ図（drawio.svg） | ドキュメント2ファイル | なし |
| **Session B** | Step 3 | 5パッケージ骨格作成 + pixi.toml更新（Gazebo含む） + ビルド確認 | 新規ファイル ~25個 | Session A完了 |
| **Session C** | Step 4 | biped_teleop_node + biped_rl_policy_node(viz/sim) + Gazeboシミュレーション + launch + 動作確認 | ノード実装3ファイル + launch + Gazebo設定 | Session B完了 |
| **Session D** | Step 5 | biped_safety_node + 設定ファイル整備 + ユニットテスト | ノード実装 + テスト + config | Session C完了 |
| **Session E** | Step 6 | 実機展開（controllers.yaml 10関節化 + controlモード + IMU統合 + Jetson launch） | 設定変更 + ノード修正 + launch | Session D完了 + Jetson環境 + IMU搭載済み実機 |

### 使い方

各セッション開始時にClaude Codeへ以下のように指示する:

```
doc/design/ros2_walking/system_plan.md のSession {X} を実行してください。
```

### 注意事項
- 各セッション終了時にユーザが手動でgit commitし、次セッションとの境界を明確にする
- Session A〜Dはソフトウェアのみ（MacBookで完結）、Session Eは実機（Jetson + IMU搭載ロボット）が必要
- Session E内の段階的テストは手動作業を含むため、1セッション内で完遂しない可能性がある

---

## Step 1: 要件定義（Session A）

`doc/design/ros2_walking/requirements.md` を作成し、以下の要件を軽量な表形式で列挙する。

### 機能要件

| ID | 要件 | 優先度 |
|---|---|---|
| FR-01 | F710rゲームパッドの左スティック上下→前後速度、左右→横速度、右スティック左右→yaw角速度として速度指令を入力 | Must |
| FR-02 | 学習済みRLポリシー（droid-walking-omni系）で速度指令から10関節の目標位置を50Hzで生成 | Must |
| FR-03 | ros2_controlのForwardCommandControllerを介して10個のRS02モータを200Hzで制御 | Must |
| FR-04 | RViz2でロボットの関節状態をリアルタイム可視化 | Must |
| FR-05 | ゲームパッドのボタンで緊急停止 | Must |
| FR-06 | 関節角度/速度リミット・姿勢異常を200Hzで監視し安全停止 | Must |
| FR-07 | ゲームパッド切断時は速度指令をゼロにする | Must |

### 非機能要件

| ID | 要件 |
|---|---|
| NFR-01 | RLポリシー推論レイテンシ < 10ms（50Hz） |
| NFR-02 | ゲームパッド→アクチュエータ端末間レイテンシ < 40ms |
| NFR-03 | 安全監視は200Hz/レイテンシ < 1ms |
| NFR-04 | Jetson Orin Nano Super（8GB RAM, ARM64）で動作 |

### 安全要件

| ID | 要件 | 閾値 |
|---|---|---|
| SR-01 | 過度な傾きで緊急停止 | Roll/Pitch > 60° |
| SR-02 | 転倒検出で緊急停止 | ベース高さ < 0.1m |
| SR-03 | 関節リミット超過で停止 | URDFリミットの100% |
| SR-04 | デッドマンスイッチ（LBボタン） | 押下中のみ歩行 |

### 成果物
- `doc/design/ros2_walking/requirements.md`

---

## Step 2: システムアーキテクチャ設計（Session A）

`doc/design/ros2_walking/fig/ros2_walking_architecture.drawio.svg` を作成する。

### パッケージ構成

```
ros2_ws/src/
  biped_description/         # [実装済み] URDF, RViz
  robstride_hardware/        # [実装済み] ros2_control HW Interface (Jetson専用)
  biped_msgs/                # [新規] カスタムメッセージ定義
  biped_teleop/              # [新規] ゲームパッドteleop (Python)
  biped_rl_policy/           # [新規] RLポリシー推論 (Python)
  biped_safety/              # [新規] 安全監視 (Python)
  biped_bringup/             # [新規] Launch統合・設定ファイル
```

### ノードグラフ（データフロー）

```
[joy_node] →(sensor_msgs/Joy)→ [teleop_twist_joy_node]
                              →(sensor_msgs/Joy)→ [biped_joy_safety_node] → /emergency_stop
                                    │
                              (geometry_msgs/Twist)
                              /cmd_vel
                                    │
                                    ▼
[joint_state_broadcaster] → [biped_rl_policy_node] (50Hz)
  (sensor_msgs/JointState)         │
  /joint_states                    │
                              (std_msgs/Float64MultiArray)
                              /forward_position_controller/commands
                                    │
                                    ▼
                          [forward_position_controller] (200Hz)
                                    │
                                    ▼
                          [robstride_hardware] (200Hz)
                                    │
                               CAN bus → RS02 x10

[imu_driver] →(sensor_msgs/Imu)→ [biped_rl_policy_node]
  /imu/data

[biped_safety_node] (200Hz) ←── /joint_states, /imu/data, /cmd_vel
  │
  └→ /safety_status, /emergency_stop

[biped_joy_safety_node] ←── /joy
  │
  └→ /emergency_stop（緊急停止ボタン、ゲームパッド切断検出）
```

### 主要な設計判断

1. **joy_node（ROS 2標準）を使用** — pygameではなく。デバイスhotplug対応・ROS 2エコシステム統合
2. **RLポリシーはPython (rclpy)** — PyTorchモデル（MLP 512-256-128）の50Hz推論はPythonで十分。libtorch化は時期尚早
3. **ForwardCommandController利用** — RL出力 `action * 0.25 + default_dof_pos` を直接コマンド。ハードウェアPD（kp=35, kd=2）が200Hzで補間
4. **観測ベクトルは50次元を厳密に再現** — `droid_env_unitree.py:714-730` と完全一致させる。simモード（Phase 1）ではGazeboのセンサデータ、実機（Phase 3）では実センサデータを使用
5. **viz/sim/controlの3モード** — vizはノード起動確認用（モックセンサ）、simはGazeboシミュレーションで歩行検証（物理フィードバック）、controlは実機制御用。`joint_interface.md`の設計と整合
6. **Gazebo HarmonicによるMacBook上の物理シミュレーション** — RLポリシーは閉ループ制御器であり、物理フィードバックなしでは歩容を検証できない。`ros-jazzy-ros-gz`と`ros-jazzy-gz-ros2-control`がpixi (robostack-jazzy) でmacOS ARM64に対応しているため、MacBook上でros2_controlを含むフルシミュレーションが可能
7. **IMUは実機に最初から搭載** — 二脚ロボットにおいてIMUは姿勢制御の最も基本的なセンサ。実機のハードウェア構成にIMUは必須

### カスタムメッセージ（biped_msgs）

- `SafetyStatus.msg` — is_safe, warning_flags, error_flags, message
- `RLPolicyState.msg` — observation[], action[], target_positions[], inference_time_ms（デバッグ用）

### 成果物
- `doc/design/ros2_walking/fig/ros2_walking_architecture.drawio.svg`（ノードグラフ・データフロー図）
- `doc/design/ros2_walking/requirements.md`（要件定義、Step 1と統合）

---

## Step 3: パッケージ骨格作成（Session B — Phase 0）

### タスク
1. `biped_msgs` パッケージ作成（ament_cmake, SafetyStatus.msg, RLPolicyState.msg）
2. `biped_teleop` パッケージ骨格（ament_python）
3. `biped_rl_policy` パッケージ骨格（ament_python）
4. `biped_safety` パッケージ骨格（ament_python）
5. `biped_bringup` パッケージ骨格（ament_cmake, launch/config）
6. `pixi.toml` に `ros-jazzy-joy`, `ros-jazzy-teleop-twist-joy`, `ros-jazzy-ros-gz`, `ros-jazzy-gz-ros2-control` 依存追加
7. `controllers.yaml` を10関節対応に拡張（現在はjoint1のみ）
8. `pixi run colcon build` でビルド確認

### 変更対象ファイル
- `ros2_ws/pixi.toml` — joy, teleop_twist_joy, Gazebo関連依存追加
- `ros2_ws/src/robstride_hardware/config/controllers.yaml` — 10関節化
- 新規パッケージ5つ（上記）

---

## Step 4: ゲームパッド→Gazeboシミュレーション歩行（Session C — Phase 1 MVP）

F710でスティックを倒すとGazebo上のロボットが物理シミュレーションで歩行する、最初のデモ。RLポリシーは閉ループ制御器であり、物理シミュレータからのセンサフィードバック（IMU、接触、関節状態）が歩容生成に必須である。

### タスク

#### 4.1 teleop_twist_joy + biped_joy_safety_node

**teleop_twist_joy（ROS 2標準パッケージ、設定のみ）:**
- `/joy` subscribe → `/cmd_vel` publish
- F710 DirectInputモードの軸マッピング（`biped_eval_gamepad.py`準拠）:
  - axis_linear.x=1 (左Y): lin_vel_x, axis_linear.y=0 (左X): lin_vel_y, axis_angular.yaw=2 (右X): ang_vel_yaw
- 符号反転: `scale_*` パラメータを負値に設定（joy_nodeはスティック上/左を負値で報告するため）
- デッドマンボタン: enable_button=4 (LB)、未押下時はゼロTwist
- デッドゾーン: joy_node側のパラメータ `deadzone=0.08` で対応
- `pixi.toml` に `ros-jazzy-teleop-twist-joy` を追加

**biped_joy_safety_node（本パッケージ提供）:**
- `/joy` subscribe → 緊急停止ボタン（STARTボタン）監視、ゲームパッド切断検出
- 緊急停止時またはゲームパッド切断時に `/emergency_stop` に `true` を publish

#### 4.2 Gazeboシミュレーション環境構築
- URDF/xacroにGazeboプラグインを追加:
  - `gz_ros2_control`プラグイン: ros2_controlをGazebo上で動作させる
  - IMUセンサプラグイン: `/imu/data` (sensor_msgs/Imu) を配信
  - 接触センサプラグイン: 足裏の接地状態を検出
- Gazeboワールドファイル作成（平面地形 + ロボットspawn）
- `controllers.yaml`はPhase 3（実機）と同一設定を使用

#### 4.3 biped_rl_policy_node（sim/vizモード）

**simモード（主要）— Gazeboシミュレーション:**
- `/cmd_vel` subscribe → `/forward_position_controller/commands` publish (50Hz)
- `/joint_states`からGazeboの実関節状態を取得（joint_state_broadcaster経由）
- `/imu/data`からGazeboのIMUデータを取得
- 観測ベクトル構築（`droid_env_unitree.py:714-730` を忠実に再現）:
  - base_lin_vel: IMU加速度積分 or 運動学推定
  - base_ang_vel: IMU角速度（`/imu/data`から取得）
  - projected_gravity: IMU姿勢（クォータニオン）から計算
  - commands: /cmd_velから（×commands_scale）
  - dof_pos - default_dof_pos: `/joint_states`から取得（×obs_scales.dof_pos）
  - dof_vel: `/joint_states`から取得（×obs_scales.dof_vel）
  - actions: 前回出力
  - gait_phase_sin/cos: 内部クロック（50Hz, 周期は学習時と同じ）
  - leg_phase: gait_phaseから導出
  - feet_pos_z: FK計算 or 定数
  - contact_state: 接触センサ or 関節トルクから推定
- アクション→関節位置: `target = action * 0.25 + default_dof_pos`
- `default_dof_pos`: hip_pitch=60°, knee=-100°, ankle=45°（`droid_train_omni_v21.py:120-122`）

**vizモード（簡易確認用）— RViz可視化のみ:**
- ノード起動確認・トピック疎通確認用途に限定
- `/cmd_vel` subscribe → `/joint_states` publish (50Hz)
- 全センサ値をモック（base_ang_vel=[0,0,0], projected_gravity=[0,0,-1], contact_state=[1,1]）
- 物理フィードバックなしのため歩行品質の検証には使用しない

#### 4.4 biped_bringup/launch

**sim_teleop.launch.py（主要）:**
- Gazebo（gz_sim） + gz_ros2_control + controller_manager + forward_position_controller + joint_state_broadcaster + biped_rl_policy_node(sim) + biped_teleop_node + joy_node + robot_state_publisher + rviz2
- `use_sim_time:=true`を全ノードに設定

**viz_teleop.launch.py（簡易確認用）:**
- joy_node + biped_teleop_node + biped_rl_policy_node(viz) + robot_state_publisher + rviz2
- ros2_control不使用、Gazebo不使用

### 検証方法（sim_teleop.launch.py）
- Gazebo上でロボットが立位姿勢で安定
- F710左スティックを前に倒す → Gazebo上でロボットが歩行
- スティックをニュートラルに戻す → 立位姿勢に復帰
- LBボタンを離す → 動作停止（ゼロ指令）
- ロボットが転倒せず安定歩行を維持

### 参照ファイル
- `rl_ws/biped_walking/envs/droid_env_unitree.py` — 観測ベクトル構築（L709-730）、アクション変換（L487）
- `rl_ws/biped_walking/biped_eval_gamepad.py` — ゲームパッド軸マッピング、推論ループ
- `rl_ws/biped_walking/train/droid_train_omni_v21.py` — env_cfg/obs_cfg/default_joint_angles

---

## Step 5: 安全監視・設定整備（Session D — Phase 2）

### タスク

#### 5.1 biped_safety_node
- `/joint_states` subscribe、200Hzで監視
- 関節位置リミットチェック（URDF値）
- 関節速度リミットチェック（RS02仕様 ~25 rad/s）
- `/safety_status` publish, 違反時 `/emergency_stop` publish

#### 5.2 設定ファイル整備
- `biped_bringup/config/joy_f710.yaml` — 軸/ボタンマッピング
- `biped_bringup/config/rl_policy.yaml` — モデルパス、obs_scales、action_scale
- `biped_bringup/config/safety_limits.yaml` — 関節リミット、姿勢リミット

#### 5.3 テスト
- 各パッケージのユニットテスト（pytest）
  - teleop: モックJoyメッセージ→緊急停止・切断検出の検証
  - rl_policy: 既知入力→既知出力（シミュレータ記録データと照合）
  - safety: リミット超過検出
- launch_testingによる統合テスト

---

## Step 6: 実機展開（Session E — Phase 3）

実機にはIMUが最初から搭載されている前提で構築する。二脚ロボットにおいてIMUは姿勢制御の最も基本的なセンサであり、実機開発の初日から利用可能であるべきもの。

### タスク

#### 6.1 IMUドライバ統合
- IMUドライバノード（imu_toolsパッケージ or メーカー提供ドライバ）
- `/imu/data` (sensor_msgs/Imu) をpublish
- biped_rl_policy_nodeの観測ベクトルをモック値から実IMUデータに切替:
  - `base_ang_vel` ← IMU角速度
  - `projected_gravity` ← IMU姿勢（クォータニオン）から計算
  - `base_lin_vel` ← IMU加速度積分 or 運動学推定
- 足接地推定（関節トルクから推定、力センサなし）

#### 6.2 biped_rl_policy_node controlモード化
- simモードと同一のトピック構成（`/forward_position_controller/commands`出力、`/joint_states`・`/imu/data`入力）
- センサデータのソースがGazeboから実ハードウェア（エンコーダ、IMU）に変わるのみ
- 観測ベクトル構築ロジックはsimモードと完全共通

#### 6.3 controllers.yaml 10関節化
現在の`joint1`のみ → 10関節全てを定義:
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

#### 6.4 Launch分離
- `real_control.launch.py`（Jetson）: robstride_hardware + controllers + imu_driver + rl_policy + safety + joy + teleop
- `real_viz.launch.py`（MacBook）: robot_state_publisher + rviz2（DDSで/joint_statesを受信）

#### 6.5 段階的ハードウェアテスト
1. IMU単体テスト（データ取得・RViz可視化確認）
2. 単関節テスト（1モータ、小振幅正弦波）
3. 全10関節、立位姿勢（default_dof_posのみ送信）
4. テストスタンドでゲームパッド入力（小振幅）
5. セーフティテザー付き自立歩行

### 変更対象ファイル
- `ros2_ws/src/robstride_hardware/config/controllers.yaml`
- `ros2_ws/src/biped_rl_policy/biped_rl_policy/rl_policy_node.py`
- `ros2_ws/src/biped_bringup/launch/` 新規launch

---

## リスクと対策

| リスク | 対策 |
|---|---|
| PyTorchとpixi環境の競合（特にJetson） | JetPack付属のシステムPyTorchを使用。pixi内ではpip経由でインストール |
| 観測ベクトルのシミュレータとの不一致 | シミュレータで記録したobs/actionペアでユニットテスト |
| controllers.yaml 1→10関節拡張でHW Interface不具合 | 単関節→2関節→全関節と段階的に拡張・テスト |
| F710のDirectInput/XInputモード差異 | 両モード用config作成、ドキュメントにスイッチ位置明記 |
| Gazebo Harmonic macOSでのパフォーマンス | GPUレンダリング無効化オプション（headlessモード）の活用。物理ステップレートの調整 |
| Gazebo↔Genesis間のsim-to-sim gap | 物理パラメータ（質量・慣性・摩擦係数）をGenesis学習環境と統一。まずは定性的な歩行安定性の確認を優先 |

---

## 全体の検証計画

| Phase | 検証内容 | 方法 |
|---|---|---|
| Phase 0 | 全パッケージビルド成功 | `pixi run colcon build` |
| Phase 1 | viz簡易確認（ノード起動・トピック疎通） + Gazeboシミュレーション歩行 | viz: トピック確認、sim: Gazebo上でF710操作・歩行安定性確認 |
| Phase 2 | 安全停止・ユニットテスト | `pixi run colcon test` + 手動テスト |
| Phase 3 | IMU確認→単関節→全関節→テストスタンド→自立歩行 | 段階的ハードウェアテスト |
