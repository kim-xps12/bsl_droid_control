# BSL-Droid ROS 2 歩行制御システム構築計画

## Context

BSL-Droid（逆関節10自由度二脚ロボット）をLogitech F710rゲームパッドで操縦するROS 2システムを構築する。RL学習済みポリシー（droid-walking-omni系、50次元観測・10次元行動）が速度指令から歩容を生成する。Genesis物理シミュレーションで検証後、実機（Jetson Orin Nano Super）に展開する段階的アプローチをとる。

実装済み資産:
- `ros2_ws/src/biped_description/` — URDF/RViz可視化（実装済み）
- `ros2_ws/src/aoba_hardware/` — ros2_control HW Interface（実装済み、Jetson専用）
- `rl_ws/` — Genesis+PPOによるRL学習環境、F710ゲームパッド操縦（実装済み）

---

## セッション分割ガイド

この計画は6ステップで構成され、1セッションで全て完遂することはできない。以下のようにセッションを分割して実行する。

### セッション一覧

| セッション | 対応Step | 内容 | 進捗 |
|---|---|---|---|
| **Session A** | Step 1 + Step 2 | 要件定義ドキュメント + アーキテクチャ図（drawio.svg） | [完了] |
| **Session B** | Step 3 | パッケージ骨格作成 + pixi.toml更新 + ビルド確認 | [完了] |
| **Session C** | Step 4 | biped_safety(joy_safety) + biped_rl_policy(sim) + Genesis sim + launch + 動作確認 | [完了] |
| **Session D** | Step 5 | biped_safety_node + 設定ファイル整備 + ユニットテスト | [未着手] |
| **Session E** | Step 6 | 実機展開（controllers.yaml 10関節化 + controlモード + IMU統合 + Jetson launch） | [未着手] |

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

## Step 1: 要件定義（Session A）[完了]

`doc/design/ros2_walking/requirements.md` を作成し、以下の要件を軽量な表形式で列挙する。

### 機能要件

| ID | 要件 | 優先度 |
|---|---|---|
| FR-01 | F710rゲームパッドの左スティック上下→前後速度、左右→横速度、右スティック左右→yaw角速度として速度指令を入力 | Must |
| FR-02 | 学習済みRLポリシー（droid-walking-omni系）で速度指令から10関節の目標位置を50Hzで生成 | Must |
| FR-03 | ros2_controlのForwardCommandControllerを介して10個のRS02モータを200Hzで制御（controlモードのみ） | Must |
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

## Step 2: システムアーキテクチャ設計（Session A）[完了]

`doc/design/ros2_walking/fig/ros2_walking_architecture.drawio.svg` を作成する。

### パッケージ構成

```
ros2_ws/src/
  biped_description/         # [実装済み] URDF, RViz
  aoba_hardware/        # [実装済み] ros2_control HW Interface (Jetson専用)
  biped_msgs/                # [実装済み] カスタムメッセージ定義
  biped_rl_policy/           # [実装済み] RLポリシー推論 (Python)
  biped_genesis_sim/         # [実装済み] Genesis物理シミュレーションブリッジ (Python)
  biped_gait_control/        # [実装済み] IK軌道歩容生成 (Python)
  biped_safety/              # [一部実装済み] 安全監視（緊急停止・ゲームパッド切断検知・将来: 関節・姿勢監視） (Python)
  biped_bringup/             # [実装済み] Launch統合・設定ファイル
```

### ノードグラフ（データフロー）— simモード（Genesis）[実装済み]

```
[joy_node] →(sensor_msgs/Joy)→ [teleop_twist_joy_node]
                              →(sensor_msgs/Joy)→ [biped_joy_safety_node] → /emergency_stop
                                    │
                              (geometry_msgs/Twist)
                              /cmd_vel
                                    │
                                    ▼
                          [genesis_sim_node]
                              │           ▲
               /policy_obs    │           │  /policy_actions
         (Float64MultiArray)  │           │  (Float64MultiArray)
                              ▼           │
                    [biped_rl_policy_node] (sim, イベント駆動)

[genesis_sim_node] → /joint_states → [robot_state_publisher] → /tf
                   → /clock
```

### ノードグラフ（データフロー）— controlモード [未実装・計画]

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
                          [aoba_hardware] (200Hz)
                                    │
                               CAN bus → RS02 x10

[imu_driver] →(sensor_msgs/Imu)→ [biped_rl_policy_node]
  /imu/data

[biped_safety_node] (200Hz) ←── /joint_states, /imu/data, /cmd_vel
  │
  └→ /safety_status, /emergency_stop
```

### 主要な設計判断

1. **joy_node（ROS 2標準）を使用** — pygameではなく。デバイスhotplug対応・ROS 2エコシステム統合
2. **RLポリシーはPython (rclpy)** — PyTorchモデル（MLP 512-256-128）の50Hz推論はPythonで十分。libtorch化は時期尚早
3. **ForwardCommandController利用（controlモード）** — RL出力 `action * 0.25 + default_dof_pos` を直接コマンド。ハードウェアPD（kp=35, kd=2）が200Hzで補間。simモードではgenesis_sim_nodeが内部PD制御を実行
4. **観測ベクトルは50次元を厳密に再現** — `droid_env_unitree.py:714-730` と完全一致させる。simモードではgenesis_sim_nodeが構築、controlモードではbiped_rl_policy_nodeが構築
5. **sim/controlの2モード** — simはGenesisシミュレーションで歩行検証（物理フィードバック）、controlは実機制御用。`joint_interface.md`の設計と整合
6. **Genesis物理エンジンの採用** [計画変更] — 当初計画のGazebo Harmonicから変更。RL学習環境と同一エンジンを使用することでsim-to-sim gapを排除。観測ベクトル構築ロジックを学習環境から直接移植可能
7. **IMUは実機に最初から搭載** — 二脚ロボットにおいてIMUは姿勢制御の最も基本的なセンサ。実機のハードウェア構成にIMUは必須

### カスタムメッセージ（biped_msgs）

- `SafetyStatus.msg` — is_safe, warning_flags, error_flags, message
- `RLPolicyState.msg` — observation[], action[], target_positions[], inference_time_ms（デバッグ用）

### 成果物
- `doc/design/ros2_walking/fig/ros2_walking_architecture.drawio.svg`（ノードグラフ・データフロー図）
- `doc/design/ros2_walking/requirements.md`（要件定義、Step 1と統合）

---

## Step 3: パッケージ骨格作成（Session B — Phase 0）[完了]

### タスク
1. `biped_msgs` パッケージ作成（ament_cmake, SafetyStatus.msg, RLPolicyState.msg）
2. `biped_safety` パッケージ骨格（ament_python（当初 biped_teleop として作成、後に統合）
3. `biped_rl_policy` パッケージ骨格（ament_python）
4. `biped_safety` パッケージ骨格（ament_python）
5. `biped_genesis_sim` パッケージ骨格（ament_python）
6. `biped_bringup` パッケージ骨格（ament_cmake, launch/config）
7. `pixi.toml` に `ros-jazzy-joy`, `ros-jazzy-teleop-twist-joy` 依存追加
8. `controllers.yaml` を10関節対応に拡張
9. `pixi run colcon build` でビルド確認

### 変更対象ファイル
- `ros2_ws/pixi.toml` — joy, teleop_twist_joy依存追加
- `ros2_ws/src/aoba_hardware/config/controllers.yaml` — 10関節化
- 新規パッケージ6つ（上記）

---

## Step 4: ゲームパッド→Genesisシミュレーション歩行（Session C — Phase 1 MVP）[完了]

F710でスティックを倒すとGenesis上のロボットが物理シミュレーションで歩行する、最初のデモ。RLポリシーは閉ループ制御器であり、物理シミュレータからのセンサフィードバック（関節状態、接触、重力方向）が歩容生成に必須である。

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

#### 4.2 Genesis物理シミュレーション環境構築 [計画変更: Gazebo → Genesis]

当初計画のGazebo Harmonicに代わり、Genesis物理エンジンを採用した。genesis_sim_nodeがRL学習環境（`droid_env_unitree.py`）と同等の物理シミュレーション・観測構築を行う。

**genesis_sim_node（biped_genesis_simパッケージ）:**
- アクション駆動型の環境ノード。RL学習時のステップ関数を忠実に再現
- Subscribe: `/policy_actions` (Float64MultiArray, 10次元), `/cmd_vel` (Twist)
- Publish: `/policy_obs` (Float64MultiArray, 50次元), `/joint_states` (JointState), `/clock` (Clock)
- 内部PD制御: kp=35, kd=2（関節別オーバーライド: knee kp=50, ankle kd=8）
- 観測ベクトル構築（50次元）は学習環境と完全一致
- アクションレイテンシ: 前フレームのアクションを適用（学習環境と同一）
- URDFはプレーンURDF（xacroではなく）を使用: `rl_ws/assets/bsl_droid_simplified_v2.urdf`
- hip_roll_inward_limit: -0.05 rad（PDターゲットクランプ、訓練環境と一致）

#### 4.3 biped_rl_policy_node（simモード）

**simモード（主要）— Genesis物理シミュレーション:**
- `/policy_obs` subscribe → 推論 → `/policy_actions` publish（イベント駆動）
- 純粋な推論ラッパー: 観測ベクトル構築はgenesis_sim_nodeが担当
- ros2_controlは使用しない
- 緊急停止時はゼロアクション出力

#### 4.4 biped_bringup/launch

**genesis_teleop.launch.py（主要）:**
- robot_state_publisher + genesis_sim_node + biped_rl_policy_node(sim) + joy_node + teleop_twist_joy_node + biped_joy_safety_node
- 段階的起動: t=0でrobot_state_publisher + genesis_sim_node、t=3sでrl_policy + joy/teleopノード群
- `use_sim_time:=true`を全ノードに設定（genesis_sim_nodeはクロックソースのため`false`）

### 検証方法（genesis_teleop.launch.py）
- Genesis上でロボットが立位姿勢で安定
- F710左スティックを前に倒す → Genesis上でロボットが歩行
- スティックをニュートラルに戻す → 立位姿勢に復帰
- LBボタンを離す → 動作停止（ゼロ指令）
- ロボットが転倒せず安定歩行を維持

### 参照ファイル
- `rl_ws/biped_walking/envs/droid_env_unitree.py` — 観測ベクトル構築（L709-730）、アクション変換（L487）
- `rl_ws/biped_walking/biped_eval_gamepad.py` — ゲームパッド軸マッピング、推論ループ
- `rl_ws/biped_walking/train/droid_train_omni_v21.py` — env_cfg/obs_cfg/default_joint_angles

---

## Step 5: 安全監視・設定整備（Session D — Phase 2）[未着手]

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

## Step 6: 実機展開（Session E — Phase 3）[未着手]

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
- `/forward_position_controller/commands`出力、`/joint_states`・`/imu/data`入力
- センサデータのソースが実ハードウェア（エンコーダ、IMU）
- 観測ベクトル構築ロジックはgenesis_sim_nodeの実装を参考にbiped_rl_policy_node内に移植

#### 6.3 controllers.yaml 10関節化
controllers.yamlは既に10関節対応済み（Session Bで完了）。

#### 6.4 Launch分離
- `real_control.launch.py`（Jetson）: aoba_hardware + controllers + imu_driver + rl_policy + safety + joy + teleop
- `real_viz.launch.py`（MacBook）: robot_state_publisher + rviz2（DDSで/joint_statesを受信）

#### 6.5 段階的ハードウェアテスト
1. IMU単体テスト（データ取得・RViz可視化確認）
2. 単関節テスト（1モータ、小振幅正弦波）
3. 全10関節、立位姿勢（default_dof_posのみ送信）
4. テストスタンドでゲームパッド入力（小振幅）
5. セーフティテザー付き自立歩行

### 変更対象ファイル
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
| GenesisのROSノードとしての安定性 | Genesis初期化タイムアウト・例外処理を実装。GPU非対応環境ではCPUフォールバック |

---

## 全体の検証計画

| Phase | 検証内容 | 方法 | 進捗 |
|---|---|---|---|
| Phase 0 | 全パッケージビルド成功 | `pixi run colcon build` | [完了] |
| Phase 1 | Genesisシミュレーション歩行 | Genesis上でF710操作・歩行安定性確認 | [完了] |
| Phase 2 | 安全停止・ユニットテスト | `pixi run colcon test` + 手動テスト | [未着手] |
| Phase 3 | IMU確認→単関節→全関節→テストスタンド→自立歩行 | 段階的ハードウェアテスト | [未着手] |
