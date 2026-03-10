# biped_bringup パッケージ モジュール設計

## 1. 概要

`biped_bringup`は、BSL-Droid歩行制御システム全体の起動と設定を統括するament_cmakeパッケージである。本パッケージは実行可能ノードを含まず、launchファイルと設定ファイルのみで構成される。

各実行環境（Genesisシミュレーション、Jetsonでの実機制御、MacBook+Jetsonの分散構成）に応じたlaunchファイルを提供し、関連する全パッケージのノードを適切なパラメータで統合起動する。

---

## 2. Launchファイル

### 2.1 genesis_teleop.launch.py（Genesisシミュレーション、MacBook）[実装済み]

Genesis物理シミュレーションによる歩行検証の起動ファイルである。MacBook単体で動作する。genesis_sim_nodeが物理シミュレーション・観測ベクトル構築・PD制御を担当し、biped_rl_policy_nodeが純粋な推論ラッパーとしてイベント駆動で動作する。ros2_controlは使用しない。

**起動ノード:**

| ノード | パッケージ | 役割 |
|---|---|---|
| robot_state_publisher | robot_state_publisher | URDF→TF変換（xacro URDF使用） |
| genesis_sim_node | biped_genesis_sim | Genesis環境ノード（物理ステップ + 観測構築 + PD制御） |
| joy_node | joy | F710rゲームパッドの入力取得 |
| teleop_twist_joy_node | teleop_twist_joy | Joy→Twist変換、デッドマンスイッチ処理 |
| biped_joy_safety_node | biped_safety | 緊急停止ボタン監視、ゲームパッド切断検出 |
| biped_rl_policy_node | biped_rl_policy | RLポリシー推論（mode=sim、イベント駆動） |

**Launch引数:**

| 引数 | デフォルト | 説明 |
|---|---|---|
| `model_path` | `""` | 学習済みモデルファイルのパス（.pt） |
| `show_viewer` | `true` | Genesisビューアの表示 |
| `genesis_urdf` | `rl_ws/assets/bsl_droid_simplified_v2.urdf` | Genesis用プレーンURDFのパス |

**動作概要:**

genesis_sim_nodeがGenesis物理エンジンを初期化し、URDFからロボットモデルをロードする。genesis_sim_nodeとbiped_rl_policy_nodeは同期イベント駆動ループを構成する: genesis_sim_nodeが物理ステップ実行後に50次元の観測ベクトルを`/policy_obs`としてpublish → biped_rl_policy_nodeが受信して推論し10次元のアクションを`/policy_actions`としてpublish → genesis_sim_nodeが受信してPD制御で関節を駆動 → 次の物理ステップ → ...

genesis_sim_nodeは`/clock`をpublishし、他の全ノードは`use_sim_time:=true`でシミュレーション時刻に同期する。genesis_sim_nodeはクロックソースであるため`use_sim_time:=false`で動作する。

起動は段階的に行う: t=0でrobot_state_publisherとgenesis_sim_node、t=3sでbiped_rl_policy_node、joy_node、teleop_twist_joy_node、biped_joy_safety_nodeを起動する。Genesis初期化に数秒を要するため、ポリシーノードの起動を遅延させる。

genesis_sim_nodeはプレーンURDF（xacroではなく）を使用する。robot_state_publisherはxacro URDFを使用してTFツリーを構築する。

**パラメータ読込:**

- joy_f710.yaml → joy_node, teleop_twist_joy_node, biped_joy_safety_node
- genesis_sim.yaml → genesis_sim_node

### 2.2 real_control.launch.py（Phase 3、Jetson）[未実装・計画]

Phase 3における実機制御の起動ファイルである。Jetson Orin Nano Super上で動作し、ros2_controlを介してRS02モータを制御する。

**起動ノード:**

| ノード | パッケージ | 役割 |
|---|---|---|
| controller_manager | controller_manager | ros2_controlコントローラ管理 |
| robstride_hardware | robstride_hardware | RS02モータHardware Interface（200Hz） |
| forward_position_controller | forward_command_controller | 位置指令のHWインターフェースへの転送 |
| joint_state_broadcaster | joint_state_broadcaster | 実エンコーダ→/joint_states配信 |
| imu_driver | （IMUドライバパッケージ） | IMUセンサデータ取得 |
| biped_rl_policy_node | biped_rl_policy | RLポリシー推論（mode=control） |
| biped_safety_node | biped_safety | 200Hz安全監視 |
| joy_node | joy | F710rゲームパッド入力 |
| teleop_twist_joy_node | teleop_twist_joy | Joy→Twist変換、デッドマンスイッチ処理 |
| biped_joy_safety_node | biped_safety | 緊急停止ボタン監視、ゲームパッド切断検出 |

**動作概要:**

robstride_hardwareがCAN bus経由でRS02モータと200Hz通信を行う。biped_rl_policy_nodeはcontrolモードで動作し、`/cmd_vel`からRLポリシー推論を行って`/forward_position_controller/commands`にFloat64MultiArrayとして出力する。forward_position_controllerがハードウェアPD制御（kp=35, kd=2）で200Hzの位置追従を行う。

biped_safety_nodeは`/joint_states`と`/imu/data`を200Hzで監視し、安全違反検出時に`/emergency_stop`を発行する。

**パラメータ読込:**

- joy_f710.yaml → joy_node, teleop_twist_joy_node, biped_joy_safety_node
- rl_policy.yaml → biped_rl_policy_node（mode=controlをオーバーライド）
- safety_limits.yaml → biped_safety_node
- controllers.yaml → controller_manager

**スケジューリング:**

Jetson上ではリアルタイムスケジューリングを適用する。robstride_hardwareはSCHED_FIFO優先度50、biped_safety_nodeはSCHED_FIFO優先度45で動作する（次期ノード設計の性能要件表に準拠）。

### 2.3 real_viz.launch.py（Phase 3、MacBook）[未実装・計画]

Phase 3においてMacBookからJetsonの実機状態を可視化するための起動ファイルである。Jetson上の`real_control.launch.py`と併用する。

**起動ノード:**

| ノード | パッケージ | 役割 |
|---|---|---|
| robot_state_publisher | robot_state_publisher | URDF→TF変換 |
| rviz2 | rviz2 | 3D可視化 |

**動作概要:**

JetsonのDDS通信経由で`/joint_states`を受信し、robot_state_publisherがTFツリーを構築してrviz2で可視化する。制御ノードは一切起動しない。

MacBookとJetsonは同一のROS_DOMAIN_ID（デフォルト42）で通信する。画像データ等の大容量トピックは必要に応じてimage_transportで圧縮転送する。

**パラメータ読込:**

- walking.rviz → rviz2

### 2.4 trajectory_replay.launch.py（軌道リプレイ、MacBook / Jetson）[実装済み]

`biped_gait_control`パッケージが提供する軌道リプレイ専用の起動ファイルである。事前設計された脚軌道を再生し、RViz2上の3Dモデルと実機の動きの一致を確認する用途に使用する。

本launchファイルは`biped_bringup`パッケージではなく`biped_gait_control`パッケージに所属する。詳細は[biped_gait_controlモジュール設計書](./gait_control.md)を参照。

---

## 3. 設定ファイル

### 3.1 config/joy_f710.yaml

Logitech F710rゲームパッド（DirectInputモード）の軸・ボタンマッピングを定義する。joy_node、teleop_twist_joy_node、biped_joy_safety_node の3ノードに対するパラメータを含む。

**joy_node パラメータ:**

| パラメータ | 値 | 説明 |
|---|---|---|
| deadzone | 0.08 | スティック中心のドリフト防止（teleop_twist_joy にはデッドゾーン機能がないため joy_node 側で処理） |

**teleop_twist_joy_node パラメータ:**

| パラメータ | 値 | 説明 |
|---|---|---|
| axis_linear.x | 1 | 左スティック上下 → 前後速度 |
| axis_linear.y | 0 | 左スティック左右 → 横速度 |
| axis_angular.yaw | 2 | 右スティック左右 → yaw角速度 |
| scale_linear.x | -0.5 | 前後速度スケール [m/s]。負値で符号反転 |
| scale_linear.y | -0.3 | 横速度スケール [m/s]。負値で符号反転 |
| scale_angular.yaw | -1.0 | yaw角速度スケール [rad/s]。負値で符号反転 |
| enable_button | 4 | LBボタン（デッドマンスイッチ、SR-04） |

**biped_joy_safety_node パラメータ:**

| パラメータ | 値 | 説明 |
|---|---|---|
| emergency_stop_buttons | [9, 10] | L3+R3同時押し込み（緊急停止、FR-05） |
| joy_timeout | 0.5 | ゲームパッド切断検出のタイムアウト [秒]（FR-07） |

軸マッピングは`rl_ws/biped_walking/biped_eval_gamepad.py`のマッピングに準拠する（要件FR-01）。`scale_*` パラメータを負値にすることで、joy_nodeが報告する符号（スティック上/左が負値）をロボット座標系（前進/左移動が正）に変換する。速度上限のデフォルト値はRL学習時の`commands_scale`に合わせて設定する。

### 3.2 config/rl_policy.yaml

RLポリシーノードのデフォルトパラメータを定義する。simモードではgenesis_sim.yamlが観測構築パラメータを提供する。

**主要パラメータ:**

| パラメータ | 値 | 説明 |
|---|---|---|
| model_path | （実行時指定） | 学習済みモデルファイルのパス |
| obs_scales.dof_pos | 1.0 | 関節偏差の観測スケール |
| obs_scales.dof_vel | 0.05 | 関節速度の観測スケール |
| action_scale | 0.25 | アクション→関節位置変換のスケール |
| default_dof_pos | [0.0, 0.0, 1.047, -1.745, 0.785, 0.0, 0.0, 1.047, -1.745, 0.785] | 各関節のデフォルト位置 [rad] |
| gait_frequency | 1.5 | 歩容位相の周波数 [Hz] |
| mode | sim | 動作モード（sim/control） |

`default_dof_pos`の値はhip_pitch=60度、knee_pitch=-100度、ankle_pitch=45度に対応する（`droid_train_omni_v21.py`の設定に準拠）。hip_yawとhip_rollのデフォルトは0.0である。関節順序はALL_JOINTS（left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee_pitch, left_ankle_pitch, right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee_pitch, right_ankle_pitch）に従う。

### 3.3 config/genesis_sim.yaml

Genesis物理シミュレーションノードのパラメータを定義する。RL学習環境（`droid_train_omni_v21.py`）の設定を忠実に再現する。

**物理パラメータ:**

| パラメータ | 値 | 説明 |
|---|---|---|
| dt | 0.02 | 制御周期 [s]（50Hz、学習時と一致） |
| substeps | 2 | 物理サブステップ数 |
| kp | 35.0 | PD制御の位置ゲイン（デフォルト） |
| kd | 2.0 | PD制御の速度ゲイン（デフォルト） |

**関節別PDゲインオーバーライド:**

| 関節 | kp | kd | 備考 |
|---|---|---|---|
| left/right_knee_pitch_joint | 50.0 | (デフォルト) | 膝の応答性強化 |
| left/right_ankle_pitch_joint | (デフォルト) | 8.0 | 足首の振動抑制 |

**初期状態:**

| パラメータ | 値 | 説明 |
|---|---|---|
| base_init_pos | [0.0, 0.0, 0.35] | 初期ベース位置 [m] |
| base_init_quat | [1.0, 0.0, 0.0, 0.0] | 初期ベース姿勢（クォータニオン） |
| show_viewer | true | Genesisビューアの表示 |

**観測構築パラメータ（学習環境と同一）:**

| パラメータ | 値 | 説明 |
|---|---|---|
| action_scale | 0.25 | アクション→関節位置変換のスケール [rad] |
| gait_frequency | 1.5 | 歩容位相の周波数 [Hz] |
| obs_scales.lin_vel | 2.0 | ベース線速度のスケーリング |
| obs_scales.ang_vel | 0.25 | ベース角速度のスケーリング |
| obs_scales.dof_pos | 1.0 | 関節偏差のスケーリング |
| obs_scales.dof_vel | 0.05 | 関節速度のスケーリング |
| commands_scale | [2.0, 2.0, 0.25] | 速度指令のスケーリング [lin_vel_x, lin_vel_y, ang_vel_yaw] |

**追加パラメータ:**

| パラメータ | 値 | 説明 |
|---|---|---|
| hip_roll_inward_limit | -0.05 | hip roll PDターゲットクランプ [rad]。脚の内向き回転を制限（学習環境と一致） |
| feet_names | [left_foot_link, right_foot_link] | 接触検出用の足リンク名 |
| contact_threshold | 0.05 | 接触検出のZ高さ閾値 [m] |

### 3.4 config/safety_limits.yaml

biped_safety_nodeの安全監視パラメータを定義する。詳細はbiped_safetyモジュール設計を参照。

> **注意:** biped_safety_nodeは現在スタブ実装であり、これらのパラメータは将来の実装で使用される。

**主要パラメータ:**

| パラメータ | 値 | 説明 |
|---|---|---|
| joint_limits.hip_yaw | [-0.785, 0.785] | hip_yaw可動範囲 [rad] |
| joint_limits.hip_roll | [-0.524, 0.524] | hip_roll可動範囲 [rad] |
| joint_limits.hip_pitch | [-1.571, 1.047] | hip_pitch可動範囲 [rad] |
| joint_limits.knee_pitch | [-2.094, 0.0] | knee_pitch可動範囲 [rad]（逆関節） |
| joint_limits.ankle_pitch | [-1.047, 1.047] | ankle_pitch可動範囲 [rad] |
| max_joint_velocity | 25.0 | 関節速度上限 [rad/s] |
| max_tilt_angle | 1.047 | 姿勢異常閾値 [rad]（60度） |
| min_base_height | 0.1 | 転倒検出閾値 [m] |
| warning_threshold_ratio | 0.8 | Warning閾値の比率 |
| cmd_vel_timeout | 1.0 | コマンドウォッチドッグのタイムアウト [s] |

関節リミットはbiped_description URDFの定義値と一致させる。URDFからの自動取得も可能であるが、本設定ファイルでの明示指定を優先する。

### 3.5 config/controllers.yaml

ros2_controlのコントローラ設定を定義する。controlモード（Phase 3、実機robstride_hardware経由）で使用する。simモード（Genesis）ではros2_controlを使用しないため、本ファイルは参照されない。

**設定内容:**

```yaml
controller_manager:
  ros__parameters:
    update_rate: 200

    forward_position_controller:
      type: forward_command_controller/ForwardCommandController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

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

controller_managerの更新レートは200Hzとし、robstride_hardwareの制御ループ周期と一致させる。forward_position_controllerは10関節全てのposition interfaceを管理する。

---

## 4. モード切替の設計思想

本パッケージのlaunchファイルは、統一関節インターフェースの設計思想に基づき、sim/controlの2モードを明確に分離している。

| 項目 | simモード (genesis_teleop.launch.py) | controlモード (real_control.launch.py) |
|---|---|---|
| 実装状態 | [実装済み] | [未実装・計画] |
| 実行環境 | MacBook | Jetson |
| 物理シミュレーション | Genesis物理エンジン | なし（実機） |
| ros2_control | 不使用（genesis_sim_node内部PD制御） | robstride_hardware（実機） |
| RLポリシー入力 | `/policy_obs`（genesis_sim_nodeから受信） | `/cmd_vel` + `/joint_states` + `/imu/data`（自前観測構築） |
| RLポリシー出力 | `/policy_actions`（genesis_sim_nodeへ送信） | `/forward_position_controller/commands` |
| センサ入力 | Genesis物理シミュレーション | 実センサ（エンコーダ、IMU） |
| 安全監視 | なし | 全チェック有効 |
| 用途 | 歩行品質検証（物理フィードバックあり） | 実機制御 |

simモードではgenesis_sim_nodeが学習環境（`droid_env_unitree.py`）と同一の物理エンジン・観測構築ロジックを使用するため、sim-to-sim gapが存在しない。モード切替は起動するlaunchファイルそのものを選択することで行う（`genesis_teleop.launch.py` / 将来の `real_control.launch.py`）。

---

## 5. 依存パッケージ

### 5.1 ビルド依存

- ament_cmake

### 5.2 実行時依存

| パッケージ | 用途 | 備考 |
|---|---|---|
| biped_description | URDF、robot_state_publisher | 実装済み |
| biped_msgs | SafetyStatus等カスタムメッセージ | 実装済み |
| biped_safety | 安全監視（緊急停止・ゲームパッド切断検出・将来: 関節・姿勢監視） | 実装済み（joy_safety） / スタブ（safety_node） |
| teleop_twist_joy | Joy→Twist変換、デッドマンスイッチ | ROS 2標準 |
| biped_rl_policy | RLポリシー推論 | 実装済み |
| biped_genesis_sim | Genesis物理シミュレーション | 実装済み（simモード） |
| robstride_hardware | ros2_control HW Interface | 実装済み、Phase 3のみ |
| joy | ゲームパッドドライバ | ROS 2標準 |
| robot_state_publisher | URDF→TF | ROS 2標準 |
| rviz2 | 3D可視化 | ROS 2標準 |
| controller_manager | ros2_controlコントローラ管理 | Phase 3のみ |
| forward_command_controller | 位置指令コントローラ | Phase 3のみ |
| joint_state_broadcaster | 関節状態配信 | Phase 3のみ |
