# biped_bringup パッケージ モジュール設計

## 1. 概要

`biped_bringup`は、BSL-Droid歩行制御システム全体の起動と設定を統括するament_cmakeパッケージである。本パッケージは実行可能ノードを含まず、launchファイルと設定ファイルのみで構成される。

各実行環境（MacBook単体での可視化、Jetsonでの実機制御、MacBook+Jetsonの分散構成）に応じたlaunchファイルを提供し、関連する全パッケージのノードを適切なパラメータで統合起動する。

---

## 2. ディレクトリ構造

```
biped_bringup/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── viz_teleop.launch.py
│   ├── sim_teleop.launch.py
│   ├── real_control.launch.py
│   └── real_viz.launch.py
├── config/
│   ├── joy_f710.yaml
│   ├── rl_policy.yaml
│   ├── safety_limits.yaml
│   ├── controllers.yaml
│   └── walking.rviz
└── doc/
```

CMakeLists.txtでは`launch/`と`config/`ディレクトリをinstall先に含める。ament_cmakeパッケージとしてビルドし、`find_package(ament_cmake REQUIRED)`のみを必須依存とする。

---

## 3. Launchファイル

### 3.1 viz_teleop.launch.py（簡易確認用、MacBook）

ノード起動確認・トピック疎通確認用の簡易起動ファイルである。MacBook単体で動作し、ros2_controlは使用しない。物理シミュレーションを行わないため、歩行品質の検証には使用しない。歩行検証にはsim_teleop.launch.pyを使用すること。

**起動ノード:**

| ノード | パッケージ | 役割 |
|---|---|---|
| joy_node | joy | F710rゲームパッドの入力取得 |
| teleop_twist_joy_node | teleop_twist_joy | Joy→Twist変換、デッドマンスイッチ処理 |
| biped_joy_safety_node | biped_teleop | 緊急停止ボタン監視、ゲームパッド切断検出 |
| biped_rl_policy_node | biped_rl_policy | RLポリシー推論（mode=viz） |
| robot_state_publisher | robot_state_publisher | URDF→TF変換 |
| rviz2 | rviz2 | 3D可視化 |

**動作概要:**

ゲームパッドの入力をjoy_nodeが`/joy`トピックとしてpublishし、teleop_twist_joy_nodeが`/cmd_vel`に変換する。biped_joy_safety_nodeは同じ`/joy`を購読し、緊急停止ボタンの監視とゲームパッド切断検出を行う。biped_rl_policy_nodeはvizモードで動作し、`/cmd_vel`からRLポリシー推論を行って`/joint_states`を直接publishする。robot_state_publisherが`/joint_states`から`/tf`を生成し、rviz2がロボットモデルを可視化する。

vizモードではIMUデータとしてモック値（base_ang_vel=[0,0,0]、projected_gravity=[0,0,-1]等）を使用する。RLポリシーは閉ループ制御器であるため、モックセンサでは正しい歩容が生成されない。このlaunchファイルはノード間のトピック接続が正しいことの確認にのみ使用する。

**パラメータ読込:**

- joy_f710.yaml → joy_node, teleop_twist_joy_node, biped_joy_safety_node
- rl_policy.yaml → biped_rl_policy_node（mode=vizをオーバーライド）
- walking.rviz → rviz2

### 3.2 sim_teleop.launch.py（Phase 1、MacBook）

Phase 1におけるGazeboシミュレーション歩行検証の起動ファイルである。MacBook単体で動作し、Gazebo Harmonicのgz_ros2_controlプラグインによりros2_controlをシミュレーション上で使用する。これにより、実機（Phase 3）と同一のトピック構成・コントローラ設定で歩行を検証できる。

**起動ノード:**

| ノード | パッケージ | 役割 |
|---|---|---|
| gz_sim | ros_gz_sim | Gazebo Harmonicシミュレータ |
| controller_manager | controller_manager | ros2_controlコントローラ管理（gz_ros2_control経由） |
| forward_position_controller | forward_command_controller | 位置指令のシミュレータへの転送 |
| joint_state_broadcaster | joint_state_broadcaster | シミュレータ関節状態→/joint_states配信 |
| joy_node | joy | F710rゲームパッドの入力取得 |
| teleop_twist_joy_node | teleop_twist_joy | Joy→Twist変換、デッドマンスイッチ処理 |
| biped_joy_safety_node | biped_teleop | 緊急停止ボタン監視、ゲームパッド切断検出 |
| biped_rl_policy_node | biped_rl_policy | RLポリシー推論（mode=sim） |
| robot_state_publisher | robot_state_publisher | URDF→TF変換 |
| rviz2 | rviz2 | 3D可視化 |

**動作概要:**

Gazebo Harmonicが物理シミュレーションを実行し、gz_ros2_controlプラグインがros2_controlのコントローラをシミュレータに接続する。biped_rl_policy_nodeはsimモードで動作し、`/cmd_vel`からRLポリシー推論を行って`/forward_position_controller/commands`にFloat64MultiArrayとして出力する。forward_position_controllerがGazeboのposition interfaceに指令を転送する。

GazeboのIMUセンサプラグインが`/imu/data`を配信し、joint_state_broadcasterがシミュレータの関節状態を`/joint_states`として配信する。biped_rl_policy_nodeはこれらの物理フィードバックを用いて観測ベクトルを構築するため、閉ループ制御による歩容生成が正しく機能する。

全ノードに`use_sim_time:=true`を設定し、Gazeboのシミュレーション時刻に同期する。

**パラメータ読込:**

- joy_f710.yaml → joy_node, teleop_twist_joy_node, biped_joy_safety_node
- rl_policy.yaml → biped_rl_policy_node（mode=simをオーバーライド）
- controllers.yaml → controller_manager（実機と同一設定）
- walking.rviz → rviz2

**URDF/Gazebo設定:**

biped_descriptionのURDF/xacroに以下のGazeboプラグインを追加する:

- `gz_ros2_control`プラグイン: ros2_controlのHardware Interfaceをシミュレータに接続
- IMUセンサプラグイン: `base_link`にIMUセンサを配置し`/imu/data`を配信
- 接触センサプラグイン（任意）: 足裏の接地状態検出

### 3.3 real_control.launch.py（Phase 3、Jetson）

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
| biped_joy_safety_node | biped_teleop | 緊急停止ボタン監視、ゲームパッド切断検出 |

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

### 3.4 real_viz.launch.py（Phase 3、MacBook）

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

---

## 4. 設定ファイル

### 4.1 config/joy_f710.yaml

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
| emergency_stop_button | 7 | STARTボタン（緊急停止、FR-05） |
| joy_timeout | 0.5 | ゲームパッド切断検出のタイムアウト [秒]（FR-07） |

軸マッピングは`rl_ws/biped_walking/biped_eval_gamepad.py`のマッピングに準拠する（要件FR-01）。`scale_*` パラメータを負値にすることで、joy_nodeが報告する符号（スティック上/左が負値）をロボット座標系（前進/左移動が正）に変換する。速度上限のデフォルト値はRL学習時の`commands_scale`に合わせて設定する。

### 4.2 config/rl_policy.yaml

RLポリシー推論ノードのパラメータを定義する。

**主要パラメータ:**

| パラメータ | 値 | 説明 |
|---|---|---|
| model_path | （実行時指定） | 学習済みモデルファイルのパス |
| config_path | （実行時指定） | cfgs.pklファイルのパス |
| obs_scales.dof_pos | 1.0 | 関節偏差の観測スケール |
| obs_scales.dof_vel | 0.05 | 関節速度の観測スケール |
| action_scale | 0.25 | アクション→関節位置変換のスケール |
| default_dof_pos | [0.0, 0.0, 1.047, -1.745, 0.785, 0.0, 0.0, 1.047, -1.745, 0.785] | 各関節のデフォルト位置 [rad] |
| gait_frequency | （学習時設定に準拠） | 歩容位相の周波数 [Hz] |
| mode | viz | 動作モード（viz/sim/control） |

`default_dof_pos`の値はhip_pitch=60度、knee_pitch=-100度、ankle_pitch=45度に対応する（`droid_train_omni_v21.py`の設定に準拠）。hip_yawとhip_rollのデフォルトは0.0である。関節順序はALL_JOINTS（left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee_pitch, left_ankle_pitch, right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee_pitch, right_ankle_pitch）に従う。

### 4.3 config/safety_limits.yaml

biped_safety_nodeの安全監視パラメータを定義する。詳細はbiped_safetyモジュール設計を参照。

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

### 4.4 config/controllers.yaml

ros2_controlのコントローラ設定を定義する。simモード（Phase 1、Gazebo gz_ros2_control経由）およびcontrolモード（Phase 3、実機robstride_hardware経由）で共通して使用する。

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

### 4.5 config/walking.rviz

RViz2の表示設定ファイルである。以下の表示要素を含む。

- RobotModel: biped_description URDFに基づくロボットモデル表示
- TF: 座標フレームの表示（base_link、各関節フレーム、足先フレーム）
- JointState: 関節状態のオーバーレイ表示（任意）
- Grid: 地面グリッド

カメラの初期視点はロボット全体が見える位置に設定する。

---

## 5. モード切替の設計思想

本パッケージのlaunchファイルは、統一関節インターフェースの設計思想に基づき、viz/sim/controlの3モードを明確に分離している。

| 項目 | vizモード (viz_teleop.launch.py) | simモード (sim_teleop.launch.py) | controlモード (real_control.launch.py) |
|---|---|---|---|
| 実行環境 | MacBook | MacBook | Jetson |
| ros2_control | 不使用 | gz_ros2_control（Gazebo経由） | robstride_hardware（実機） |
| RLポリシー出力先 | /joint_states（直接publish） | /forward_position_controller/commands | /forward_position_controller/commands |
| センサ入力 | モック値 | Gazeboシミュレーション（IMU、接触、関節状態） | 実センサ（エンコーダ、IMU） |
| 安全監視 | なし | 関節リミットのみ | 全チェック有効 |
| 用途 | ノード起動確認・トピック疎通確認 | 歩行品質検証（物理フィードバックあり） | 実機制御 |

RLポリシーは閉ループ制御器であり、物理フィードバックなしでは歩容を正しく生成できない。そのため歩行検証にはsimモード（Gazeboシミュレーション）を使用する。vizモードはノード起動確認のみに使用し、歩行品質の検証には使用しない。

simモードとcontrolモードはトピック構成が同一（`/forward_position_controller/commands`出力、`/joint_states`・`/imu/data`入力）であり、センサデータのソースがGazeboから実ハードウェアに変わるのみである。これにより、sim→realの移行が最小限のコード変更で完結する。

Gazebo Harmonicのgz_ros2_controlプラグインにより、MacBook上でもros2_controlが利用可能である（robostack-jazzyの`ros-jazzy-gz-ros2-control`パッケージ）。モード切替はlaunch引数ではなく、起動するlaunchファイルそのものを選択することで行う。

---

## 6. 依存パッケージ

### 6.1 ビルド依存

- ament_cmake

### 6.2 実行時依存

| パッケージ | 用途 | 備考 |
|---|---|---|
| biped_description | URDF、robot_state_publisher | 実装済み |
| biped_msgs | SafetyStatus等カスタムメッセージ | 新規パッケージ |
| biped_teleop | ゲームパッド安全機能（緊急停止・切断検出） | 新規パッケージ |
| teleop_twist_joy | Joy→Twist変換、デッドマンスイッチ | ROS 2標準 |
| biped_rl_policy | RLポリシー推論 | 新規パッケージ |
| biped_safety | 安全監視 | 新規パッケージ |
| robstride_hardware | ros2_control HW Interface | 実装済み、Phase 3のみ |
| joy | ゲームパッドドライバ | ROS 2標準 |
| robot_state_publisher | URDF→TF | ROS 2標準 |
| rviz2 | 3D可視化 | ROS 2標準 |
| ros_gz_sim | Gazebo Harmonicシミュレータ起動 | simモード（Phase 1） |
| ros_gz_bridge | Gazebo↔ROS 2トピックブリッジ | simモード（Phase 1） |
| gz_ros2_control | Gazebo用ros2_controlプラグイン | simモード（Phase 1） |
| controller_manager | ros2_controlコントローラ管理 | simモード + Phase 3 |
| forward_command_controller | 位置指令コントローラ | simモード + Phase 3 |
| joint_state_broadcaster | 関節状態配信 | simモード + Phase 3 |
