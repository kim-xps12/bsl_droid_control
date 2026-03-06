# BSL-Droid ROS 2 歩行制御システム — アーキテクチャ設計

## 1. 概要

本書はBSL-DroidのROS 2歩行制御システムのアーキテクチャを定義する。Logitech F710rゲームパッドで速度指令を入力し、RL学習済みポリシーが歩容を生成、ros2_controlを介してRS02モータを制御する。

### 1.1 設計目標

- ゲームパッドによるテレオペレーション歩行の実現
- RViz可視化（Phase 1）から実機制御（Phase 3）への段階的移行
- biped_description, robstride_hardwareパッケージとの統合
- 200Hzリアルタイム制御と50HzRLポリシー推論の両立

### 1.2 基盤システムのアーキテクチャ図

以下の図は本歩行制御システムの基盤となる設計を示す。

- [制御層階層構造図](./fig/system_architecture.drawio.svg) — 5層制御階層（High-Level → Gait → State Estimation → RT Control → Hardware）
- [分散システム構成図](../system/fig/distributed_system_architecture.drawio.svg) — Jetson/MacBook間のノード配置とROS 2 DDS通信
- [データフロー図](./fig/data_flow.drawio.svg) — センサ→状態推定→歩容生成→モータ制御のデータフロー
- [開発ワークフロー図](./fig/development_workflow.drawio.svg) — MacBook開発→Git→Jetsonデプロイの手順

## 2. システムアーキテクチャ全体図

![ROS 2 Walking System Architecture](./fig/ros2_walking_architecture.drawio.svg)

### 2.1 パッケージ構成

本システムは実装済み2パッケージ＋新規5パッケージで構成される。

| パッケージ | 種別 | ビルド | 役割 |
|---|---|---|---|
| `biped_description` | 実装済み | ament_cmake | URDF定義、RViz可視化 |
| `robstride_hardware` | 実装済み | ament_cmake | ros2_control HW Interface (Jetson専用) |
| `biped_msgs` | **新規** | ament_cmake | カスタムメッセージ定義 |
| `biped_teleop` | **新規** | ament_python | ゲームパッド安全機能（緊急停止・切断検出）。速度指令変換は teleop_twist_joy を利用 |
| `biped_rl_policy` | **新規** | ament_python | RLポリシー推論 |
| `biped_safety` | **新規** | ament_python | 安全監視 |
| `biped_bringup` | **新規** | ament_cmake | Launch統合・設定ファイル |

### 2.2 ノードグラフ

メインデータフローは以下の経路をたどる:

```
F710r Gamepad → [joy_node] → /joy → [teleop_twist_joy_node] → /cmd_vel → [biped_rl_policy_node] → /fwd_pos_ctrl/commands → [forward_position_controller] → [robstride_hardware] → CAN → RS02 Motors
                                  → /joy → [biped_joy_safety_node] → /emergency_stop
```

フィードバックループ:
```
[robstride_hardware] → [joint_state_broadcaster] → /joint_states → [biped_rl_policy_node] (観測構築)
                                                   /joint_states → [robot_state_publisher] → /tf → [RViz2]
[imu_driver] → /imu/data → [biped_rl_policy_node] (姿勢観測)
```

安全監視（破線 = サブスクライブ）:
```
[biped_safety_node] ← /joint_states, /imu/data, /cmd_vel
                    → /safety_status, /emergency_stop
```

## 3. 主要トピック一覧

| トピック名 | メッセージ型 | 周波数 | Publisher | Subscriber |
|---|---|---|---|---|
| `/joy` | sensor_msgs/Joy | ~100Hz | joy_node | teleop_twist_joy_node, biped_joy_safety_node |
| `/cmd_vel` | geometry_msgs/Twist | ~50Hz | teleop_twist_joy_node | biped_rl_policy_node, biped_safety_node |
| `/joint_states` | sensor_msgs/JointState | 200Hz | joint_state_broadcaster (実機) / biped_rl_policy_node (viz) | biped_rl_policy_node, robot_state_publisher, biped_safety_node |
| `/forward_position_controller/commands` | std_msgs/Float64MultiArray | 50Hz | biped_rl_policy_node | forward_position_controller |
| `/imu/data` | sensor_msgs/Imu | 200Hz | imu_driver | biped_rl_policy_node, biped_safety_node |
| `/safety_status` | biped_msgs/SafetyStatus | 200Hz | biped_safety_node | (モニタリング) |
| `/emergency_stop` | std_msgs/Bool | イベント | biped_safety_node, biped_joy_safety_node | 全ノード |
| `/rl_policy_state` | biped_msgs/RLPolicyState | 50Hz | biped_rl_policy_node | (デバッグ) |
| `/tf` | tf2_msgs/TFMessage | 200Hz | robot_state_publisher | RViz2 |

## 4. 主要な設計判断

### 4.1 joy_node + teleop_twist_joy（ROS 2標準）を使用

pygameや自前のteleop実装ではなく、ROS 2標準の`joy_node` + `teleop_twist_joy`を使用する。理由:
- デバイスhotplug対応
- ROS 2エコシステムとの統合（rqt, rosbag等でJoyメッセージを記録・再生可能）
- QoS設定やDDSを通じた分散動作が容易
- `teleop_twist_joy`はデッドマンスイッチ・軸マッピング・速度スケーリングをパラメータのみで設定可能であり、自前実装の保守コストを排除できる
- 緊急停止・切断検出は`biped_joy_safety_node`（biped_teleopパッケージ）が分担する

### 4.2 RLポリシーはPython (rclpy)

PyTorchモデル（MLP 512-256-128）の50Hz推論はPythonで十分にレイテンシ要件（<10ms）を満たす。libtorchへの移植はプロファイリング結果を見てから判断する。

### 4.3 ForwardCommandController利用

RL出力 `action * 0.25 + default_dof_pos` を直接ForwardCommandControllerに送信する。ハードウェアPD（kp=35, kd=2）が200Hzで位置追従を行う。JointTrajectoryControllerではなくForwardCommandControllerを選択した理由:
- RLポリシーが50Hz毎に目標位置を更新するため、軌道補間は不要
- ハードウェアPDが200Hzで補間を行うため十分な滑らかさを確保

### 4.4 unitree_rl_gymと同様の50次元観測ベクトル採用

`droid_env_unitree.py:714-730`の観測ベクトル構築を完全に再現する。シミュレータとの不一致はポリシー性能劣化に直結するため、ユニットテストでシミュレータ記録データとの照合を行う。

### 4.5 viz/controlモード切替

- **vizモード**（Phase 1）: biped_rl_policy_nodeが`/joint_states`をpublish → robot_state_publisher → RViz2で可視化。ros2_controlは不要。MacBookのみで動作。
- **controlモード**（Phase 3）: biped_rl_policy_nodeが`/forward_position_controller/commands`をpublish → ros2_control → robstride_hardware → CAN → モータ。Jetsonで動作。

この切替はLaunch引数（`mode:=viz|control`）で制御する。統一関節インターフェース設計のRouter設計とも整合する。

## 5. 制御階層との対応

本歩行制御システムは[制御層階層構造](./fig/system_architecture.drawio.svg)の以下のレイヤーに対応する:

| 制御層 | 周波数 | 本システムのノード | 備考 |
|---|---|---|---|
| High-Level Control | 10-50Hz | （Phase 3以降） | ゲームパッドが代替 |
| Gait Generation | 50Hz | biped_rl_policy_node | RL推論による歩容生成 |
| State Estimation | 200Hz | （Phase 3で実装） | vizモードではモック |
| RT Control & Safety | 200Hz | biped_safety_node, forward_position_controller, robstride_hardware | 安全監視＋モータ制御 |
| Hardware | - | RS02 x10, IMU | CAN bus, I2C/SPI |

## 6. 分散構成との対応

分散システムアーキテクチャに基づき、ノードを以下のように配置する:

| ノード | vizモード（MacBook） | controlモード（Jetson） |
|---|---|---|
| joy_node | MacBook | Jetson（USBゲームパッド接続先） |
| teleop_twist_joy_node | MacBook | Jetson |
| biped_joy_safety_node | MacBook | Jetson |
| biped_rl_policy_node | MacBook | Jetson |
| biped_safety_node | - | Jetson |
| forward_position_controller | - | Jetson |
| robstride_hardware | - | Jetson |
| joint_state_broadcaster | - | Jetson |
| imu_driver | - | Jetson |
| robot_state_publisher | MacBook | 両方 |
| RViz2 | MacBook | MacBook（DDS経由） |

## 7. 段階的実装

| Phase | Session | 内容 | 動作環境 |
|---|---|---|---|
| Phase 0 | Session B | パッケージ骨格作成・ビルド確認 | MacBook |
| Phase 1 | Session C | ゲームパッド→RViz可視化（vizモード） | MacBook |
| Phase 2 | Session D | 安全監視・設定整備・テスト | MacBook |
| Phase 3 | Session E | 実機展開（IMU統合・controlモード） | Jetson + MacBook |

詳細はシステム構築計画を参照。

## 8. 参考文献

- [MIT Cheetah Software](https://github.com/mit-biomimetics/Cheetah-Software) — 四脚ロボット制御ソフトウェア
- [Unitree ROS](https://github.com/unitreerobotics/unitree_ros) — Unitree四脚ロボットROS統合
- [Isaac Gym Legged Robots](https://github.com/leggedrobotics/legged_gym) — RL歩行学習環境
- [ROS 2 Control](https://control.ros.org/) — ros2_control公式ドキュメント
- [robot_localization](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html) — EKF/UKFベース状態推定
