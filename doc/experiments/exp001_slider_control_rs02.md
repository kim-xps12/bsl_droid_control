# EXP-001: RS02実機スライダー制御実験

## 実験概要

| 項目 | 内容 |
|------|------|
| 実験ID | EXP-001 |
| 目的 | スライダーGUIでRS02モーターを実機制御し、ros2_control統合の動作検証を行う |
| 対象ハードウェア | RobStride RS02 (Motor ID: 127) |
| 実行環境 | Jetson Orin Nano Super (ros2_control) + MacBook (GUI) |
| 通信 | SocketCAN (can1, 1Mbps) + ROS 2 DDS (WiFi/Ethernet) |

## 背景

歩容生成ノードの開発とRViz可視化が完了し、次のステップとして実機モーターの制御実験を行う。まず単関節での動作確認を行い、その後多関節制御へ拡張する。

**分散構成の理由**: MacBookでGUI操作を行い、Jetsonで実機制御を実行することで、開発効率とデバッグ性を向上させる。

## システムアーキテクチャ（分散構成）

![dio.svg](./exp001_slider_control_architecture.drawio.svg)

### アーキテクチャ設計の重要ポイント

**なぜforward_position_controllerはJetson側に配置するのか？**

ros2_controlの設計上、以下の理由でコントローラーはHardware Interfaceと**同一プロセス・同一マシン**に配置する必要があります：

1. **Controller Managerの統合アーキテクチャ**
   - `ros2_control_node`が Controller Manager と Hardware Interface を同一プロセスで管理
   - リアルタイムループ（200Hz）で `read()` → `update()` → `write()` を実行

2. **command_interfaceは共有メモリ**
   - コントローラーとHardware Interface間のデータ受け渡しはメモリ上のポインタ
   - ネットワーク越しではアクセス不可（ゼロコピー設計）

3. **リアルタイム性の保証**
   - ネットワーク遅延を挟むとリアルタイム制御が不可能

**結論**: 
- ✅ **MacBook**: Slider GUI（ROSトピックでpublish）
- ✅ **Jetson**: ros2_control全体（Controller Manager + forward_position_controller + RobStrideHardware）

## 制御フロー

![](./exp001_slider_control_sequence.drawio.svg)

### 送信側（コマンド）
1. スライダーGUI: ユーザー操作で目標角度を設定
2. `Float64MultiArray`として `/forward_position_controller/commands` へ50Hz publish
3. `forward_position_controller`: command_interfaceへ書き込み
4. `RobStrideHardware::write()`: MIT Modeフレーム生成、CAN送信

### 受信側（状態）
1. `RobStrideHardware::read()`: CAN受信、位置/速度/トルク取得
2. state_interfaceへ書き込み
3. `joint_state_broadcaster`: `/joint_states`へpublish
4. `robot_state_publisher`: TF計算、配信
5. RViz2: 可視化

## 実装ファイル

| ファイル | 役割 |
|---------|------|
| `biped_teleop/biped_teleop/joint_slider_gui.py` | PyQt5スライダーGUI |
| `biped_teleop/launch/slider_control.launch.py` | 統合launchファイル |
| `robstride_hardware/urdf/robstride_system.urdf.xacro` | ros2_control設定 |
| `robstride_hardware/config/controllers.yaml` | コントローラー設定 |

## パラメータ

### スライダーGUI
| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `topic` | `/forward_position_controller/commands` | Publishトピック |
| `min_deg` | -90.0 | 最小角度 [deg] |
| `max_deg` | 90.0 | 最大角度 [deg] |
| `publish_rate` | 50.0 | Publish周波数 [Hz] |

### MIT Mode制御ゲイン
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `kp` | 1.5 | 位置ゲイン [Nm/rad] |
| `kd` | 0.01 | ダンピングゲイン [Nm/(rad/s)] |

## 実験手順

### 【Jetson側】事前準備
```bash
# 1. CANインターフェース確認
ip link show can1

# 2. CAN有効化（必要な場合）
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

# 3. ROS_DOMAIN_ID設定（MacBookと合わせる）
export ROS_DOMAIN_ID=0  # 任意の値（0-101）
```

### 【Jetson側】ros2_control起動
```bash
cd ~/Projects/bsl_droid_ros2/ros2_ws

# 4. ビルド
pixi run colcon build --packages-select robstride_hardware

# 5. ros2_control起動（GUIなし）
pixi run ros2 launch robstride_hardware bringup.launch.py
```

### 【MacBook側】スライダーGUI起動
```bash
# 1. ROS_DOMAIN_ID設定（Jetsonと合わせる）
export ROS_DOMAIN_ID=0

# 2. ワークスペース移動
cd ~/Projects/bsl_droid_ros2/ros2_ws

# 3. ビルド（初回のみ）
pixi run colcon build --packages-select biped_teleop

# 4. スライダーGUI起動
pixi run ros2 run biped_teleop joint_slider_gui

# または、角度範囲を指定して起動
pixi run ros2 run biped_teleop joint_slider_gui \
    --ros-args \
    -p min_deg:=-30.0 \
    -p max_deg:=30.0
```

### 【MacBook側】動作確認
```bash
# トピック監視（Jetson側のコントローラーに届いているか確認）
pixi run ros2 topic echo /forward_position_controller/commands

# モーター状態監視（Jetson側からのフィードバック）
pixi run ros2 topic echo /joint_states

# コントローラー状態確認（Jetson側を指定）
pixi run ros2 control list_controllers

# ノード一覧確認（両マシンのノードが見えるか）
pixi run ros2 node list
```

### ネットワーク疎通確認

**接続が確立しない場合のトラブルシュート:**

```bash
# 【両マシンで実行】ROS 2 Discovery確認
pixi run ros2 topic list

# 【MacBookで実行】Jetsonのトピックが見えるか
pixi run ros2 topic info /joint_states

# 【Jetsonで実行】MacBookからのコマンドが届くか
pixi run ros2 topic hz /forward_position_controller/commands

# multicast疎通確認（WiFi環境で問題が起きやすい）
# Jetson IP: 192.168.1.100, MacBook IP: 192.168.1.200 と仮定
ping 192.168.1.100  # MacBook → Jetson
ping 192.168.1.200  # Jetson → MacBook
```

## 安全対策

| リスク | 対策 |
|--------|------|
| 過大な角度指令 | スライダー範囲制限（±90°）、初回は±30°で |
| 急激な動作 | kp/kdを小さめに設定 |
| 暴走 | GUI終了時に0°復帰処理を実装 |
| 物理的干渉 | モーターを安全に固定してから実験 |
| ネットワーク切断 | Jetson側でタイムアウト処理（コマンド未受信で停止） |
| 複数マシンからの誤操作 | ROS_DOMAIN_IDで環境を分離 |

## 分散構成の利点と注意点

### 利点
- ✅ **開発効率**: MacBookの快適な環境でGUI操作・デバッグ
- ✅ **安全性**: 物理的に離れた場所から制御可能
- ✅ **可視化**: MacBookでRViz2を同時起動して状態監視
- ✅ **柔軟性**: 複数のGUIやプランナーを同時接続可能

### 注意点
- ⚠️ **ネットワーク遅延**: WiFi環境では50ms程度の遅延が発生しうる
- ⚠️ **Discovery問題**: multicast対応ルーターが必要（企業WiFiでは制限あり）
- ⚠️ **帯域**: `/joint_states`の配信レート（50Hz）がネットワーク負荷になる可能性
- ⚠️ **セキュリティ**: ROS_DOMAIN_IDのみでは不十分、VPN推奨

## 期待される結果

### MacBook側（GUI）
1. スライダー操作で `/forward_position_controller/commands` がpublishされる
2. `/joint_states` がJetsonから受信され、実機の角度が確認できる
3. RViz2を起動すれば、モデルが実機と同期して動く

### Jetson側（実機制御）
1. MacBookからのコマンドを受信し、RS02が追従動作する
2. CAN通信でモーター状態を取得し、`/joint_states`をpublish
3. ターミナルにros2_control_nodeのログが表示される

### 期待される通信遅延
- **コマンド遅延**: MacBook → Jetson: 10-50ms（WiFi環境）
- **状態フィードバック**: Jetson → MacBook: 10-50ms
- **往復遅延**: 合計 20-100ms（50Hzでは許容範囲）

## 次のステップ

- [ ] 単関節動作確認
- [ ] kp/kdゲイン調整
- [ ] 多関節対応（biped_descriptionのURDFと統合）
- [ ] 歩容生成ノードとの接続

## 関連ドキュメント

- [doc/distributed_architecture.md](../doc/distributed_architecture.md) - 分散システム設計
- [doc/next_nodes_design.md](../doc/next_nodes_design.md) - ノード設計
- [robstride_hardware/doc/](../ros2_ws/src/robstride_hardware/doc/) - ハードウェアIF詳細
