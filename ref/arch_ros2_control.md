# ros2_control アーキテクチャ

RobStride RS02アクチュエータ制御にros2_controlフレームワークを採用する設計についてまとめる。

## 概要

ros2_controlはROS 2におけるリアルタイムハードウェア制御の標準フレームワーク。

![Architecture Diagram](arch_ros2_control.drawio)

## Controller Manager

- 単一のRT制御ループを管理（SCHED_FIFO priority 50）
- 200Hzで `read() → update() → write()` サイクルを実行
- ユーザは制御ループ自体を実装しない

## Hardware Interface

ユーザが実装するのは以下のメソッドのみ:

| メソッド | 役割 |
|---------|------|
| `on_init()` | パラメータ読み込み、初期化 |
| `export_state_interfaces()` | 状態（位置、速度など）の公開 |
| `export_command_interfaces()` | コマンド（目標位置など）の公開 |
| `read()` | ハードウェアから状態を読み取り |
| `write()` | ハードウェアへコマンドを送信 |

## 利点

- **標準Controller利用可能**: `joint_trajectory_controller`, `forward_command_controller` 等
- **MoveIt2統合**: モーションプランニングとの連携が容易
- **管理ツール完備**: `ros2 control` CLIでController切り替え、状態監視
- **診断機能**: オーバーラン検出、タイミング統計の自動収集

## RT性能

| 項目 | 値 |
|------|-----|
| 制御周波数 | 200Hz（5ms周期） |
| スケジューラ | SCHED_FIFO |
| 優先度 | 50 |
| メモリ | mlockall()によりページフォルト防止 |

## 参考リンク

- [ros2_control Documentation](https://control.ros.org/)
- [ros2_controllers](https://github.com/ros-controls/ros2_controllers)
- [Writing a Hardware Interface](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_interface.html)

## 実装例

詳細な実装コードは [robstride_hardware_impl.md](robstride_hardware_impl.md) を参照。
