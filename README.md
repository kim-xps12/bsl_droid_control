# BSL Droid ROS 2

BSL-DroidのROS 2制御システム

## 概要

Jetson Orin Nano SuperとMacBookの分散環境で動作する二脚ロボット制御システムです。

### システム構成

| ホスト | 役割 | プラットフォーム |
|--------|------|------------------|
| Jetson Orin Nano Super | 実機のリアルタイム制御 | linux-aarch64 |
| MacBook | 各種可視化・机上開発 | osx-arm64 |

### ロボット仕様

- **構造**: 逆関節（鳥脚型）二脚ロボット
- **自由度**: 10 DOF（片脚5関節 × 2脚）
- **アクチュエータ**: RobStride RS02（CAN通信）

## 環境構築

### 前提条件
以下のツールが導入済みであること

- [pixi](https://pixi.sh/)
- Git

### セットアップ

```bash
# リポジトリをクローン
git clone <repository-url>
cd bsl_droid_ros2

# ros2_wsディレクトリに移動
cd ros2_ws

# 依存関係をインストール（プラットフォーム自動検出）
pixi install

# ワークスペースをビルド
pixi run colcon build --symlink-install
```

### pixi環境について

本プロジェクトは [pixi](https://pixi.sh/) を使用してROS 2環境を管理しています。
`ros2_ws/pixi.toml` でプラットフォーム別の依存関係を定義：

```toml
[workspace]
platforms = ["linux-aarch64", "osx-arm64"]

# 共通依存関係（両プラットフォームで利用可能）
[dependencies]
ros-jazzy-desktop = ">=0.11.0,<0.12"
ros-jazzy-xacro = "*"
ros-jazzy-robot-state-publisher = "*"
pyqt = ">=5.15"
# ...

# Jetson専用依存関係（ros2_control関連）
[target.linux-aarch64.dependencies]
ros-jazzy-ros2-control = "*"
ros-jazzy-controller-manager = "*"
# ...
```

#### プラットフォーム別機能

| 機能 | Jetson | MacBook |
|------|--------|---------|
| URDF可視化 | ✅ | ✅ |
| 関節操作GUI | ✅ | ✅ |
| ros2_control | ✅ | ❌ |
| 実機制御 | ✅ | ❌ |

> ⚠️ **注意**: ros2_controlはRoboStack JazzyのmacOS ARM64ビルドが未提供のため、MacBookでは利用不可

## クイックスタート

本節のコマンドは全てカレントディレクトリを`ros2_ws`へ移してから行う．

```bash
# ros2_wsへ移動
cd ros2_ws
```

### RViz2でロボットモデルを可視化

```bash
# rviz2によるロボットモデルの表示のみ
# JointStatesがpublishされるまでは表示される形状が不定
pixi run ros2 launch biped_description display_rviz_only.launch.py

# スライダーによるJointStatesのpublisherも併せて起動
pixi run ros2 launch biped_description display_custom.launch.py
```

### 歩容サンプル実装の表示

```bash
# 歩容生成 + RViz可視化（推奨・1コマンドで完結）
pixi run ros2 launch biped_gait_control gait_visualization.launch.py

# または以下のように分割して起動もできる
# ターミナル1: RViz2
pixi run ros2 launch biped_description display_rviz_only.launch.py

# ターミナル2: 歩容生成ノード
pixi run ros2 launch biped_gait_control gait_control.launch.py
```

パラメータを変えて起動する場合は以下．
```bash
# パラメータをカスタマイズ
pixi run ros2 launch biped_gait_control gait_visualization.launch.py \
    step_frequency:=1.0 \
    step_height:=0.05
```
> **注意**: `display.launch.py` は `joint_state_publisher_gui` を起動するため、
> 外部からの `/joint_states` と競合します。外部ソースを使用する場合は
> `display_rviz_only.launch.py` を使用してください。

### トピックの確認

```bash
# 関節状態を確認
pixi run ros2 topic echo /joint_states

# TFツリーを確認
pixi run ros2 run tf2_tools view_frames
```

## パッケージ一覧

| パッケージ | 説明 | 状態 |
|-----------|------|------|
| `biped_description` | URDFモデル・RViz2可視化・関節操作GUI | ✅ 完成 |
| `biped_gait_control` | 歩容生成のサンプル実装（50Hz関節角度出力） | ✅ 完成 |
| `robstride_hardware` | ros2_control用ハードウェアインターフェース | 🔄 開発中 |
| `pub_sub_cpp` | ROS 2チュートリアル（C++） | 📚 サンプル |
| `pub_sub_python` | ROS 2チュートリアル（Python） | 📚 サンプル |

## ディレクトリ構造

```
bsl_droid_ros2/
├── README.md                 # 本ドキュメント
├── doc/                      # プロジェクト全体の設計資料
│   ├── next_nodes_design.md
│   ├── distributed_architecture.md
│   └── *.drawio
├── ref/                      # 参考資料・リファレンス実装
│   └── RobStride_Control/    # モーター制御ライブラリ
└── ros2_ws/                  # ROS 2ワークスペース
    ├── pixi.toml             # pixi環境設定
    └── src/
        ├── biped_description/    # ロボットモデル・URDF
        ├── biped_gait_control/   # 歩容パターン生成
        ├── robstride_hardware/   # ハードウェアIF
        └── ...
```

## 既知の問題

### macOSでのthread affinity警告

```
Problem to set affinity of thread with id [...] to value 0. 
Error 'Protocol family not supported'
```

macOSではDDSのスレッドアフィニティ設定がサポートされていないため表示されますが、**動作には影響しません**。

### KDL root link inertia警告

```
[WARN] The root link base_link has an inertia specified in the URDF...
```

KDLライブラリの制限による警告で、**可視化には影響しません**。

## ドキュメント

### パッケージドキュメント

- [biped_description技術仕様](ros2_ws/src/biped_description/doc/technical_specification.md) - ロボットモデル詳細
- [biped_gait_control README](ros2_ws/src/biped_gait_control/doc/README.md) - 歩容生成パッケージ概要
- [biped_gait_control技術仕様](ros2_ws/src/biped_gait_control/doc/technical_specification.md) - 運動学・軌道生成詳細

### 設計資料

- [次期ノード設計](doc/next_nodes_design.md) - state_estimator, safety_monitor等の設計
- [分散システム設計](doc/distributed_architecture.md) - Jetson/MacBook間の通信設計

## ライセンス

MIT License

## 変更履歴

| 日付 | 変更内容 |
|------|----------|
| 2026-01-23 | biped_gait_controlパッケージ追加（歩容パターン生成）、display_rviz_only.launch.py追加 |
| 2026-01-23 | biped_descriptionパッケージ追加、クロスプラットフォーム対応 |
