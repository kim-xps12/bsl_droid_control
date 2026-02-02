# BSL-Droid Control

BSL-Droid（二脚ロボット）の制御システム

## 概要

Jetson Orin Nano SuperとMacBookの分散環境で動作する二脚ロボット制御システムです。ROS 2によるリアルタイム制御と、Genesis物理シミュレータによる強化学習環境を提供します。

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

- [pixi](https://pixi.sh/) — ROS 2ワークスペース（ros2_ws）用
- [uv](https://docs.astral.sh/uv/) — 強化学習環境（rl_ws）用
- Git

#### uvのインストール

```bash
# macOS / Linux
curl -LsSf https://astral.sh/uv/install.sh | sh

# Windows (PowerShell)
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

### セットアップ

```bash
# リポジトリをクローン（submoduleを含む）
git clone --recursive --shallow-submodules https://github.com/kim-xps12/bsl_droid_control.git
cd bsl_droid_control

# 既にclone済みの場合はsubmoduleを取得
git submodule update --init --depth 1

# ros2_wsディレクトリに移動
cd ros2_ws

# 依存関係をインストール（プラットフォーム自動検出）
pixi install

# ワークスペースをビルド
pixi run colcon build --symlink-install
```

### Git Submoduleについて

このリポジトリは以下のsubmoduleを含んでいます：

| Submodule | パス | 用途 |
|-----------|------|------|
| [Genesis](https://github.com/Genesis-Embodied-AI/Genesis) | `rl_ws/genesis_official/` | 物理シミュレータ（強化学習環境） |
| [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) | `rl_ws/mujoco_menagerie/` | ロボットモデル集（Sim2Sim比較用） |

- **新規clone時**: `--recursive`オプションを付けてcloneしてください
- **既存リポジトリ**: `git submodule update --init`を実行してください
- **高速化**: `--shallow-submodules`または`--depth 1`でsubmoduleの履歴を省略できます

> **Note**: 強化学習環境（rl_ws）を使用しない場合、submoduleの取得は不要です。

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

## 強化学習環境（rl_ws）

歩容獲得のための強化学習環境を `rl_ws/` に用意しています。[Genesis](https://genesis-world.readthedocs.io/)物理シミュレータとPPO実装を使用します。

### セットアップ

```bash
cd rl_ws

# 依存関係をインストール
uv sync

# 動作確認
uv run python -c "import genesis as gs; print(f'Genesis {gs.__version__} loaded')"
```

### トレーニング

#### Go2サンプル（参考用）

```bash
cd rl_ws

# ヘッドレスでトレーニング（デフォルト: 4096環境）
uv run python genesis_official/examples/locomotion/go2_train.py

# 少数環境でテスト
uv run python genesis_official/examples/locomotion/go2_train.py -B 64

# イテレーション数を指定（チェックポイントは100回ごとに保存）
uv run python genesis_official/examples/locomotion/go2_train.py -B 64 --max_iterations 200
```

**注意**: モデルチェックポイントはイテレーション0と100の倍数で保存されます（100, 200, 300...）。
`--max_iterations 100`の場合、`model_0.pt`は生成されますが、100回目の完了前に終了すると`model_100.pt`は生成されません。

### TensorBoard監視

```bash
cd rl_ws
uv run python -m tensorboard.main --logdir logs

# ブラウザで http://localhost:6006 を開く
```

### ポリシー評価

```bash
cd rl_ws

# デフォルト（model_100.ptを読み込み）
uv run python genesis_official/examples/locomotion/go2_eval.py -e go2-walking

# チェックポイント指定
uv run python genesis_official/examples/locomotion/go2_eval.py -e go2-walking --ckpt 0
```

#### BSL-Droid二脚ロボット

二脚ロボット用の強化学習環境と訓練スクリプトを提供しています。

**実験名の規則**:

| ロボット | 実験名形式 | 例 |
|----------|-----------|-----|
| BSL-Droid Simplified | `droid-walking-v{N}` | droid-walking-v19 |

```bash
cd rl_ws

# トレーニング（droid-walking）のv1の例
uv run python biped_walking/train/droid_train_v1.py --max_iterations 500
```

### ポリシー評価

統一評価スクリプト`biped_eval.py`で全バージョンを評価できます。

```bash
cd rl_ws

# GUI付き評価
uv run python biped_walking/biped_eval.py -e droid-walking-v1

# 特定チェックポイントを指定
uv run python biped_walking/biped_eval.py -e droid-walking-v1 --ckpt 400

# ヘッドレス評価
uv run python biped_walking/biped_eval.py -e droid-walking-v1 --no-viewer --duration 10

# MP4録画
uv run python biped_walking/biped_eval.py -e droid-walking-v1 --record output.mp4 --duration 10
```

詳細は [rl_ws/README.md](rl_ws/README.md) を参照してください。

## ディレクトリ構造

```
bsl_droid_control/
├── README.md                 # 本ドキュメント
├── AGENTS.md                 # AI Coding Agent向け指示
├── CLAUDE.md                 # Claude Code向け設定
├── doc/                      # プロジェクト全体の設計資料
│   ├── design/               # システム設計ドキュメント
│   ├── experiments/          # 実験レポート群
│   └── next_nodes_design.md
├── ref/                      # 参考資料・リファレンス実装
│   └── RobStride_Control/    # モーター制御ライブラリ
├── rl_ws/                    # 強化学習環境（Genesis + uv）
│   ├── pyproject.toml        # uv環境設定
│   ├── biped_walking/        # 二脚ロボット環境・訓練スクリプト
│   │   ├── envs/             # 環境定義（biped_env.py, droid_env.py）
│   │   ├── train/            # 訓練スクリプト群
│   │   ├── biped_eval.py     # 統一評価スクリプト（Genesis）
│   │   └── biped_eval_mujoco.py  # MuJoCoでのsim2simによる評価スクリプト
│   ├── assets/               # ロボットモデル（URDF/MJCF）
│   ├── scripts/              # 分析・デバッグスクリプト
│   ├── genesis_official/     # Genesis公式リポジトリ（submodule）
│   ├── mujoco_menagerie/     # MuJoCoモデル集（submodule）
│   └── logs/                 # トレーニングログ・チェックポイント
└── ros2_ws/                  # ROS 2ワークスペース（pixi管理）
    ├── pixi.toml             # pixi環境設定
    └── src/
        ├── biped_description/    # URDFモデル・RViz2可視化・GUI
        ├── biped_gait_control/   # 歩容パターン生成
        ├── robstride_hardware/   # ros2_control用IF（開発中）
        └── pub_sub_*/            # ROS 2チュートリアル
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
- [rl_ws README](rl_ws/README.md) - 強化学習環境の詳細

### 設計資料

- [次期ノード設計](doc/next_nodes_design.md) - state_estimator, safety_monitor等の設計
- [分散システム設計](doc/design/distributed_architecture.md) - Jetson/MacBook間の通信設計
- [統一関節インターフェース設計](doc/design/unified_joint_interface_architecture.md) - 関節制御アーキテクチャ

### 実験レポート

実験レポートは `doc/experiments/` 以下に格納されています。各実験ディレクトリには主レポート（`.md`）と関連する図が含まれます。

## ライセンス

MIT License

## 変更履歴

| 日付 | 変更内容 |
|------|----------|
| 2026-02-01 | README全体を実態に即して更新（ディレクトリ構造、ドキュメントリンク、MuJoCo評価追加） |
| 2026-02-01 | 強化学習環境を整理（biped_walking/配下にenvs, train, 評価スクリプトを集約） |
| 2026-02-01 | droid-walking系実験追加、統一評価スクリプト（biped_eval.py）導入 |
| 2026-01-25 | biped_descriptionにスライダーGUIを統合 |
| 2026-01-23 | biped_gait_controlパッケージ追加（歩容パターン生成）、display_rviz_only.launch.py追加 |
| 2026-01-23 | biped_descriptionパッケージ追加、クロスプラットフォーム対応 |
