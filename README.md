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
| Genesis物理シミュレーション | ❌ | ✅ |
| RLポリシー推論 | ✅ | ✅ |
| ゲームパッド遠隔操作 | ✅ | ✅ |
| 安全監視 | ✅ | ✅ |
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
# rviz2によるロボットモデルの表示のみ（外部から/joint_statesを供給）
# JointStatesがpublishされるまでは表示される形状が不定
pixi run ros2 launch aoba_description display_rviz_only.launch.py

# aobaモデルのシンプルな表示＋スライダー
pixi run ros2 launch aoba_description display.launch.py

# aoba_description: スライダーGUI + 歩行待機姿勢ボタン付き
pixi run ros2 launch aoba_description display_custom.launch.py

# PlotJugglerによるリアルタイム関節角度可視化付き
pixi run ros2 launch aoba_description display_custom.launch.py plot:=true
```

PlotJugglerのレイアウトファイル（`ros2_ws/src/aoba_description/config/plotjuggler_joint_states.xml`）は、監視対象トピックの追加・プロット配置の変更に応じて更新が必要になる。更新手順は以下の通り:

1. `plot:=true` で起動
2. PlotJuggler上で Streaming → ROS 2 Topic Subscriber からトピックを追加（例: `/forward_position_controller/commands`）
3. プロットエリアにドラッグして配置を調整
4. File → Save Layout で同じパスに上書き保存
5. `pixi run build` でinstall先に反映

### 軌道リプレイ（RViz / 実機検証）

事前設計された脚軌道を再生し、RViz2上の3Dモデルと実機の動きが一致するかを確認する。

```bash
# 足軌道（CamberTrajectory + IK）— デフォルト
pixi run ros2 launch biped_gait_control trajectory_replay.launch.py

# 単関節の正弦波振動（ハードウェア個別検証）
pixi run ros2 launch biped_gait_control trajectory_replay.launch.py config_file:=replay_oscillation.yaml

# ウェイポイント補間による再生
pixi run ros2 launch biped_gait_control trajectory_replay.launch.py config_file:=replay_waypoint.yaml

# 実機制御モード（ros2_control経由でモータ駆動）
pixi run ros2 launch biped_gait_control trajectory_replay.launch.py mode:=control
```

### RL歩行テレオペ（Genesis）

RLポリシーによる歩行制御をゲームパッドで操作する。

```bash
pixi run ros2 launch biped_bringup genesis_teleop.launch.py
```

### 実機制御（Jetson専用）

```bash
# 全10関節の ros2_control 起動（デュアルCANバス）
pixi run ros2 launch aoba_hardware bringup.launch.py

# 単一モータテスト（自動テスト → 統計分析 → シャットダウン）
pixi run ros2 launch aoba_hardware single_motor_test.launch.py
pixi run ros2 launch aoba_hardware single_motor_test.launch.py target_position:=1.57

# 複数モータゼロ点テスト（モータ構成を引数で指定）
pixi run ros2 launch aoba_hardware multi_motor_zero_test.launch.py motors:='can1:11,12,13,14,15 can2:21,22,23,24,25'

# CAN レイテンシ計測
pixi run ros2 run aoba_hardware can_latency_test can1 11 1000
pixi run ros2 run aoba_hardware multi_bus_latency_test can1:11,12,13,14,15 can2:21,22,23,24,25
```

### トピックの確認

```bash
# 関節状態を確認
pixi run ros2 topic echo /joint_states

# TFツリーを確認
pixi run ros2 run tf2_tools view_frames
```

## パッケージ一覧

| パッケージ | 説明 |
|-----------|------|
| `biped_bringup` | 起動・設定統合（genesis_teleop） |
| `biped_description` | URDFモデル・RViz2可視化・関節操作GUI |
| `biped_gait_control` | 歩容生成・軌道リプレイ（50Hz関節角度出力） |
| `biped_genesis_sim` | Genesis物理シミュレータブリッジ |
| `biped_msgs` | カスタムメッセージ定義（SafetyStatus / RLPolicyState） |
| `biped_rl_policy` | RLポリシー推論ノード（simモード） |
| `biped_safety` | 安全監視ノード（緊急停止・ゲームパッド切断検知） |
| `aoba_description` | Aobaロボット URDFモデル・RViz2可視化・関節操作GUI |
| `aoba_hardware` | ros2_control用ハードウェアインターフェース（Jetson専用） |

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

# ゲームパッド操縦（F710等）
uv run python biped_walking/biped_eval.py -e droid-walking-v1 --gamepad

# ゲームパッド＋横速度・yaw速度の操縦を有効化
uv run python biped_walking/biped_eval.py -e droid-walking-v1 --gamepad --gamepad-vel-y 0.2 --gamepad-vel-yaw 1.0
```

詳細は [rl_ws/README.md](rl_ws/README.md) を参照してください。

## Claude Code スキル

`.claude/skills/` 配下のスキルのうち、プロジェクト固有でないもの（`drawio-svg`, `svg-logo-designer` 等）は [claude-code-skills](https://github.com/kim-xps12/claude-code-skills) リポジトリで管理しており、シンボリックリンクで参照している。

共有スキルの追加・セットアップ手順は上記リポジトリのREADMEを参照。

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
│   │   ├── biped_eval.py     # 統一評価スクリプト（Genesis、ゲームパッド対応）
│   │   └── biped_eval_mujoco.py  # MuJoCoでのsim2simによる評価スクリプト
│   ├── assets/               # ロボットモデル（URDF/MJCF）
│   ├── scripts/              # 分析・デバッグスクリプト
│   ├── genesis_official/     # Genesis公式リポジトリ（submodule）
│   ├── mujoco_menagerie/     # MuJoCoモデル集（submodule）
│   └── logs/                 # トレーニングログ・チェックポイント
└── ros2_ws/                  # ROS 2ワークスペース（pixi管理）
    ├── pixi.toml             # pixi環境設定
    └── src/
        ├── biped_bringup/           # 起動・設定統合
        ├── biped_description/       # URDFモデル・RViz2可視化・GUI
        ├── biped_gait_control/      # 歩容パターン生成・軌道リプレイ
        ├── biped_genesis_sim/       # Genesis物理シミュレータブリッジ
        ├── biped_msgs/              # カスタムメッセージ定義
        ├── biped_rl_policy/         # RLポリシー推論
        ├── biped_safety/            # 安全監視（緊急停止・ゲームパッド切断検知・将来: 関節・姿勢監視）
        ├── aoba_hardware/      # ros2_control用ハードウェアインターフェース（Jetson専用）
        └── pub_sub_*/               # ROS 2チュートリアル
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

- [biped_gait_control README](ros2_ws/src/biped_gait_control/doc/README.md) - 歩容生成パッケージ概要・技術仕様
- [rl_ws README](rl_ws/README.md) - 強化学習環境の詳細

### 設計ドキュメント

- [ROS 2歩行制御アーキテクチャ](doc/design/ros2_walking/architecture.md) - システム全体設計
- [ROS 2歩行制御要件定義](doc/design/ros2_walking/requirements.md) - 機能・非機能要件
- [ROS 2歩行制御開発計画](doc/design/ros2_walking/system_plan.md) - フェーズ別実装計画


## ライセンス

MIT License

## 変更履歴

| 日付 | 変更内容 |
|------|----------|
| 2026-03-10 | ros2_wsドキュメントを実態に即して更新（新規6パッケージ追加、コマンド二重管理解消、.gitignore整備） |
| 2026-02-01 | README全体を実態に即して更新（ディレクトリ構造、ドキュメントリンク、MuJoCo評価追加） |
| 2026-02-01 | 強化学習環境を整理（biped_walking/配下にenvs, train, 評価スクリプトを集約） |
| 2026-02-01 | droid-walking系実験追加、統一評価スクリプト（biped_eval.py）導入 |
| 2026-01-25 | biped_descriptionにスライダーGUIを統合 |
| 2026-01-23 | biped_gait_controlパッケージ追加（歩容パターン生成）、display_rviz_only.launch.py追加 |
| 2026-01-23 | biped_descriptionパッケージ追加、クロスプラットフォーム対応 |
