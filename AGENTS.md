# AGENTS.md — bsl_droid_control

## 目的
- このリポジトリは「BSL-Droid」の制御に関するソフトウェア全般を扱う
- 動作環境はJetson Orin Nano Super（実機・RT制御）とMacBook（可視化・机上開発）の分散構成を前提としている


## プラットフォーム別の注意事項
- Jetson Orin Nano Super(linux-aarch64): ros2_control利用可
- M4 MacBook Air(osx-arm64): ros2_control不可（RoboStack JazzyのmacOS ARM64ビルド未提供）

## 作業ディレクトリ

このリポジトリでは各環境のworkspaceへ`cd`してからコマンドを実行する必要がある．

- ROS 2環境: `cd ros2_ws` → `pixi run ...`
- 強化学習環境: `cd rl_ws` → `uv run ...`

ルール: `cd` と後続コマンドを `&&` で連結してはならない。`settings.json` の allow パターンはコマンド先頭からマッチするため、`cd ... && cmd` 形式ではパターンに一致せず許可ダイアログが発生する。`cd` と後続コマンドは別々の Bash ツール呼び出しに分けること（作業ディレクトリは Bash 呼び出し間で保持される）。

## ROS 2環境（ros2_ws）

### 開発環境の仮想化

pixiでROS 2環境を管理している．依存関係の解消およびビルドコマンドは以下の通り．

```bash
cd ros2_ws
pixi install
pixi run build
```

> **注意**: `pixi run build` は `pixi.toml` の `[tasks]` で定義されたビルドコマンドを実行する。`--cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3` はタスク定義に含まれているため、手動で指定する必要はない。pyenvなど他のPython環境がある場合でもpixi環境のPythonが自動的に使用される。

### よく使う起動コマンド

```bash
cd ros2_ws

# URDF可視化（カスタムGUI付き）
pixi run ros2 launch biped_description display_custom.launch.py
pixi run ros2 launch aoba_description display_custom.launch.py
pixi run ros2 launch aoba_description display_custom.launch.py plot:=true  # PlotJuggler付き

# 外部から /joint_states を供給する場合（競合回避）
pixi run ros2 launch biped_description display_rviz_only.launch.py
pixi run ros2 launch aoba_description display_rviz_only.launch.py

# 軌道リプレイ（足軌道 / 単関節振動 / ウェイポイント補間）
pixi run ros2 launch biped_gait_control trajectory_replay.launch.py
pixi run ros2 launch biped_gait_control trajectory_replay.launch.py config_file:=replay_oscillation.yaml
pixi run ros2 launch biped_gait_control trajectory_replay.launch.py config_file:=replay_waypoint.yaml

# RL歩行テレオペ（Genesis物理シミュレーション + ゲームパッド）
pixi run ros2 launch biped_bringup genesis_teleop.launch.py

# 実機制御（Jetson専用）
pixi run ros2 launch aoba_hardware bringup.launch.py
```

### 注意事項

- `display.launch.py` は `joint_state_publisher_gui` を起動するため、外部から`/joint_states` を与える時競合する
- `display_rviz_only.launch.py`を常用するのが好ましい
- 生成物（例: `frames_*.gv`）はワークスペース直下に出ることがある

## 強化学習環境（rl_ws）

強化学習ワークスペース`rl_ws/`はuvで管理されている．ROS 2環境（ros2_ws）とは独立した仮想環境である．

### 開発環境の仮想化

依存関係の解消コマンドは以下の通り．

```bash
cd rl_ws
uv sync
```

### コマンド実行形式

ルール: `rl_ws`ディレクトリに移動してから`uv run`で実行すること．

```bash
# 正しい形式
cd rl_ws
uv run python biped_walking/train/droid_train_v{N}.py --max_iterations 500
uv run python biped_walking/biped_eval.py -e droid-walking-v{N}

# 誤った形式（動作しない）
uv run python rl_ws/biped_walking/...  # rl_wsの外から実行不可
python biped_walking/...               # uvなしでは依存関係が解決されない
```

### 主要スクリプト

| 用途 | コマンド |
|------|---------|
| トレーニング（droid） | `uv run python biped_walking/train/droid_train_v{N}.py --max_iterations 500` |
| 評価（GUI付き） | `uv run python biped_walking/biped_eval.py -e {実験名}` |
| 評価（ヘッドレス） | `uv run python biped_walking/biped_eval.py -e {実験名} --no-viewer --duration 10` |
| 評価（ゲームパッド） | `uv run python biped_walking/biped_eval.py -e {実験名} --gamepad` |
| TensorBoard | `uv run python -m tensorboard.main --logdir logs/` |

### 実験名の規則

| ロボット | 形式 | 例 |
|----------|------|-----|
| biped_digitigrade | `biped-walking-v{N}` | biped-walking-v22 |
| BSL-Droid Simplified | `droid-walking-v{N}` | droid-walking-v19 |
| BSL-Droid Omni | `droid-walking-omni-v{N}` | droid-walking-omni-v1 |

評価スクリプトは`-e`オプションで実験名を指定することで、任意のバージョンを評価できる統一インターフェースとなっている．

### 注意事項

- **カレントディレクトリ**: 必ず`rl_ws`に`cd`してから実行
- **パス指定**: スクリプトパスは`rl_ws`からの相対パスで指定
- **ログ出力**: チェックポイントは`rl_ws/logs/{実験名}/`に保存される
- **設定ファイル**: 各実験の設定は`rl_ws/logs/{実験名}/cfgs.pkl`に保存される
- **実験ルール**: 実験に関連する作業は対応する`doc/experiments/exp{NNN}/exp{NNN}_rules.md`を参照

## 実際に実行する動作確認
- GUIを伴う実装の有無によらず、コーディング完了後は実際に対象を起動し、エラーなく終了することを必ず確認する
- 各仮想環境のworkspaceへ`cd`して実行する必要があることを念頭におく

## トラブルシュート
- macOSでのthread affinity警告は無視可
- KDL root link inertia警告は可視化に影響なし

## ビルド警告の取り扱い【必須】

Coding Agentはビルド時に発生する**すべての警告**に対処する義務がある．「ツールチェーン起因」「実害なし」等の理由で警告を無視することは許されない．

### 原則

1. **プロジェクト側で修正可能な警告**: 直ちにソースコードや設定ファイルを修正して解消すること
2. **ツールチェーン・外部依存起因で直接修正不可能な警告**: CMakeやビルドスクリプトの設定で警告を抑制する対策を講じること（例: `set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS ON)`, 環境変数の設定等）
3. **対策が不明な場合**: 調査を行い、対策案をユーザに提示すること．「無視してよい」という結論は出さない

### 具体的な既知ケース

| 警告 | 対処 |
|------|------|
| `cmake_minimum_required` deprecation | `VERSION 3.14` 以上を指定 |
| `install_name_tool: warning: changes being made to the file will invalidate the code signature` | macOS + conda-forge環境固有．ビルドツールチェーン起因で直接修正不可だが、`MACOSX_RPATH`設定やcodesign再署名スクリプトの導入を検討すること |

## コード品質ルール【必須】

このリポジトリではPythonおよびC++コードの品質を担保するため、以下のツールを使用する。Coding Agentは必ずこれらのルールに従うこと。

### Python 自動チェック（Claude Code Hooks）

`.claude/settings.json`で設定されたhooksにより、Edit/Writeツール実行後に自動でruffとmypyが実行される。Coding Agentはフィードバックされたエラーを修正すること。

### Python 手動チェックコマンド

```bash
cd rl_ws

# Ruff: lintチェック
uv run ruff check biped_walking/ scripts/ assets/export_urdf.py check_training_logs.py

# Ruff: 自動修正
uv run ruff check --fix biped_walking/ scripts/ assets/export_urdf.py check_training_logs.py

# Ruff: フォーマット
uv run ruff format biped_walking/ scripts/ assets/export_urdf.py check_training_logs.py

# Mypy: 型チェック
uv run mypy biped_walking/ scripts/ assets/export_urdf.py check_training_logs.py
```

### Python コーディング規約

| 項目 | ルール |
|------|--------|
| スタイル | PEP-8準拠 |
| 行長 | 120文字以内 |
| import順序 | isortルールに従う（標準ライブラリ → サードパーティ → ローカル） |
| 型ヒント | 新規関数には可能な限り型ヒントを付与 |
| 命名規則 | PEP-8に準拠（ただし数式変数として大文字行列名は許容） |

### 許容される例外

以下のケースは警告を無視してよい:
- `E741`: 数式で使用する`l`, `O`, `I`などの変数名
- `N806`: 行列を表す大文字変数名（例: `R`, `J`, `M`）
- `B008`: Pydantic等でのdefault引数での関数呼び出し

### Python 設定ファイル

- `rl_ws/pyproject.toml`: ruff, mypyの設定
- `.claude/settings.json`: Claude Code hooksの設定
- `.claude/hooks/lint-check.sh`: Python自動チェックスクリプト

### C++ 自動チェック（Claude Code Hooks）

`.claude/settings.json`で設定されたhooksにより、`ros2_ws/src/aoba_hardware/` 以下のC++ファイル（`.cpp`, `.hpp`）のEdit/Write後に自動でclang-formatとcpplintが実行される。Coding Agentはフィードバックされたエラーを修正すること。

### C++ 手動チェックコマンド

```bash
cd ros2_ws

# clang-format: フォーマット適用
pixi run format-cpp

# clang-format: フォーマットチェック（差分表示のみ）
pixi run check-format-cpp

# cpplint: スタイルチェック
pixi run lint-cpp
```

### C++ コーディング規約

| 項目 | ルール |
|------|--------|
| スタイル | Google C++ Style Guide準拠（ROS 2カスタマイズ） |
| 行長 | 100文字以内 |
| インデント | 2スペース |
| ヘッダガード | `#pragma once` を使用 |
| include順序 | 自ヘッダ → C system → C++ system → 外部ライブラリ → プロジェクト |
| 命名規則 | snake_case（関数・変数）、PascalCase（クラス・構造体） |
| ライセンスヘッダ | 全ファイル先頭に `// Copyright (c) 2024-2025, Yutaro KIMURA (B-SKY Lab)` + SPDX |

### C++ 設定ファイル

- `ros2_ws/src/aoba_hardware/.clang-format`: clang-formatの設定
- `ros2_ws/src/aoba_hardware/.clang-tidy`: clang-tidyの設定（Jetson上での手動実行用）
- `.claude/hooks/cpp-lint-check.sh`: C++自動チェックスクリプト

## ドキュメント類の作成ルール
- 文書は`markdown`形式のドキュメントとして作成すること
- 必ず日本語で執筆し，N1相当の流暢で自然な日本語を用いること．
- 図は`drawio.svg`形式のファイル名に`drawio`の規格に準じたxmlタグを格納することで，独立して作成すること．
- 図をmarkdownドキュメント内にアスキーアートで描くことは行なってはいけない．二重管理の状態を避けるために厳守する．
- ドキュメント（README.md, 設計書等）にパッケージや機能の開発ステータス（「開発中」「完成」等）を記載しない．ステータスはソースコード自体が示すものであり，ドキュメント上のステータス表記は陳腐化して誤解を招くリスクがメンテナンスコストに見合わない．

### 実験レポートの構成ルール[厳守]

`doc/experiments/` 以下に実験レポートを作成する際は、以下のルールに従うこと。

#### ディレクトリ構造

```
doc/experiments/
├── README.md                          # 実験一覧インデックス（任意）
├── roadmap_unified_interface.md       # 横断的ドキュメント（直下に残す）
├── exp001_slider_control_rs02/
│   ├── exp001_slider_control_rs02.md              # 主レポート（必須）
│   ├── exp001_slider_control_rs02_architecture.drawio.svg
│   └── exp001_slider_control_rs02_sequence.drawio.svg
├── exp002_biped_sim2sim/
│   └── exp002_biped_sim2sim.md
└── ...
```

#### 命名規則

| 要素 | 形式 | 例 |
|------|------|-----|
| ディレクトリ名 | `exp{NNN}_{実験名}` | `exp001_slider_control_rs02` |
| 主レポート | `exp{NNN}_{実験名}.md` | `exp001_slider_control_rs02.md` |
| 補足ファイル | `exp{NNN}_{実験名}_{補足}.{拡張子}` | `exp001_slider_control_rs02_architecture.drawio.svg` |

- `{NNN}`: 3桁の通し番号（001, 002, ...）
- `{実験名}`: snake_caseで簡潔に（長すぎる場合は略称を使用）
- `{補足}`: 図や画像の内容を示すsuffix（`_architecture`, `_sequence`, `_screenshot_01` 等）

#### ルールの意図

1. **自己完結性**: ファイル単体で実験番号と内容が判別できる（移動・共有に強い）
2. **検索性**: `exp002`で検索すれば関連ファイルがすべてヒット
3. **整理性**: 各実験の関連ファイルがディレクトリ内にカプセル化される

#### 補足ファイルの扱い

- 補足ファイル（図・画像等）は必ず主レポート（`.md`）から参照すること
- 参照されていない孤立ファイルは作成しない
- 参照例: `![システムアーキテクチャ](./exp001_slider_control_rs02_architecture.drawio.svg)`

#### 実験ルールファイル（rules.md）【重要】

各実験ディレクトリには `exp{NNN}_rules.md` を作成し、その実験固有の手順・コマンド・ルールを記載する。以下は例である．

```
doc/experiments/exp007_droid_rl_walking_ref_unitree/
├── exp007_droid_rl_walking_ref_unitree.md  # 実験概要・インデックス（主レポート）
├── exp007_report_v1.md                     # V1の設計・実装・結果
├── exp007_report_v2.md                     # V2の設計・実装・結果
├── exp007_report_v3.md                     # V3の設計・実装・結果（以降同様に追加）
├── exp007_rules.md                         # 実験ルール・手順・コマンド・レポートテンプレート
└── exp007_unitree_rl_gym_survey.md         # 調査結果（任意）
```

**Coding Agentへの義務**:

実験に関連する作業（コード修正、トレーニング実行、評価、デバッグ等）を行う際は、**必ず対応する `exp{NNN}_rules.md` を最初に読み込み、記載されたルールに従うこと**。

```
# 例: exp004に関連する作業を行う場合
1. doc/experiments/exp004_droid_rl_walking/exp004_rules.md を読む
2. 記載されたバージョン管理原則、コマンド、トラブルシューティングを確認
3. ルールに従って作業を実行
```

**rules.mdに記載すべき内容**:
- 実験目的に基づく振る舞いの原則
- 実験目的に基づく考察の基本軸や方針
- バージョン管理原則（コピー原則、最小変更原則等）
- 主要コマンド（トレーニング、評価、分析等）
- スクリプトの使い方
- トラブルシューティング
- 実験固有の注意事項

#### 実験記録の責務分離【重要】

実験ディレクトリ内のドキュメントは、以下のように責務を明確に分離すること:

| ファイル | 責務 | 記載する内容 |
|----------|------|--------------|
| `exp{NNN}_{実験名}.md` | 実験概要・インデックス | 実験の背景・目的・バージョン一覧へのリンク。詳細な実験結果は含めない |
| `exp{NNN}_report_v{M}.md` | バージョン別実験結果 | 各バージョンの設計・実装・結果・考察を自己完結的に記載（**1バージョン = 1ファイル**） |
| `exp{NNN}_rules.md` | 実験ルール・手順 | コマンド、スクリプト使用法、バージョン管理原則、レポートテンプレート、トラブルシューティング |
| `exp{NNN}_{調査名}_survey.md` | 先行研究調査 | 関連事例、研究、先行事例、論文などの調査結果**のみ**を記載。実験結果やそれに基づく考察とは分離 |

**バージョン別レポート原則**:
- **1バージョン = 1ファイル**: 各バージョンの実験結果は個別ファイル（`exp{NNN}_report_v{M}.md`）に記録
- 主レポート（`exp{NNN}_{実験名}.md`）にはバージョン一覧とリンクのみを記載
- 各バージョンファイルは自己完結的に記述（そのファイルだけで実験内容が理解できる）

**分離の意図**:
- **実験概要**: 実験全体の俯瞰と各バージョンへのナビゲーション
- **バージョン別結果**: 各試行の詳細を独立管理（ファイルサイズ肥大化防止、参照しやすさ）
- **調査結果**: 外部知識を参照可能な形で整理（引用元の明確化・再利用性）
- **ルール**: 作業手順を標準化（効率化・ミス防止）

これらを混在させると、後から特定の情報を探す際に困難になる。

### DrawIO図の作成ルール

`.drawio.svg`ファイルの作成・編集時は、drawioスキル（`.claude/skills/drawio/SKILL.md`）に従うこと。形式はDrawIO XML（`<mxfile>`）のみ許可。純粋なSVG形式は禁止。
- 主要な実行コマンドはプロジェクトルートの`README.md`に記載し，パッケージ直下のドキュメントには二重管理を防ぐため記載しない．コマンドに言及したい場合は「ルートのREADME.mdを参照のこと」で扱うこと
