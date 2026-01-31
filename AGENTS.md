# AGENTS.md — bsl_droid_control

## 目的
- このリポジトリは「BSL-Droid」の制御に関するソフトウェア全般を扱う
- 動作環境はJetson Orin Nano Super（実機・RT制御）とMacBook（可視化・机上開発）の分散構成を前提としている

## リポジトリ構成（要点）

```
bsl_droid_control/
├── ros2_ws/                        # ROS 2ワークスペース（pixi管理）
│   └── src/
│       ├── biped_description/      # URDF/可視化/GUI
│       ├── biped_gait_control/     # 歩容生成サンプル
│       ├── robstride_hardware/     # ros2_control用IF（Jetson専用・開発中）
│       └── pub_sub_cpp|python/     # ROS 2チュートリアル
├── rl_ws/                          # 強化学習環境（uv管理）
│   ├── biped_walking/              # 二脚ロボット環境・訓練スクリプト
│   │   ├── envs/                   # 環境定義（biped_env.py, droid_env.py）
│   │   └── train/                  # 訓練スクリプト群
│   ├── assets/                     # URDFモデル
│   ├── logs/                       # 訓練ログ・チェックポイント
│   ├── genesis_official/           # Genesis物理シミュレータ（submodule）
│   └── mujoco_menagerie/           # MuJoCoモデル集（submodule）
├── doc/                            # 設計資料
│   ├── design/                     # システム設計ドキュメント
│   └── experiments/                # 実験レポート群
└── ref/                            # 参考実装・外部ライブラリ
```

## セットアップ

### ROS 2環境（ros2_ws）

pixiでROS 2環境を管理している．

```bash
cd ros2_ws
pixi install
pixi run colcon build --symlink-install
```

### 強化学習環境（rl_ws）

uvで強化学習環境を管理している．

```bash
cd rl_ws
uv sync
```

### プラットフォーム注意
- **Jetson (linux-aarch64)**: ros2_control利用可
- **MacBook (osx-arm64)**: ros2_control不可（RoboStack JazzyのmacOS ARM64ビルド未提供）

## 作業ディレクトリ

このリポジトリでは各環境のworkspaceへ`cd`してからコマンドを実行する必要がある．

- **ROS 2環境**: `cd ros2_ws && pixi run ...`
- **強化学習環境**: `cd rl_ws && uv run ...`

## ROS 2環境（ros2_ws）

### よく使う起動コマンド

```bash
cd ros2_ws

# URDF可視化（カスタムGUI付き）
pixi run ros2 launch biped_description display_custom.launch.py

# 歩容生成 + RViz可視化
pixi run ros2 launch biped_gait_control gait_visualization.launch.py

# 外部から /joint_states を供給する場合（競合回避）
pixi run ros2 launch biped_description display_rviz_only.launch.py
```

### 注意事項

- `display.launch.py` は `joint_state_publisher_gui` を起動するため、外部から`/joint_states` を与える時競合する
- `display_rviz_only.launch.py`を常用するのが好ましい
- 生成物（例: `frames_*.gv`）はワークスペース直下に出ることがある

## 強化学習環境（rl_ws）

強化学習ワークスペース`rl_ws/`はuvで管理されている．ROS 2環境（ros2_ws）とは独立した仮想環境である．

### コマンド実行形式

**必須**: `rl_ws`ディレクトリに移動してから`uv run`で実行すること．

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
| TensorBoard | `uv run tensorboard --logdir logs/` |

### 実験名の規則

| ロボット | 形式 | 例 |
|----------|------|-----|
| biped_digitigrade | `biped-walking-v{N}` | biped-walking-v22 |
| BSL-Droid Simplified | `droid-walking-v{N}` | droid-walking-v19 |

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

## ドキュメント類の作成ルール
- 文書は`markdown`形式のドキュメントとして作成すること
- 必ず日本語で執筆し，N1相当の流暢で自然な日本語を用いること．
- 図は`drawio.svg`形式のファイル名に`drawio`の規格に準じたxmlタグを格納することで，独立して作成すること．
- 図をmarkdownドキュメント内にアスキーアートで描くことは行なってはいけない．二重管理の状態を避けるために厳守する．

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
doc/experiments/exp004_droid_rl_walking/
├── exp004_droid_rl_walking.md      # 実験結果と知見（累積）
├── exp004_rules.md                 # 実験ルール・手順・コマンド
└── exp004_reward_design_survey.md  # 調査結果（任意）
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
| `exp{NNN}_{実験名}.md` | 実験結果と知見 | 各バージョンの目的・変更内容・結果を累積。失敗した試みも含めて記述し、次の改善に活かす |
| `exp{NNN}_rules.md` | 実験ルール・手順 | コマンド、スクリプト使用法、バージョン管理原則、トラブルシューティング |
| `exp{NNN}_{調査名}_survey.md` | 先行研究調査 | 関連事例、研究、先行事例、論文などの調査結果**のみ**を記載。実験結果やそれに基づく考察とは分離 |

**分離の意図**:
- **実験結果**: 自分たちが行った試行錯誤と学びを記録（再現性・知見の蓄積）
- **調査結果**: 外部知識を参照可能な形で整理（引用元の明確化・再利用性）
- **ルール**: 作業手順を標準化（効率化・ミス防止）

これらを混在させると、後から特定の情報を探す際に困難になる。

### DrawIO図の作成ルール【厳守】

**ファイル形式**: `.drawio.svg` 拡張子を使用し、中身は **DrawIO XML形式** (`<mxfile>`) で作成すること。

#### DO（必須）
```xml
<!-- 正しい形式：DrawIO XML形式 -->
<mxfile host="65bd71144e">
    <diagram name="Diagram Name" id="diagram-id">
        <mxGraphModel dx="800" dy="600" grid="1" gridSize="10" ...>
            <root>
                <mxCell id="0"/>
                <mxCell id="1" parent="0"/>
                <!-- 図形要素をここに配置 -->
                <mxCell id="box1" value="Text" style="rounded=1;whiteSpace=wrap;html=1;..." 
                        parent="1" vertex="1">
                    <mxGeometry x="100" y="100" width="120" height="60" as="geometry"/>
                </mxCell>
            </root>
        </mxGraphModel>
    </diagram>
</mxfile>
```

#### DON'T（禁止）
```xml
<!-- 誤った形式：純粋なSVG形式（DrawIOで開けない） -->
<svg xmlns="http://www.w3.org/2000/svg" ...>
    <rect x="100" y="100" width="120" height="60"/>
</svg>
```

#### 理由
- **編集可能性**: DrawIOアプリで開いて後から編集できる
- **表示可能性**: Markdownプレビューで画像として表示される
- **一元管理**: ソースファイルと表示ファイルが同一（二重管理なし）

#### 図形の指定方法
DrawIO標準図形を使用する際は、適切な名前空間を指定すること：

```xml
<!-- 矩形 -->
<mxCell id="rect1" value="Label" 
        style="rounded=1;whiteSpace=wrap;html=1;fillColor=#dae8fc;strokeColor=#6c8ebf;" 
        parent="1" vertex="1">
    <mxGeometry x="100" y="100" width="120" height="60" as="geometry"/>
</mxCell>

<!-- 矢印（接続） -->
<mxCell id="arrow1" value="Label" 
        style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;strokeWidth=2;" 
        parent="1" source="box1" target="box2" edge="1">
    <mxGeometry relative="1" as="geometry"/>
</mxCell>
```

#### 検証方法
作成後、以下を確認すること：
1. DrawIOアプリ（draw.io）で開けるか → 編集可能性の確認
2. Markdownプレビューで画像表示されるか → 表示可能性の確認
3. ファイルを`cat`して`<mxfile>`タグから始まるか → 形式の確認

#### テンプレート
新規DrawIO図を作成する際は、以下のテンプレートを使用すること：

```xml
<mxfile host="65bd71144e">
    <diagram name="[図の名前]" id="[ユニークID]">
        <mxGraphModel dx="800" dy="600" grid="1" gridSize="10" guides="1" tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1" pageWidth="827" pageHeight="1169" math="0" shadow="0">
            <root>
                <mxCell id="0"/>
                <mxCell id="1" parent="0"/>
                <!-- ここに図形要素を追加 -->
            </root>
        </mxGraphModel>
    </diagram>
</mxfile>
```

**重要**: この形式以外で`.drawio.svg`ファイルを作成することは厳禁。違反すると「Not a diagram file」エラーが発生し、DrawIOで開けなくなる。
- 主要な実行コマンドはプロジェクトルートの`README.md`に記載し，パッケージ直下のドキュメントには二重管理を防ぐため記載しない．コマンドに言及したい場合は「ルートのREADME.mdを参照のこと」で扱うこと

## 参照すべきドキュメント
- `README.md`
- `doc/design/distributed_architecture.md`
- `doc/next_nodes_design.md`
- `ros2_ws/src/biped_description/doc/technical_specification.md`
