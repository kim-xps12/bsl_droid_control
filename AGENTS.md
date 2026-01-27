# AGENTS.md — bsl_droid_ros2

## 目的
このリポジトリは「BSL-Droid」のROS 2制御システム。Jetson Orin Nano Super（実機・RT制御）とMacBook（可視化・机上開発）の分散構成を前提にしています。

## リポジトリ構成（要点）
- `ros2_ws/` — ROS 2ワークスペース（実作業は基本ここ）
- `ros2_ws/src/biped_description/` — URDF/可視化/GUI
- `ros2_ws/src/biped_gait_control/` — 歩容生成
- `ros2_ws/src/robstride_hardware/` — ros2_control用IF（開発中）
- `doc/` — 設計資料
- `ref/` — 参考実装・外部ライブラリ

## セットアップ（pixi）
このプロジェクトはpixiでROS 2環境を管理します。

```bash
cd ros2_ws
pixi install
pixi run colcon build --symlink-install
```

### プラットフォーム注意
- **Jetson (linux-aarch64)**: ros2_control利用可
- **MacBook (osx-arm64)**: ros2_control不可（RoboStack JazzyのmacOS ARM64ビルド未提供）

## 作業ディレクトリ
このリポジトリではROS 2の作業環境は`ros2_ws`以下にpixiで仮想化しているため，ビルドやノードの実行をする際にはカレントディレクトリを`ros2_ws`に移動した後に`pixi run hogehoge...`の形式で実行する必要がある．

## 実際に実行する動作確認
- GUIを伴う実装の有無によらず、コーディング完了後は実際に対象を起動し、エラーなく終了することを必ず確認する
- 各仮想環境のworkspaceへ`cd`して実行する必要があることを念頭におく

## よく使う起動コマンド

```bash
# URDF可視化（カスタムGUI）
pixi run ros2 launch biped_description display_custom.launch.py

# 歩容生成 + RViz可視化
pixi run ros2 launch biped_gait_control gait_visualization.launch.py

# 外部から /joint_states を供給する場合
pixi run ros2 launch biped_description display_rviz_only.launch.py
```

## トラブルシュート
- macOSでのthread affinity警告は無視可
- KDL root link inertia警告は可視化に影響なし

## ドキュメント類の作成ルール
- 文書は`markdown`形式のドキュメントとして作成すること
- 必ず日本語で執筆し，N1相当の流暢で自然な日本語を用いること．
- 図は`drawio.svg`形式のファイル名に`drawio`の規格に準じたxmlタグを格納することで，独立して作成すること．
- 図をmarkdownドキュメント内にアスキーアートで描くことは行なってはいけない．二重管理の状態を避けるために厳守する．

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

## 作業時の注意
- 実行コマンドは原則 `ros2_ws/` 配下で行う
- `display.launch.py` は `joint_state_publisher_gui` を起動するため、外部から`/joint_states` を与える時競合するゆえに`display_rviz_only.launch.py`を常用するのが好ましい
- 生成物（例: `frames_*.gv`）はワークスペース直下に出ることがある



## 参照すべきドキュメント
- `README.md`
- `doc/distributed_architecture.md`
- `doc/next_nodes_design.md`
- `ros2_ws/src/biped_description/doc/technical_specification.md`
