---
name: drawio-svg
description: DrawIO互換SVG図（.drawio.svg）の作成・編集ルール。.drawio.svgファイルを作成・編集する際に必ず適用する。
user-invocable: false
---

# DrawIO互換SVG図の作成・編集

## 概要

`.drawio.svg` は **SVGラッパー + DrawIO XMLの埋め込み** という二層構造を持つ。

- **SVGレイヤー**: ブラウザ/Markdownプレビューでの視覚表示を担当
- **content属性**: DrawIOアプリでの編集データ（HTMLエンコード済みmxfile XML）

両レイヤーが同期していないと、DrawIOで開けない・表示が壊れるといった問題が起きる。
手動でHTMLエンコードすると高確率でエラーになるため、**必ず付属スクリプトを使う**。

### 禁止形式

| 形式 | 問題 |
|---|---|
| `<mxfile>` のみ（素のDrawIO XML） | ブラウザで表示不可 |
| `<svg>` のみ（素のSVG） | DrawIOで編集不可 |
| 手動HTMLエンコード | エンコードミスでDrawIOが開けない |

## ツールスクリプト

`scripts/drawio_svg.py` が付属する。HTMLエンコード・viewBox計算・検証を自動で行う。

```bash
# スクリプトのパス（スキルディレクトリからの相対パス）
SCRIPT=".claude/skills/drawio-svg/scripts/drawio_svg.py"

# ビルド: 部品ファイルから .drawio.svg を生成
python $SCRIPT build --mxfile /tmp/mxfile.xml --svg-body /tmp/body.svg -o output.drawio.svg

# 検証: 既存ファイルの整合性チェック
python $SCRIPT validate file.drawio.svg

# 抽出: 編集のためにmxfile XMLを取り出す
python $SCRIPT extract file.drawio.svg -o /tmp/mxfile.xml
```

## 新規作成の手順

### ステップ1: mxfile XMLを作成

`/tmp/drawio_mxfile.xml` に以下の構造で書く:

```xml
<mxfile host="65bd71144e">
    <diagram name="[図の名前]" id="[ケバブケースID]">
        <mxGraphModel dx="800" dy="600" grid="1" gridSize="10" guides="1" tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1" pageWidth="827" pageHeight="1169" math="0" shadow="0">
            <root>
                <mxCell id="0"/>
                <mxCell id="1" parent="0"/>
                [mxCell要素をここに記述]
            </root>
        </mxGraphModel>
    </diagram>
</mxfile>
```

### ステップ2: SVGボディを作成

`/tmp/drawio_body.svg` にSVG描画要素を書く。mxCellの座標・色と一致させること。

### ステップ3: ビルド＆検証

```bash
python .claude/skills/drawio-svg/scripts/drawio_svg.py build \
    --mxfile /tmp/drawio_mxfile.xml \
    --svg-body /tmp/drawio_body.svg \
    -o [出力パス]

python .claude/skills/drawio-svg/scripts/drawio_svg.py validate [出力パス]
```

validateがERRORを出したら修正して再ビルドする。

## 既存ファイルの編集

1. **Read** でファイルを読む
2. **extract** で生のmxfile XMLを取り出す:
   ```bash
   python .claude/skills/drawio-svg/scripts/drawio_svg.py extract [ファイル] -o /tmp/drawio_mxfile.xml
   ```
3. mxfile XMLを編集し `/tmp/drawio_mxfile.xml` に保存
4. SVGボディ部分（`<g>...</g>` の中身）を編集し `/tmp/drawio_body.svg` に保存
5. **build** で再組み立て → **validate** で検証

mxCellとSVG要素は**必ず両方同時に更新**すること。片方だけ更新すると不整合になる。

## デザイン定数

### カラーパレット（この色のみ使用する）

| 用途 | fillColor | strokeColor | テキスト色 |
|---|---|---|---|
| 青（通常/進行中） | `#dae8fc` | `#6c8ebf` | `#6c8ebf` |
| 緑（成功/完了） | `#d5e8d4` | `#82b366` | `#82b366` |
| 赤（失敗/エラー） | `#fdedec` | `#e74c3c` | `#e74c3c` |
| 灰（中立/基盤） | `#f5f5f5` | `#666666` | `#666666` |
| 黄（注意/報酬） | `#fff2cc` | `#d6b656` | `#d6b656` |
| 紫（特殊/拡張） | `#e1d5e7` | `#9673a6` | `#9673a6` |

- 本文テキスト色: `#333`
- 矢印・線のデフォルト色: `#555`
- 線幅: `stroke-width="2"`

### フォント

全テキスト要素に `font-family="Helvetica, sans-serif"` を固定適用する。

| テキスト種別 | font-size | font-weight |
|---|---|---|
| タイトル / ボックスラベル | `14` | `bold` |
| サブテキスト / 注釈 | `12` | `normal` |
| 小注釈 | `11` | `normal` |

### レイアウト定数

| パラメータ | 値 |
|---|---|
| ボックス角丸 | `rx="10" ry="10"` |
| ボックスデフォルト幅 | `220` |
| ボックスデフォルト高さ（1行） | `50` |
| ボックスデフォルト高さ（2行） | `60` |
| ボックス間の垂直間隔 | `50`（矢印のスペース含む） |
| ボックス間の水平間隔 | `30` |
| テキストのY中央オフセット（1行ボックス） | ボックスy + 高さ×0.6 |
| テキストのY中央オフセット（2行ボックス1行目） | ボックスy + 高さ×0.4 |
| テキストのY中央オフセット（2行ボックス2行目） | ボックスy + 高さ×0.7 |

## SVG描画要素のリファレンス

### 矩形ボックス

```xml
<!-- mxCell定義 -->
<mxCell id="[id]" value="[ラベル]"
        style="rounded=1;whiteSpace=wrap;html=1;fillColor=[fillColor];strokeColor=[strokeColor];fontSize=[size];fontStyle=1;"
        parent="1" vertex="1">
    <mxGeometry x="[x]" y="[y]" width="[w]" height="[h]" as="geometry"/>
</mxCell>

<!-- 対応するSVG要素 -->
<rect x="[x]" y="[y]" width="[w]" height="[h]" rx="10" ry="10"
      fill="[fillColor]" stroke="[strokeColor]" stroke-width="2"/>
<text x="[x + w/2]" y="[テキストY]" text-anchor="middle"
      font-family="Helvetica, sans-serif" font-size="[size]" font-weight="bold"
      fill="#333">[ラベル]</text>
```

### 矢印（直線）

```xml
<!-- mxCell定義 -->
<mxCell id="[id]" value=""
        style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;strokeWidth=2;"
        parent="1" source="[sourceId]" target="[targetId]" edge="1">
    <mxGeometry relative="1" as="geometry"/>
</mxCell>

<!-- 対応するSVG要素（垂直下向き矢印） -->
<line x1="[始点x]" y1="[始点y]" x2="[終点x]" y2="[終点y]"
      stroke="#555" stroke-width="2" marker-end="url(#arrowhead)"/>
```

### 矢印（折れ線・パス）

```xml
<path d="M [x1] [y1] L [x2] [y2] L [x3] [y3]"
      fill="none" stroke="#555" stroke-width="2" marker-end="url(#arrowhead)"/>
```

### 矢印ラベル付き

```xml
<text x="[矢印中間x + オフセット]" y="[矢印中間y]"
      text-anchor="start" font-family="Helvetica, sans-serif"
      font-size="12" font-weight="bold" fill="[色]">[ラベル]</text>
```

### 破線矢印

```xml
<path d="M [x1] [y1] L [x2] [y2]"
      fill="none" stroke="[色]" stroke-width="2" stroke-dasharray="6 4"
      marker-end="url(#arrowhead)"/>
```

### 色分け矢印マーカー

矢印の色を変える場合は `--defs` オプションでカスタムdefsファイルを指定する:

```xml
<defs>
    <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="10" refY="3.5" orient="auto">
        <polygon points="0 0, 10 3.5, 0 7" fill="#555"/>
    </marker>
    <marker id="arrowhead-green" markerWidth="10" markerHeight="7" refX="10" refY="3.5" orient="auto">
        <polygon points="0 0, 10 3.5, 0 7" fill="#82b366"/>
    </marker>
</defs>
```

## 整合性チェックリスト

`validate` コマンドが自動で以下をチェックする:

- ファイルが `<svg` タグで始まる
- `content` 属性にデコード可能なmxfile XMLが含まれる
- mxfile XMLが妥当なXMLとしてパースできる
- `<diagram>` と `<mxGraphModel>` 構造が存在する
- SVG描画要素（rect, text, line, path等）が存在する
- mxCellのvertex/edge数とSVG要素数が概ね一致する
