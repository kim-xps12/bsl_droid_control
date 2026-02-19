---
name: drawio-svg
description: DrawIO互換SVG図（.drawio.svg）の作成・編集ルール。.drawio.svgファイルを作成・編集する際に必ず適用する。
user-invocable: false
---

# DrawIO互換SVG図の作成仕様【厳守】

## 1. ファイル形式の定義

`.drawio.svg` ファイルは **SVGラッパー + DrawIO XMLの埋め込み** という二層構造を持つ。

```
┌─ <svg content="HTMLエンコード済み<mxfile>"> ─┐
│   <defs>矢印マーカー定義</defs>              │
│   <g>                                         │
│     SVG描画要素（rect, text, path...）        │
│   </g>                                         │
└────────────────────────────────────────────────┘
```

- **SVGレイヤー**: ブラウザ/Markdownプレビューでの視覚表示を担当
- **content属性**: DrawIOアプリでの編集データを担当

### 禁止形式

| 形式 | 問題 |
|---|---|
| `<mxfile>` のみ（素のDrawIO XML） | ブラウザで表示不可 |
| `<svg>` のみ（素のSVG） | DrawIOで編集不可 |

## 2. 作成手順（必ずこの順序で実行）

### ステップ1: DrawIO XMLデータの設計

図の論理構造をmxCell要素として定義する。

### ステップ2: SVG描画要素の作成

ステップ1のmxCellに対応するSVG要素（rect, text, path等）を作成する。
**mxCellのstyleプロパティとSVG要素の属性値は必ず一致させること。**

### ステップ3: DrawIO XMLのHTMLエンコード

ステップ1のXMLを以下のルールでエンコードする:
- `<` → `&lt;`
- `>` → `&gt;`
- `"` → `&quot;`
- `&` → `&amp;`（他のエンコード文字内の`&`は除く）

### ステップ4: SVGファイルの組み立て

後述のテンプレートに従い、エンコード済みDrawIO XMLを `content` 属性に埋め込む。

### ステップ5: viewBoxとサイズの算出

全SVG要素の座標から以下を計算する:
- `minX` = 全要素の最小x座標
- `minY` = 全要素の最小y座標
- `maxX` = 全要素の最大 (x + width)
- `maxY` = 全要素の最大 (y + height)
- `viewBox` = `"{minX - 0.5} {minY - 0.5} {maxX - minX + 1} {maxY - minY + 1}"`
- `width` = `"{maxX - minX + 1}px"`
- `height` = `"{maxY - minY + 1}px"`

## 3. 完全テンプレート

**新規作成時は必ずこのテンプレートを使用する。** プレースホルダ `[...]` のみ変更すること。

```xml
<svg host="65bd71144e" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" version="1.1" width="[幅]px" height="[高さ]px" viewBox="[viewBox]" style="background: transparent;" content="[HTMLエンコード済みmxfile XML]">
    <defs>
        <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="10" refY="3.5" orient="auto">
            <polygon points="0 0, 10 3.5, 0 7" fill="#555"/>
        </marker>
    </defs>
    <g>
        [SVG描画要素]
    </g>
</svg>
```

### DrawIO XMLテンプレート（content属性に埋め込む前の生データ）

```xml
<mxfile host="65bd71144e">
    <diagram name="[図の名前]" id="[ケバブケースID]">
        <mxGraphModel dx="800" dy="600" grid="1" gridSize="10" guides="1" tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1" pageWidth="827" pageHeight="1169" math="0" shadow="0">
            <root>
                <mxCell id="0"/>
                <mxCell id="1" parent="0"/>
                [mxCell要素]
            </root>
        </mxGraphModel>
    </diagram>
</mxfile>
```

## 4. デザイン定数

### 4.1 カラーパレット（この色のみ使用する）

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

### 4.2 フォント

全テキスト要素に以下を固定適用する:

```
font-family="Helvetica, sans-serif"
```

| テキスト種別 | font-size | font-weight |
|---|---|---|
| タイトル / ボックスラベル | `14` | `bold` |
| サブテキスト / 注釈 | `12` | `normal` |
| 小注釈 | `11` | `normal` |

### 4.3 レイアウト定数

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

## 5. SVG描画要素のリファレンス

### 5.1 矩形ボックス

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

### 5.2 矢印（直線）

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

### 5.3 矢印（折れ線・パス）

```xml
<!-- 対応するSVG要素（L字・コの字パス） -->
<path d="M [x1] [y1] L [x2] [y2] L [x3] [y3]"
      fill="none" stroke="#555" stroke-width="2" marker-end="url(#arrowhead)"/>
```

### 5.4 矢印ラベル付き

矢印の横にラベルを配置する場合、`<text>` を矢印の中間点付近に配置する:

```xml
<text x="[矢印中間x + オフセット]" y="[矢印中間y]"
      text-anchor="start" font-family="Helvetica, sans-serif"
      font-size="12" font-weight="bold" fill="[色]">[ラベル]</text>
```

### 5.5 破線矢印

```xml
<path d="M [x1] [y1] L [x2] [y2]"
      fill="none" stroke="[色]" stroke-width="2" stroke-dasharray="6 4"
      marker-end="url(#arrowhead)"/>
```

### 5.6 色分け矢印マーカー

矢印の色を変える場合は、`<defs>` に色別のマーカーを追加する:

```xml
<marker id="arrowhead-[色名]" markerWidth="10" markerHeight="7" refX="10" refY="3.5" orient="auto">
    <polygon points="0 0, 10 3.5, 0 7" fill="[色コード]"/>
</marker>
```

## 6. mxCell と SVG の整合性チェックリスト

ファイル作成完了後、以下を必ず確認する:

- [ ] ファイルが `<svg` タグで始まる
- [ ] `content` 属性に `&lt;mxfile` で始まるHTMLエンコード済みXMLが含まれる
- [ ] 全mxCell（vertex）に対応するSVG `<rect>` + `<text>` がある
- [ ] 全mxCell（edge）に対応するSVG `<line>` or `<path>` がある
- [ ] mxCellのx, y, width, heightとSVG要素の座標が一致する
- [ ] mxCellのfillColor/strokeColorとSVG要素のfill/strokeが一致する
- [ ] viewBoxが全要素を包含している
- [ ] `width` と `height` が viewBox と整合している

## 7. 既存ファイルの編集

既存の `.drawio.svg` を編集する場合:

1. **Read** でファイルを読み込む
2. `content` 属性内のDrawIO XMLを確認し、変更対象のmxCellを特定する
3. mxCellとSVG要素の**両方を同時に更新**する（片方だけ更新すると不整合が起きる）
4. viewBoxとwidth/heightを再計算する
