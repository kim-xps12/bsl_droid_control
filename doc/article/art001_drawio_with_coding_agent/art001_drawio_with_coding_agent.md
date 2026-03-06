# Coding Agentに編集可能な図付きでドキュメントを書かせたい！

## 2025/03 更新: Anthropic公式のskill-creatorでブラッシュアップしました

本記事で紹介した `.drawio.svg` の運用について、Anthropic公式の **skill-creator** プラグインを使ってスキルの品質を改善しました。主な変更点は以下の通りです。

### 改善内容

1. **ビルドスクリプトの導入** (`scripts/drawio_svg.py`)
   - HTMLエンコードを手動で行う必要がなくなった（エンコードミスによる「DrawIOで開けない」問題を解決）
   - viewBox・width/heightの自動計算（レンダリング崩れの防止）
   - `build` / `validate` / `extract` の3コマンドで作成・検証・編集を一貫して行える

2. **SKILL.mdの刷新**
   - 旧: AGENTS.mdにルールを直接記載 → 新: `.claude/skills/drawio-svg/` としてスキル化
   - 「mxfile XMLとSVGボディを別ファイルに書いてスクリプトで結合」というワークフローに変更
   - 手動HTMLエンコードを明示的に禁止

### 最新のスキル構成

```
.claude/skills/drawio-svg/
├── SKILL.md              # スキル定義（トリガー条件・作成手順・デザイン定数）
└── scripts/
    └── drawio_svg.py     # ビルド/検証/抽出ツール
```

以下に各ファイルの全文を掲載します。読者の環境に `.claude/skills/drawio-svg/` ディレクトリを作成し、これらのファイルを配置すればそのまま利用できます。

<details>
<summary>SKILL.md（クリックで展開）</summary>

```markdown
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

## 新規作成の手順

### ステップ1: mxfile XMLを作成

`/tmp/drawio_mxfile.xml` に以下の構造で書く:

（テンプレートはSKILL.md本体を参照）

### ステップ2: SVGボディを作成

`/tmp/drawio_body.svg` にSVG描画要素を書く。mxCellの座標・色と一致させること。

### ステップ3: ビルド＆検証

python .claude/skills/drawio-svg/scripts/drawio_svg.py build \
    --mxfile /tmp/drawio_mxfile.xml \
    --svg-body /tmp/drawio_body.svg \
    -o [出力パス]

python .claude/skills/drawio-svg/scripts/drawio_svg.py validate [出力パス]

## 既存ファイルの編集

1. Readでファイルを読む
2. extractで生のmxfile XMLを取り出す
3. mxCellとSVG要素の両方を更新
4. buildで再組み立て → validateで検証
```

</details>

<details>
<summary>scripts/drawio_svg.py（クリックで展開）</summary>

```python
#!/usr/bin/env python3
"""DrawIO SVG build / validate / extract tool.

Eliminates manual HTML-encoding errors and viewBox miscalculations
when creating or editing .drawio.svg files.
"""

import argparse
import html
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def html_encode_for_attr(text: str) -> str:
    """Encode raw XML for embedding in an SVG content attribute."""
    text = text.replace("&", "&amp;")
    text = text.replace("<", "&lt;")
    text = text.replace(">", "&gt;")
    text = text.replace('"', "&quot;")
    return text


def html_decode_attr(text: str) -> str:
    """Decode HTML entities from an SVG content attribute."""
    return html.unescape(text)


def _find_attr(tag_str: str, attr: str):
    """Extract a numeric attribute value from an SVG element string."""
    m = re.search(rf'{attr}\s*=\s*["\']([^"\']*)["\']', tag_str)
    if m:
        try:
            return float(m.group(1))
        except ValueError:
            return None
    return None


def calculate_viewbox(svg_body: str) -> tuple:
    """Return (min_x, min_y, width, height) covering all SVG elements."""
    min_x = float("inf")
    min_y = float("inf")
    max_x = float("-inf")
    max_y = float("-inf")

    def expand(x, y):
        nonlocal min_x, min_y, max_x, max_y
        min_x = min(min_x, x)
        min_y = min(min_y, y)
        max_x = max(max_x, x)
        max_y = max(max_y, y)

    # <rect>, <image>, <foreignObject> with x/y/width/height
    for m in re.finditer(r"<(?:rect|image|foreignObject)\b[^>]*?>", svg_body):
        tag = m.group(0)
        x = _find_attr(tag, "x")
        y = _find_attr(tag, "y")
        w = _find_attr(tag, "width")
        h = _find_attr(tag, "height")
        if x is not None and y is not None:
            expand(x, y)
            if w is not None and h is not None:
                expand(x + w, y + h)

    # <text> with x/y
    for m in re.finditer(r"<text\b[^>]*?>", svg_body):
        tag = m.group(0)
        x = _find_attr(tag, "x")
        y = _find_attr(tag, "y")
        if x is not None and y is not None:
            expand(x, y)
            expand(x + 10, y + 5)

    # <line> with x1/y1/x2/y2
    for m in re.finditer(r"<line\b[^>]*?>", svg_body):
        tag = m.group(0)
        for pair in [("x1", "y1"), ("x2", "y2")]:
            x = _find_attr(tag, pair[0])
            y = _find_attr(tag, pair[1])
            if x is not None and y is not None:
                expand(x, y)

    # <path d="..."> - extract M and L coordinates
    for m in re.finditer(r"<path\b[^>]*?>", svg_body):
        tag = m.group(0)
        d_match = re.search(r'd\s*=\s*"([^"]*)"', tag)
        if d_match:
            for coord in re.finditer(r"[ML]\s*([\d.]+)\s+([\d.]+)", d_match.group(1)):
                expand(float(coord.group(1)), float(coord.group(2)))

    # <circle> with cx/cy/r
    for m in re.finditer(r"<circle\b[^>]*?>", svg_body):
        tag = m.group(0)
        cx = _find_attr(tag, "cx")
        cy = _find_attr(tag, "cy")
        r = _find_attr(tag, "r")
        if cx is not None and cy is not None and r is not None:
            expand(cx - r, cy - r)
            expand(cx + r, cy + r)

    # <ellipse> with cx/cy/rx/ry
    for m in re.finditer(r"<ellipse\b[^>]*?>", svg_body):
        tag = m.group(0)
        cx = _find_attr(tag, "cx")
        cy = _find_attr(tag, "cy")
        rx = _find_attr(tag, "rx")
        ry = _find_attr(tag, "ry")
        if None not in (cx, cy, rx, ry):
            expand(cx - rx, cy - ry)
            expand(cx + rx, cy + ry)

    if min_x == float("inf"):
        return (0, 0, 800, 600)

    padding = 10
    return (
        min_x - padding,
        min_y - padding,
        max_x - min_x + 2 * padding,
        max_y - min_y + 2 * padding,
    )


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

DEFAULT_DEFS = """\
    <defs>
        <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="10" refY="3.5" orient="auto">
            <polygon points="0 0, 10 3.5, 0 7" fill="#555"/>
        </marker>
    </defs>"""


def cmd_build(args):
    """Build a .drawio.svg from raw mxfile XML and SVG body elements."""
    mxfile_xml = Path(args.mxfile).read_text(encoding="utf-8").strip()
    svg_body = Path(args.svg_body).read_text(encoding="utf-8").strip()

    # Validate the mxfile XML parses correctly
    try:
        ET.fromstring(mxfile_xml)
    except ET.ParseError as e:
        print(f"ERROR: mxfile XML is not valid: {e}", file=sys.stderr)
        sys.exit(1)

    encoded = html_encode_for_attr(mxfile_xml)

    vb = calculate_viewbox(svg_body)
    viewbox = f"{vb[0]:.1f} {vb[1]:.1f} {vb[2]:.1f} {vb[3]:.1f}"
    width = f"{vb[2]:.0f}px"
    height = f"{vb[3]:.0f}px"

    # Read custom defs or use default
    if args.defs:
        defs = Path(args.defs).read_text(encoding="utf-8").strip()
    else:
        defs = DEFAULT_DEFS

    svg = (
        f'<svg host="65bd71144e" xmlns="http://www.w3.org/2000/svg"'
        f' xmlns:xlink="http://www.w3.org/1999/xlink" version="1.1"'
        f' width="{width}" height="{height}" viewBox="{viewbox}"'
        f' style="background: transparent;"'
        f' content="{encoded}">\n'
        f"{defs}\n"
        f"    <g>\n"
        f"{svg_body}\n"
        f"    </g>\n"
        f"</svg>\n"
    )

    Path(args.output).write_text(svg, encoding="utf-8")
    print(f"OK: {args.output}  ({width} x {height},  viewBox={viewbox})")


def cmd_validate(args):
    """Validate a .drawio.svg file."""
    path = Path(args.file)
    content = path.read_text(encoding="utf-8")
    errors = []
    warnings = []

    # 1. Must start with <svg
    if not content.strip().startswith("<svg"):
        errors.append("File does not start with <svg> tag")

    # 2. content attribute must exist and decode to valid mxfile XML
    m = re.search(r'content="([^"]*)"', content)
    if not m:
        errors.append("No content attribute found in <svg> tag")
    else:
        encoded = m.group(1)
        decoded = html_decode_attr(encoded)

        if not decoded.strip().startswith("<mxfile"):
            errors.append(
                f"Decoded content does not start with <mxfile>. "
                f"Starts with: {decoded[:80]!r}"
            )

        try:
            root = ET.fromstring(decoded)
            diagram = root.find("diagram")
            if diagram is None:
                warnings.append("mxfile has no <diagram> child")
            else:
                model = diagram.find("mxGraphModel")
                if model is None:
                    warnings.append("diagram has no <mxGraphModel> child")
        except ET.ParseError as e:
            errors.append(f"Decoded mxfile XML is not valid: {e}")

    # 3. viewBox present
    if 'viewBox="' not in content:
        warnings.append("No viewBox attribute found")

    # 4. SVG drawing elements exist
    has_drawing = any(
        f"<{tag}" in content
        for tag in ("rect", "text", "line", "path", "circle", "ellipse")
    )
    if not has_drawing:
        warnings.append("No SVG drawing elements found")

    # 5. Check mxCell count vs SVG element count
    if m:
        decoded = html_decode_attr(m.group(1))
        vertex_count = len(re.findall(r'vertex="1"', decoded))
        edge_count = len(re.findall(r'edge="1"', decoded))
        rect_count = len(re.findall(r"<rect\b", content))
        line_path_count = len(re.findall(r"<(?:line|path)\b", content))

        if vertex_count > 0 and rect_count == 0:
            warnings.append(
                f"{vertex_count} mxCell vertices but 0 SVG rects - "
                f"SVG layer may be missing"
            )
        if edge_count > 0 and line_path_count == 0:
            warnings.append(
                f"{edge_count} mxCell edges but 0 SVG lines/paths - "
                f"SVG layer may be missing"
            )

    # Report
    ok = True
    if errors:
        ok = False
        print(f"ERRORS ({path}):")
        for e in errors:
            print(f"  x {e}")
    if warnings:
        print(f"WARNINGS ({path}):")
        for w in warnings:
            print(f"  ! {w}")
    if ok and not warnings:
        print(f"OK: {path}")

    sys.exit(0 if ok else 1)


def cmd_extract(args):
    """Extract raw mxfile XML from a .drawio.svg file."""
    content = Path(args.file).read_text(encoding="utf-8")
    m = re.search(r'content="([^"]*)"', content)
    if not m:
        print("ERROR: No content attribute found", file=sys.stderr)
        sys.exit(1)

    decoded = html_decode_attr(m.group(1))

    # Pretty-print if possible
    try:
        root = ET.fromstring(decoded)
        ET.indent(root)
        decoded = ET.tostring(root, encoding="unicode")
    except ET.ParseError:
        pass  # output as-is

    if args.output:
        Path(args.output).write_text(decoded, encoding="utf-8")
        print(f"Extracted to: {args.output}", file=sys.stderr)
    else:
        print(decoded)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(
        description="Build, validate, and extract DrawIO-compatible SVG files"
    )
    sub = p.add_subparsers(dest="command")

    # build
    bp = sub.add_parser("build", help="Build .drawio.svg from mxfile XML + SVG body")
    bp.add_argument("--mxfile", required=True, help="Path to raw mxfile XML file")
    bp.add_argument("--svg-body", required=True, help="Path to SVG body elements file")
    bp.add_argument("-o", "--output", required=True, help="Output .drawio.svg path")
    bp.add_argument("--defs", help="Path to custom <defs> block (optional)")

    # validate
    vp = sub.add_parser("validate", help="Validate a .drawio.svg file")
    vp.add_argument("file", help="Path to .drawio.svg file")

    # extract
    ep = sub.add_parser("extract", help="Extract raw mxfile XML from .drawio.svg")
    ep.add_argument("file", help="Path to .drawio.svg file")
    ep.add_argument("-o", "--output", help="Output file (default: stdout)")

    args = p.parse_args()
    if args.command == "build":
        cmd_build(args)
    elif args.command == "validate":
        cmd_validate(args)
    elif args.command == "extract":
        cmd_extract(args)
    else:
        p.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
```

</details>

---

## はじめに

Claude CodeやCursorなどのCoding Agentを使ってドキュメントを書かせると，図が必要な場面で困ることがありませんか？アスキーアートで描かれた図は後から編集できませんし，画像生成AIで作った図も同様です．

本記事では，`drawio.svg`形式を活用することで，Coding Agentに「後から人間が編集可能な図」を生成させる方法を紹介します．実際に筆者のプロジェクトで運用している方法ですので，ぜひ参考にしてみてください．

## 想定環境
- Visual Studio Codeで開発をしている
- markdown形式でドキュメントを書いている
- DrawIO拡張を利用している

## 課題：Coding Agentが作る図は編集しづらい

Coding Agentにシステム構成図やフローチャートを依頼すると，以下のような出力が返ってくることが多いです．

### パターン1: アスキーアート

```
┌─────────────┐    ┌─────────────┐
│   Client    │───▶│   Server    │
└─────────────┘    └─────────────┘
```

見た目は悪くないのですが，要素を追加したりレイアウトを調整したりする際に全体を書き直す必要があります．ちょっとした修正でも面倒ですね．

### パターン2: 純粋な`.drawio`形式

coding agentに`.drawio`ファイルとして描かせることもできますが，これはこれで別の問題が出てきます．

- Markdownプレビューで図としてレンダリング表示できない(ことがある)
- 結局PNG/JPEGにエクスポートする必要があり，編集用ソースと表示用画像の二重管理になる

これはこれで地味に使いづらさがあります．

## 解決策：`.drawio.svg`形式

VSCodeのDrawIO拡張では，純粋な`.drawio`形式の他に`.drawio.svg`や`.drawio.png`形式でも扱うことができます．これのうち．`.drawio.svg`は，以下を満たすことができて非常に便利です．

1. Markdownプレビューで画像として表示される
1. DrawIOアプリで開いて編集できる
1. テキストベースなのでCoding Agentに適当な指示を出しても編集できる（pngの方はバイナリで画像を持ってしまうためagentと相性が悪い）

これによって「ソースファイルと表示ファイルが同一」になり，二重管理が生じなくなります．嬉しい．

以下の図は，この方法によるワークフローを示しています．

![](./art001_drawio_with_coding_agent_workflow.drawio.svg)

この図自体も`.drawio.svg`形式で作成されているため，VSCodeのdrawio拡張で開いて編集できます．以下の生コードを`piypiyo.drawio.svg`として保存してvscodeから開けばOKです．


<details>
<summary>art001_drawio_with_coding_agent_workflow.drawio.svg の生コード</summary>

```xml
<svg host="65bd71144e" xmlns="http://www.w3.org/2000/svg" style="background: transparent; background-color: transparent;" xmlns:xlink="http://www.w3.org/1999/xlink" version="1.1" width="721px" height="190px" viewBox="-0.5 -0.5 721 190" content="&lt;mxfile&gt;&lt;diagram name=&quot;DrawIO Figure Generation Workflow&quot; id=&quot;workflow-001&quot;&gt;1ZjbjtsgEIafhttVjA/Bl3bibCv1cJFKvSaG2LS2iQg59ekLNsQk9h6kZjfdaBXBz2DMNxOGWeDP6uOjwJvyKye0AnBCjsCfAwgRjNW3Fk6dEEKvEwrBSCc5wpL9oUacGHXHCN1eGErOK8k2l2LOm4bm8kLDQvDDpdmaV5erbnBBB8Iyx9VQ/cmILM22wkmvf6KsKM3K4cQM1NjaGmFbYsIPjuRnwJ8JzmXXqo8zWml0Fks3b/HE6Pm9BG3kayZIJtWe2kl7XO3M9uYCHz5/B1kI0jlIYNuYAbQAmQ+QB5K0bUCAkG2k2iZOQeqDbAriAMTKONIG8aS18UGyMA31qG7WWYlDazPvbTo+8mSZS3pUW0pLWVdK8FRzKwX/TWe84kIpDW+UZbpmVXUl4YoVjermCgpVerqnQjLlzcQM1IwQvUx6KJmkyw3O9ZoHFbpKE3zXEKp5TfTjeSOX5qU82+/i04tUf+gA4xO9Jj06knHII+U1leKkTMwotNFifhyB6R76SAusSelEmW80bIK7OD+5DwDVMDEwHg9iNxIOyWP27cfyoSYDn/RsvCfoOe5yPQOgv16vYZ4P3KhGSLSKwugF2PA2sINL1h4cwj5rLuzoBrDVKaHe+5r2jBPWFEpL+uEbISeYovUo8ihHdLV+H+R3ZU7U0cb4APpDpz9s98VtiYcUkWCMOIIrP3qvIL8n8XJX42YAHGQBSBP9l8UgjkDsP5tcpiCdggS1xilA0W1PIpTT8ZNohcIgfOnYv5GTorseRfpS5I14KdRpXzlBeQCFIJkOwFOibkamy4UsecEbXGW9epVAe5svnG8Mzl9UypPhiXeSj+V5e9GCV/gnz+Hf8p3IbYicc5vEoqBycAjrrTzrKUErLNn+8hL4z9jhCPbBHerjYncAu9zdg/hO4P1R8C8cNB8HvEvYJe8cyO8PvuFyeMG0ZcDcFgaBLQN809DJYKGrgjh5osDwbBWhEsbMKgunLGmH4nNt0yWbqUk/KDUKCuxQNBIM/azOeN6nqP+4XDGBY/OWe+1rP29Ttpz7b1C3qG5fI7djzv8Z/Owv&lt;/diagram&gt;&lt;/mxfile&gt;">
    <defs/>
    <g>
        <g>
            <rect x="160" y="0" width="400" height="30" fill="none" stroke="none" pointer-events="all"/>
        </g>
        <g>
            <g transform="translate(-0.5 -0.5)">
                <switch>
                    <foreignObject style="overflow: visible; text-align: left;" pointer-events="none" width="100%" height="100%" requiredFeatures="http://www.w3.org/TR/SVG11/feature#Extensibility">
                        <div xmlns="http://www.w3.org/1999/xhtml" style="display: flex; align-items: unsafe center; justify-content: unsafe center; width: 398px; height: 1px; padding-top: 15px; margin-left: 161px;">
                            <div style="box-sizing: border-box; font-size: 0; text-align: center; color: #000000; ">
                                <div style="display: inline-block; font-size: 16px; font-family: &quot;Helvetica&quot;; color: light-dark(#000000, #ffffff); line-height: 1.2; pointer-events: all; font-weight: bold; white-space: normal; word-wrap: normal; ">
                                    DrawIO形式による図生成ワークフロー
                                </div>
                            </div>
                        </div>
                    </foreignObject>
                    <text x="360" y="20" fill="light-dark(#000000, #ffffff)" font-family="&quot;Helvetica&quot;" font-size="16px" text-anchor="middle" font-weight="bold">
                        DrawIO形式による図生成ワークフロー
                    </text>
                </switch>
            </g>
        </g>
        <g>
            <rect x="0" y="80" width="120" height="60" rx="9" ry="9" fill="#fff2cc" stroke="#d6b656" pointer-events="all" style="fill: light-dark(rgb(255, 242, 204), rgb(40, 29, 0)); stroke: light-dark(rgb(214, 182, 86), rgb(109, 81, 0));"/>
        </g>
        <g>
            <g transform="translate(-0.5 -0.5)">
                <switch>
                    <foreignObject style="overflow: visible; text-align: left;" pointer-events="none" width="100%" height="100%" requiredFeatures="http://www.w3.org/TR/SVG11/feature#Extensibility">
                        <div xmlns="http://www.w3.org/1999/xhtml" style="display: flex; align-items: unsafe center; justify-content: unsafe center; width: 118px; height: 1px; padding-top: 110px; margin-left: 1px;">
                            <div style="box-sizing: border-box; font-size: 0; text-align: center; color: #000000; ">
                                <div style="display: inline-block; font-size: 12px; font-family: &quot;Helvetica&quot;; color: light-dark(#000000, #ffffff); line-height: 1.2; pointer-events: all; font-weight: bold; white-space: normal; word-wrap: normal; ">
                                    AGENTS.md
                                </div>
                            </div>
                        </div>
                    </foreignObject>
                    <text x="60" y="114" fill="light-dark(#000000, #ffffff)" font-family="&quot;Helvetica&quot;" font-size="12px" text-anchor="middle" font-weight="bold">
                        AGENTS.md
                    </text>
                </switch>
            </g>
        </g>
        <g>
            <rect x="200" y="80" width="120" height="60" rx="9" ry="9" fill="#dae8fc" stroke="#6c8ebf" pointer-events="all" style="fill: light-dark(rgb(218, 232, 252), rgb(29, 41, 59)); stroke: light-dark(rgb(108, 142, 191), rgb(92, 121, 163));"/>
        </g>
        <g>
            <g transform="translate(-0.5 -0.5)">
                <switch>
                    <foreignObject style="overflow: visible; text-align: left;" pointer-events="none" width="100%" height="100%" requiredFeatures="http://www.w3.org/TR/SVG11/feature#Extensibility">
                        <div xmlns="http://www.w3.org/1999/xhtml" style="display: flex; align-items: unsafe center; justify-content: unsafe center; width: 118px; height: 1px; padding-top: 110px; margin-left: 201px;">
                            <div style="box-sizing: border-box; font-size: 0; text-align: center; color: #000000; ">
                                <div style="display: inline-block; font-size: 12px; font-family: &quot;Helvetica&quot;; color: light-dark(#000000, #ffffff); line-height: 1.2; pointer-events: all; font-weight: bold; white-space: normal; word-wrap: normal; ">
                                    Coding Agent
                                </div>
                            </div>
                        </div>
                    </foreignObject>
                    <text x="260" y="114" fill="light-dark(#000000, #ffffff)" font-family="&quot;Helvetica&quot;" font-size="12px" text-anchor="middle" font-weight="bold">
                        Coding Agent
                    </text>
                </switch>
            </g>
        </g>
        <g>
            <rect x="400" y="80" width="120" height="60" rx="9" ry="9" fill="#d5e8d4" stroke="#82b366" pointer-events="all" style="fill: light-dark(rgb(213, 232, 212), rgb(31, 47, 30)); stroke: light-dark(rgb(130, 179, 102), rgb(68, 110, 44));"/>
        </g>
        <g>
            <g transform="translate(-0.5 -0.5)">
                <switch>
                    <foreignObject style="overflow: visible; text-align: left;" pointer-events="none" width="100%" height="100%" requiredFeatures="http://www.w3.org/TR/SVG11/feature#Extensibility">
                        <div xmlns="http://www.w3.org/1999/xhtml" style="display: flex; align-items: unsafe center; justify-content: unsafe center; width: 118px; height: 1px; padding-top: 110px; margin-left: 401px;">
                            <div style="box-sizing: border-box; font-size: 0; text-align: center; color: #000000; ">
                                <div style="display: inline-block; font-size: 12px; font-family: &quot;Helvetica&quot;; color: light-dark(#000000, #ffffff); line-height: 1.2; pointer-events: all; font-weight: bold; white-space: normal; word-wrap: normal; ">
                                    .drawio.svg
                                </div>
                            </div>
                        </div>
                    </foreignObject>
                    <text x="460" y="114" fill="light-dark(#000000, #ffffff)" font-family="&quot;Helvetica&quot;" font-size="12px" text-anchor="middle" font-weight="bold">
                        .drawio.svg
                    </text>
                </switch>
            </g>
        </g>
        <g>
            <rect x="600" y="80" width="120" height="60" rx="9" ry="9" fill="#f8cecc" stroke="#b85450" pointer-events="all" style="fill: light-dark(rgb(248, 206, 204), rgb(81, 45, 43)); stroke: light-dark(rgb(184, 84, 80), rgb(215, 129, 126));"/>
        </g>
        <g>
            <g transform="translate(-0.5 -0.5)">
                <switch>
                    <foreignObject style="overflow: visible; text-align: left;" pointer-events="none" width="100%" height="100%" requiredFeatures="http://www.w3.org/TR/SVG11/feature#Extensibility">
                        <div xmlns="http://www.w3.org/1999/xhtml" style="display: flex; align-items: unsafe center; justify-content: unsafe center; width: 118px; height: 1px; padding-top: 110px; margin-left: 601px;">
                            <div style="box-sizing: border-box; font-size: 0; text-align: center; color: #000000; ">
                                <div style="display: inline-block; font-size: 12px; font-family: &quot;Helvetica&quot;; color: light-dark(#000000, #ffffff); line-height: 1.2; pointer-events: all; font-weight: bold; white-space: normal; word-wrap: normal; ">
                                    人間による編集
                                </div>
                            </div>
                        </div>
                    </foreignObject>
                    <text x="660" y="114" fill="light-dark(#000000, #ffffff)" font-family="&quot;Helvetica&quot;" font-size="12px" text-anchor="middle" font-weight="bold">
                        人間による編集
                    </text>
                </switch>
            </g>
        </g>
        <g>
            <path d="M 120 110 L 191.76 110" fill="none" stroke="#000000" stroke-width="2" stroke-miterlimit="10" pointer-events="stroke" style="stroke: light-dark(rgb(0, 0, 0), rgb(255, 255, 255));"/>
            <path d="M 197.76 110 L 189.76 114 L 191.76 110 L 189.76 106 Z" fill="#000000" stroke="#000000" stroke-width="2" stroke-miterlimit="10" pointer-events="all" style="fill: light-dark(rgb(0, 0, 0), rgb(255, 255, 255)); stroke: light-dark(rgb(0, 0, 0), rgb(255, 255, 255));"/>
        </g>
        <g>
            <g transform="translate(-0.5 -0.5)">
                <switch>
                    <foreignObject style="overflow: visible; text-align: left;" pointer-events="none" width="100%" height="100%" requiredFeatures="http://www.w3.org/TR/SVG11/feature#Extensibility">
                        <div xmlns="http://www.w3.org/1999/xhtml" style="display: flex; align-items: unsafe center; justify-content: unsafe center; width: 1px; height: 1px; padding-top: 110px; margin-left: 160px;">
                            <div style="box-sizing: border-box; font-size: 0; text-align: center; color: #000000; background-color: #ffffff; ">
                                <div style="display: inline-block; font-size: 10px; font-family: &quot;Helvetica&quot;; color: light-dark(#000000, #ffffff); line-height: 1.2; pointer-events: all; background-color: light-dark(#ffffff, var(--ge-dark-color, #121212)); white-space: nowrap; ">
                                    参照
                                </div>
                            </div>
                        </div>
                    </foreignObject>
                    <text x="160" y="113" fill="light-dark(#000000, #ffffff)" font-family="&quot;Helvetica&quot;" font-size="10px" text-anchor="middle">
                        参照
                    </text>
                </switch>
            </g>
        </g>
        <g>
            <path d="M 320 110 L 391.76 110" fill="none" stroke="#000000" stroke-width="2" stroke-miterlimit="10" pointer-events="stroke" style="stroke: light-dark(rgb(0, 0, 0), rgb(255, 255, 255));"/>
            <path d="M 397.76 110 L 389.76 114 L 391.76 110 L 389.76 106 Z" fill="#000000" stroke="#000000" stroke-width="2" stroke-miterlimit="10" pointer-events="all" style="fill: light-dark(rgb(0, 0, 0), rgb(255, 255, 255)); stroke: light-dark(rgb(0, 0, 0), rgb(255, 255, 255));"/>
        </g>
        <g>
            <g transform="translate(-0.5 -0.5)">
                <switch>
                    <foreignObject style="overflow: visible; text-align: left;" pointer-events="none" width="100%" height="100%" requiredFeatures="http://www.w3.org/TR/SVG11/feature#Extensibility">
                        <div xmlns="http://www.w3.org/1999/xhtml" style="display: flex; align-items: unsafe center; justify-content: unsafe center; width: 1px; height: 1px; padding-top: 110px; margin-left: 360px;">
                            <div style="box-sizing: border-box; font-size: 0; text-align: center; color: #000000; background-color: #ffffff; ">
                                <div style="display: inline-block; font-size: 10px; font-family: &quot;Helvetica&quot;; color: light-dark(#000000, #ffffff); line-height: 1.2; pointer-events: all; background-color: light-dark(#ffffff, var(--ge-dark-color, #121212)); white-space: nowrap; ">
                                    生成
                                </div>
                            </div>
                        </div>
                    </foreignObject>
                    <text x="360" y="113" fill="light-dark(#000000, #ffffff)" font-family="&quot;Helvetica&quot;" font-size="10px" text-anchor="middle">
                        生成
                    </text>
                </switch>
            </g>
        </g>
        <g>
            <path d="M 520 110 L 591.76 110" fill="none" stroke="#000000" stroke-width="2" stroke-miterlimit="10" pointer-events="stroke" style="stroke: light-dark(rgb(0, 0, 0), rgb(255, 255, 255));"/>
            <path d="M 597.76 110 L 589.76 114 L 591.76 110 L 589.76 106 Z" fill="#000000" stroke="#000000" stroke-width="2" stroke-miterlimit="10" pointer-events="all" style="fill: light-dark(rgb(0, 0, 0), rgb(255, 255, 255)); stroke: light-dark(rgb(0, 0, 0), rgb(255, 255, 255));"/>
        </g>
        <g>
            <g transform="translate(-0.5 -0.5)">
                <switch>
                    <foreignObject style="overflow: visible; text-align: left;" pointer-events="none" width="100%" height="100%" requiredFeatures="http://www.w3.org/TR/SVG11/feature#Extensibility">
                        <div xmlns="http://www.w3.org/1999/xhtml" style="display: flex; align-items: unsafe center; justify-content: unsafe center; width: 1px; height: 1px; padding-top: 110px; margin-left: 560px;">
                            <div style="box-sizing: border-box; font-size: 0; text-align: center; color: #000000; background-color: #ffffff; ">
                                <div style="display: inline-block; font-size: 10px; font-family: &quot;Helvetica&quot;; color: light-dark(#000000, #ffffff); line-height: 1.2; pointer-events: all; background-color: light-dark(#ffffff, var(--ge-dark-color, #121212)); white-space: nowrap; ">
                                    編集
                                </div>
                            </div>
                        </div>
                    </foreignObject>
                    <text x="560" y="113" fill="light-dark(#000000, #ffffff)" font-family="&quot;Helvetica&quot;" font-size="10px" text-anchor="middle">
                        編集
                    </text>
                </switch>
            </g>
        </g>
        <g>
            <rect x="160" y="160" width="400" height="30" fill="none" stroke="none" pointer-events="all"/>
        </g>
        <g>
            <g transform="translate(-0.5 -0.5)">
                <switch>
                    <foreignObject style="overflow: visible; text-align: left;" pointer-events="none" width="100%" height="100%" requiredFeatures="http://www.w3.org/TR/SVG11/feature#Extensibility">
                        <div xmlns="http://www.w3.org/1999/xhtml" style="display: flex; align-items: unsafe center; justify-content: unsafe center; width: 398px; height: 1px; padding-top: 175px; margin-left: 161px;">
                            <div style="box-sizing: border-box; font-size: 0; text-align: center; color: #666666; ">
                                <div style="display: inline-block; font-size: 11px; font-family: &quot;Helvetica&quot;; color: light-dark(#666666, #959595); line-height: 1.2; pointer-events: all; white-space: normal; word-wrap: normal; ">
                                    ポイント：生成された図をDrawIOで開いて編集できる
                                </div>
                            </div>
                        </div>
                    </foreignObject>
                    <text x="360" y="178" fill="#666666" font-family="&quot;Helvetica&quot;" font-size="11px" text-anchor="middle">
                        ポイント：生成された図をDrawIOで開いて編集できる
                    </text>
                </switch>
            </g>
        </g>
    </g>
    <switch>
        <g requiredFeatures="http://www.w3.org/TR/SVG11/feature#Extensibility"/>
        <a transform="translate(0,-5)" xlink:href="https://www.drawio.com/doc/faq/svg-export-text-problems" target="_blank">
            <text text-anchor="middle" font-size="10px" x="50%" y="100%">
                Text is not SVG - cannot display
            </text>
        </a>
    </switch>
</svg>
```
</details>

svgだとQiitaやslackにそのままコピペはできないので，本当は`.drawio.png`の方が好きなのですが...トレードオフですね．良いやり方をご存知の方がいらしたらコメントください

## 導入方法

では，具体的な導入方法を見ていきましょう．

### 1. AGENTS.mdにルールを記載する

Coding Agentがドキュメントを作成する際に参照するルールファイル（`AGENTS.md`や`CLAUDE.md`など）に，DrawIO図の作成ルールを明記します．

以下は実際に筆者のプロジェクトで使用しているルールの抜粋です．

```markdown

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
```


### 2. テンプレートを提供する

Coding Agentが適切な形式で図を生成できるよう，汎用的なテンプレートを用意します．
これがあると，Agentが迷わずに正しい形式で出力してくれやすくなります．

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

### 3. 基本的な図形要素のサンプルを示す

Coding Agentが適切な図形を生成できるように，よく使う要素のサンプルを提示しておくと効果的らしいです．

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

### 4. アスキーアートを禁止する

アスキーアートを明確に禁止しておくとさらに効果的なようです．

```markdown
## ドキュメント類の作成ルール
- 図は`drawio.svg`形式のファイル名に`drawio`の規格に準じたxmlタグを格納することで，独立して作成すること．
- 図をmarkdownドキュメント内にアスキーアートで描くことは行なってはいけない．二重管理の状態を避けるために厳守する．
```

禁止を明記しないと，Coding Agentはお手軽なアスキーアートに逃げがちな気がします．

## 運用上の工夫

### ファイル命名規則

図ファイルは主ドキュメントと同じディレクトリに配置し，一貫した命名規則を適用します．

```
doc/experiments/exp001_slider_control/
├── exp001_slider_control.md                        # 主ドキュメント
├── exp001_slider_control_architecture.drawio.svg   # アーキテクチャ図
└── exp001_slider_control_sequence.drawio.svg       # シーケンス図
```

### 参照方法

Markdownからの参照は相対パスで行います．

```markdown
![システムアーキテクチャ](./exp001_slider_control_architecture.drawio.svg)
```

### 検証ポイント

> **注記**: 以下は記事初版（v1）のアプローチです。最新版では `scripts/drawio_svg.py validate` コマンドで自動検証できます（冒頭の更新セクション参照）。

生成された図が正しいか確認させるために，以下のチェックリストもルールと一緒に書いておくと安心です．

```markdown
1. **ファイル形式**: `cat`コマンドで`<mxfile>`タグから始まるか確認
2. **編集可能性**: DrawIOアプリで開けるか確認
3. **表示可能性**: Markdownプレビューで画像として表示されるか確認
```

## まとめ

今回の方法を導入することで，以下のメリットが得られました．

1. 編集効率の向上: Coding Agentが生成した図をベースに，人間が微調整できる
1. 二重管理の解消: 編集用ソースと表示用画像が同一ファイル
1. 一貫性の確保: ルールを明文化することで，図の品質が安定する

これにより，Coding Agentが生成するドキュメントの図に関するQoLが非常に上がります．

## おわりに

本稿ではCoding Agentに編集可能な図を生成させるため`.drawio.svg`形式の活用方法を紹介しました．Vive CodingによるQoL向上が得られれば幸いです．それでは．

---

## Appendix: AGENTS.mdにおけるドキュメント作成ルールの全文（v1）

> **注記**: 以下は記事初版（v1）時点のAGENTS.mdルールです。現在は `.claude/skills/drawio-svg/` スキルとして再構成されています（冒頭の更新セクション参照）。v1では `.drawio.svg` ファイルの中身を `<mxfile>` のみで構成していましたが、現在は `<svg>` ラッパー + content属性に埋め込んだmxfile XMLの二層構造を採用しています。

以下は，筆者のプロジェクトで実際に使用していた`AGENTS.md`からドキュメント作成に関連する箇所をそのまま抜粋したものです．

```markdown

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
```

https://gist.github.com/kim-xps12/4db5059b3aef1371ee74560c33dc24be