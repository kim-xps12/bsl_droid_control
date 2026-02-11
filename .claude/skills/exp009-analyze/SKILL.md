---
name: exp009-analyze
description: 物理シミュレータ内でのロボットの挙動を詳細に分析し，評価する
argument-hint: "[version-number] [optional: observation-text]"
---

# EXP009 分析・評価ワークフロー

V$ARGUMENTS の物理シミュレータ上での挙動を詳細に分析し、レポートの考察セクションを更新する。

## 必読ドキュメント

作業開始前に以下を必ず読むこと:
1. `doc/experiments/exp009_droid_rl_walking_omni/exp009_rules.md`
2. `doc/experiments/exp009_droid_rl_walking_omni/exp009_commands.md`（分析スクリプトの使い方）

## Step 0: 引数解析と所感取得

`$ARGUMENTS` を以下のルールで解析する:

1. 先頭トークンをバージョン番号 **N** として取得
2. 残りのテキストがあれば、それを **人間の目視所感** として使用
3. テキストがない場合、`doc/experiments/exp009_droid_rl_walking_omni/visual_obs_v{N}.md` を探す
4. ファイルも存在しない場合 → AskUserQuestion で所感を入力させる

前バージョン番号 **N-1** も算出する。

## Step 1: データ収集

以下のコマンドを `cd rl_ws` の上で実行する。独立コマンドは並列実行可。
**注意**: `PYTHONUNBUFFERED=1` は `settings.json` で自動適用されるため、明示的な付与は不要。

### 1.1 基本データ収集（CSV評価が未実施の場合のみ）

eval CSVファイルが `logs/droid-walking-omni-v{N}/eval_*.csv` に存在するか確認し、なければ実行:

```bash
# V{N} CSV評価
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 10 --csv
# V{N-1} CSV評価
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N-1} --no-viewer --duration 10 --csv
```

### 1.1b Yaw信頼性評価（前進方向・固定コマンド、必須）

ランダムコマンド評価（上記1.1）では **4秒ごとのコマンド切替がYawドリフトを隠蔽** するため、
Yaw評価には固定コマンド評価が必須。`eval_*_cmd_0.30_0.00_0.00_s1.csv` が存在しなければ実行:

```bash
# V{N} 前進固定コマンド（20秒）
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.3 0.0 0.0
# V{N-1} 前進固定コマンド（20秒）
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N-1} --no-viewer --duration 20 --csv --command 0.3 0.0 0.0
```

**重要**: Yaw改善率はこの固定コマンド評価データのみから算出すること。ランダムコマンド評価からのYaw改善率は、コマンド切替による隠蔽のため信頼できない。

### 1.2 分析スクリプト実行（全て実行）

```bash
# 報酬コンポーネント比較
uv run python scripts/show_reward_components.py droid-walking-omni-v{N-1} droid-walking-omni-v{N}

# CSV時系列分析
uv run python scripts/analyze_eval_csv.py {N} {N-1} --prefix droid-walking-omni-v

# 横方向揺れ詳細分析
uv run python scripts/analyze_lateral_sway.py {N} {N-1} --prefix droid-walking-omni-v

# 振動・高周波分析（目視で振動が指摘された場合に特に有用）
uv run python scripts/analyze_vibration.py {N} {N-1} --prefix droid-walking-omni-v
```

### 1.3 追加分析（所感に応じて選択）

人間の所感に基づき、以下から必要な追加分析を選択実行する:

- **方向別問題が指摘された場合**: 方向別評価CSV（既存なら読み込み）を用いた方向別分析
- **Yaw問題の深掘り**: Step 1.1b の固定コマンドCSVに対して `analyze_eval_csv.py` で時系列分析。
  `analyze_yaw_drift_v2.py` 系はexp008固有（パスがハードコード）のため exp009 では使用しない
- **接触問題**: `uv run python scripts/analyze_contact_chattering_v2.py` 系

## Step 2: 二者対話による多角的分析（Sequential Task 方式）

2つの専門視点（DA: データ分析 / Lead: 物理解釈）による対話を、リーダーが仲介する **Sequential Task パターン** で実施する。
各ラウンドは独立した Task（サブエージェント）呼び出しで実行し、リーダーが結果を次の Task に明示的に受け渡す。

### 2.1 対話フロー概要

| ラウンド | 役割 | 入力 | 出力 |
|---------|------|------|------|
| Round 1 | DA | 全分析データ + 所感 | DA_R1: 定量分析レポート |
| Round 2 | Lead | DA_R1 + 参照ドキュメント + 所感 | LEAD_R1: 物理解釈 + 追加データ要求 |
| Round 3 | DA | DA_R1 + LEAD_R1 + 元データ | DA_R2: 仮説検証データ |
| Round 4 | Lead | DA_R1 + LEAD_R1 + DA_R2 | LEAD_FINAL: 統合考察・改善案 |

### 2.2 Round 1: DA 初期定量分析

Task ツールで実行（結果を **DA_R1** として保持）:

```
subagent_type: general-purpose
name: da-round1
```

**プロンプト**:

> あなたは「目の前にあるデータ分析を精密に行う professional assistance agent」です。
> 名前: **DA**
>
> ## あなたの役割
> - 収集された定量データを精密に読み解き、異常値・変化・相関を特定する
> - 主観的解釈は避け、数値に基づく客観的な分析に徹する
>
> ## 入力データ
> {Step 1 で収集した全分析結果をここに貼り付ける}
>
> ## 人間の目視所感
> {所感テキストをここに貼り付ける}
>
> ## 指示
> 以下を実行し、結果をテキストで出力せよ（ファイル書き込み不要）:
> 1. 収集データの中から V{N-1}→V{N} で最も顕著な変化・異常を **3-5項目** 特定し、具体的数値とともに報告
> 2. 人間の所感と定量データの対応関係を分析（所感を裏付ける/矛盾するデータを明示）
> 3. 注目すべき相関・トレードオフがあれば指摘
> 4. **Yaw指標の信頼性チェック**: ランダムコマンド評価のYaw指標にコマンド切替の影響がないか確認。
>    Yaw改善率の算出にはランダムコマンド評価データを使用してはならず、固定コマンド評価データのみを使用すること
>
> **出力形式**: 150-300字程度の簡潔な分析レポート。箇条書き推奨。

### 2.3 Round 2: Lead 物理解釈・仮説構築

Task ツールで実行（結果を **LEAD_R1** として保持）:

```
subagent_type: general-purpose
name: lead-round1
```

**プロンプト**:

> あなたは「ロボティクス・物理学に精通した全体を俯瞰できる lead agent」です。
> 名前: **Lead**
>
> ## あなたの役割
> - DA の定量分析結果を物理的・力学的メカニズムで解釈する
> - 因果関係の特定、仮説の構築を主導する
> - 過去の知見（exp007/exp008 の教訓）との整合性を確認する
>
> ## 参照すべきドキュメント
> 以下を読み込んで過去の知見を把握すること:
> - `~/.claude/projects/-Users-yutaro-kimura-bsl-droid-control/memory/MEMORY.md`
> - `doc/experiments/exp009_droid_rl_walking_omni/exp009_rules.md`
> - 必要に応じて `doc/experiments/exp007_droid_rl_walking_ref_unitree/exp007_unitree_rl_gym_survey.md`
>
> ## DA の定量分析結果
> {DA_R1 をここに貼り付ける}
>
> ## 人間の目視所感
> {所感テキストをここに貼り付ける}
>
> ## 指示
> 以下を実行し、結果をテキストで出力せよ（ファイル書き込み不要）:
> 1. DA の報告を物理的・力学的メカニズムに基づき解釈する
>    - 因果連鎖の特定（A → B → C → 観察された現象）
>    - exp008 の類似パターンとの照合（特にカスケード悪化パターン）
>    - penalty/positive ratio が健全域か overkill 域かの判定
> 2. 仮説を提示し、その検証に必要な **追加データ・観点** を DA に要求する形で記述
>
> **出力形式**: 150-300字程度。仮説 → 根拠 → 追加データ要求の構造で記述。

### 2.4 Round 3: DA 仮説検証データ提示

Task ツールで実行（結果を **DA_R2** として保持）:

```
subagent_type: general-purpose
name: da-round2
```

**プロンプト**:

> あなたは DA（データ分析エージェント）です。
>
> ## これまでの対話
> ### DA 初期分析（Round 1）
> {DA_R1 をここに貼り付ける}
>
> ### Lead 物理解釈（Round 2）
> {LEAD_R1 をここに貼り付ける}
>
> ## 元データ
> {Step 1 で収集した全分析結果をここに貼り付ける}
>
> ## 指示
> Lead が要求した追加データ・観点について、元データから該当部分を抽出・再分析し回答せよ。
> Lead の仮説を **裏付ける/反証する** 具体的数値を提示すること。
>
> **出力形式**: 150-300字程度。要求された項目ごとにデータで回答。

### 2.5 Round 4: Lead 統合考察・改善案

Task ツールで実行（結果を **LEAD_FINAL** として保持）:

```
subagent_type: general-purpose
name: lead-final
```

**プロンプト**:

> あなたは Lead（ロボティクス・物理学俯瞰エージェント）です。
>
> ## これまでの対話
> ### DA 初期分析（Round 1）
> {DA_R1 をここに貼り付ける}
>
> ### Lead 物理解釈（Round 2）
> {LEAD_R1 をここに貼り付ける}
>
> ### DA 仮説検証データ（Round 3）
> {DA_R2 をここに貼り付ける}
>
> ## 参照すべきドキュメント
> 以下を読み込んで過去の知見を把握すること:
> - `~/.claude/projects/-Users-yutaro-kimura-bsl-droid-control/memory/MEMORY.md`
> - `doc/experiments/exp009_droid_rl_walking_omni/exp009_rules.md`
>
> ## 人間の目視所感
> {所感テキストをここに貼り付ける}
>
> ## 指示
> 全ラウンドの対話を踏まえ、以下を含む **最終統合考察** を出力せよ（ファイル書き込み不要）:
> 1. **悪化メカニズムの因果関係**: 根本原因 → 中間過程 → 観察された現象
> 2. **成功点の特定と理由**: 改善が見られた項目とそのメカニズム
> 3. **改善案**（1変更1検証原則を遵守）: 各案に期待効果・リスク・根拠を明記
> 4. **推奨案の決定**: 最も優先すべき1案を選定し理由を述べる
>
> **出力形式**: 構造化テキスト。各セクションを見出しで区切る。

### 2.6 リーダーによる確認

全 Round 完了後、リーダーは DA_R1, LEAD_R1, DA_R2, LEAD_FINAL の内容を確認し、
必要に応じて追加の分析スクリプト実行や補足 Round の Task を実行する。

## Step 3: レポート更新

既存の `doc/experiments/exp009_droid_rl_walking_omni/exp009_report_v{N}.md` の**考察セクション**を更新する。

### 更新内容

#### 3.1 「目視評価所見」サブセクション
人間の所感を事実として記載する。

```markdown
### 目視評価所見

目視評価において以下が観察された:
- {所感の内容を箇条書きで整理}
```

#### 3.2 「対話による多角的分析」サブセクション
DA と Lead の各 Round 結果（DA_R1, LEAD_R1, DA_R2, LEAD_FINAL）を以下の形式で記載する:

```markdown
### 対話による多角的分析

以下、データ分析専門エージェント（DA）とロボティクス・物理学俯瞰エージェント（Lead）の対話形式で考察を行う。

---

**DA（Round 1: 初期定量分析）**: {DA_R1 の内容}

**Lead（Round 2: 物理解釈・仮説構築）**: {LEAD_R1 の内容}

**DA（Round 3: 仮説検証データ）**: {DA_R2 の内容}

**Lead（Round 4: 統合考察・改善案）**: {LEAD_FINAL の内容}

---
```

#### 3.3 「成功点」「課題」「次バージョンへの提案」
対話の結論に基づいて更新する。

### レポート更新のルール
- 結果セクションは変更しない（事実のみが記載済み）
- 考察セクションのみを更新対象とする
- 期待・目標の記述は断定表現を避ける
- 「次バージョンへの提案」は1変更1検証原則を遵守
