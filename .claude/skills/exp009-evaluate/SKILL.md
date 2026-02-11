---
name: exp009-evaluate
description: 収集済みデータに基づくエージェント対話分析とレポート更新
argument-hint: "[version-number] [optional: observation-text]"
---

# EXP009 評価・レポートワークフロー

V$ARGUMENTS の収集済みデータに基づき、二者対話による多角的分析を行い、レポートを更新する。
事前に `/exp009-collect` でデータ収集が完了していること。

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

## スクリプト利用ルール（全Stepに適用）

追加分析やデータ加工でpythonスクリプトを使いたい場合:

1. まず `exp009_commands.md` Section 5 から用途に合うスクリプトを探す
2. 適したスクリプトがあればそれを使用する
3. ない場合のみ `scripts/` 直下に汎用的な引数インターフェースで新規作成する

## Step 1: 分析データ取得

`/exp009-collect` で生成済みの CSV を入力として、分析スクリプトを再実行し出力を取得する。
これらのスクリプトは CSV を読むだけなので高速に完了する。

以下のコマンドを `cd rl_ws` の上で実行する。独立コマンドは並列実行可。
**注意**: `PYTHONUNBUFFERED=1` は `settings.json` で自動適用されるため、明示的な付与は不要。

### 1.1 前提チェック

V{N} のトレーニングスクリプト `biped_walking/train/droid_train_omni_v{N}.py` から `lin_vel_x_range` と `lin_vel_y_range` の上限値（**vx_max**, **vy_max**）を取得する。

以下のファイルが存在することを確認する。なければ `/exp009-collect` の実行を促す:

- `logs/droid-walking-omni-v{N}/eval_*.csv`（ランダムコマンド）
- 4方向 × 3 seed の固定コマンドCSV（計12ファイル。ファイル名例: `eval_*_cmd_{vx_max:.2f}_0.00_0.00_s1.csv`）
- V{N-1} の同等ファイル（最低限ランダムコマンド + FWD固定コマンド）

### 1.2 分析スクリプト実行

```bash
# 報酬コンポーネント比較
uv run python scripts/show_reward_components.py droid-walking-omni-v{N-1} droid-walking-omni-v{N}

# CSV時系列分析
uv run python scripts/analyze_eval_csv.py {N} {N-1} --prefix droid-walking-omni-v

# 横方向揺れ詳細分析
uv run python scripts/analyze_lateral_sway.py {N} {N-1} --prefix droid-walking-omni-v

# 振動・高周波分析
uv run python scripts/analyze_vibration.py {N} {N-1} --prefix droid-walking-omni-v
```

各スクリプトの出力を保持し、Step 2 の対話プロンプトで使用する。

### 1.3 結果セクション記入

レポートの**結果セクション**（考察セクションより上）に空欄テーブルがあれば、収集済みデータから事実を記入する。

#### 1.3a 訓練結果テーブル

```bash
uv run python scripts/analyze_training_log.py droid-walking-omni-v{N}
```

出力から最終報酬・ピーク報酬・エピソード長・Last 50 std を抽出し、テーブルに記入する。
V{N-1} の値も同様に取得して変化率を算出する。

#### 1.3b ランダムコマンド評価結果

`eval_*.csv`（ランダムコマンド）から主要指標を抽出して記入する。
具体的な指標はレポートのテーブル定義に従う。

#### 1.3c 方向別評価結果テーブル

4方向 × 3 seed の固定コマンドCSVから以下を算出し記入する:

各方向について 3 試行（seed 1-3）の mean ± std:
- **コマンド方向速度**: コマンド方向の平均速度（定常状態 t>2s）
- **直交方向速度**: コマンド直交方向の平均速度
- **Yawドリフト**: 20秒間の最終Yaw角
- **Roll std**: Roll角の標準偏差
- **hip_pitch相関**: L/R hip_pitch の相関係数
- **追従率**: コマンド方向速度 / コマンド速度

CSVの読み取りと集計には既存スクリプトまたは直接のCSV読み込みを使用する。

#### 1.3d 方向間バランステーブル

1.3c の追従率から算出:
- 追従率 mean（4方向の平均）
- 追従率 std（4方向のばらつき）
- FWD/BWD追従率差
- LFT/RGT追従率差

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
> {Step 1 で取得した全分析結果をここに貼り付ける}
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
> {Step 1 で取得した全分析結果をここに貼り付ける}
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
- 結果セクションの空欄テーブルは Step 1.3 で事実データを記入する（既に記入済みの値は変更しない）
- 考察セクションは Step 3 で更新する
- 期待・目標の記述は断定表現を避ける
- 「次バージョンへの提案」は1変更1検証原則を遵守
