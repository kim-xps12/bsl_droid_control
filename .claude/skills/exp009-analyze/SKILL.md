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
- **Yaw問題**: `uv run python scripts/analyze_yaw_drift_v2.py` 系
- **接触問題**: `uv run python scripts/analyze_contact_chattering_v2.py` 系

## Step 2: 二者対話による多角的分析

Agent Teams を作成し、2つの専門エージェントの対話によって多角的な考察を実施する。

### 2.1 チーム作成

```
TeamCreate: team_name="exp009-analyze-v{N}"
```

### 2.2 タスク作成

| タスクID | タスク内容 | 担当 | 依存 |
|---------|----------|------|------|
| 1 | 初期定量分析 | data-analyst | - |
| 2 | 物理解釈・仮説構築 | lead-physicist | 1 |
| 3 | 仮説検証データ提示 | data-analyst | 2 |
| 4 | 統合考察・改善案 | lead-physicist | 3 |
| 5 | レポート更新 | リーダー | 4 |

### 2.3 data-analyst の起動

```
subagent_type: general-purpose
team_name: exp009-analyze-v{N}
name: data-analyst
```

**data-analyst へのプロンプト**:

> あなたは「目の前にあるデータ分析を精密に行う professional assistance agent」です。
> 名前: **DA**
>
> ## あなたの役割
> - 収集された定量データを精密に読み解き、異常値・変化・相関を特定する
> - 主観的解釈は避け、数値に基づく客観的な分析に徹する
> - 対話相手の lead-physicist の質問やコメントに対して、データで裏付けた回答をする
>
> ## 入力データ
> {Step 1 で収集した全分析結果をここに貼り付ける}
>
> ## 人間の目視所感
> {所感テキストをここに貼り付ける}
>
> ## 指示
> 1. まず、収集データの中から最も顕著な変化・異常を3-5項目特定し、定量的に報告せよ
> 2. 人間の所感と定量データの対応関係を分析せよ
> 3. 分析結果を SendMessage で lead-physicist に送信せよ
> 4. lead-physicist からの質問・コメントにはデータで回答せよ
> 5. 対話は2-3往復を目安とする。各発言は150-300字程度に収めよ
>
> **重要**: 全ての発言を SendMessage でリーダーにもCC（同内容を送信）すること。
> TaskList を確認し、該当タスクを完了状態に更新すること。

### 2.4 lead-physicist の起動

data-analyst の初期分析完了後に起動する。

```
subagent_type: general-purpose
team_name: exp009-analyze-v{N}
name: lead-physicist
```

**lead-physicist へのプロンプト**:

> あなたは「ロボティクス・物理学に精通した全体を俯瞰できる lead agent」です。
> 名前: **Lead**
>
> ## あなたの役割
> - data-analyst の定量分析結果を物理的・力学的メカニズムで解釈する
> - 因果関係の特定、仮説の構築、改善提案の設計を主導する
> - 過去の知見（exp007/exp008 の教訓）との整合性を確認する
>
> ## 参照すべきドキュメント
> 以下を読み込んで過去の知見を把握すること:
> - `/Users/yutaro.kimura/.claude/projects/-Users-yutaro-kimura-bsl-droid-control/memory/MEMORY.md`
> - `doc/experiments/exp009_droid_rl_walking_omni/exp009_rules.md`
> - 必要に応じて `doc/experiments/exp007_droid_rl_walking_ref_unitree/exp007_unitree_rl_gym_survey.md`
>
> ## 人間の目視所感
> {所感テキストをここに貼り付ける}
>
> ## 指示
> 1. data-analyst から受信した定量分析を読み込む
> 2. 物理的・力学的メカニズムに基づく解釈を提供する
>    - 因果連鎖の特定（A → B → C → 観察された現象）
>    - exp008 の類似パターンとの照合（特にカスケード悪化パターン）
>    - penalty/positive ratio が健全域か overkill 域かの判定
> 3. 解釈を SendMessage で data-analyst に送信し、追加データを要求する
> 4. 2-3往復の対話後、以下を含む最終統合を SendMessage でリーダーに送信:
>    - 悪化メカニズムの因果関係
>    - 成功点の特定と理由
>    - 改善案（1変更1検証原則を遵守、期待効果・リスク・根拠を明記）
>    - 推奨案の決定
>
> **重要**: 全ての発言を SendMessage でリーダーにもCC（同内容を送信）すること。
> TaskList を確認し、該当タスクを完了状態に更新すること。

### 2.5 対話の仲介

リーダーは対話を監視し、必要に応じて:
- 追加の分析スクリプト実行を指示
- 論点の整理・方向修正
- 対話が発散した場合の収束指示

### 2.6 チーム解散

全タスク完了後:
1. data-analyst と lead-physicist にシャットダウンリクエストを送信
2. `TeamDelete` でチームを解散

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
DA と Lead の対話を以下の形式で記載する:

```markdown
### 対話による多角的分析

以下、データ分析専門エージェント（DA）とロボティクス・物理学俯瞰エージェント（Lead）の対話形式で考察を行う。

---

**DA**: {DAの発言。定量データに基づく分析}

**Lead**: {Leadの発言。物理的解釈と仮説}

**DA**: {追加データの提示}

**Lead**: {統合考察と改善案}

---
```

#### 3.3 「成功点」「課題」「次バージョンへの提案」
対話の結論に基づいて更新する。

### レポート更新のルール
- 結果セクションは変更しない（事実のみが記載済み）
- 考察セクションのみを更新対象とする
- 期待・目標の記述は断定表現を避ける
- 「次バージョンへの提案」は1変更1検証原則を遵守
