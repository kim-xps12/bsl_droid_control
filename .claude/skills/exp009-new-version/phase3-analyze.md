# Phase 3: 訓練後分析フロー（Agent Teams）

訓練完了後、以下のフローで分析を実行する。Agent Teams を活用して Step A のデータ収集を並列化する。

## フロー

```
Step A: データ収集（Agent Teams で並列化）
    ↓
Step B: 定量分析（リーダーが実行）
    ↓
Step C: サーベイ知見との照合（Teammate に委任可能）
    ↓
Step D: 独立考察（リーダーが実行）【最重要・省略禁止】
    ↓
Step E: レポート執筆（リーダーが実行）
```

## Agent Teams セットアップ

### 1. チーム作成

```
TeamCreate: team_name="exp009-v{N}"
```

### 2. タスク作成

以下のタスクを `TaskCreate` で作成し、依存関係を設定する:

| タスクID | タスク内容 | 担当 | 依存 |
|---------|----------|------|------|
| 1 | Step A: 基本評価（ヘッドレス）実行 | data-collector | - |
| 2 | Step A: CSV評価（V{N}）実行 | data-collector | - |
| 3 | Step A: CSV評価（V{N-1}）実行 | data-collector | - |
| 4 | Step A: 訓練ログ分析実行 | data-collector | - |
| 5 | Step A: 報酬コンポーネント分析実行 | data-collector | - |
| 6 | Step A: CSV時系列分析実行 | data-collector | 2, 3 |
| 6b | Step A: 横方向揺れ詳細分析実行 | data-collector | 2, 3 |
| 6c | Step A: 方向別評価 FWD (3 trials) | data-collector | - |
| 6d | Step A: 方向別評価 BWD (3 trials) | data-collector | - |
| 6e | Step A: 方向別評価 LFT (3 trials) | data-collector | - |
| 6f | Step A: 方向別評価 RGT (3 trials) | data-collector | - |
| 7 | Step B: 定量分析（方向別集計含む） | リーダー | 1-6f |
| 8 | Step C: サーベイ照合 | survey-analyst | 7 |
| 9 | Step D: 独立考察 | リーダー | 7, 8 |
| 10 | Step E: レポート執筆 | リーダー | 9 |

### 3. Teammate "data-collector" の起動

`Task` ツールで general-purpose エージェントを起動:

```
subagent_type: general-purpose
team_name: exp009-v{N}
name: data-collector
```

**data-collector へのプロンプト**:

> exp009 V{N} の訓練後データ収集を実行する。以下のコマンドを全て `cd rl_ws` の上で実行し、結果を整理して報告せよ。
> 独立したコマンドは並列実行してよい。
> **注意**: `PYTHONUNBUFFERED=1` は `settings.json` の `env` セクションで全 Bash コマンドに自動適用されるため、コマンドへの明示的な付与は不要。
>
> 1. 基本評価: `uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 10`
> 2. CSV評価（V{N}）: `uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 10 --csv`
> 3. CSV評価（V{N-1}）: `uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N-1} --no-viewer --duration 10 --csv`
> 4. 訓練ログ分析: `uv run python scripts/analyze_training_log.py droid-walking-omni-v{N}`
> 5. 報酬コンポーネント分析: `uv run python scripts/show_reward_components.py droid-walking-omni-v{N-1} droid-walking-omni-v{N}`
> 6. CSV時系列分析（2,3 完了後）: `uv run python scripts/analyze_eval_csv.py {N} {N-1} --prefix droid-walking-omni-v`
> 7. 横方向揺れ詳細分析（2,3 完了後）: `uv run python scripts/analyze_lateral_sway.py {N} {N-1} --prefix droid-walking-omni-v`
> 8. 方向別評価 Forward（順次実行、各20秒）:
>    `uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.3 0.0 0.0 --seed 1`
>    `uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.3 0.0 0.0 --seed 2`
>    `uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.3 0.0 0.0 --seed 3`
> 9. 方向別評価 Backward（順次実行）:
>    `uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command -0.3 0.0 0.0 --seed 1`
>    `uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command -0.3 0.0 0.0 --seed 2`
>    `uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command -0.3 0.0 0.0 --seed 3`
> 10. 方向別評価 Left（順次実行）:
>    `uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.0 0.3 0.0 --seed 1`
>    `uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.0 0.3 0.0 --seed 2`
>    `uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.0 0.3 0.0 --seed 3`
> 11. 方向別評価 Right（順次実行）:
>    `uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.0 -0.3 0.0 --seed 1`
>    `uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.0 -0.3 0.0 --seed 2`
>    `uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.0 -0.3 0.0 --seed 3`
>
> **注意**: 方向別評価（8-11）はGPU占有のため順次実行すること。1-7の独立コマンドとは並列実行可能。
> 方向別評価の各コマンドのコンソール出力（X/Y速度、Yawドリフト、hip_pitch相関）を方向ごとに整理して報告すること。
>
> 完了後、全コマンドの出力を整理して SendMessage でリーダーに報告すること。
> TaskList を確認し、該当タスクを完了状態に更新すること。

### 4. Teammate "survey-analyst" の起動

data-collector 完了後・Step B 完了後に起動:

```
subagent_type: general-purpose
team_name: exp009-v{N}
name: survey-analyst
```

**survey-analyst へのプロンプト**:

> exp009 V{N} の実験結果について、サーベイ知見との照合（Step C）を実行する。
>
> 1. `doc/experiments/exp007_droid_rl_walking_ref_unitree/exp007_unitree_rl_gym_survey.md` を読み込む
> 2. リーダーから共有された Step B の定量分析結果と照合する
> 3. 実験結果がサーベイの理論的予測と一致するかを検証する
> 4. 照合結果を SendMessage でリーダーに報告すること。
> 5. TaskList を確認し、該当タスクを完了状態に更新すること。

## Step B: 定量分析（リーダー）

data-collector から受け取ったデータを用いて:

1. V{N-1}→V{N} の全指標比較表を作成
2. 報酬バランスの定量評価（正の報酬合計 vs ペナルティ絶対値合計の比率）
3. 報酬項目数のチェック（15-17項目の推奨範囲内か）
4. 方向別評価の集計:
   - 各方向3試行からの mean ± std を算出（X速度、Y速度、Yawドリフト、Roll std、hip_pitch相関）
   - コマンド方向速度追従率 = mean(コマンド方向実速度) / 0.3
   - 方向間バランス: 4方向の追従率のmean/std
   - FWD/BWD差、LFT/RGT差（方向非対称性の検出）

## Step D: 独立考察（リーダー）【最重要】

**このステップを省略してデータ収集から直接レポート執筆に進むことは禁止。**
**前バージョンのレポートの考察をそのまま流用することも禁止。ゼロベースで再考察すること。**

Step B（定量分析）と Step C（サーベイ照合）の結果を入力とし、以下を考察する:

1. **動作悪化メカニズムの因果関係の特定**: 報酬コンポーネント間の相互作用を分析
2. **報酬設計全体の体系的整理**: 寄与度ランキング、報酬バランス、削減候補の選定
3. **複数改善案の設計と評価**: 各案の期待効果・リスク・理論的根拠を明記
4. **推奨案の決定**: 1変更1検証の原則を遵守した段階的実施計画

## Step E: レポート執筆（リーダー）

Step A-D の成果を `report-template.md` のテンプレートに統合:
- 結果セクション: 事実のみ（Step A-B のデータ）
- 考察セクション: Step C-D の成果を反映
- 期待・目標の記述は断定表現を避ける

## チーム解散

全タスク完了後、`TeamDelete` でチームを解散する。
