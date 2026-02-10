---
name: exp007-new-version
description: Create a new exp007 training version with full E2E workflow (prepare, train, analyze, report)
disable-model-invocation: true
argument-hint: "[version-number]"
---

# EXP007 新バージョン作成ワークフロー

バージョン V$ARGUMENTS を作成し、E2E ワークフローを実行する。

## 事前準備

以下のドキュメントを読み込む:

1. `doc/experiments/exp007_droid_rl_walking_ref_unitree/exp007_rules.md` — バージョン管理原則
2. `doc/experiments/exp007_droid_rl_walking_ref_unitree/exp007_workflow.md` — 詳細手順・レポートテンプレート
3. `doc/experiments/exp007_droid_rl_walking_ref_unitree/exp007_reward_design.md` — 報酬設計原則

## E2E ワークフロー

`exp007_workflow.md` に記載された以下のフローを順次実行する:

### Phase 1: 準備

1. 前バージョンのレポート `exp007_report_v{N-1}.md` を読み込み、「次バージョンへの提案」を確認
2. トレーニングスクリプトをコピー原則に従い作成（`exp007_rules.md` Section 1.3）
3. 必要に応じて `droid_env_unitree.py` に報酬関数を追加（`exp007_rules.md` Section 1.1, 1.2）
4. lint・型チェック（`AGENTS.md` コード品質ルール参照）
5. レポートスケルトン `exp007_report_v{N}.md` を `exp007_workflow.md` Section 1 のテンプレートに従い作成
6. 主レポート `exp007_droid_rl_walking_ref_unitree.md` のバージョン一覧を更新

### Phase 2: 訓練

以下の手順で訓練をバックグラウンド実行し、リアルタイム監視用の tmux ペインを作成する。

#### 2.1 訓練のバックグラウンド起動

Bash ツールを `run_in_background: true` で実行する。

`cd rl_ws` を先に別の Bash ツール呼び出しで実行してから（cwdは保持される）:
```bash
uv run python biped_walking/train/droid_train_unitree_v{N}.py --max_iterations 500
```

#### 2.2 tmux 監視ペインの作成

訓練起動後、ツール結果に含まれる `output_file` パスを用いて、tmux 監視ペインを作成する。

tmux が利用可能か確認:
```bash
tmux list-sessions 2>/dev/null
```

利用可能な場合（exit code 0）:
```bash
tmux split-window -v -d -l 15 "tail -f {output_file}"
```
- `-v`: 下方向に分割
- `-d`: フォーカスを移動しない
- `-l 15`: 高さ15行
- `{output_file}` は 2.1 のツール結果から取得したパスに置換する

利用不可の場合、以下のコマンドをユーザに提示する:
```
tail -f {output_file}
```

#### 2.3 訓練完了後の後処理

訓練完了の通知を受けたら、監視ペインを閉じてから Phase 3 に進む:
```bash
tmux list-panes -F '#{pane_id} #{pane_current_command}' | grep tail | awk '{print $1}' | xargs -I{} tmux kill-pane -t {}
```
失敗した場合（ユーザが既に閉じた等）は無視してよい。

### Phase 3: 訓練後分析

`exp007_workflow.md` Section 3 のフローに従い、以下を順次実行:

- **Step A**: データ収集（評価・ログ分析・CSV分析・報酬コンポーネント比較）
- **Step B**: 定量分析（比較表作成・報酬バランス評価）
- **Step C**: サーベイ知見との照合（`exp007_unitree_rl_gym_survey.md`）
- **Step D**: 独立考察（因果分析・改善案設計）【最重要、省略禁止】
- **Step E**: レポート執筆（結果=事実のみ、考察=分析）

コマンドの詳細は `doc/experiments/exp007_droid_rl_walking_ref_unitree/exp007_commands.md` を参照。

### Phase 4: 完了報告

- 結果サマリをユーザに提示
- 次バージョンの推奨案を提示
