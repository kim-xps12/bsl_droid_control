---
name: exp008-new-version
description: Create a new exp008 training version with full E2E workflow. Uses agent teams for parallel data collection in Phase 3 analysis.
disable-model-invocation: true
argument-hint: "[version-number]"
---

# EXP008 新バージョン作成ワークフロー

バージョン V$ARGUMENTS を作成し、E2E ワークフローを実行する。

## フロー概要

```
Phase 1: 準備（単独）
    ↓
Phase 2: 訓練（単独・長時間）
    ↓
Phase 3: 訓練後分析（Agent Teams で並列化）
    ↓
Phase 4: 完了報告（単独）
```

## Phase 1: 準備

[phase1-prepare.md](phase1-prepare.md) のチェックリストに従い、以下を順次実行する:

1. 前バージョンのレポート `exp008_report_v{N-1}.md` を読み込み、「次バージョンへの提案」を確認
2. トレーニングスクリプトをコピー原則に従い作成
   - `cp biped_walking/train/droid_train_narrow_v{N-1}.py biped_walking/train/droid_train_narrow_v{N}.py`（`cd rl_ws` 済みの状態で実行。`&&` で連結しないこと）
3. 必要に応じて `droid_env_unitree.py` に報酬関数を追加
4. lint・型チェック（`AGENTS.md` コード品質ルール参照）
5. レポートスケルトン `exp008_report_v{N}.md` を [report-template.md](report-template.md) に従い作成
6. 主レポート `exp008_droid_rl_walking_narrow.md` のバージョン一覧を更新

## Phase 2: 訓練

以下の手順で訓練をバックグラウンド実行し、リアルタイム監視用の tmux ペインを作成する。

### 2.1 訓練のバックグラウンド起動

Bash ツールを `run_in_background: true` で実行する。確認なしで自動実行する。

`cd rl_ws` を先に別の Bash ツール呼び出しで実行してから（cwdは保持される）:
```bash
uv run python biped_walking/train/droid_train_narrow_v{N}.py --max_iterations 500
```

### 2.2 tmux 監視ペインの作成

訓練起動後、ツール結果に含まれる `output_file` パスを用いて、tmux 監視ペインを作成する。

tmux セッションが存在するか確認:
```bash
tmux list-sessions 2>/dev/null
```

**セッションが存在しない場合（exit code 1）**: 自分でセッションを作成する:
```bash
tmux new-session -d -s main
```

セッションが存在する状態で、監視ペインを作成:
```bash
tmux split-window -v -d -l 15 "tail -f {output_file}"
```
- `-v`: 下方向に分割
- `-d`: フォーカスを移動しない
- `-l 15`: 高さ15行
- `{output_file}` は 2.1 のツール結果から取得したパスに置換する

### 2.3 訓練完了後の後処理

訓練完了の通知を受けたら、監視ペインを閉じてから Phase 3 に進む。

**注意**: パイプを含む複合コマンドは `settings.json` の権限パターンでマッチしないため、2段階に分けて実行する:

```bash
# Step 1: tail を実行しているペインIDを確認
tmux list-panes -F '#{pane_id} #{pane_current_command}'
```

出力例: `%3 tail` → `%3` がターゲット

```bash
# Step 2: 該当ペインを閉じる
tmux kill-pane -t %3
```

失敗した場合（ユーザが既に閉じた等）は無視してよい。

## Phase 3: 訓練後分析（Agent Teams）

[phase3-analyze.md](phase3-analyze.md) のフローに従い、Agent Teams を活用してデータ収集を並列化する。

**重要**: Step D（独立考察）は省略禁止。データ収集から直接レポート執筆に進むことは禁止。
考察すべき項目:
1. 動作悪化メカニズムの因果関係の特定
2. 報酬設計全体の体系的整理（寄与度ランキング・報酬バランス・削減候補）
3. 複数改善案の設計と評価（期待効果・リスク・理論的根拠）
4. 推奨案の決定（1変更1検証の原則を遵守）

## Phase 4: 完了報告

- 結果サマリをユーザに提示
- 次バージョンの推奨案を提示
