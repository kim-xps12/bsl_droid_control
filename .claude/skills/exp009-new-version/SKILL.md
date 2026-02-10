---
name: exp009-new-version
description: Create a new exp009 training version: script generation, report skeleton, and training start.
disable-model-invocation: true
argument-hint: "[version-number]"
---

# EXP009 新バージョン作成ワークフロー

バージョン V$ARGUMENTS を作成し、訓練を開始する。

## フロー概要

```
Phase 1: 準備（単独）
    ↓
Phase 2: 訓練開始（単独）
    ↓
完了
```

## Phase 1: 準備

[phase1-prepare.md](phase1-prepare.md) のチェックリストに従い、以下を順次実行する:

1. 前バージョンのレポート `exp009_report_v{N-1}.md` を読み込み、「次バージョンへの提案」を確認
2. トレーニングスクリプトをコピー原則に従い作成
   - `cp biped_walking/train/droid_train_omni_v{N-1}.py biped_walking/train/droid_train_omni_v{N}.py`（`cd rl_ws` 済みの状態で実行。`&&` で連結しないこと）
3. 必要に応じて `droid_env_unitree.py` に報酬関数を追加
4. lint・型チェック（`AGENTS.md` コード品質ルール参照）
5. レポートスケルトン `exp009_report_v{N}.md` を [report-template.md](report-template.md) に従い作成
6. 主レポート `exp009_droid_rl_walking_omni.md` のバージョン一覧を更新

## Phase 2: 訓練開始

以下の手順で訓練をバックグラウンド実行し、リアルタイム監視用の tmux ペインを作成する。

### 2.1 訓練のバックグラウンド起動

Bash ツールを `run_in_background: true` で実行する。確認なしで自動実行する。

`cd rl_ws` を先に別の Bash ツール呼び出しで実行してから（cwdは保持される）:
```bash
uv run python biped_walking/train/droid_train_omni_v{N}.py --max_iterations 500
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

## スコープ外

訓練完了後の分析（データ収集・定量分析・サーベイ照合・独立考察・レポート執筆）は本スキルのスコープ外とする。
