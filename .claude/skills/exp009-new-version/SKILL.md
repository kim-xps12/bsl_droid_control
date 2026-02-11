---
name: exp009-new-version
description: Create a new exp009 training version: script generation, report skeleton, and training start.
disable-model-invocation: true
argument-hint: "[version-number] [--no-train]"
---

# EXP009 新バージョン作成ワークフロー

## 引数の解析

`$ARGUMENTS` を空白で分割し、以下を抽出する:
- **バージョン番号**: 最初の数値トークン（例: `8`）
- **`--no-train` フラグ**: 存在する場合、Phase 2（訓練開始）をスキップする

例:
- `/exp009-new-version 8` → V8 を作成し、訓練を開始する
- `/exp009-new-version 8 --no-train` → V8 を作成するが、訓練は開始しない

以降、バージョン番号を `{N}` として参照する。

## フロー概要

```
Phase 1: 準備（スクリプト作成・コード品質チェック）
    ↓
Phase 2: 訓練開始（tmux）  ← --no-train 指定時はスキップ
    ↓
Phase 3: レポート準備（訓練実行中に並行作業）
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

## Phase 2: 訓練開始

> **`--no-train` 指定時**: このフェーズ全体をスキップし、Phase 3 に進む。ユーザに以下を報告する:
> - 「訓練はスキップしました。手動で開始する場合は以下のコマンドを実行してください:」
> - ```
>   cd rl_ws
>   uv run python biped_walking/train/droid_train_omni_v{N}.py --max_iterations 4000
>   ```

以下の手順で tmux セッション内で訓練を起動する。SSH セッションが切断されても tmux 内で訓練が継続される。

### 2.1 tmux セッションの確認/作成

tmux セッションが存在するか確認:
```bash
tmux list-sessions 2>/dev/null
```

**セッションが存在しない場合（exit code 1）**: セッションを作成する:
```bash
tmux new-session -d -s main
```

### 2.2 訓練の tmux ウィンドウ起動

`cd rl_ws` を先に別の Bash ツール呼び出しで実行してから（cwdは保持される）、以下を順次実行する。確認なしで自動実行する。

ログファイルパス: `/tmp/exp009_train_v{N}.log`

**Step 1**: tmux 内に訓練用ウィンドウを作成:
```bash
tmux new-window -t main -n "train-v{N}"
```

**Step 2**: 訓練コマンドを送信（`$(pwd)` で現在の rl_ws 絶対パスを使用）:
```bash
tmux send-keys -t "main:train-v{N}" "cd $(pwd) && uv run python biped_walking/train/droid_train_omni_v{N}.py --max_iterations 4000 2>&1 | tee /tmp/exp009_train_v{N}.log" Enter
```

- `tee`: 訓練出力を tmux ウィンドウ内のターミナルとログファイルの両方に出力
- SSH 切断後も tmux セッション内で訓練が継続する（`tmux attach -t main` で再接続可能）

### 2.3 監視ペインの作成

訓練出力を監視するペインを作成する:
```bash
tmux split-window -v -d -l 15 "tail -f /tmp/exp009_train_v{N}.log"
```
- `-v`: 下方向に分割
- `-d`: フォーカスを移動しない
- `-l 15`: 高さ15行

### 2.4 訓練状態の確認方法

訓練の進捗はログファイルで確認可能:
```bash
tail -20 /tmp/exp009_train_v{N}.log
```

訓練ウィンドウの存在確認:
```bash
tmux list-windows -t main
```

## Phase 3: レポート準備

訓練は tmux 内で実行中。待ち時間を活用してレポートを準備する。

1. レポートスケルトン `exp009_report_v{N}.md` を [report-template.md](report-template.md) に従い作成
   - 「結果」セクションは訓練完了後に記入する旨を明記
   - 設計詳細・パラメータ変更を記載
2. 主レポート `exp009_droid_rl_walking_omni.md` のバージョン一覧を更新

## スコープ外

訓練完了後の分析（データ収集・定量分析・サーベイ照合・独立考察・レポート執筆）は本スキルのスコープ外とする。
