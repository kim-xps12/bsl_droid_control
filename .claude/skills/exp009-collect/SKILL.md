---
name: exp009-collect
description: 物理シミュレータ評価のためのデータ収集・分析スクリプト実行
argument-hint: "[version-number] [optional: observation-text]"
---

# EXP009 データ収集ワークフロー

V$ARGUMENTS の物理シミュレータ評価に必要なデータを収集する。
収集完了後、ユーザーが別途 `/exp009-evaluate` を実行してレポート更新を行う。
**禁止事項**: このスキル内で `/exp009-evaluate` を実行してはならない。データ収集と完了チェックまでで必ず終了すること。

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

## Step 1: データ収集

以下のコマンドを `cd rl_ws` の上で実行する。
**注意**: `PYTHONUNBUFFERED=1` は `settings.json` で自動適用されるため、明示的な付与は不要。

### 1.1 一括データ収集（batch_eval.py による並列実行）

トレーニングスクリプト `biped_walking/train/droid_train_omni_v{N}.py` を読み、以下を取得する:

- `lin_vel_x_range` → 上限値を **vx_max** とする（例: `[−0.3, 0.3]` → `vx_max = 0.3`）
- `lin_vel_y_range` → 上限値を **vy_max** とする（例: `[−0.15, 0.15]` → `vy_max = 0.15`）

V{N-1} のトレーニングスクリプトからも `lin_vel_x_range` の上限値（**vx_max_prev**）を取得する。

以下の2コマンドを**並列で**（別々のBash tool callとして同時に）実行する:

```bash
# V{N}: ランダム評価 + 方向別4方向×3seed（並列実行）
uv run python scripts/batch_eval.py \
    -e droid-walking-omni-v{N} \
    --vx-max {vx_max} --vy-max {vy_max} --workers 3
```

```bash
# V{N-1}: ランダム評価 + FWD固定コマンド（Yaw信頼性評価用）
uv run python scripts/batch_eval.py \
    -e droid-walking-omni-v{N-1} \
    --no-directional \
    --extra-cmd "{vx_max_prev} 0.0 0.0"
```

`batch_eval.py` は既存CSVを自動スキップするため、再実行しても安全。
詳細は `exp009_commands.md` Section 5.10 を参照。

**重要**: Yaw改善率はFWD固定コマンド評価データのみから算出すること。ランダムコマンド評価からのYaw改善率は、コマンド切替による隠蔽のため信頼できない。

### 1.2 分析スクリプト実行（全て並列実行）

以下の4コマンドを**並列で**（別々のBash tool callとして同時に）実行する:

```bash
# 報酬コンポーネント比較
uv run python scripts/show_reward_components.py droid-walking-omni-v{N-1} droid-walking-omni-v{N}
```

```bash
# CSV時系列分析
uv run python scripts/analyze_eval_csv.py {N} {N-1} --prefix droid-walking-omni-v
```

```bash
# 横方向揺れ詳細分析
uv run python scripts/analyze_lateral_sway.py {N} {N-1} --prefix droid-walking-omni-v
```

```bash
# 振動・高周波分析（目視で振動が指摘された場合に特に有用）
uv run python scripts/analyze_vibration.py {N} {N-1} --prefix droid-walking-omni-v
```

### 1.3 追加分析（所感に応じて選択）

人間の所感に基づき、以下から必要な追加分析を選択実行する:

- **方向別問題が指摘された場合**: 方向別評価CSV（既存なら読み込み）を用いた方向別分析
- **Yaw問題の深掘り**: Step 1.1b の固定コマンドCSVに対して `analyze_eval_csv.py` で時系列分析。
  固定コマンドCSVを指定するには `--epoch` にファイル名のepoch部分全体を渡す（`exp009_commands.md` Section 5.4 参照）:
  ```bash
  uv run python scripts/analyze_eval_csv.py {N} {N-1} --prefix droid-walking-omni-v --epoch {EPOCH}_cmd_0.30_0.00_0.00_s1
  ```
  `analyze_yaw_drift_v2.py` 系はexp008固有（パスがハードコード）のため exp009 では使用しない
- **接触問題**: `uv run python scripts/analyze_contact_chattering_v2.py` 系

## 完了チェック

以下が揃っていることを確認して終了:

- [ ] `logs/droid-walking-omni-v{N}/eval_*.csv`（ランダムコマンド）
- [ ] `logs/droid-walking-omni-v{N}/eval_*_cmd_0.30_0.00_0.00_s1.csv`（FWD固定コマンド）
- [ ] 4方向 × 3 seed の固定コマンドCSV（計12ファイル）
- [ ] V{N-1} の同等ファイル
- [ ] 各分析スクリプトの実行完了

**このスキルはここで終了する。`/exp009-evaluate` を自動実行してはならない。**
ユーザーへの案内: 次のステップとして `/exp009-evaluate {N} {所感テキスト}` を手動で実行してください。
