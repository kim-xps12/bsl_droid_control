---
name: exp009-collect
description: 物理シミュレータ評価のためのデータ収集・分析スクリプト実行
argument-hint: "[version-number] [optional: observation-text]"
---

# EXP009 データ収集ワークフロー

V$ARGUMENTS の物理シミュレータ評価に必要なデータを収集する。
収集完了後、`/exp009-evaluate` で対話分析・レポート更新を行う。

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

## 完了チェック

以下が揃っていることを確認して終了:

- [ ] `logs/droid-walking-omni-v{N}/eval_*.csv`（ランダムコマンド）
- [ ] `logs/droid-walking-omni-v{N}/eval_*_cmd_0.30_0.00_0.00_s1.csv`（固定コマンド）
- [ ] V{N-1} の同等ファイル
- [ ] 各分析スクリプトの実行完了

次のステップ: `/exp009-evaluate {N} {所感テキスト}`
