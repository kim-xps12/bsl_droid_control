# V6: action_rate scale増加（ジタバタ挙動抑制）

## 概要

V4で観測されたジタバタ挙動（hip_yaw 3.62Hz振動、L_hip_rollアクション変化率+32%、R_hip_roll速度RMS 1.52倍）の抑制を目指す。V4レポートの「V6以降の候補」第1案に基づき、action_rate scaleを-0.005→-0.01（2倍増）に変更する。action_rateペナルティはアクション間の変化量の二乗に比例するため、高周波関節振動に対して直接コストを課す。

## 前バージョンからの改善事項

V4レポートの「次バージョンへの提案」に記載された「V6以降の候補」第1案に基づく変更。

| パラメータ | V5値 | V6値 | 変更理由 |
|-----------|------|------|---------|
| action_rate scale | -0.005 | -0.01 | V4で観測されたhip_yaw 3.62Hz振動の抑制。-0.005はexp008の前進歩行用設計で全方向歩行には不十分。保守的に2倍増で試行 |

## 設計詳細

### action_rate penaltyの役割

action_rateペナルティは `Σ(action_t - action_{t-1})²` に比例する。高周波振動はステップ間のアクション差分が大きいため、このペナルティで直接抑制される。

V4の分析で判明した問題:
- hip_yaw両側の主周波数が3.62 Hzに遷移（V3: 0.12-1.25 Hz）
- 高周波成分比率: L_hip_yaw 62.5%（1.44x）、R_hip_yaw 72.1%（1.68x）
- L_hip_rollアクション変化率が+32%突出
- action_rateペナルティは+24.6%悪化（-0.0313→-0.0390）

### scale選択の根拠

V4レポートは-0.01 or -0.02を候補としている。overkillリスク（exp008 V5の教訓: penalty/positive ratio 0.186でoverkill zone）を考慮し、保守的に-0.01（2倍増）を選択。V4のratio 0.077は健全域にあるため、action_rate単独の2倍増ではoverkillに至りにくいと判断。

### 報酬構成

16項目（V5と同一構成、action_rate scaleのみ変更）。

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_omni_v6.py --max_iterations 4000
```

## 評価コマンド

```bash
cd rl_ws
# ランダムコマンド評価（従来）
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v6 --no-viewer --duration 10
# 方向別評価（Forward例、4方向x3試行の詳細はexp009_commands.md Section 6参照）
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v6 --no-viewer --duration 20 --csv --command 0.3 0.0 0.0 --seed 1
```

## 結果

（訓練完了後に記入）

### 訓練結果

| 指標 | V6 | V5 | 変化 |
|------|-----|-----|------|
| 最終報酬 | - | - | - |
| エピソード長 | - | - | - |
| Last 50 std | - | - | - |

### 評価結果（ランダムコマンド）

（訓練完了後に記入）

### 方向別評価結果

（訓練完了後に記入）

### 方向間バランス

（訓練完了後に記入）

## 考察と改善案

（訓練完了後に記入）

### 成功点
- （訓練完了後に記入）

### 課題
- （訓練完了後に記入）

### 次バージョンへの提案
- （訓練完了後に記入）

## まとめ

（訓練完了後に記入）

## 備考

- 本バージョンの変更はaction_rate scaleのみ（-0.005→-0.01）。他の報酬構成・環境設定・コマンド設定はV5と完全同一
- V4レポートの「V6以降の候補」第1案に基づく変更
- V5の結果が未確定のため、V4のベースライン指標との比較も並行して記録する
- 1変更1検証原則に基づき、action_rate scaleのみを変更
- hip_yaw_pos scale増加（V4レポート第2案）はV6の結果に応じてV7で検討
