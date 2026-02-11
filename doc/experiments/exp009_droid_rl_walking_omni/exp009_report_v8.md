# V8: gait_frequency 1.2→1.5 Hz（交互歩行回復）

## 概要

V6をベースに、gait_frequencyを1.2Hzから1.5Hzに25%増加させることで、V5-V7で一貫して悪化し続けたhip_pitch交互歩行の回復を目指す。V7の関節グループ別Kp/Kd変更は一旦revertし、V6のPD制御ゲインに復帰する。

exp008 V20で0.9→1.2Hz（33%増）が6+指標同時改善を実証した実績に基づき、「Dynamic parameters (timing) > reward shaping」の原則を全方向歩行（exp009）にも適用する。

## 前バージョンからの改善事項

V7レポートの「次バージョンへの提案」に基づき、V6ベース + gait_frequency変更を実施。

| パラメータ | V7値 | V8値 | 変更理由 |
|-----------|------|------|---------|
| gait_frequency | 1.2 Hz | **1.5 Hz** | exp008 V20の実績に基づく。片足支持時間短縮→交互歩行回復 |
| kp_overrides (knee) | 50.0 | **削除（デフォルトKp=35.0）** | ankle Kp問題との分離不可→一旦revert |
| kp_overrides (ankle) | 20.0 | **削除（デフォルトKp=35.0）** | 報酬関数の構造的欺瞞+接地バウンス→revert必須 |
| kd_overrides (knee) | 3.0 | **削除（デフォルトKd=2.0）** | knee Kp revertに伴い一旦revert |
| kd_overrides (ankle) | 5.0 | 5.0（変更なし） | exp008 V25から継承（PDアンダーダンピング修正） |

### V7のKp/Kd revertの理由

V7のankle Kp=20は以下の二重問題を引き起こした:
- (A) 制御帯域が歩行周波数以下に低下し接地バウンスが多発（接触遷移レート9.02/s、exp009全バージョン最悪）
- (B) ankle速度低下がcontact_no_velペナルティを構造的に騙し、ratio 0.065の「偽改善」

Kp/Kd調整は交互歩行を直接改善するメカニズムを持たず、gait_frequencyで安定ベースラインを確立した上でV9以降で再検討する順序が合理的。

## 設計詳細

### gait_frequency 1.5 Hzの選択根拠

1. **exp008 V20の前例**: 0.9→1.2Hz（33%増）で以下を同時達成
   - X vel +22.4%
   - hip_pitch corr ALL-TIME BEST
   - lateral std -52.9%
   - Yaw≈0°
   - L/R contact symmetry回復

2. **因果メカニズム**: 片足支持時間短縮→hip_rollバランス負荷軽減→交互歩行回復

3. **変更幅**: 1.2→1.5Hz = 25%増（exp008の33%より保守的）

4. **V5-V7の教訓**: 報酬パラメータ（V5: iterations増、V6: action_rate）やPD制御（V7: Kp/Kd）の調整では交互歩行崩壊を解決できなかった→動的タイミングパラメータの変更が必要

### PD制御ゲイン（V6と同一）

| 関節グループ | Kp | Kd | 備考 |
|---|---|---|---|
| hip_yaw/hip_roll/hip_pitch | 35.0 | 2.0 | 全関節デフォルト |
| knee_pitch | 35.0 | 2.0 | V7のKp=50/Kd=3から一旦revert |
| ankle_pitch | 35.0 | 5.0 | Kdのみexp008 V25から継承 |

### 報酬構成

16項目（V6と完全同一、報酬scale変更なし）。

### 成功指標

| 指標 | 目標 | V7値（参考） | V6値（ベース） |
|------|------|-------------|---------------|
| hip_pitch相関 | < -0.2（V4レベル回復） | +0.150 | +0.036 |
| 遷移レート | < 7/s（V6レベル） | 9.02/s | 6.52/s |
| L/R周波数 | 一致 | 不一致 | 不一致 |

### リスク

- feet_swing_height悪化（exp008 V20で+17.5%の前例。短いスイング時間→高さ目標到達困難）
- 1.5Hzが高すぎる場合、全方向歩行の自由度が制約される可能性

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_omni_v8.py --max_iterations 4000
```

## 評価コマンド

```bash
cd rl_ws
# ランダムコマンド評価（従来）
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v8 --no-viewer --duration 10
# 方向別評価（Forward例、4方向x3試行の詳細はexp009_commands.md Section 6参照）
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v8 --no-viewer --duration 20 --csv --command 0.3 0.0 0.0 --seed 1
```

## 結果

（訓練完了後に記入）

### 訓練結果

| 指標 | V8 | V7 | 変化 |
|------|-----|-----|------|
| 最終報酬 | - | 60.006 | - |
| ピーク報酬 | - | 60.569 | - |
| エピソード長 | - | ~997-1001 | - |
| Last 50 std | - | 0.2455 | - |

### 評価結果（ランダムコマンド）

（訓練完了後に記入）

### 方向別評価結果

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

- V6をベースにgait_frequencyのみ変更（1変更1検証原則に基づく）
- V7の関節グループ別Kp/Kd（knee Kp=50/Kd=3、ankle Kp=20）は一旦revert
- Kp/Kd調整の方向性自体はCLOSEDではない。V8でgait_frequencyによる安定ベースラインを確立後、V9以降で1関節グループずつmoderate変更で再検討
- CLOSEDアプローチ（具体値）: ankle Kp=20（V7で棄却）、複数Kp/Kd同時変更（V7で棄却）
