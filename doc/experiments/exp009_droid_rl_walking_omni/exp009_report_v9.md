# V9: knee Kp=50（単独、関節グループ別Kp/Kd再検討の第一歩）

## 概要

V8をベースに、knee_pitch Kp 35.0→50.0（単独変更）を実施する。V8でgait_frequency 1.5Hzによる交互歩行回復が達成されたことで、V7レポートで計画した「gait_frequency確立後のKp/Kd再検討」の条件が成立した。V7ではknee Kp=50 + ankle Kp=20の同時変更で効果分離が不可能だったため、V9ではknee Kp=50を単独で検証し、1.5Hz歩行環境における膝応答性の強化効果を評価する。

## 前バージョンからの改善事項

V8レポートの「次バージョンへの提案」に基づき、knee Kp=50を単独で実施。

| パラメータ | V8値 | V9値 | 変更理由 |
|-----------|------|------|---------|
| knee_pitch Kp | 35.0（デフォルト） | **50.0（kp_overrides）** | V7計画のKp/Kd再検討第一歩。膝は最大負荷関節。swing中の制御力増強で正報酬回復を狙う |

### V9でknee Kp=50を単独変更する理由

1. **V7の教訓**: knee Kp=50 + ankle Kp=20の同時変更で効果分離不可。1変更1検証原則に基づき単独検証
2. **物理的合理性**: 膝は最大負荷関節であり、Unitree G1でもhipの1.5倍のKpを使用
3. **V8の正報酬低下への対策**: swing_duration -59.2%、step_length -28.4%を膝制御力増強で改善しうる
4. **リスク低減**: ankle Kp減少とは異なり、制御帯域低下・接地バウンス・報酬欺瞞のリスクがない
5. **高速歩行環境への適応**: gait_frequency=1.5Hzでは片足支持時間が0.667sと短く、関節の応答性がより重要

## 設計詳細

### PD制御ゲイン

| 関節グループ | Kp | Kd | 備考 |
|---|---|---|---|
| hip_yaw/hip_roll/hip_pitch | 35.0 | 2.0 | 全関節デフォルト |
| knee_pitch | **50.0** | 2.0 | V9変更（V8: 35.0） |
| ankle_pitch | 35.0 | 5.0 | Kdのみexp008 V25から継承 |

### 報酬構成

16項目（V8と完全同一、報酬scale変更なし）。

### 成功指標

| 指標 | 目標 | V8値（ベース） |
|------|------|---------------|
| hip_pitch相関 | < -0.2（維持） | -0.229 |
| 正報酬合計 | > 2.5 | 1.99 |
| 横方向増幅率 | < 2.0x | 2.68x |
| 膝range非対称度 | < 20% | 監視対象 |

### リスク

- knee range非対称が拡大する可能性（V7ではL +40%だが、ankle Kp=20の影響を含む）
- action_rate増加の可能性（高Kp resistanceに対するポリシーの過剰入力）

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_omni_v9.py --max_iterations 4000
```

## 評価コマンド

```bash
cd rl_ws
# ランダムコマンド評価（従来）
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v9 --no-viewer --duration 10
# 方向別評価（Forward例、4方向x3試行の詳細はexp009_commands.md Section 6参照）
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v9 --no-viewer --duration 20 --csv --command 0.3 0.0 0.0 --seed 1
```

## 結果

（訓練完了後に記入予定）

### 訓練結果

| 指標 | V9 | V8 | 変化 |
|------|-----|-----|------|
| 最終報酬 | - | 60.856 | - |
| ピーク報酬 | - | 61.485 | - |
| エピソード長 | - | ~994 | - |
| Last 50 std | - | 0.3028 | - |

### 評価結果（ランダムコマンド）

（訓練完了後に記入予定）

### 方向別評価結果

（訓練完了後に記入予定）

## 考察と改善案

（訓練完了後に記入予定）

### 成功点
- （訓練完了後に記入予定）

### 課題
- （訓練完了後に記入予定）

### 次バージョンへの提案
- （訓練完了後に記入予定）

## まとめ

exp009 V9はV8ベース + knee_pitch Kp 35.0→50.0（単独変更）を実施した。V7レポートで計画した「gait_frequency確立後のKp/Kd再検討」の第一歩として、膝の剛性増加による1.5Hz歩行への適応改善を検証する。（結果は訓練完了後に記入予定）

## 備考

- V8をベースにknee_pitch Kpのみ変更（1変更1検証原則に基づく）
- 報酬構成は16項目（V8と同一、scale変更なし）
- gait_frequency 1.5Hz、ankle_pitch Kd=5.0はV8から継承
- V7で同時変更したankle Kp=20はV8で既にrevert済み（今回は変更なし）
