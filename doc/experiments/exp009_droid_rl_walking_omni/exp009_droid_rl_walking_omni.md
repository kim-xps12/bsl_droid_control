# EXP009: BSL-Droid Simplified V2による全方向歩行学習実験

## 概要

BSL-Droid Simplified V2（胴体幅75%縮小モデル）に対して、exp008で確立された歩行制御を基に全方向（前進・後退・左右並進）の速度コマンド追従歩行を実現する。exp008 V25をベースに、速度指令のランダマイズ機構を導入する。

## 実験の進め方

実験ルール・手順は以下を参照:
- [exp009_rules.md](exp009_rules.md) - 原則・ルール（メインエントリ）
- [exp009_workflow.md](exp009_workflow.md) - ワークフロー・レポートテンプレート
- [exp009_commands.md](exp009_commands.md) - コマンドリファレンス
- [exp009_reward_design.md](exp009_reward_design.md) - 報酬設計原則・トラブルシューティング

## 背景

### exp008からの継承

exp008（25バージョンの反復チューニング）で以下が確立された:
- PD target clampによる内股解消（V13-V14で確立、V25まで継続検証）
- gait_frequency 1.2Hzによる多指標同時改善（V20で確立）
- 16-17項目の報酬構成（V23-V25で安定）
- ankle_pitch kd個別設定によるPDダンピング改善（V25で導入）

詳細は exp008の各バージョンレポートを参照。

### 全方向歩行の動機

exp008では前進歩行のみを学習対象としていた（lin_vel_x_range: [0.25, 0.35] m/s、Y/Yaw: 0）。
実機応用に向け、以下の能力を獲得する必要がある:
1. 前後方向の速度追従（前進・後退）
2. 左右並進の速度追従
3. 将来的にはYaw旋回も含む全方位コマンド追従

## ロボット仕様

### BSL-Droid Simplified V2（exp008と同一）

| 項目 | V1（exp007） | V2（exp008/exp009） | 変更率 |
|------|-------------|---------------------|--------|
| 胴体幅（torso_width） | 0.18 m | 0.135 m | 75% |
| 胴体奥行（torso_depth） | 0.14 m | 0.14 m | - |
| 胴体高さ（torso_height） | 0.16 m | 0.16 m | - |
| 股関節オフセット（hip_offset_y） | 0.10 m | 0.075 m | 75% |
| DOF | 10 | 10 | - |
| 総質量 | 約5.8 kg | 約5.8 kg | - |

---

## バージョン別実験レポート

各バージョンの詳細な設計・実装・結果は以下の個別ファイルを参照：

| バージョン | ファイル | 概要 | 主な結果 |
|-----------|----------|------|----------|
| V1 | [exp009_report_v1.md](exp009_report_v1.md) | exp008 V25ベース + 全方向コマンド対応 | 訓練収束（reward=54.9）、内股維持（±10°）、hip_pitch corr崩壊（-0.254）、Yaw drift +39°。velocity_deficit構造的不適合が主因 |
| V2 | [exp009_report_v2.md](exp009_report_v2.md) | velocity_deficit scale縮小（-0.5→-0.1） | velocity_deficit -80.9%達成、ratio 0.080回復、action_rate -31.7%、hip_pitch corr +52.8%（-0.389）、L/R周波数同期化。Yaw drift不変（+40.6°）、R²崩壊（0.122）、収束未完了（std=0.398） |
| V3 | [exp009_report_v3.md](exp009_report_v3.md) | 訓練イテレーション数増加（500→2000） | reward 59.89（+6.5%）、Last 50 std 0.228（-42.7%）、ratio 0.071。学習飽和（Q3→Q4 +0.6）。base_height +88.9%・Action RMS +216%（重心引き上げ戦略）。FWD/BWD追従78%、LFT 41.5%、RGT 27.0%。R²=0.042崩壊持続。BWD転倒あり |
| V4 | [exp009_report_v4.md](exp009_report_v4.md) | is_moving判定のバグ修正（X方向のみ→XYノルム） | reward 58.93（-1.6%）、std 0.376（+64.9%）。LFT Yaw +103°→+5.5°劇的改善、BWD転倒解消。一方、**ジタバタ挙動**発生（hip_yaw 3.62Hz振動）、全方向追従率低下（52.0%→V3: 56.2%）、RGT 19.7%最低。is_moving修正は正しいが報酬ランドスケープの構造的変更に2000iterationsでは適応不足 |
| V5 | [exp009_report_v5.md](exp009_report_v5.md) | 訓練イテレーション数増加（2000→4000） | **収束不足仮説を定量的に棄却**。報酬ピーク後下降（59.09→57.12）、Last 50 std +9.5%悪化。hip_yaw >2Hz成分L 85.6%に増大、hip_roll action RMS累積6.7x。4方向追従率45.8%（V4: 52.0%）。RGT改善（+15.3pp）だがBWD -25.0pp悪化。根本原因はaction_rate scale -0.005の過小 |
| V6 | [exp009_report_v6.md](exp009_report_v6.md) | action_rate scale増加（-0.005→-0.01、ジタバタ挙動抑制） | （訓練前） |
| V7 | [exp009_report_v7.md](exp009_report_v7.md) | 関節グループ別Kp/Kd導入（Unitree G1比率参考: 膝Kp=50/Kd=3、足首Kp=20/Kd=5） | （訓練前） |
| V8 | [exp009_report_v8.md](exp009_report_v8.md) | V6ベース + gait_frequency 1.2→1.5 Hz（交互歩行回復、V7 Kp/Kd revert） | （訓練前） |
| V9 | [exp009_report_v9.md](exp009_report_v9.md) | V8ベース + knee Kp=50（単独、関節グループ別Kp/Kd再検討の第一歩） | （訓練前） |
| V10 | [exp009_report_v10.md](exp009_report_v10.md) | V9ベース + vyコマンド範囲の楕円化（±0.3→±0.15 m/s） | （訓練前） |
| V11 | [exp009_report_v11.md](exp009_report_v11.md) | V10ベース + vyコマンド範囲の最適化（±0.15→±0.20 m/s） | （訓練前） |
| V12 | [exp009_report_v12.md](exp009_report_v12.md) | V10ベース + 速度二乗EMA対称性報酬の追加（symmetry_vel_ema 0.3） | reward 66.66（+9.0%）、振動大幅抑制（高周波比率14.7→7.8%）、接地対称化（48.2/50.8%）、FWD追従率90.5%。ただしsymmetry_vel_emaの構造的欠陥により右側活動量を選択的に削減するtrivial solutionを獲得。R hip_pitch歩行パターン崩壊（主周波数0.12Hz）、knee非対称度+18.6%悪化、左膝過伸展。速度RMS対称化はrange非対称を隠蔽。追加分析でbase_height_target引き下げを却下、Mirror Augmentationを推奨 |
| V13 | [exp009_report_v13.md](exp009_report_v13.md) | V11ベース + Mirror Augmentation（L↔R反転データ拡張） | 対称性改善（hip_pitch非対称度+28.5%→+9.8%）だが、gait_phaseミラー変換バグにより位相制御崩壊（Yawドリフト7.3倍悪化: 12.7°→92.4°/20s）。バグ修正をV14で実施 |
| V14 | [exp009_report_v14.md](exp009_report_v14.md) | V13ベース + gait_phaseミラー変換バグ修正（obs[42:44]符号反転追加） | 姿勢安定性大幅改善（Roll std -24.3%, Pitch std -40.5%）、マイクロスタンス完全解消、hip_pitch相関-0.502に大幅改善、FWD追従率93.2%。一方hip_pitch非対称度が方向反転(-57.7%)、LFT/RGT追従率低下 |
| V15 | [exp009_report_v15.md](exp009_report_v15.md) | V14ベース + 横方向コマンド速度範囲拡大（lin_vel_y_range ±0.20→±0.30） | （訓練前） |
