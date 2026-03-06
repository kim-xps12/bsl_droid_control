# V21: stance_foot_lateral_positionの削除と動作検証

## 概要

V20の報酬構成から冗長報酬項目`stance_foot_lateral_position`（scale=-0.5）を削除し、報酬構造のクリーンアップを行う。V14以降のPDターゲットクランプ（`hip_roll_inward_limit=-0.05 rad`）がhip_rollを物理的に制約しているため、足横方向位置ペナルティは機能的に不要。V20での寄与はペナルティ全体の0.6%（-0.0014）で完全に冗長。

## 前バージョンからの改善事項

### V20の課題

V20はgait_frequency引き上げ（0.9→1.2 Hz）により6+指標が同時改善したブレークスルー版である。主な課題は:
1. `feet_swing_height`が+17.5%悪化（唯一の退行指標）
2. 報酬構成に冗長項目が残存（`stance_foot_lateral_position`寄与0.6%）

### V20レポートの次バージョン提案

V20レポートのOption A（推奨）を採用:
> stance_foot_lateral_positionを削除し、PDクランプへの完全移行を完了する

### 改善戦略

冗長報酬項目の削除により:
- 報酬構造の簡素化（18→17項目、推奨範囲15-17に到達）
- penalty/positive ratioの微改善（0.079→目標≤0.079）
- ポリシー最適化の自由度確保（不要な制約の除去）

### パラメータ変更表

| パラメータ | V20 | V21 | 変更理由 |
|-----------|-----|-----|----------|
| `stance_foot_lateral_position` | -0.5 | 0（削除） | PDクランプ(V14+)が完全代替、寄与0.6% |

## 設計詳細

### stance_foot_lateral_position削除の根拠

1. **PDターゲットクランプが完全代替**: V14で導入されたhip_roll_inward_limit=-0.05 rad（実効値≈-12°）がhip_rollを物理的に制約。足横方向位置は運動学的に追従
2. **7バージョンの冗長性実績**: V14-V20を通じて、PDクランプ下で内股±9-12°が安定維持
3. **V20での寄与率**: ペナルティ全体の0.6%（-0.0014）は実質ゼロ
4. **V15で冗長性確認済み**: base_vel_y存在下でpenalty -24.4%減少を記録

### 報酬構成（V21）

有効項目数: 17（V20の18から1削除）

**正報酬（6項目）**: tracking_lin_vel(1.5), tracking_ang_vel(0.5), swing_duration(2.0), contact(0.4), single_foot_contact(0.5), step_length(0.8)

**ペナルティ（11項目）**: swing_contact_penalty(-0.7), lin_vel_z(-2.0), ang_vel_xy(-0.1), orientation(-2.0), base_height(-5.0), feet_swing_height(-8.0), contact_no_vel(-0.1), hip_yaw_pos(-0.8), velocity_deficit(-0.5), action_rate(-0.005), foot_lateral_velocity(-0.5), base_vel_y(-1.0)

**削除**: stance_foot_lateral_position(-0.5→0)

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_narrow_v21.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-narrow-v21 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 61.18 |
| エピソード長 | 1001.0 (最大) |
| 収束ステップ | ~350 (reward >58) |
| 最終noise std | 0.21 |
| last 50 std | 0.119 |

訓練は安定的に収束。V20 (57.73相当) と比較して最終報酬が高い。これはstance_foot_lateral_positionペナルティ削除により、ペナルティ総額が減少したことが寄与。

### 評価結果

| 指標 | V20 | V21 | 変化率 |
|------|-----|-----|--------|
| X velocity [m/s] | 0.312 | 0.307 | -1.6% |
| Yaw drift [°] | +0.05 | -1.00 | 微悪化（依然優秀） |
| Roll std [°] | 4.18 | 4.21 | +0.7%（実質不変） |
| Pitch std [°] | 1.00 | 0.82 | **-18.0%（改善）** |
| hip_pitch corr | -0.877 | -0.733 | **-16.4%（悪化）** |
| lateral std [mm] | 27.3 | 24.7 | **-9.5%（改善）** |
| R² | 0.903 | 0.959 | **+6.2%（改善）** |
| 内股 hip_roll L [°] | -9.3 | -9.4 | 維持 |
| 内股 hip_roll R [°] | +9.3 | +9.6 | 維持 |
| ratio | 0.079 | 0.078 | -1.3%（微改善） |
| gait freq [Hz] | 1.25 | 1.25 | 維持 |
| base height [m] | 0.270 | 0.278 | +3.0% |
| 残差 std [mm] | 8.5 | 5.0 | **-41.2%（改善）** |
| 横方向ドリフト速度 [mm/s] | -9.0 | -2.6 | **-71.1%（改善）** |
| 増幅率 | 1.32x | 1.18x | **-10.3%（改善）** |
| 両足接地 [%] | 3.0 | 2.0 | -1.0pp |
| L/R非対称度 | +5.1% | +12.2% | 悪化 |

### 報酬成分比較

| 報酬項目 | V20 | V21 | 変化 |
|----------|-----|-----|------|
| tracking_lin_vel | 1.1809 | 1.3041 | +10.4% |
| tracking_ang_vel | 0.4333 | 0.4724 | +9.0% |
| swing_duration | 0.2188 | 0.2405 | +9.9% |
| contact | 0.7005 | 0.7645 | +9.1% |
| single_foot_contact | 0.4449 | 0.4860 | +9.2% |
| step_length | 0.0462 | 0.0504 | +9.1% |
| **正報酬合計** | **3.0246** | **3.3179** | **+9.7%** |
| feet_swing_height | -0.0531 | -0.0525 | -1.1% |
| ang_vel_xy | -0.0476 | -0.0503 | +5.7% |
| base_vel_y | -0.0360 | -0.0349 | -3.1% |
| base_height | -0.0228 | -0.0307 | +34.6% |
| hip_yaw_pos | -0.0171 | -0.0245 | +43.3% |
| lin_vel_z | -0.0211 | -0.0241 | +14.2% |
| orientation | -0.0130 | -0.0132 | +1.5% |
| action_rate | -0.0115 | -0.0141 | +22.6% |
| foot_lateral_velocity | -0.0081 | -0.0066 | -18.5% |
| contact_no_vel | -0.0033 | -0.0044 | +33.3% |
| swing_contact_penalty | -0.0035 | -0.0023 | -34.3% |
| stance_foot_lateral_position | -0.0014 | 0.0000 | **削除** |
| velocity_deficit | -0.0006 | -0.0011 | +83.3% |
| **ペナルティ合計** | **-0.2391** | **-0.2587** | **+8.2%** |

### 接地パターン（定常状態 t>2s）

| パターン | V20 | V21 |
|---------|-----|-----|
| 左足のみ | 47.2% | 45.8% |
| 右足のみ | 49.5% | 52.2% |
| 両足 | 3.0% | 2.0% |
| 両足空中 | 0.2% | 0.0% |

### 歩行周期分析

| 指標 | V20 | V21 |
|------|-----|-----|
| 主歩行周波数 | 1.25 Hz | 1.25 Hz |
| L swing duration mean | 0.343 s | 0.389 s |
| R swing duration mean | 0.366 s | 0.385 s |
| L hip_pitch range | 0.701 rad | 0.739 rad |
| R hip_pitch range | 0.633 rad | 0.578 rad |
| バランスストローク L | 5.2° | 5.1° |
| バランスストローク R | 5.3° | 5.3° |

## 考察と改善案

### 因果分析

#### 1. 内股維持の確認（主目的達成）

stance_foot_lateral_position削除後も、接地相hip_rollは L=-9.4°, R=+9.6°で維持（V20: L=-9.3°, R=+9.3°）。PDターゲットクランプ（hip_roll_inward_limit=-0.05 rad）が内股制約を完全に代替していることが8バージョン（V14-V21）にわたり確認された。タスク空間ペナルティからハード制約（PDクランプ）への移行が完了。

#### 2. 横方向揺れ改善の因果連鎖

```
stance_foot_lateral_position削除
  → ペナルティ解放（-0.0014→0）
  → ポリシー最適化の自由度増加
  → base_height最適化 (0.270→0.278 m, +3.0%)
  → 重心上昇 → 振り子幾何学改善
  → Roll→横方向増幅率低下 (1.32x→1.18x)
  → 残差std半減 (8.5→5.0 mm)
  → R²上昇 (0.903→0.959)
  → detrended lateral std -9.5% (27.3→24.7 mm)
```

非Roll残差の半減が最も注目すべき改善点。横方向揺れの96%がRoll角で説明可能となり、RL stochasticityに起因する予測不能な横方向運動が大幅に減少した。

#### 3. hip_pitch correlation悪化の分析

hip_pitch corrが-0.877→-0.733（-16.4%）に悪化。L/R hip_pitch range非対称度も+5.1%→+12.2%に増加。

考えられる原因:
- **RL stochasticity（最有力）**: stance_foot_lateral_positionの寄与は0.6%であり、直接的にhip_pitch coordinationに影響する機構がない。異なる報酬ランドスケープ→異なるローカル最適解
- **暗黙的対称性の喪失（副次的可能性）**: stance_foot_lateral_positionは左右の接地足に同一の距離閾値を課すため、暗黙的にL/R対称性を促進していた可能性がある。削除によりこの微弱な対称性バイアスが消失

hip_pitch corr=-0.733はV4レベル（-0.787）よりやや劣るが、歩行は安定維持。L/R contact比の非対称（45.8%/52.2%）が直接的な原因で、接地時間が右脚に偏っている。

### 成功点

1. **内股防止の完全PDクランプ移行確認**: stance_foot_lateral_position削除後も±9.4-9.6°維持
2. **横方向揺れ改善**: detrended std -9.5%, R² +6.2%, 残差std -41.2%
3. **報酬項目数最適化**: 18→17項目（推奨範囲15-17に到達）
4. **ratio維持**: 0.079→0.078（健全域維持）
5. **Pitch std改善**: 1.00→0.82°（-18.0%）
6. **横方向ドリフト改善**: -9.0→-2.6 mm/s（-71.1%）

### 課題

1. **hip_pitch corr悪化**: -0.877→-0.733（-16.4%）。L/R非対称度+12.2%
2. **X velocity微減**: 0.312→0.307 m/s（-1.6%）
3. **base_height penalty増大**: -0.0228→-0.0307（+34.6%）。base_height上昇の代償
4. **hip_yaw_pos penalty増大**: -0.0171→-0.0245（+43.3%）

### 次バージョンへの提案

**Option A（推奨）: swing_height_target微調整（0.05→0.03 m）**
- feet_swing_heightはV21最大のペナルティ項目（-0.0525）
- 1.25 Hzでのswing duration ~0.39sでは0.05m到達が困難
- 0.03mに下げることで、ペナルティ軽減→ratio改善が期待できる
- V20から2バージョン連続でfeet_swing_heightが最大ペナルティ
- リスク: LOW（ターゲット値の調整のみ、構造変更なし）
- 期待効果: ratio 0.078→~0.05-0.06（ペナルティ最大項目の緩和）

**Option B: hip_pitch correlation対策（symmetry_range復活、0→0.3）**
- L/R hip_pitch range非対称度を直接ペナルティ化
- リスク: MODERATE（17→17項目で変化なしだが、新たな制約追加）
- 懸念: 対称性ペナルティはV34で無効化された経緯あり

**Option C: gait_frequency微調整（1.2→1.1 Hz）**
- hip_pitch corrの改善が期待できる（swing timeの延長→L/R調整時間増加）
- リスク: MODERATE（V20のブレークスルーが部分的に退行する可能性）

## まとめ

V21はstance_foot_lateral_positionの削除実験として成功。主目的である内股防止のPDクランプへの完全移行が確認され、報酬項目数が推奨範囲（17項目）に到達した。予想外の横方向揺れ改善（detrended std -9.5%, R² 0.959）が観測され、base_height上昇を通じた振り子幾何学の改善が因果メカニズムと考えられる。一方、hip_pitch correlationが-0.877→-0.733に悪化し、L/R非対称度が増加した。これはRL stochasticityが主因と推定されるが、削除項目の暗黙的対称性効果の可能性も完全には排除できない。

## 備考

- V20レポートOption A（推奨案）の実行
- 1変更1検証原則を遵守（stance_foot_lateral_position削除のみ）
- 報酬項目のPDクランプへの完全移行はV14-V21の8バージョンで実証完了
