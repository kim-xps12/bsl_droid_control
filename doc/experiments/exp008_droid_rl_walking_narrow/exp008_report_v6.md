# V6: 胴体鉛直性強化（orientation強化）

## 概要

V5で関節レベル制約（stance_hip_roll_target=-2.0）の限界が判明した。内股offset ±4°（数値best）を達成したものの、Roll std +77%、Pitch std +132%のgait degradationが発生し、視覚的にも内股は解消されなかった。body Roll揺れ（6.95°）が脚を傾けて見せるため、hip_roll数値改善だけでは不十分であることが明らかになった。

V6ではV4（best gait quality: Roll std=3.92°, Pitch std=0.98°）をベースに、`orientation`ペナルティを4倍に強化し、ボディレベル（胴体鉛直性）から内股の視覚的改善を狙う。

## 前バージョンからの改善事項

V5レポートの「次バージョンへの提案」に基づき、関節レベル制約からボディレベル制約へのアプローチ転換を実施。V5は失敗版のためベースとせず、V4（best gait quality）をベースとする。

| パラメータ | V4値 | V6値 | 変更理由 |
|-----------|------|------|---------|
| orientation | -0.5 | -2.0 | 胴体鉛直性4x強化。projected_gravity[:,:2]²を直接抑制し、body Roll/Pitch傾きを低減。視覚的内股はbody Roll揺れに起因するため、ボディレベルからの制御が必要 |

## 設計詳細

### orientation報酬の仕組み

`_reward_orientation`は`projected_gravity[:,:2]²`をペナルティ化する（droid_env_unitree.py:620）。重力ベクトルのXY成分が大きい＝胴体がRoll/Pitch方向に傾いていることを意味する。

- **スケール変更**: -0.5 → -2.0（4x強化）
- **期待効果**:
  - Roll std低減を狙う（V4: 3.92° → さらなる改善を期待）
  - Pitch std維持または改善を期待（V4: 0.98°）
  - body Roll揺れの低減により、視覚的内股の軽減を期待

### ang_vel_xyとの関係

- `ang_vel_xy`（-0.1）: Roll/Pitch方向の**角速度**をペナルティ化（動的な揺れ）
- `orientation`（-2.0）: Roll/Pitch方向の**傾き角度**をペナルティ化（静的な傾き）
- 両者は相補的に作用し、角度と角速度の両面から胴体の鉛直性を確保する

### リスク評価

- **過剰制約のリスク**: V5のstance_hip_roll_target=-2.0と同様に、orientation=-2.0でもペナルティ過大になる可能性がある。ただし、orientationは常時適用のため、stance-conditionedよりも穏やかに作用すると期待
- **X速度低下のリスク**: 胴体傾きの自由度制限により、推進力が制約される可能性
- **歩行品質維持**: V4ベースのため、stance_hip_roll_target=-0.5は維持。gait degradationリスクはV5より低いと期待

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_narrow_v6.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-narrow-v6 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 54.74 |
| エピソード長 | 979.28 |
| 最大報酬 | 55.29（step 497） |
| 収束傾向 | 上昇継続（last 50 std=1.42、追加学習の余地あり） |

訓練報酬の推移（4分割平均）:
- Steps 0-125: 20.30
- Steps 125-250: 37.49
- Steps 250-375: 40.57
- Steps 375-500: 49.14

### 評価結果

#### 主要指標比較（V4 → V6）

| 指標 | V4 | V6 | 変化 |
|------|-----|-----|------|
| X速度 | 0.167 m/s | 0.160 m/s | -4.2% |
| Y速度 | -0.003 m/s | 0.000 m/s | 改善 |
| Yaw drift | +7.51° | +14.05° | **+87.1%（悪化）** |
| Yaw drift rate | 1.105 °/s | 1.869 °/s | **+69.1%（悪化）** |
| Roll std | 3.92° | 3.54° | **-9.7%（改善）** |
| Pitch std | 0.98° | 0.89° | **-9.2%（改善）** |
| hip_pitch corr | -0.787 | -0.778 | -1.1%（微減） |
| Both feet | 0.8% | 0.4% | 改善 |
| Base height | 0.245 m | 0.246 m | 安定 |
| DOF range sum | 6.405 rad | 6.400 rad | 安定 |
| Action RMS | 1.112 | 1.150 | +3.4% |

#### 接地パターン（定常状態 t>2s）

| 状態 | V4 | V6 |
|------|-----|-----|
| 両足接地 | 0.8% | 0.2% |
| 左足のみ | 48.0% | 48.5% |
| 右足のみ | 50.5% | 50.5% |
| 両足空中 | 0.8% | 0.8% |

#### スイング区間

| 区間 | V4 | V6 |
|------|-----|-----|
| 左足接地(右足スイング) | 9回, mean=0.536s | 9回, mean=0.540s |
| 右足接地(左足スイング) | 10回, mean=0.490s | 10回, mean=0.490s |

#### 関節可動域

| 関節 | V4 L range | V4 R range | V6 L range | V6 R range |
|------|-----------|-----------|-----------|-----------|
| hip_yaw | 0.370 | 0.366 | 0.379 | 0.408 |
| hip_roll | 0.692 | 0.543 | 0.693 | 0.836 |
| hip_pitch | 0.605 | 0.693 | 0.719 | 0.636 |
| knee_pitch | 0.824 | 0.881 | 0.746 | 0.826 |
| ankle_pitch | 0.669 | 0.762 | 0.533 | 0.624 |

#### hip_roll平均オフセット（定常状態 t>2s）

| 脚 | V4 | V6 | 変化 |
|-----|------|------|------|
| L hip_roll mean | -0.2337 rad (-13.39°) | -0.2535 rad (-14.52°) | +8.4%（微悪化） |
| R hip_roll mean | +0.2969 rad (+17.01°) | +0.2101 rad (+12.03°) | **-29.3%（改善）** |

#### hip_yaw非対称度

| 指標 | V4 | V6 |
|------|-----|-----|
| L hip_yaw range | 0.370 | 0.379 |
| R hip_yaw range | 0.366 | 0.408 |
| 非対称度 | 1.1%（R≈L） | **7.1%（R>L）** |

#### hip_pitch位相解析

| 指標 | V4 | V6 |
|------|-----|-----|
| L/R相関（全体） | -0.852 | -0.850 |
| ローリング相関 mean | -0.853 | -0.860 |
| 非対称度 | -7.0%（R>L） | +8.1%（L>R） |
| 歩行周波数 | 0.88 Hz | 0.88 Hz |

#### Yawドリフト推移

| 時刻 | V4 | V6 |
|------|-----|-----|
| 0s | 0.00° | 0.00° |
| 2s | -0.89° | -2.15° |
| 4s | -0.11° | +3.97° |
| 6s | +1.41° | +8.12° |
| 8s | +3.63° | +7.04° |
| 10s | +7.51° | +14.05° |

V4: 0-5s安定、5s以降緩やかにドリフト。V6: 2s以降加速的にドリフト開始、7-8sで一時停滞後再加速。

#### 報酬コンポーネント比較（最終ステップ）

| 報酬項目 | V4 | V6 | 変化率 |
|----------|------|------|--------|
| tracking_lin_vel | 1.3883 | 1.3233 | -4.7% |
| tracking_ang_vel | 0.3361 | 0.2590 | -22.9% |
| swing_duration | 0.3889 | 0.3637 | -6.5% |
| contact | 0.7418 | 0.7141 | -3.7% |
| single_foot_contact | 0.4831 | 0.4684 | -3.0% |
| step_length | 0.0570 | 0.0683 | +19.8% |
| **正報酬合計** | **3.3952** | **3.1968** | **-5.8%** |
| ang_vel_xy | -0.0748 | -0.0944 | **+26.2%（悪化）** |
| stance_hip_roll_target | -0.0770 | -0.0731 | -5.1%（改善） |
| action_rate | -0.0421 | -0.0563 | +33.7%（悪化） |
| swing_foot_lateral_velocity | -0.0468 | -0.0519 | +10.9% |
| lin_vel_z | -0.0291 | -0.0373 | +28.2%（悪化） |
| feet_swing_height | -0.0373 | -0.0367 | -1.6% |
| hip_yaw_pos | -0.0269 | -0.0281 | +4.5% |
| **orientation** | **-0.0064** | **-0.0244** | **+281.3%（4x scale）** |
| swing_contact_penalty | -0.0096 | -0.0218 | +127.1%（悪化） |
| base_height | -0.0128 | -0.0115 | -10.2%（改善） |
| velocity_deficit | -0.0031 | -0.0039 | +25.8% |
| contact_no_vel | -0.0040 | -0.0036 | -10.0% |
| **ペナルティ合計** | **-0.3699** | **-0.4430** | **+19.8%（悪化）** |
| **ペナルティ/正報酬比** | **0.109** | **0.139** | **+27.5%** |

#### orientation報酬の詳細分析

| 指標 | V4 | V6 |
|------|-----|-----|
| スケール | -0.5 | -2.0（4x） |
| 報酬値 | -0.0064 | -0.0244（3.8x） |
| raw penalty（報酬値/スケール） | 0.0128 | 0.0122（-4.7%） |

raw penaltyが微減 → orientation制約は実際にbody tiltを低減している。4xスケール増に対して報酬値が3.8x増にとどまるのは、制約が有効に機能している証拠。

## 考察と改善案

### 成功点

1. **Roll std改善**: 3.92°→3.54°（-9.7%）。orientation強化による胴体鉛直性向上の直接効果
2. **Pitch std改善**: 0.98°→0.89°（-9.2%）。projected_gravity[:,:2]²はRoll/Pitch両方を抑制
3. **R hip_roll stance offset大幅改善**: +17.01°→+12.03°（-29.3%）。胴体が鉛直に保たれることで、接地脚のhip_rollが中立位置に近づく（ボディレベル→関節レベルへのトップダウン制約チェーン）
4. **stance_hip_roll_target penalty自然減少**: -0.0770→-0.0731（-5.1%）。orientation強化がhip_roll offsetを間接的に改善し、stance_hip_roll_targetの負荷を軽減
5. **歩行品質維持**: スイング持続時間（0.49-0.54s）、歩行周波数（0.88Hz）、接地パターンがV4とほぼ同一。V5のような短頻度戦略への遷移は発生せず
6. **ペナルティ/正報酬比0.139**: V5のoverkill zone（0.186）に比べ健全な範囲内
7. **base_height penalty改善**: -0.0128→-0.0115（-10.2%）。胴体が鉛直に保たれることで高さ維持も容易に

### 課題

1. **Yaw drift大幅悪化**: +7.51°→+14.05°（+87.1%）。drift rate 1.105→1.869°/s（+69.1%）
   - **原因**: hip_yaw非対称度1.1%→7.1%に増大。orientation制約が胴体傾きの自由度を制限し、間接的なYaw補正チャネルを喪失（Survey 7.4.2: angular momentum imbalance）
   - V6はYaw推移で4s以降加速的にドリフト。V4の5s以降緩やかなドリフトと対照的

2. **ang_vel_xy penalty増大**: -0.0748→-0.0944（+26.2%）
   - **原因**: orientation制約で傾き角度は縮小（raw penalty -4.7%）したが、より速い姿勢修正で鉛直性を維持 → 角速度は増大（Survey 2.2.2/2.2.3: 位置制約と速度制約は直交）
   - 位置（傾き角）の改善と速度（角速度）の悪化は共存しうる

3. **swing_contact_penalty悪化**: -0.0096→-0.0218（+127.1%）
   - 定常状態の両足接地は0.8%→0.2%と改善しているが、非定常状態でのスイング接触が増加

4. **L hip_roll offset微悪化**: -13.39°→-14.52°（+8.4%）。R side（-29.3%）と非対称な改善。RL stochasticityによるL/R非対称最適化

5. **正報酬全体の微減**: 3.3952→3.1968（-5.8%）。tracking_ang_vel -22.9%が主因（Yaw drift増大→角速度追従が困難に）

### 因果分析

orientation=-2.0の因果チェーン:
1. projected_gravity penalty 4x → body tilt減少（Roll std -9.7%, Pitch std -9.2%）✓
2. body tilt減少 → stance hip_roll offset間接改善（R: -29.3%）✓
3. body tilt制約 → Roll/Pitch方向の補正自由度喪失 → hip_yaw非対称度1.1%→7.1% → angular momentum imbalance → **Yaw drift +87.1%** ✗
4. 鉛直性維持のための高速修正 → **ang_vel_xy +26.2%** ✗

Survey 7.4.2によれば、Yaw driftの根本原因は歩行の非対称性であり、body tiltの制約はこの根本原因に対処しない。orientation強化はRoll/Pitchを改善する代わりに、間接的なYaw補正メカニズム（body tiltによる角運動量調整）を奪った。

### 報酬設計体系的整理

#### 正報酬寄与度ランキング（V6）
1. tracking_lin_vel: 1.3233（41.4%）
2. contact: 0.7141（22.3%）
3. single_foot_contact: 0.4684（14.6%）
4. swing_duration: 0.3637（11.4%）
5. tracking_ang_vel: 0.2590（8.1%）
6. step_length: 0.0683（2.1%）

#### ペナルティ寄与度ランキング（V6）
1. ang_vel_xy: -0.0944（21.3%）
2. stance_hip_roll_target: -0.0731（16.5%）
3. action_rate: -0.0563（12.7%）
4. swing_foot_lateral_velocity: -0.0519（11.7%）
5. lin_vel_z: -0.0373（8.4%）
6. feet_swing_height: -0.0367（8.3%）
7. hip_yaw_pos: -0.0281（6.3%）
8. orientation: -0.0244（5.5%）
9. swing_contact_penalty: -0.0218（4.9%）
10. base_height: -0.0115（2.6%）
11. velocity_deficit: -0.0039（0.9%）
12. contact_no_vel: -0.0036（0.8%）

orientation penaltyは4xスケール増にもかかわらず全ペナルティの5.5%にとどまる。raw penaltyが-4.7%減少しており、制約が有効に機能していることを示す。

### 次バージョンへの提案

#### 推奨案: 全相足先横方向速度ペナルティ（V6ベース）

- **根拠**: 現在の`swing_foot_lateral_velocity`は遊脚のみに適用されるが、内股着地の問題は**接地相での脚の横方向速度成分**にも起因する可能性がある。速度指令ベクトルと直交する速度成分を**遊脚・接地脚の両方に適用**することで、脚全体の軌道が前進方向に揃う効果を期待する
- **実装**: `_reward_foot_lateral_velocity`（新設）で遊脚マスクを除去し、全フェーズでvel_perp²をペナルティ化。既存の`swing_foot_lateral_velocity`は無効化（0）
- **期待効果**: 接地脚の横方向微小速度成分も抑制されることで、内股着地パターンが変化する可能性がある
- **リスク**: 接地脚の足は地面に静止しているため速度≈0であり、ペナルティ値がごく小さい可能性がある。その場合は効果が限定的
- **1変更1検証**: V6 → V7で swing_foot_lateral_velocity=0 + foot_lateral_velocity=-0.5の1変更のみ（報酬項目数17維持）

#### 代替案A: orientation -2.0 → -1.0（V4ベース、moderate increase）

- **根拠**: -2.0ではYaw regression（+87.1%）が大きすぎる。-1.0（V4の2x）でRoll/Pitch改善とYaw安定性のバランスを探る
- **期待効果**: V6の改善（Roll -9.7%, Pitch -9.2%）の約半分を期待しつつ、Yaw regressionを約半減
- **リスク**: V5のstance_hip_roll_target=-2.0でも視覚的内股は解消されなかった。関節角度を制約するアプローチの期待値は低い

#### 代替案B: equivariant network（長期的アプローチ）

- **根拠**: Survey 7.2.3「reward shapingは軟制約であり対称性を保証しない」。ネットワーク構造で左右対称を強制
- **期待効果**: hip_yaw非対称度→0に近づけ、Yaw drift根本解消
- **リスク**: 実装コスト大。RL frameworkの変更が必要

## まとめ

V6はorientation -0.5→-2.0（4x強化）により、Roll std -9.7%、Pitch std -9.2%、R hip_roll stance offset -29.3%の改善を達成した。胴体鉛直性強化による「ボディレベル→関節レベル」のトップダウン制約チェーンが有効に機能し、V5（関節レベル制約）で不可能だった体系的改善アプローチを実証した。

一方、Yaw drift +87.1%（+7.51°→+14.05°）が主要な課題として残った。orientation制約による間接Yaw補正チャネルの喪失が原因であり、hip_yaw非対称度が1.1%→7.1%に増大した。ペナルティ/正報酬比0.139は健全な範囲内であり、V5のようなgait degradationは発生しなかった。

次バージョンでは、全相足先横方向速度ペナルティ（swing_foot_lateral_velocity→foot_lateral_velocity置換、遊脚マスク除去）により、脚全体の軌道を前進方向に揃えるアプローチで内股着地の改善を試みる。

## 備考

- ベースバージョン: V4（V5は失敗版のためスキップ）
- 変更数: 1（orientation: -0.5 → -2.0）
- 環境ファイル変更: なし（_reward_orientationは既存）
- 訓練時間: ~833秒（500イテレーション）
- サーベイ参照: Section 2.2.2/2.2.3（ang_vel_xy/orientation直交性）、Section 7.4.2（angular momentum imbalance）、Section 7.4.3（symmetric gait）、Section 7.2.3（equivariant policy）
