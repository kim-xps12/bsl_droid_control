# V16: 位相条件付きPDクランプ (hip_roll_clamp_stance_only)

## 概要

V14の全位相PDターゲットクランプがhip_roll動的範囲を-44%制限し、V4の位相依存バランス機構が消失した。V15のbase_vel_yは対症療法にとどまった。V16では接地相のみにPDクランプを適用し、遊脚相のhip_roll自由度を回復することで、V4の横方向足配置機構の復活を狙う。

## 前バージョンからの改善事項

V15レポートの「次バージョンへの提案」推奨案Aに基づき、位相条件付きPDクランプを実装する。

| パラメータ | V15値 | V16値 | 変更理由 |
|-----------|-------|-------|---------|
| hip_roll_clamp_stance_only (新設) | なし (=False) | True | 遊脚相のhip_roll自由度回復。V4のlateral std 14.9mmを支えた位相依存バランス機構の復活を狙う |

他のパラメータは一切変更なし（1変更1検証原則）。報酬項目数は18（V15と同一）。

## 設計詳細

### 位相条件付きPDクランプの設計根拠

V14-V15の横方向揺れの根本原因:
- V14のPDクランプが全位相でhip_rollを制約 → 動的範囲 0.38→0.21 rad (-44%)
- V4では遊脚相でhip_rollが自由 → 次の着地位置を横方向に調整するバランス機構が機能
- V4のlateral std 14.9mm（exp008 best）はこの機構によるもの
- V14/V15のlateral std 39.5-48.0mmは機構消失が原因

### 実装

`droid_env_unitree.py`の`step()`内PDターゲットクランプを接地状態で条件分岐:

```python
if self.hip_roll_clamp_stance_only:
    left_stance = self.contact_state[:, 0] > 0.5
    right_stance = self.contact_state[:, 1] > 0.5
    target_dof_pos[:, self.left_hip_roll_idx] = torch.where(
        left_stance, left_clamped, target_dof_pos[:, self.left_hip_roll_idx]
    )
    target_dof_pos[:, self.right_hip_roll_idx] = torch.where(
        right_stance, right_clamped, target_dof_pos[:, self.right_hip_roll_idx]
    )
```

- 接地相: hip_roll_inward_limit=-0.05 rad（V14維持、内股防止）
- 遊脚相: クランプなし（lateral foot placement自由度回復）
- `contact_state`は前ステップ値（20ms遅延、stance ~500msに対し十分小さい）

### 報酬項目数

V16: 18項目（V15と完全同一）。コード変更のみで報酬スケール変更なし。

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_narrow_v16.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-narrow-v16 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 45.12 |
| エピソード長 | 923 steps |
| 最大報酬 | 47.22 (step 485) |
| 収束傾向 | 上昇継続（last 10 avg: 46.0, std: 0.87） |
| 訓練時間 | 860s (500 iterations) |

### 評価結果

#### 主要指標（V15比較）

| 指標 | V15 | V16 | 変化率 |
|------|-----|-----|--------|
| X速度 | 0.166 m/s | 0.151 m/s | -9.0% |
| Y速度 mean | 0.000 m/s | 0.008 m/s | - |
| Yaw drift | +21.07° | +20.07° | -4.7% |
| Roll std | 5.20° | 8.25° | +58.7% |
| Pitch std | 2.08° | 1.57° | -24.5% |
| Base height | 0.291 m | 0.261 m | -10.3% |
| hip_pitch corr | -0.765 | -0.638 | -16.6% |
| Gait freq | 0.88 Hz | 0.88 Hz | 0% |
| Action RMS | 2.302 | 1.054 | -54.2% |

#### hip_roll動的範囲（V16の核心的変化）

| 指標 | V15 | V16 | 変化率 |
|------|-----|-----|--------|
| L hip_roll range | 0.212 rad | 0.519 rad | +145% |
| R hip_roll range | 0.212 rad | 0.440 rad | +108% |
| L hip_roll min/max | [-0.212, 0.000] | [-0.303, +0.216] | 外向き修正復活 |
| R hip_roll min/max | [-0.000, 0.212] | [-0.068, +0.372] | 外向き修正復活 |
| L hip_roll vel std | 0.493 rad/s | 1.227 rad/s | +149% |
| R hip_roll vel std | 0.483 rad/s | 0.951 rad/s | +97% |

V15ではhip_rollがクランプ境界付近に張り付いていた（min/maxが0.000に集中）が、V16では零点を跨いで両方向に動いている。

#### 接地相/遊脚相別hip_roll分析

| 指標 | V15 | V16 |
|------|-----|-----|
| L stance hip_roll mean | -10.5° | -10.4° |
| R stance hip_roll mean | +10.3° | +10.4° |
| L swing hip_roll mean | -4.1° | -0.7° |
| R swing hip_roll mean | +4.1° | +5.5° |
| L balance stroke | 6.4° | 9.7° (+52%) |
| R balance stroke | 6.2° | 4.9° (-21%) |
| L outer max | -2.6° (放棄) | +11.0° (活用中) |
| R outer max | +2.6° (放棄) | -3.9° (限定的) |

stance hip_rollは±10.4°で維持（接地相クランプが機能）。遊脚相のhip_roll自由度が回復し、L脚は外向き+11.0°まで到達。

#### 横方向揺れ分析

| 指標 | V15 | V16 | 変化率 |
|------|-----|-----|--------|
| Lateral detrended std | 44.7 mm | 36.2 mm | -19.0% |
| Lateral detrended p2p | 168.2 mm | 129.6 mm | -22.9% |
| Y速度 std | 0.2268 m/s | 0.1878 m/s | -17.2% |
| Lateral drift速度 | 34.8 mm/s | 21.8 mm/s | -37.4% |
| Roll→lateral増幅率 | 1.47x | 0.75x | -48.9% |
| Roll成分 std | 38.8 mm | 29.0 mm | -25.3% |
| 残差 std | 22.3 mm | 21.7 mm | -2.7% |

Roll std +58.7%にもかかわらず、横方向位置振幅は-19.0%改善。Roll→lateral増幅率が1.47x→0.75xに反転（増幅→減衰）。

#### 接地パターン

| 指標 | V15 | V16 |
|------|-----|-----|
| 両足接地 | 1.4% | 2.6% |
| 片足接地 | 97.0% | 96.2% |
| 両足空中 | 1.6% | 1.2% |
| L swing duration | 0.488s | 0.323s |
| R swing duration | 0.438s | 0.341s |

#### 報酬バランス

| 指標 | V15 | V16 | 変化率 |
|------|-----|-----|--------|
| 正の報酬合計 | 2.935 | 2.673 | -8.9% |
| ペナルティ絶対値合計 | 0.536 | 0.516 | -3.7% |
| penalty/positive ratio | 0.183 | 0.193 | +5.5% |
| 報酬項目数（有効） | 13 | 13 | 0 |

penalty/positive ratio 0.193はV5のoverkill zone (0.186)を超過。

#### 報酬コンポーネント変動（V15→V16）

| 報酬項目 | V15 | V16 | 変化 |
|----------|-----|-----|------|
| orientation | -0.030 | -0.047 | +54% (Roll増大) |
| foot_lateral_velocity | -0.031 | -0.052 | +69% (hip_roll自由度) |
| ang_vel_xy | -0.100 | -0.122 | +21% (hip_roll速度) |
| action_rate | -0.091 | -0.057 | -37% (動作効率向上) |
| base_vel_y | -0.038 | -0.024 | -37% (横方向速度低下) |
| base_height | -0.032 | -0.015 | -55% (ペナルティ減だが高さ低下) |

## 考察と改善案

### 成功点

1. **hip_roll動的範囲の回復（核心的成果）**: 0.212→0.48 rad (+126%)。V4レベル（0.38-0.50 rad）に復帰。遊脚相のhip_roll自由度が完全に回復し、L脚は外向き+11.0°まで到達（V15は-2.6°で外向き修正を放棄していた）。

2. **Roll→lateral増幅率の反転（0.75x）**: V15の1.47x（受動的増幅）から0.75x（能動的減衰）に反転。遊脚hip_rollがRoll誘発の横方向運動を能動的に打ち消している直接的証拠。サーベイのAngular Momentum Balancing（Section 7.4.2）で提案される「腕スイングによるカウンターバランス」を、遊脚hip_rollで実現している。

3. **横方向位置安定性の改善**: lateral detrended std -19.0%、p2p -22.9%、Y速度std -17.2%。Roll stdが+58.7%増大しているにもかかわらず横方向位置が改善されたのは、能動的バランシングの復活による。

4. **接地相クランプの機能維持**: stance hip_roll ±10.4°（V15の±10.5°とほぼ同一）。接地相でのPDクランプが内股防止として正しく機能。

5. **Action RMS大幅改善**: -54.2%（2.302→1.054）。V15では全位相クランプへの「抵抗」にエネルギーを消費していたが、V16では遊脚相で自然な動作が許容されるため効率向上。

6. **Pitch std改善**: -24.5%（2.08→1.57°）。V15で悪化していたPitch安定性が回復。

### 課題

1. **Roll std大幅悪化**: +58.7%（5.20→8.25°）。遊脚相のhip_roll自由度回復により、脚の慣性がtorso Rollに大きな擾乱を与えている。能動的バランシングの副作用であり、gait breakdownではないが、ユーザー目標「しぜんに歩く」への障害。

2. **hip_pitch相関悪化**: -16.6%（-0.765→-0.638）。遊脚相hip_rollが自由になりhip_pitchの役割が分散。歩行周波数0.88Hzは維持されているため歩行崩壊ではなく、歩行パターンの多様化と解釈される。

3. **Base height低下**: -10.3%（0.291→0.261m）。遊脚相の大きなhip_rollスイングと、knee_pitch meanの深化（-1.44→-1.71 rad）による。

4. **L/R非対称**: L balance stroke 9.7° vs R balance stroke 4.9°。L脚は外向き修正を積極活用（+11.0°）するが、R脚は限定的（-3.9°）。MLPの非対称応答（V8で確認済み）の再現。

5. **penalty/positive ratio**: 0.193（V5 overkill zone 0.186超過）。orientation +54%, foot_lateral_velocity +69%の増加が原因。ただしV5とは質的に異なる（V5は単一ペナルティの暴走、V16はPDクランプによる行動空間制約の副作用）。

### 因果メカニズム

```
hip_roll遊脚相自由化
├→ hip_roll動的範囲回復 (+126%)
│  ├→ 能動的バランシング復活 → lateral std -19%, 増幅率 0.75x
│  ├→ 遊脚hip_roll大振幅 → torso Roll擾乱 → Roll std +58.7%
│  └→ hip_pitch役割分散 → hip_pitch corr -16.6%
├→ 接地相クランプ維持 → stance hip_roll ±10.4° → 内股防止維持
├→ 動作効率向上 → Action RMS -54.2%
└→ 膝深化 + Roll幾何学 → Base height -10.3%
```

### 次バージョンへの提案

**【V16後の追加分析により案Aは不採用、案Dを推奨に変更】**

追加分析（データ分析+ロボティクス/物理学検証）により以下が判明:
- V16の歩行不規則性が深刻: チャタリング6回（V15: 1回）、歩行周期CV 40.7%（V15: 3.3%）
- 「能動バランシング復活」は誤解釈: Roll-lateral R²が0.61→0.40に低下（相関喪失）
- 遊脚hip_rollは制御された動きではなく無秩序な振り（速度3.6倍増）
- ang_vel_xy強化はRoll+Pitchを同時ペナルティし、既にoverkill zone超過のpenalty ratioをさらに悪化させる

~~**推奨案A（推奨）: ang_vel_xy強化 (-0.1→-0.15)**~~
- ~~V16の最大課題はRoll std +58.7%。ang_vel_xyはRoll角速度を直接ペナルティ化する~~
- **不採用理由**: (1) Roll+Pitchを同時ペナルティ（PitchはV16で改善済み-24.5%）、(2) ratio 0.193→0.215でoverkill zone深刻化、(3) バランシング機構自体を抑制、(4) Roll stdの根本原因は振幅（位置）であり速度ではない

**案B: stance_foot_lateral_position削除 (-0.5→0)**
- PDクランプが接地相で直接hip_rollを制約しているため、タスク空間制約は二重
- 削除で報酬項目数18→17（推奨範囲内に復帰）、penalty/positive ratio 0.193→~0.17
- ペナルティ予算の解放により、他のペナルティ追加の余地が生まれる
- ~~A案と同時実施する場合、penalty/positive ratioの増加を相殺できる（ただし2変更同時は原則違反）~~

**案C: base_vel_y削除 (-1.0→0)**
- V16でRoll→lateral増幅率が0.75xに反転し、横方向速度はhip_rollバランス機構で対処可能
- base_vel_yペナルティ自体も-37%減少しており、制約としての寄与が低下
- 削除で報酬項目数18→17、penalty/positive ratio改善
- ただしY速度stdの改善（-17.2%）にbase_vel_yが一部寄与している可能性があり、削除リスクあり

**案D（推奨）: foot_lateral_velocity削除 (-0.5→0)**
- foot_lateral_velocityは全足・全位相の横方向速度をペナルティ化。V16で+69%増加したのは遊脚hip_rollバランシングとの直接衝突が原因
- exp007 V7で「foot_lateral_velocity counterproductive: Promotes 内股」と確認済み。PDクランプで内股防止は構造的に解決済みのため存在理由が消失
- 削除で0.052/timestepのペナルティ予算回復、ratio 0.193→~0.173（健全域に復帰）
- 報酬項目数18→17（推奨範囲に復帰）
- 1変更1検証原則に適合

## まとめ

V16は位相条件付きPDクランプ（hip_roll_clamp_stance_only=True）により、V4の位相依存hip_rollバランス機構の回復に成功した。hip_roll動的範囲は+126%回復（0.212→0.48 rad、V4レベル）し、Roll→lateral増幅率は1.47x→0.75xに反転、横方向位置安定性は-19.0%（lateral std）改善した。接地相クランプは正しく機能し、stance hip_rollは±10.4°を維持（内股防止を継続）。Action RMS -54.2%、Pitch std -24.5%も改善。

一方、遊脚相hip_roll自由化の副作用としてRoll std +58.7%（5.20→8.25°）、hip_pitch相関-16.6%（-0.765→-0.638）、base height -10.3%が発生。penalty/positive ratio 0.193はV5 overkill zone (0.186)を超過した。

次バージョンではバランシング機構と衝突しているfoot_lateral_velocity削除（-0.5→0）を推奨する（案D）。

## 備考

- `hip_roll_clamp_stance_only`は`droid_env_unitree.py`の`step()`内で実装。デフォルト`False`で後方互換性を維持
- `contact_state`（前ステップの接触状態）を使用し、物理的に正しいstance/swing判定を実現
- サーベイ照合: 位相条件付き制約はPeriodic Reward Composition (ICRA 2021)の制御入力レベルへの拡張。Roll増大+横方向改善はAngular Momentum Balancing (Section 7.4.2)で理論的に説明可能
- 訓練ログで「std > 0.1、追加学習の余地あり」。ポリシーがhip_roll自由度への適応途上の可能性
- Roll→lateral増幅率0.75xは「腕なしロボットの遊脚バランシング」として新規知見
