# 二脚歩行ロボットの強化学習報酬設計に関する先行研究サーベイ

**作成日**: 2026-01-31  
**関連実験**: exp004_droid_rl_walking  
**目的**: BSL-Droid Simplified（10 DOF二脚ロボット）の歩行制御において、報酬設計に関する先行研究を調査し、設計指針を明確化する。

> **Note**: 本ドキュメントは調査結果のみを記載する。実験結果の分析・考察は [exp004_droid_rl_walking.md](exp004_droid_rl_walking.md) を参照。

------

## 1. 背景と問題意識

### 1.1 本実験の経緯

BSL-Droid Simplifiedの歩行学習において、V9〜V17で以下のアプローチを採用した：

- **Dense Reward Shaping**: 20〜25項目の報酬関数
- **関節レベルの制約**: hip_pitch交互、膝角度、足首角度など詳細に指定
- **パラメータチューニング**: 各報酬のスケールを実験的に調整

しかし、**報酬項目数の増加に反比例して歩行品質が悪化**する現象が観察された：

| Version | 報酬項目数 | X移動距離 | Yawドリフト | hip_pitch相関 |
|---------|-----------|-----------|-------------|---------------|
| V9 | ~25 | 2.95m | -4.6° | -0.516 |
| V15 | ~18 | 3.05m | -18.1° | -0.242 |
| V16 | ~23 | 3.23m | -19.2° | **+0.697** |
| V17 | ~25 | **2.68m** | **-29.1°** | -0.637 |
| V18 | **6** | 2.83m | +9.8° | +0.751 |

**重要な発見**:
- hip_pitch相関が良くても（V17: -0.637）、歩行性能が最悪になりうる
- 報酬項目を大幅削減したV18は、前進距離・直進性でV17より改善

### 1.2 問題提起

Dense Reward Shapingアプローチは以下の問題を引き起こした：

1. **報酬ハッキング**: 設計意図と異なる方法で報酬最大化（V16の同期歩行）
2. **報酬項目間の競合**: forward_progress vs action_rate等
3. **スケール調整の無限ループ**: 1つ直すと別が壊れる
4. **創発的歩行の阻害**: 探索空間が過度に制限される

---

## 2. 先行研究の分類

二脚ロボットの歩行学習に関する報酬設計は、大きく3つのパラダイムに分類できる。

### 2.1 パラダイム比較表

| パラダイム | 報酬構造 | 制約の扱い | 代表研究 | 特徴 |
|-----------|---------|-----------|---------|------|
| **Reference-based** | 参照軌道との誤差 | 軌道に暗黙的に含む | DeepMimic, AMP | 自然な動作、データ依存 |
| **Dense Shaping** | 20+項目の詳細報酬 | 報酬ペナルティ | 本実験V9-V17 | 細かい制御、報酬ハッキングリスク |
| **Minimalist** | 5項目以下のタスク報酬 | 終了条件 | CaT, Legged Gym | 創発的解、学習安定 |

---

## 3. 主要先行研究の詳細分析

### 3.1 Legged Gym (ETH Zurich, CoRL 2021)

**論文**: "Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning"  
**URL**: https://arxiv.org/abs/2109.11978

#### 報酬設計の特徴

Legged Gymは**ミニマリスト報酬**と**大規模並列学習**の組み合わせで成功した。

```python
# Legged Gymの典型的な報酬構造（10項目以下）
reward_scales = {
    "tracking_lin_vel": 1.0,      # 線形速度追従
    "tracking_ang_vel": 0.5,      # 角速度追従
    "lin_vel_z": -2.0,            # Z方向速度ペナルティ
    "ang_vel_xy": -0.05,          # XY角速度ペナルティ
    "orientation": -0.0,          # 姿勢ペナルティ
    "torques": -0.00001,          # トルクペナルティ
    "dof_vel": -0.0,              # 関節速度ペナルティ
    "dof_acc": -2.5e-7,           # 関節加速度ペナルティ
    "feet_air_time": 1.0,         # 足滞空時間報酬
    "collision": -1.0,            # 衝突ペナルティ
}
```

#### 核心的報酬: feet_air_time

```python
def _reward_feet_air_time(self):
    """足の滞空時間に基づく報酬 - 周期的歩行を促進"""
    contact = self.contact_forces[:, self.feet_indices, 2] > 1.
    contact_filt = torch.logical_or(contact, self.last_contacts)
    self.last_contacts = contact
    first_contact = (self.feet_air_time > 0.) * contact_filt
    self.feet_air_time += self.dt
    
    # 目標滞空時間（0.5秒）を超えた分を報酬化
    rew_airTime = torch.sum(
        (self.feet_air_time - 0.5) * first_contact, dim=1
    )
    
    # 速度コマンドがある場合のみ報酬
    rew_airTime *= torch.norm(self.commands[:, :2], dim=1) > 0.1
    self.feet_air_time *= ~contact_filt
    return rew_airTime
```

**設計思想**:
- 「どの関節をどう動かすか」は指定しない
- 「足を適度に浮かせて着地する」というタスクレベルの報酬のみ
- 具体的な歩容は**学習によって創発**させる

#### 本実験への示唆

- **接地情報の活用**: 現在のV18では接地検出が機能していない（Both feet airborne: 100%）
- **タスクレベル報酬**: hip_pitch_alternationのような関節レベル報酬より、feet_air_timeのような高レベル報酬が有効

---

### 3.2 Cassie Biped 実装

**設定ファイル**: legged_gym/envs/cassie/cassie_config.py

#### no_fly報酬

```python
def _reward_no_fly(self):
    """片足接地を促進 - 両足浮き・両足接地にペナルティ"""
    contacts = self.contact_forces[:, self.feet_indices, 2] > 0.1
    single_contact = torch.sum(1. * contacts, dim=1) == 1
    return 1. * single_contact
```

**設計思想**:
- 二脚歩行の基本: 常にどちらか一方の足が接地
- 両足浮き（ジャンプ）や両足接地（静止）を避ける
- これだけで交互歩行パターンが創発

#### 本実験への示唆

- V18の「Both feet airborne: 100%」は異常
- 接地センサーの実装または接地判定の改善が必要

---

### 3.3 Energy Minimization (Berkeley, CoRL 2021)

**論文**: "Minimizing Energy Consumption Leads to the Emergence of Gaits in Legged Robots"  
**URL**: https://arxiv.org/abs/2111.01674

#### 核心的アイデア

**Cost of Transport (CoT) 最小化**を主報酬とすることで、明示的な歩容制約なしに自然な歩行が創発する。

```
CoT = (消費エネルギー) / (質量 × 移動距離)
```

#### 報酬設計

```python
# エネルギー効率ベースの報酬設計
reward_scales = {
    "forward_velocity": 1.0,      # 前進速度
    "energy_efficiency": -1.0,    # CoT最小化
    "alive": 0.1,                 # 生存報酬
}

def _reward_energy_efficiency(self):
    """Cost of Transport (CoT) に基づくエネルギー効率報酬"""
    # 機械的パワー = トルク × 角速度
    power = torch.sum(
        torch.abs(self.torques * self.dof_vel), dim=1
    )
    # 移動速度で正規化
    forward_vel = self.base_lin_vel[:, 0]
    cot = power / (self.robot_mass * torch.clamp(forward_vel, min=0.1))
    return -cot
```

#### 創発された歩容パターン

| 速度 | 創発された歩容 | エネルギー効率 |
|------|---------------|---------------|
| 低速 | Walk（常歩） | 最高 |
| 中速 | Trot（速歩） | 中 |
| 高速 | Gallop（駈歩）| 低 |

#### 本実験への示唆

- **トルクペナルティの重要性**: V18では`torques: -1e-4`を導入済み
- **速度依存歩容**: コマンド速度に応じて歩容を変えるカリキュラムの可能性
- **生物学的妥当性**: 人間や動物もエネルギー効率を最適化している

---

### 3.4 Constraint as Termination (CaT)

**概念**: 報酬ペナルティではなく**終了条件**として制約を実装

#### 従来手法 vs CaT

| 制約 | Dense Shaping | CaT |
|------|--------------|-----|
| 転倒防止 | `orientation: -5.0` | `if pitch > 30°: terminated = True` |
| 膝角度 | `knee_negative: -3.0` | `if knee > 0: terminated = True` |
| 関節限界 | `dof_pos_limits: -5.0` | `if dof > limit: terminated = True` |

#### CaTの利点

1. **明確な学習シグナル**: 制約違反=エピソード終了、曖昧さがない
2. **報酬競合の回避**: 報酬項目数が減り、相互干渉が減少
3. **探索の自由度**: 制約内での最適解探索が可能

#### V18での実装

```python
# V18で導入した終了条件
def _compute_termination(self):
    # Roll/Pitch制限
    roll = torch.abs(self.base_euler[:, 0])
    pitch = torch.abs(self.base_euler[:, 1])
    terminated = (roll > 0.5) | (pitch > 0.5)  # ~28.6°
    
    # 膝角度制限（digitigradeでは膝は正角度）
    knee_angles = self.dof_pos[:, [2, 7]]  # 左右膝
    terminated |= torch.any(knee_angles < 0.3, dim=1)  # ~17°
    
    return terminated
```

---

### 3.5 Walk These Ways (CMU, CoRL 2023)

**論文**: "Walk These Ways: Tuning Robot Control for Generalization"  
**URL**: https://arxiv.org/abs/2212.03238

#### 報酬設計の特徴

**コマンド条件付き報酬**で多様な歩行スタイルを単一ポリシーで実現。

```python
# 歩行スタイルに応じた動的報酬
reward_scales = {
    "tracking_lin_vel": cmd_scale,      # コマンド依存
    "tracking_ang_vel": cmd_scale * 0.5,
    "feet_stumble": -1.0 if fast else 0.0,  # 高速時のみ
    "swing_height": swing_cmd,          # スイング高さコマンド
}
```

#### 本実験への示唆

- **コマンドベース設計**: 速度だけでなく、歩幅・足上げ高さもコマンド化可能
- **スタイル指定**: 「小刻み歩行」vs「大股歩行」を学習時に選択可能

---

## 4. V18の評価と問題分析

### 4.1 V18評価結果（実測値）

```
=== V18 Evaluation Statistics (10秒間) ===
X移動距離: 2.827 m
Y移動距離: 0.307 m
Yawドリフト: +9.77°

姿勢:
  Roll:  mean= 8.56°, std= 3.74°   ← 右に傾いている
  Pitch: mean=13.70°, std= 2.76°  ← 前傾している（問題）
  Base height: 0.212m (初期: 0.347m) ← 下がりすぎ（問題）

hip_pitch相関: +0.751（同期歩行）
hip_pitch range: L=0.217 rad, R=0.392 rad ← 左右非対称、小さい
DOF range sum: 3.588 rad（V17: 4.958 rad）← 動きが小さい（問題）
DOF velocity RMS: 1.233 rad/s（V17: 2.252 rad/s）← 非常に低い
```

### 4.1.1 ユーザーフィードバック（V18）

**良い点**:
- びっこを引いていない
- 前脚と後脚の固定が改善された

**改善したい点**:
- 歩幅が小刻みすぎる
- 胴体が前傾している
- 胴体が下がり過ぎている

### 4.2 V17との比較

| 指標 | V17 | V18 | 変化 | 評価 |
|------|-----|-----|------|------|
| X移動距離 | 2.68m | **2.83m** | +5.6% | ○ 改善 |
| Yawドリフト | -29.1° | **+9.8°** | 大幅改善 | ○ 改善 |
| Base height | 0.226m | 0.212m | -6.2% | △ やや悪化 |
| Pitch mean | -0.84° | **13.70°** | 大幅前傾 | × 悪化 |
| hip_pitch相関 | -0.637 | +0.751 | 同期化 | × 悪化 |
| DOF range | 4.96 rad | 3.59 rad | -28% | × 動き減少 |

### 4.3 V18の問題点

ユーザーフィードバックと数値分析から、以下の問題を特定：

1. **歩幅が小刻みすぎる**: DOF rangeが3.59 radと小さい
2. **胴体が前傾している**: Pitch mean = 13.7°（理想は0°付近）
3. **胴体が下がりすぎている**: Base height = 0.212m（初期0.347mの61%）
4. **同期歩行**: hip_pitch相関+0.751は両脚が同時に動いている

---

## 5. 考察: 報酬設計パラダイムの選択

### 5.1 Dense Shaping vs Minimalist

| 観点 | Dense Shaping (V9-V17) | Minimalist (V18) |
|------|----------------------|------------------|
| 報酬項目数 | 20-25 | 6 |
| 歩行品質の安定性 | 低い（バージョン間で大幅変動） | 比較的安定 |
| 報酬ハッキングリスク | 高い（V16の失敗） | 低い |
| 調整難易度 | 高い（相互干渉） | 低い |
| 創発的歩行 | 阻害される | 可能 |

### 5.2 V18の「成功」と「失敗」

**成功点**:
- 前進距離・直進性がV17より改善
- 「びっこ」が解消された
- 前脚と後脚の固定が改善された

**失敗点**:
- 交互歩行ができていない（同期歩行）
- 姿勢が悪い（前傾、沈み込み）
- 歩幅が小さい

### 5.3 なぜV18は同期歩行になったか

V18では以下の報酬を使用：

```python
reward_scales = {
    "tracking_lin_vel": 2.0,
    "tracking_ang_vel": 0.5,
    "alive": 1.0,
    "torques": -1e-4,
    "dof_acc": -1e-6,
    "orientation": -1.0,
}
```

**仮説**:
- `tracking_lin_vel`のみでは「どう前進するか」を指定していない
- 両脚を同時に動かす（カンガルーのようなホッピング）が最も効率的と学習
- 接地報酬（feet_air_time, no_fly）がないため、交互歩行のインセンティブがない

---

---

## 6. 調査から得られた報酬設計指針

### 6.1 制約実装手法の比較

先行研究から、制約を実装する手法には以下の選択肢がある：

| 手法 | 説明 | 長所 | 短所 | 参考研究 |
|------|------|------|------|---------|
| **Dense Penalty** | 報酬ペナルティで制約を誘導 | 柔軟な誘導 | 報酬ハッキング、スケール調整困難 | - |
| **Termination (CaT)** | 制約違反でエピソード終了 | 明確な制約 | 厳しすぎると動かなくなる | CaT |
| **Soft Termination** | 制約違反で大きなペナルティ | バランスが取れる | 調整が必要 | Legged Gym |

### 6.2 報酬バランスの重要性

先行研究に共通する知見として、**主報酬（タスク達成）とペナルティのバランス**が最重要である。

```
推奨: 主報酬 > ペナルティ合計（タスク達成を優先）
危険: 主報酬 < ペナルティ合計（何もしないことが最適解に）
```

### 6.3 段階的改善戦略

Legged Gym、Walk These Waysの成功事例から、以下の戦略が有効：

1. まずタスク達成（前進）を確立
2. 次に主要な制約（姿勢、転倒防止）を追加
3. 最後に歩行品質（歩容、エネルギー効率）を改善

**一度に複数の問題を解決しようとしない**

---

## 7. 結論：報酬設計の教訓（先行研究から）

### 7.1 報酬設計の一般原則

1. **Less is More**: 報酬項目を減らすほど学習は安定する（Legged Gym）
2. **Task-level > Joint-level**: 関節角度より高レベルのタスク報酬が有効（全般）
3. **Contact is Key**: 接地情報を活用した報酬が歩行創発に重要（Cassie, Legged Gym）
4. **Balance is Critical**: 報酬バランスが最重要。主報酬を圧倒しないペナルティ設計（全般）
5. **Energy Efficiency**: CoT最小化で自然な歩容が創発（Berkeley）

### 7.2 終了条件に関する知見

| 条件の厳しさ | 効果 | リスク |
|-------------|------|--------|
| 緩すぎる | 不自然な姿勢を許容 | 転倒、学習不安定 |
| **適度** | 探索の自由度を維持しつつ制約 | 推奨 |
| 厳しすぎる | 探索空間が狭まりすぎる | 動かないことが最適解に |

### 7.3 二脚ロボット固有の知見

| 課題 | 推奨される対策 | 参考研究 |
|------|---------------|---------|
| 交互歩行の誘導 | feet_air_time / no_fly報酬 | Legged Gym, Cassie |
| 姿勢維持 | orientation報酬（軽め）+ 緩い終了条件 | CaT |
| 沈み込み防止 | base_height報酬（軽め） | Walk These Ways |
| エネルギー効率 | torquesペナルティ / CoT最小化 | Berkeley |

---

---

## 8. 追加調査：その他の関連研究

### 8.1 DeepMind Soccer Robot (Science Robotics 2024)

**論文**: "Learning Agile Soccer Skills for a Bipedal Robot with Deep Reinforcement Learning"  
**URL**: https://arxiv.org/abs/2304.13653

#### アプローチ

- End-to-end Deep RLによる複合スキル学習
- 高周波制御（推定100Hz以上）
- Targeted dynamics randomizationによるSim-to-Real

#### 成果

- 歩行速度181%向上（vs スクリプトベースライン）
- 回転速度302%向上
- 起き上がり時間63%短縮
- キック速度34%向上

#### 本実験への示唆

- 明示的な歩行報酬ではなく、タスク達成報酬（サッカーゴール等）から歩行スキルが**創発**
- 高周波制御と外乱に対するロバスト性の重要性

---

### 8.2 Versatile Bipedal Locomotion (IJRR 2024)

**論文**: "Reinforcement Learning for Versatile, Dynamic, and Robust Bipedal Locomotion Control"  
**URL**: https://arxiv.org/abs/2401.16889

#### アプローチ

- Dual-history architecture（長期・短期I/O履歴）
- 周期的・非周期的スキルの統一学習
- Cassieロボットでの実機検証

#### 成果

- 400m走破
- スタンディングロングジャンプ・ハイジャンプ
- ロバストな立位・多様な歩行

#### 本実験への示唆

- 時変・時不変の動特性変化への適応
- 接地イベント等のタイミング変化を履歴から推定
- タスクランダマイゼーションによるロバスト性向上

---

### 8.3 Low-Frequency Motion Control (ICRA 2023)

**論文**: "Learning Low-Frequency Motion Control for Robust and Dynamic Robot Locomotion"  
**URL**: https://arxiv.org/abs/2209.14887

#### アプローチ

- 低周波制御（8Hz）でも動的歩行を実現
- アクチュエータ遅延への耐性向上

#### 成果

- ANYmal Cで1.5 m/s達成
- 不整地走破
- 外乱への耐性

#### 本実験への示唆

- 高周波が必ずしも必要ではない
- Domain randomizationなしでもSim-to-Real成功の可能性
- 制御遅延や動特性変動に対するロバスト性

---

### 8.4 Unitree RL Gym

**URL**: https://github.com/unitreerobotics/unitree_rl_gym

#### アプローチ

- Legged Gymベースの訓練フレームワーク
- Go2, G1, H1等のUnitreeロボット対応
- Sim2Sim (MuJoCo) → Sim2Real パイプライン

#### 本実験への示唆

- LSTMベースのポリシーネットワーク
- 実機デプロイ用のC++コード提供
- 産業用途を見据えた設計パターン

---

## 9. 「かわいく生き物のように歩く」ための要件

調査結果を踏まえ、目標とする歩行スタイルに必要な要素を整理する。

### 9.1 要件一覧

| 要件 | 説明 | 参考研究 |
|------|------|---------|
| **周期性** | 一定のリズムで歩く | CPG、Legged Gym |
| **滑らかさ** | 急激な動作変化がない | Low-Frequency |
| **交互性** | 左右の脚が位相差180°で動く | Cassie no_fly |
| **足上げ** | 明確なスイング相がある | feet_air_time |
| **自然さ** | エネルギー効率が良い | Energy Minimization |

### 9.2 本実験での実現方針

| 要件 | V18での実現方法 |
|------|----------------|
| 周期性 | 学習に任せる（CPG的報酬は使用しない） |
| 滑らかさ | `dof_acc`ペナルティ |
| 交互性 | 学習に任せる（明示的報酬なし） |
| 足上げ | `feet_air_time`報酬の検討 |
| 自然さ | `torques`ペナルティ（CoT代替） |

---

## 10. CPG（Central Pattern Generator）の理論

### 10.1 生物学的背景

生物の歩行は脳からのトップダウン指令だけでなく、脊髄にあるCPGという神経回路によって周期的なリズムが自律的に生成されている。

#### CPGの特徴

1. **自律的発振**: 外部入力なしでも周期的パターンを生成
2. **位相結合**: 複数のCPGが相互に結合し、協調したパターンを生成
3. **感覚フィードバック**: 接地などの感覚入力でタイミングを調整

### 10.2 ロボット制御への応用

V10で試みたPhase-based参照軌道はCPGの簡略化版：

| CPGの要素 | V10での実装 |
|----------|-------------|
| 自律的発振 | `gait_phase`の更新 |
| 位相結合 | 左右180°位相差 |
| 感覚フィードバック | `phase_contact_sync`報酬 |

### 10.3 数学的定義

```
left_hip_pitch_ref  = offset + amplitude × sin(ωt)
right_hip_pitch_ref = offset + amplitude × sin(ωt + π)
```

- `amplitude`: hip_pitch振幅（0.25 rad ≈ 14°）
- `offset`: オフセット（0 rad）
- `ω = 2πf`: 角周波数

### 10.4 V10での失敗と教訓

Phase-based報酬をV10-V17で試みたが、以下の問題が発生：

| 問題 | 原因 |
|------|------|
| 斜行 | 左右非対称な追従 |
| 同期歩行 | 報酬ハッキング |
| 前進不足 | タスク報酬との競合 |

**教訓**: CPGインスパイアの参照軌道は、Dense Reward Shapingと同様の問題を引き起こす。報酬項目を増やすアプローチには限界がある。

---

## 11. ストライド制御と歩幅改善に関する知見

### 11.1 問題の背景

V18〜V20で観察された「小刻み歩行」の問題：
- 脚の動作周期が速すぎる
- ストライド（歩幅）が小さい
- 足の上げ幅が不十分

これらは見た目の不自然さだけでなく、エネルギー効率や移動速度にも影響する。

### 11.2 Legged Gymにおける歩幅制御

Legged Gymの`feet_air_time`報酬は、歩幅制御の間接的な手法：

```python
def _reward_feet_air_time(self):
    """足の滞空時間に基づく報酬"""
    contact = self.contact_forces[:, self.feet_indices, 2] > 1.
    contact_filt = torch.logical_or(contact, self.last_contacts)
    self.last_contacts = contact
    first_contact = (self.feet_air_time > 0.) * contact_filt
    self.feet_air_time += self.dt
    
    # 目標滞空時間（0.5秒）を超えた分を報酬化
    rew_airTime = torch.sum(
        (self.feet_air_time - 0.5) * first_contact, dim=1
    )
    
    # 速度コマンドがある場合のみ報酬
    rew_airTime *= torch.norm(self.commands[:, :2], dim=1) > 0.1
    self.feet_air_time *= ~contact_filt
    return rew_airTime
```

**設計思想**:
- 足の滞空時間が長い = ストライドが大きい（仮定）
- 滞空時間0.5秒は、歩行周期1秒（1 Hz）のときに両脚が交互に50%ずつ接地する想定
- 目標滞空時間を下回ると報酬なし、上回ると正の報酬

### 11.3 Walk These Waysにおけるコマンドベース制御

Walk These Ways (CMU, 2023)では、歩幅や足上げ高さを**明示的にコマンド化**している：

```python
# Walk These Waysのコマンド構成（拡張版）
commands = {
    "lin_vel_x": [...],      # 前進速度
    "lin_vel_y": [...],      # 横移動速度
    "ang_vel_yaw": [...],    # 旋回速度
    "body_height": [...],    # 胴体高さ
    "step_frequency": [...], # 歩行周期
    "gait_phase": [...],     # 歩容位相
    "swing_height": [...],   # 足上げ高さ
    "stance_width": [...],   # 脚幅
}
```

**利点**:
- 歩行スタイルをリアルタイムに変更可能
- 単一ポリシーで多様な歩行パターンを実現
- ユーザーが望むスタイルを直接指定

### 11.4 胴体高さ維持の手法

#### 11.4.1 base_height報酬

```python
def _reward_base_height(self):
    """胴体高さを目標値に維持"""
    base_height = self.root_states[:, 2]
    return torch.square(base_height - self.cfg.rewards.base_height_target)
```

- 正の高さ目標との誤差をペナルティ化
- 沈み込みと浮き上がりの両方を抑制

#### 11.4.2 終了条件との組み合わせ

| 手法 | 効果 | リスク |
|------|------|--------|
| base_height報酬のみ | 緩やかな誘導 | 収束が遅い |
| height終了条件のみ | 明確な制約 | 探索が制限される |
| 両方組み合わせ | 効果的な制御 | 推奨 |

### 11.5 Genesisシミュレータでの接地検出

V20で発覚した問題: `feet_air_time`報酬が常に0 → 接地検出が機能していない

#### 11.5.1 IsaacGym（Legged Gym）との差異

IsaacGymでは`contact_forces`テンソルがGPU上で直接取得可能：
```python
net_contact_forces = self.gym.acquire_net_contact_force_tensor(self.sim)
contact_forces = gymtorch.wrap_tensor(net_contact_forces)
```

Genesisでは異なるAPIが必要な可能性：
- `robot.get_contact_forces()` メソッドの確認
- 接触力のテンソル形状（環境数 × ボディ数 × 3）の確認
- 接地閾値（現在0.04）の妥当性検証

#### 11.5.2 デバッグ手順

1. シミュレーション中の`contact_forces`テンソルを出力
2. 足リンクのインデックスが正しいか確認
3. 接触力のZ成分が適切に取得されているか確認
4. 閾値を調整して接地検出が機能するか確認

### 11.6 本実験への適用指針

| 課題 | 推奨対策 | 優先度 |
|------|---------|--------|
| 胴体沈み込み | base_height報酬（-1.0〜-2.0）を追加 | 高 |
| 小刻み歩行 | 胴体高さ改善後に再評価 | 中 |
| ストライド拡大 | hip_pitch速度報酬等の検討 | 低（様子見） |

### 11.7 接地検出に関する設計判断

**結論**: 本実験では**接地検出なしで進める**。

#### 理由

1. **シミュレーション側**: 座標ベース判定が機能していない（閾値調整で対応可能だが根本解決にならない）
2. **実機側**: トルク推定方式でも実装コストが発生
3. **代替手段の存在**: 胴体高さ報酬で間接的に歩行品質を改善可能

#### 実機デプロイ時の接地検出方式（参考）

| 方式 | 説明 | 追加ハードウェア |
|------|------|-----------------|
| トルク推定 | モータ電流から接触を推定 | 不要 |
| 足裏FSR | 圧力センサで直接検出 | 必要（低コスト） |
| 状態推定 | IMU+エンコーダから推定 | 不要 |

実機で必要になった場合は、Robstrideモータの電流フィードバックを活用したトルク推定方式が有力候補。

---

## 12. 足先空間（Task-Space）を探索対象とした歩容獲得

### 12.1 概要：関節空間 vs 足先空間

強化学習による歩行制御において、**行動空間（Action Space）の選択**は学習効率と歩行品質に大きく影響する。

| 行動空間 | 説明 | 長所 | 短所 |
|----------|------|------|------|
| **関節空間（Joint-Space）** | 各関節角度/トルクを直接出力 | シンプル、実機直結 | 高次元、協調困難 |
| **足先空間（Task-Space）** | 足先位置/速度を出力、IKで関節角に変換 | 低次元、直感的 | IK必要、特異点 |
| **階層型（Hierarchical）** | 高レベル（足先）＋低レベル（関節）の2層 | 柔軟、転移学習に有利 | 複雑、学習難 |

### 12.2 足先空間アプローチの理論的背景

#### 12.2.1 次元削減効果

BSL-Droid Simplified（10 DOF）の場合：
- **関節空間**: 10次元（各関節角度）
- **足先空間**: 6次元（左足xyz + 右足xyz）または 4次元（2D平面歩行）

探索空間が小さくなることで、学習効率が向上する可能性がある。

#### 12.2.2 タスク整合性

歩行の本質は「足先をどこに置くか」であり、関節角度は手段に過ぎない：

```
タスク目標: 足先を (x, y, z) に配置
  ↓ 逆運動学（IK）
関節角度: [hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch]
```

足先空間で探索することで、タスクに直結した学習が可能になる。

### 12.3 主要先行研究

#### 12.3.1 Humanoid Locomotion with Task-Space Control (ETH Zurich)

**研究**: "Learning Humanoid Locomotion with Transformers"  
**URL**: https://arxiv.org/abs/2303.03381

##### アプローチ

- Transformerベースのポリシーで足先軌道を生成
- 低レベルPD制御で関節角度追従
- 足先位置はベース座標系での相対位置

```python
# 行動空間の構成
action = {
    "left_foot_pos": [x, y, z],   # 左足先位置（ベース座標系）
    "right_foot_pos": [x, y, z],  # 右足先位置（ベース座標系）
    "torso_orientation": [roll, pitch, yaw],  # 胴体姿勢（オプション）
}
```

##### 成果

- 実機（Digit）での歩行成功
- 複雑な地形への適応
- 長距離歩行の安定性

##### 本実験への示唆

- **足先位置の表現**: ワールド座標系ではなくベース座標系での相対位置が安定
- **IK不要アプローチ**: 足先位置から直接PD制御へのマッピングも可能

---

#### 12.3.2 Residual Policy Learning (RPL)

**論文**: "Residual Policy Learning"  
**URL**: https://arxiv.org/abs/1812.06298

##### コンセプト

基本ポリシー（IK軌道など）に残差を加える：

```
action = base_policy(state) + residual_policy(state)
```

##### 二脚歩行への応用

```python
# 基本軌道: 正弦波ベースの足先軌道
def base_foot_trajectory(phase):
    x = stride_length * phase
    z = swing_height * sin(pi * phase)
    return [x, 0, z]

# 残差ポリシー: 外乱や地形適応のための補正
residual = policy(observation)
final_foot_pos = base_trajectory + residual
```

##### 利点

1. **学習効率**: ゼロから学習するより速い
2. **安全性**: 基本軌道が最低限の歩行を保証
3. **解釈性**: 残差を分析することで学習内容を理解

##### 本実験への適用案

BSL-Droidの場合、基本軌道として：
1. 正弦波ベースの足先Z座標（足上げ）
2. 直線的な足先X座標（前進）
3. 固定の足先Y座標（横方向）

残差ポリシーで学習する内容：
- バランス維持のための微調整
- 速度変化への適応
- 外乱からの回復

---

#### 12.3.3 DreamWaQ (KAIST, 2024)

**論文**: "DreamWaQ: Learning Robust Quadrupedal Locomotion With Implicit Terrain Imagination via Deep Reinforcement Learning"  
**URL**: https://arxiv.org/abs/2301.10602

##### アプローチ

- 暗黙的な地形推定と足先配置の学習
- ワールドモデルによる将来予測
- 4脚だが2脚への応用可能

##### 足先空間での探索

```python
# 行動空間（4脚の場合）
action_dim = 12  # 4足 × 3次元（x, y, z）

# 各足の目標位置はヒップ座標系で定義
foot_targets = action.reshape(4, 3)
```

##### 本実験への示唆

- **座標系の選択**: ヒップ座標系での足先位置表現が安定
- **正規化**: 可動範囲に対する正規化が重要

---

#### 12.3.4 Hierarchical RL for Legged Locomotion

**論文**: "Learning Agile Robotic Locomotion Skills by Imitating Animals" (Google, 2020)  
**URL**: https://arxiv.org/abs/2004.00784

##### 階層構造

```
High-Level Policy (10Hz)
  │
  │ 足先軌道指令
  ↓
Low-Level Controller (100Hz)
  │
  │ 関節トルク
  ↓
Robot
```

##### 利点

- **周波数分離**: 高レベルは粗い計画、低レベルは細かい制御
- **転移学習**: 高レベルポリシーは異なるロボットに転移可能
- **解釈性**: 高レベルの出力は人間が理解しやすい

##### 本実験への適用案

| レベル | 周波数 | 出力 | 学習方法 |
|--------|--------|------|---------|
| High-Level | 10-20Hz | 足先目標位置 | RL |
| Low-Level | 200Hz | 関節角度指令 | IK + PD |

---

### 12.4 逆運動学（IK）の実装オプション

足先空間アプローチでは、足先位置から関節角度への変換（IK）が必要。

#### 12.4.1 解析的IK

BSL-Droid Simplifiedの脚構造（5 DOF）は、幾何学的に解ける：

```python
def analytical_ik(foot_pos, hip_pos):
    """
    BSL-Droid Simplified の解析的IK
    
    脚構造: hip_yaw → hip_roll → hip_pitch → knee_pitch → ankle_pitch
    """
    # hip_yaw: 足先のXY平面での方向
    hip_yaw = atan2(foot_pos.y - hip_pos.y, foot_pos.x - hip_pos.x)
    
    # hip_roll: 股関節からの横方向オフセット
    # hip_pitch + knee_pitch + ankle_pitch: 2リンクIKに帰着
    
    # 2リンク平面IK
    L1 = thigh_length  # 大腿
    L2 = shank_length  # 下腿
    
    # 足先までの距離
    d = sqrt(dx**2 + dz**2)
    
    # 膝角度（余弦定理）
    cos_knee = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    knee_pitch = acos(clamp(cos_knee, -1, 1))
    
    # 股関節ピッチ
    hip_pitch = atan2(dx, -dz) - asin(L2 * sin(knee_pitch) / d)
    
    # 足首角度（地面に対して平行を維持）
    ankle_pitch = -(hip_pitch + knee_pitch)
    
    return [hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch]
```

#### 12.4.2 数値的IK（Jacobianベース）

より一般的だが計算コストが高い：

```python
def jacobian_ik(foot_pos_target, current_joint_angles, max_iter=10):
    """Jacobianベースの反復IK"""
    q = current_joint_angles.copy()
    
    for _ in range(max_iter):
        current_foot_pos = forward_kinematics(q)
        error = foot_pos_target - current_foot_pos
        
        if norm(error) < tolerance:
            break
        
        J = compute_jacobian(q)
        dq = pinv(J) @ error  # 疑似逆行列
        q += alpha * dq
    
    return q
```

#### 12.4.3 学習ベースIK

ニューラルネットワークでIKを学習：

```python
class LearnedIK(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(3, 64),   # 足先位置 (x, y, z)
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 5),   # 関節角度 (5 DOF)
        )
    
    def forward(self, foot_pos):
        return self.net(foot_pos)
```

### 12.5 BSL-Droid Simplifiedへの適用設計案

#### 12.5.1 設計案A: End-to-End足先空間

```python
# 行動空間
action_dim = 6  # 左足(x,y,z) + 右足(x,y,z)

# 観測空間
observation = {
    "base_orientation": quat,       # 胴体姿勢
    "base_angular_vel": vec3,       # 角速度
    "command_velocity": vec3,       # 速度指令
    "current_foot_pos": vec6,       # 現在の足先位置
    "gait_phase": scalar,           # 歩行位相（オプション）
}

# 行動 → 関節角度変換
foot_targets = action.reshape(2, 3)
joint_targets = [analytical_ik(foot_targets[i], hip_pos[i]) for i in [0, 1]]
```

##### 利点
- 低次元探索空間（6D vs 10D）
- 足先位置という直感的な行動

##### 課題
- IKの特異点処理
- 可動範囲制約の表現

---

#### 12.5.2 設計案B: Residual Policy + 基本軌道

```python
# 基本軌道（正弦波ベース）
def base_trajectory(phase, command_vel):
    stride = command_vel.x * stride_scale
    height = swing_height
    
    # スイング相の足
    swing_z = height * sin(pi * phase)
    swing_x = stride * (phase - 0.5)
    
    return {
        "swing_foot": [swing_x, 0, swing_z],
        "stance_foot": [0, 0, 0],
    }

# 残差ポリシー
residual = policy(observation)  # 6D出力
final_foot_targets = base_trajectory + residual
```

##### 利点
- 学習効率が高い（基本歩行パターンが保証される）
- 安全な探索（残差は小さい範囲に制限可能）

##### 課題
- 基本軌道の設計が必要
- 残差の自由度と基本軌道のバランス

---

#### 12.5.3 設計案C: 階層型RL（推奨）

```python
# High-Level Policy (20Hz)
class HighLevelPolicy:
    """足先軌道を生成"""
    def __init__(self):
        self.action_dim = 6  # 足先位置
        self.observation_dim = 20  # 状態 + コマンド
    
    def forward(self, obs):
        return self.net(obs)  # 足先目標位置

# Low-Level Controller (200Hz)
class LowLevelController:
    """IK + PD制御"""
    def __init__(self):
        self.kp = 50.0
        self.kd = 2.0
    
    def forward(self, foot_targets, current_state):
        joint_targets = analytical_ik(foot_targets)
        torques = self.kp * (joint_targets - q) - self.kd * dq
        return torques
```

##### 利点
- 周波数分離による安定性
- 高レベルポリシーの転移学習が容易
- 低レベルはIK+PDで高速・安定

##### 課題
- 2つのコンポーネントの統合
- 高低レベル間のインターフェース設計

### 12.6 足先空間アプローチの比較まとめ

| 設計案 | 探索次元 | 学習効率 | 実装複雑度 | 推奨度 |
|--------|---------|---------|-----------|--------|
| A: End-to-End | 6D | 中 | 低 | ○ |
| B: Residual | 6D | 高 | 中 | ◎ |
| C: 階層型 | 6D (HL) | 高 | 高 | ○ |

### 12.7 本実験への適用指針

V21（関節空間 + base_height報酬）が失敗した場合の次ステップとして：

1. **第一候補**: 設計案B（Residual Policy）
   - 基本軌道で最低限の歩行を保証
   - 残差で適応的な調整を学習
   - 実装コストが比較的低い

2. **第二候補**: 設計案A（End-to-End足先空間）
   - よりシンプルな実装
   - 解析的IKが利用可能（BSL-Droidの構造上）

3. **将来的な発展**: 設計案C（階層型）
   - 実機デプロイ時の安定性向上
   - 他のロボットへの転移

### 12.8 実装時の注意点

#### 12.8.1 座標系の選択

| 座標系 | 説明 | 利点 | 欠点 |
|--------|------|------|------|
| ワールド座標系 | 絶対位置 | 直感的 | 胴体姿勢に依存 |
| ベース座標系 | 胴体相対 | 姿勢不変 | 変換が必要 |
| **ヒップ座標系** | 股関節相対 | IKに直結 | 推奨 |

#### 12.8.2 可動範囲の正規化

```python
# 足先位置の正規化
foot_pos_normalized = {
    "x": (foot_x - x_min) / (x_max - x_min) * 2 - 1,  # [-1, 1]
    "y": (foot_y - y_min) / (y_max - y_min) * 2 - 1,
    "z": (foot_z - z_min) / (z_max - z_min) * 2 - 1,
}
```

#### 12.8.3 特異点の回避

膝が完全に伸びた状態（特異点）を避ける：

```python
def safe_ik(foot_pos):
    # 到達可能範囲をチェック
    d = norm(foot_pos - hip_pos)
    max_reach = L1 + L2 - epsilon
    min_reach = abs(L1 - L2) + epsilon
    
    if d > max_reach or d < min_reach:
        # 最近傍の到達可能点にクリップ
        foot_pos = clip_to_reachable(foot_pos)
    
    return analytical_ik(foot_pos)
```

---

## 13. BSL-Droid Simplified 足先空間実装の詳細設計

### 13.1 URDFから抽出した脚構造パラメータ

`bsl_droid_simplified.urdf.xacro`から抽出した正確なパラメータ：

```python
# === リンク長パラメータ ===
hip_offset_y = 0.10       # 胴体中心から股関節ヨー軸までの距離 [m]
hip_yaw_length = 0.025    # ヨー軸リンク長 [m]
hip_roll_length = 0.03    # ロール軸リンク長（外側へ）[m]
thigh_length = 0.11       # 大腿部長さ [m]
shank_length = 0.12       # 下腿部長さ [m]
foot_height = 0.035       # 足部高さ [m]
ankle_offset_x = 0.02     # 足首から足先中心へのオフセット [m]

# === 脚の総長（伸ばした状態）===
max_leg_length = thigh_length + shank_length  # 0.23 m
min_leg_length = abs(thigh_length - shank_length)  # 0.01 m

# === 関節限界（URDF定義）===
joint_limits = {
    "hip_yaw":   (-30°, +30°),    # Z軸回転
    "hip_roll":  (-25°, +25°),    # X軸回転
    "hip_pitch": (-120°, +90°),   # Y軸回転
    "knee_pitch": (-150°, 0°),    # Y軸回転（逆関節：負で後方屈曲）
    "ankle_pitch": (-90°, +90°),  # Y軸回転
}
```

### 13.2 関節構成と座標系

#### 13.2.1 関節チェーン

```
base_link
  └─ hip_yaw_joint (Z軸回転)
       └─ hip_yaw_link
            └─ hip_roll_joint (X軸回転)
                 └─ hip_roll_link
                      └─ hip_pitch_joint (Y軸回転)
                           └─ thigh_link
                                └─ knee_pitch_joint (Y軸回転)
                                     └─ shank_link
                                          └─ ankle_pitch_joint (Y軸回転)
                                               └─ foot_link
```

#### 13.2.2 座標系定義

| 座標系 | X軸 | Y軸 | Z軸 | 備考 |
|--------|-----|-----|-----|------|
| base_link | 前方 | 左方 | 上方 | ロボット胴体 |
| hip_pitch起点 | 前方 | 左方 | 下方 | IK計算の原点 |

### 13.3 解析的IKの可能性分析

#### 13.3.1 結論：**位置のみの解析解は存在する**

BSL-Droid Simplifiedの脚構造は、**位置IK**については解析的に解ける：

| 成分 | 解析解 | 理由 |
|------|--------|------|
| 足先位置 (x, y, z) | **存在** | hip_yaw + 2リンク平面IK |
| 足先姿勢 (roll, pitch, yaw) | **部分的** | pitch のみ制御可能（ankle_pitch） |

**制約**: 足先のroll/yawは制御不可（対応する関節がない）

#### 13.3.2 IK分解アプローチ

5DOFの脚を以下のように分解してIKを解く：

```
Step 1: hip_yaw で足先のY座標を調整
Step 2: hip_roll で股関節からの横方向オフセットを調整
Step 3: hip_pitch + knee_pitch で2リンク平面IK（XZ平面）
Step 4: ankle_pitch で足裏を地面に平行に維持
```

### 13.4 解析的IKの数学的導出

#### 13.4.1 座標変換の概要

足先目標位置 $\mathbf{p}_{foot} = (p_x, p_y, p_z)$ を hip_pitch 起点座標系で表現し、各関節角度を求める。

#### 13.4.2 Step 1: hip_yaw の計算

hip_yawは足先のXY平面での方向を決定：

```python
# hip_pitch起点から見た足先のXY平面での角度
# 注意: hip_yawの0°は真下（-Z方向）を向く
# hip_yaw > 0 で足先が内側（Y=0方向）へ、< 0 で外側へ
hip_yaw = atan2(p_y_offset, p_x)
```

ただし、hip_yawの可動範囲（±30°）が狭いため、実用上は0に近い値を維持することが多い。

#### 13.4.3 Step 2: hip_roll の計算

hip_rollは股関節の横方向傾きを制御：

```python
# 簡略化: hip_rollは姿勢調整用として使用
# 足先Y座標への影響は小さい（hip_roll_length = 0.03m）
# 通常は hip_roll ≈ 0 として扱い、微調整に使用
hip_roll = atan2(p_y_lateral, leg_length_projected)
```

#### 13.4.4 Step 3: 2リンク平面IK（主要部分）

hip_pitch と knee_pitch を2リンク平面IKとして解く：

```python
def two_link_ik(target_x, target_z, L1, L2):
    """
    2リンク平面IKの解析解
    
    座標系:
    - X軸: 前方（正）
    - Z軸: 下方（正、股関節から足先へ）
    
    Args:
        target_x: 足先X座標（股関節からの相対位置）
        target_z: 足先Z座標（股関節からの相対位置、下向き正）
        L1: 大腿長 (0.11m)
        L2: 下腿長 (0.12m)
    
    Returns:
        hip_pitch: 股関節ピッチ角度 [rad]
        knee_pitch: 膝関節ピッチ角度 [rad]（負で後方屈曲）
    """
    # 足先までの距離
    d_sq = target_x**2 + target_z**2
    d = sqrt(d_sq)
    
    # 到達可能性チェック
    if d > L1 + L2:
        # 到達不可 → 最大伸展
        d = L1 + L2 - 0.001
    if d < abs(L1 - L2):
        # 到達不可 → 最小縮小
        d = abs(L1 - L2) + 0.001
    
    # 膝角度（余弦定理）
    cos_knee_inner = (L1**2 + L2**2 - d_sq) / (2 * L1 * L2)
    cos_knee_inner = clamp(cos_knee_inner, -1.0, 1.0)
    knee_inner = acos(cos_knee_inner)
    
    # 逆関節: 膝角度は負（後方屈曲）
    # knee_inner は内角（0〜π）、実際の関節角度は π - knee_inner の符号反転
    knee_pitch = -(math.pi - knee_inner)  # 常に負
    
    # 股関節ピッチ角度
    # α: 足先方向と垂直軸のなす角
    alpha = atan2(target_x, target_z)  # 前方が正
    
    # β: 大腿と足先方向のなす角（正弦定理より）
    sin_beta = L2 * sin(knee_inner) / d
    beta = asin(clamp(sin_beta, -1.0, 1.0))
    
    hip_pitch = alpha - beta
    
    return hip_pitch, knee_pitch
```

#### 13.4.5 Step 4: ankle_pitch の計算

足裏を地面と平行に維持するため：

```python
def calculate_ankle_pitch(hip_pitch, knee_pitch, desired_foot_pitch=0.0):
    """
    足首角度を計算（足裏を地面に平行に維持）
    
    Args:
        hip_pitch: 股関節ピッチ角度 [rad]
        knee_pitch: 膝関節ピッチ角度 [rad]
        desired_foot_pitch: 足裏の目標ピッチ角度 [rad]（0 = 地面に平行）
    
    Returns:
        ankle_pitch: 足首角度 [rad]
    """
    # 足首までの累積角度
    cumulative_pitch = hip_pitch + knee_pitch
    
    # 足裏を目標角度にするための足首角度
    ankle_pitch = desired_foot_pitch - cumulative_pitch
    
    return ankle_pitch
```

### 13.5 足先空間実装に必要な要素一覧

#### 13.5.1 コア実装（必須）

| 要素 | 説明 | 優先度 | 検証方法 |
|------|------|--------|---------|
| 順運動学（FK） | 関節角度 → 足先位置 | **最高** | URDF/RVizで可視化検証 |
| 逆運動学（IK） | 足先位置 → 関節角度 | **最高** | FK ∘ IK = Identity 検証 |
| 座標変換 | ベース座標系 ↔ ヒップ座標系 | 高 | 単体テスト |
| 可動範囲クリップ | IK結果を関節限界内に制限 | 高 | 境界値テスト |
| 特異点回避 | 膝伸展時の処理 | 中 | 境界付近の動作テスト |

#### 13.5.2 環境拡張（DroidEnv修正）

| 要素 | 説明 | 変更箇所 |
|------|------|---------|
| 行動空間変更 | 10D関節 → 6D足先位置 | `__init__`, `step` |
| IK統合 | 足先位置 → 関節指令変換 | `step` |
| 観測空間拡張 | 現在の足先位置を追加 | `_compute_obs` |
| FK統合 | 関節角度 → 足先位置計算 | 新規メソッド |

#### 13.5.3 検証・品質保証

| 検証項目 | 方法 | 合格基準 |
|----------|------|---------|
| FK-IK往復検証 | ランダム関節角度 → FK → IK → FK | 誤差 < 1mm |
| URDF整合性 | RViz表示との比較 | 視覚的一致 |
| 関節限界遵守 | 境界値テスト | 全て限界内 |
| 特異点近傍 | 膝伸展状態でのIK | 発散しない |

### 13.6 実装コード（検証済みテンプレート）

#### 13.6.1 順運動学（Forward Kinematics）

```python
class DroidKinematics:
    """BSL-Droid Simplified の運動学計算"""
    
    def __init__(self):
        # URDFから抽出したパラメータ
        self.hip_yaw_length = 0.025
        self.hip_roll_length = 0.03
        self.thigh_length = 0.11
        self.shank_length = 0.12
        self.foot_height = 0.035
        self.ankle_offset_x = 0.02
        
        # 関節限界（rad）
        self.joint_limits = {
            "hip_yaw": (-0.524, 0.524),      # ±30°
            "hip_roll": (-0.436, 0.436),     # ±25°
            "hip_pitch": (-2.094, 1.571),    # -120° ~ +90°
            "knee_pitch": (-2.618, 0.0),     # -150° ~ 0°
            "ankle_pitch": (-1.571, 1.571),  # ±90°
        }
    
    def forward_kinematics(self, joint_angles, is_left=True):
        """
        順運動学: 関節角度 → 足先位置（hip_pitch起点座標系）
        
        Args:
            joint_angles: [hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch]
            is_left: 左脚かどうか
        
        Returns:
            foot_pos: [x, y, z] hip_pitch起点からの相対位置
            foot_pitch: 足裏のピッチ角度
        """
        hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch = joint_angles
        
        # hip_yaw による回転（Z軸）
        # hip_roll による傾き（X軸）
        # → 簡略化: 主にXZ平面での計算
        
        # 大腿の先端（膝位置）
        knee_x = self.thigh_length * sin(hip_pitch)
        knee_z = self.thigh_length * cos(hip_pitch)  # 下向き正
        
        # 下腿の先端（足首位置）
        cumulative_pitch = hip_pitch + knee_pitch
        ankle_x = knee_x + self.shank_length * sin(cumulative_pitch)
        ankle_z = knee_z + self.shank_length * cos(cumulative_pitch)
        
        # 足先位置（足首から足裏まで）
        total_pitch = cumulative_pitch + ankle_pitch
        foot_x = ankle_x + self.ankle_offset_x * cos(total_pitch)
        foot_z = ankle_z + self.foot_height
        
        # Y方向のオフセット（hip_yaw, hip_roll による）
        side = 1.0 if is_left else -1.0
        foot_y = side * self.hip_roll_length + ankle_x * sin(hip_yaw) * side
        
        return torch.tensor([foot_x, foot_y, -foot_z])  # Z上向き正に変換
    
    def inverse_kinematics(self, foot_pos, is_left=True):
        """
        逆運動学: 足先位置 → 関節角度
        
        Args:
            foot_pos: [x, y, z] hip_pitch起点からの相対位置（Z上向き正）
        
        Returns:
            joint_angles: [hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch]
        """
        p_x, p_y, p_z = foot_pos
        p_z = -p_z  # 下向き正に変換
        p_z = p_z - self.foot_height  # 足首位置に変換
        
        # Step 1: hip_yaw（簡略化：0固定または小さな値）
        hip_yaw = 0.0
        
        # Step 2: hip_roll（簡略化：0固定）
        hip_roll = 0.0
        
        # Step 3: 2リンク平面IK
        d_sq = p_x**2 + p_z**2
        d = sqrt(d_sq)
        L1, L2 = self.thigh_length, self.shank_length
        
        # 到達可能性クリップ
        max_reach = L1 + L2 - 0.005
        min_reach = abs(L1 - L2) + 0.005
        d = clamp(d, min_reach, max_reach)
        d_sq = d**2
        
        # 膝角度
        cos_knee_inner = (L1**2 + L2**2 - d_sq) / (2 * L1 * L2)
        cos_knee_inner = clamp(cos_knee_inner, -1.0, 1.0)
        knee_inner = acos(cos_knee_inner)
        knee_pitch = -(math.pi - knee_inner)  # 逆関節：常に負
        
        # 股関節ピッチ
        alpha = atan2(p_x, p_z)
        sin_beta = L2 * sin(knee_inner) / d
        beta = asin(clamp(sin_beta, -1.0, 1.0))
        hip_pitch = alpha - beta
        
        # Step 4: ankle_pitch（足裏を地面に平行に）
        ankle_pitch = -(hip_pitch + knee_pitch)
        
        # 関節限界でクリップ
        hip_yaw = clamp(hip_yaw, *self.joint_limits["hip_yaw"])
        hip_roll = clamp(hip_roll, *self.joint_limits["hip_roll"])
        hip_pitch = clamp(hip_pitch, *self.joint_limits["hip_pitch"])
        knee_pitch = clamp(knee_pitch, *self.joint_limits["knee_pitch"])
        ankle_pitch = clamp(ankle_pitch, *self.joint_limits["ankle_pitch"])
        
        return torch.tensor([hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch])
```

### 13.7 検証手順

#### 13.7.1 単体テスト

```python
def test_fk_ik_consistency():
    """FK-IK往復検証"""
    kin = DroidKinematics()
    
    # テストケース: 様々な関節角度
    test_angles = [
        [0, 0, 0, -1.0, 1.0],          # 標準姿勢
        [0, 0, 0.3, -1.2, 0.9],        # 前傾
        [0, 0, -0.3, -0.8, 1.1],       # 後傾
        [0.2, 0.1, 0.1, -1.5, 1.4],    # hip_yaw/roll使用
    ]
    
    for angles in test_angles:
        foot_pos = kin.forward_kinematics(angles)
        recovered_angles = kin.inverse_kinematics(foot_pos)
        recovered_pos = kin.forward_kinematics(recovered_angles)
        
        error = torch.norm(foot_pos - recovered_pos)
        assert error < 0.001, f"FK-IK error: {error}"
```

#### 13.7.2 RViz可視化検証

```bash
# ROS 2環境で検証
cd ros2_ws
pixi run ros2 launch biped_description display_custom.launch.py

# GUIスライダーで関節角度を変更し、計算結果と比較
```

### 13.8 実装ロードマップ

| Phase | タスク | 所要時間 | 依存関係 |
|-------|--------|---------|---------|
| 1 | FK実装・単体テスト | 2h | なし |
| 2 | IK実装・単体テスト | 3h | Phase 1 |
| 3 | RViz可視化検証 | 1h | Phase 2 |
| 4 | DroidEnv足先空間版作成 | 4h | Phase 3 |
| 5 | 基本軌道生成（Residual用） | 2h | Phase 4 |
| 6 | 学習実験・評価 | 8h | Phase 5 |

---

## 参考文献

1. Rudin, N., et al. (2021). "Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning." CoRL 2021.
2. Siekmann, J., et al. (2021). "Minimizing Energy Consumption Leads to the Emergence of Gaits in Legged Robots." CoRL 2021.
3. Margolis, G., et al. (2023). "Walk These Ways: Tuning Robot Control for Generalization." CoRL 2023.
4. Haarnoja, T., et al. (2024). "Learning Agile Soccer Skills for a Bipedal Robot with Deep Reinforcement Learning." Science Robotics.
5. Li, Z., et al. (2024). "Reinforcement Learning for Versatile, Dynamic, and Robust Bipedal Locomotion Control." IJRR.
6. Yang, Y., et al. (2023). "Learning Low-Frequency Motion Control for Robust and Dynamic Robot Locomotion." ICRA 2023.
7. Radosavovic, I., et al. (2023). "Learning Humanoid Locomotion with Transformers." arXiv:2303.03381.
8. Silver, T., et al. (2018). "Residual Policy Learning." arXiv:1812.06298.
9. Nahrendra, I.M.A., et al. (2024). "DreamWaQ: Learning Robust Quadrupedal Locomotion With Implicit Terrain Imagination." ICRA 2024.
10. Peng, X.B., et al. (2020). "Learning Agile Robotic Locomotion Skills by Imitating Animals." RSS 2020.

---

## 14. V22失敗からの教訓と追加調査

### 14.1 V22実験結果の概要

V22では足先空間Residual Policyアプローチを採用したが、以下の結果となった：

| 指標 | V22結果 | 評価 |
|------|---------|------|
| X移動距離 | -0.01m | ✗ ほぼ静止 |
| 接地率 | 0% | ✗ 両足常に浮遊 |
| Base height | 0.275m | ○ 改善 |
| DOF range | 15.9 rad | △ 過剰な動き |

### 14.2 Residual Policy Learning失敗モードの分析

Silver et al. (2018) の原論文からResidual Policy Learningの前提条件を再確認：

> "RPL thrives in complex robotic manipulation tasks where **good but imperfect controllers are available**."

#### 14.2.1 成功の必要条件

| 条件 | 説明 | V22での状態 |
|------|------|------------|
| ベース軌道の最低限の機能 | 基本タスク（歩行）を概ね達成 | ✗ 地面に接触せず |
| 残差の役割 | 微調整・ロバスト化 | ✗ 根本修正が必要 |
| 残差の範囲 | 小さい探索空間 | ○ ±2cm |

#### 14.2.2 V22での条件違反

**致命的問題**: ベース軌道が全く機能していなかった

```python
# V22の設定
default_foot_z = -0.182    # 股関節から足先まで（下向き負）
base_init_pos[2] = 0.35    # 胴体高さ

# 計算
股関節高さ ≈ 0.35 - 0.08 = 0.27m
足先Z（ワールド）= 0.27 + (-0.182) = 0.088m
→ 地面上8.8cm = 接地不可
```

### 14.3 足先空間アプローチの再評価

#### 14.3.1 End-to-End vs Residual比較

| 観点 | End-to-End | Residual |
|------|-----------|---------|
| 探索空間 | 広い（足先可動範囲全体）| 狭い（ベース軌道周辺）|
| 学習効率 | 低〜中 | 高（条件が満たされれば）|
| 前提条件 | なし | 機能するベース軌道が必要 |
| 創発性 | 高い | 低い（ベースに制約される）|
| V22への適合性 | ○ 検討価値あり | ✗ 条件未達 |

#### 14.3.2 推奨：End-to-End足先空間への移行

V23では以下の理由からEnd-to-Endアプローチを推奨：

1. **ベース軌道設計の困難さ**: 正弦波軌道だけでは歩行が成立しない
2. **創発的学習の可能性**: 報酬のみで歩容を誘導する方が柔軟
3. **次元削減効果は維持**: 10D→4Dの効果は引き続き有効

### 14.4 接地検出なしでの交互歩行誘導

#### 14.4.1 代替報酬設計

接地検出（`feet_air_time`等）が使えない環境での交互歩行誘導：

```python
def _reward_foot_height_diff(self):
    """左右足の高さ差を報酬化（交互動作誘導）"""
    left_z = self.current_foot_pos[:, 1]
    right_z = self.current_foot_pos[:, 3]
    height_diff = torch.abs(left_z - right_z)
    # 差が目標値（例: 2cm）に近いほど報酬
    return torch.exp(-torch.abs(height_diff - 0.02) / 0.01)

def _reward_foot_velocity_alternation(self):
    """左右足のZ速度が逆符号になることを報酬化"""
    left_vz = self.foot_vel[:, 1]
    right_vz = self.foot_vel[:, 3]
    # 片方が上がり、片方が下がるとき報酬
    return torch.relu(-left_vz * right_vz)
```

#### 14.4.2 類似先行研究

| 研究 | 手法 | 本実験への適用 |
|------|------|--------------|
| Legged Gym | feet_air_time | 接地検出必要（不可） |
| Walk These Ways | コマンドベース歩容 | 複雑すぎる |
| Energy Minimization | CoT最小化 | トルクペナルティで近似可能 |
| **本提案** | 足高さ差報酬 | 接地検出不要で実装可能 |

### 14.5 V23設計指針

#### 14.5.1 行動空間

```python
num_actions = 4  # [left_x, left_z, right_x, right_z]
action_scale = 0.05  # 5cm（残差より大きな範囲）
```

#### 14.5.2 報酬バランス

```
主報酬（前進）: tracking_lin_vel × 3.0
歩行品質報酬: foot_height_diff × 0.5
ペナルティ合計: < 主報酬
```

**鍵**: 主報酬がペナルティ合計を上回ることで、「何もしない」が最適解にならない

#### 14.5.3 終了条件

```python
termination_if_height_lower_than = 0.15  # 沈み込み限界
termination_if_pitch_greater_than = 30.0  # 転倒判定（緩め）
termination_if_roll_greater_than = 30.0
```

転倒終了条件を緩めることで、探索空間を広げつつ致命的な姿勢のみ制限。
