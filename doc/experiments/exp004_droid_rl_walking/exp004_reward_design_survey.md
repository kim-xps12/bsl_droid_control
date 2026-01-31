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

## 参考文献

1. Rudin, N., et al. (2021). "Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning." CoRL 2021.
2. Siekmann, J., et al. (2021). "Minimizing Energy Consumption Leads to the Emergence of Gaits in Legged Robots." CoRL 2021.
3. Margolis, G., et al. (2023). "Walk These Ways: Tuning Robot Control for Generalization." CoRL 2023.
4. Haarnoja, T., et al. (2024). "Learning Agile Soccer Skills for a Bipedal Robot with Deep Reinforcement Learning." Science Robotics.
5. Li, Z., et al. (2024). "Reinforcement Learning for Versatile, Dynamic, and Robust Bipedal Locomotion Control." IJRR.
6. Yang, Y., et al. (2023). "Learning Low-Frequency Motion Control for Robust and Dynamic Robot Locomotion." ICRA 2023.
