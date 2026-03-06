# V9: 報酬項目の整理と両脚対称性強化

## 概要

V8はhip_pitch相関を+0.828から-0.382に改善し、交互歩行の基本パターンを獲得した。しかし、hip_pitch_antiphase報酬の設計欠陥により「右脚だけ動かす」局所最適に収束し、左右非対称な動作とYawドリフト+113°が発生した。

V9では、V8レポートの振り返り分析に基づき、以下の方針で改善を行う：

1. **報酬項目の整理**：設計ミス・効果未確認の報酬を削除し複雑化を抑制
2. **hip_pitch_antiphase報酬の修正**：両脚動作を必須条件とした修正版に置換
3. **両脚動作報酬の新規追加**：片脚静止戦略を明示的に抑制
4. **対称性・Yaw追従の強化**：symmetry、tracking_ang_vel報酬を強化

## 前バージョンからの改善事項

V8レポート（exp007_report_v8.md）の「次バージョンへの提案」および「V1-V8振り返り分析」に基づく改善：

### V8の根本原因分析

1. **hip_pitch_antiphase報酬の設計欠陥**
   - 「速度の積が負」を報酬化したが、片方が静止（速度=0）でも報酬0（ペナルティなし）
   - ポリシーは「右脚だけ動かし、左脚は静止」という局所最適を発見
   - **両脚が等しく動くことを保証していない**

2. **symmetry報酬の不十分さ**
   - symmetry報酬値: 0.165（低い）
   - symmetryスケール: 0.3（hip_pitch_antiphase: 0.8と比較して弱い）

3. **Yawドリフトの物理的原因**
   - 右脚のみが大きく動くと、その反作用でYaw方向に角運動量が発生
   - tracking_ang_vel報酬のスケールが0.5と弱く、修正できない

4. **報酬項目の肥大化**
   - V8で23項目（V1の14から64%増加）
   - 設計ミスのある報酬が蓄積（alternating_gait, foot_flat, step_length等）

### V9パラメータ変更一覧

| パラメータ | V8値 | V9値 | 変更理由 |
|-----------|------|------|---------|
| step_length | 0.3 | **削除** | 効果未確認、複雑化の原因 |
| foot_flat | -5.0 | **削除** | 設計ミス（ankle_pitchのみ対象） |
| ankle_roll | -2.0 | **削除** | hip_posと重複 |
| hip_pitch_antiphase | 0.8 | **削除** | V9で修正版に置換 |
| **hip_pitch_antiphase_v2** | (なし) | **0.8** | 【新規】両脚動作必須版 |
| **both_legs_active** | (なし) | **0.5** | 【新規】両脚動作報酬 |
| symmetry | 0.3 | **1.0** | 3.3倍強化 |
| tracking_ang_vel | 0.5 | **1.0** | 2倍強化、Yawドリフト対策 |
| ang_vel_range | [0, 0] | **[-0.2, 0.2]** | Yaw訓練追加 |

**報酬項目数**: 23 → 21（削除4、追加2）

## 設計詳細

### 1. 報酬項目の整理（ロールバック方向）

V8レポートの「V1-V8振り返り分析」では、報酬項目の肥大化と設計ミスの蓄積により破綻方向に進んでいると指摘された。V9では以下の報酬を削除：

| 削除報酬 | 削除理由 |
|----------|---------|
| step_length | 効果未確認、V7で追加したが歩幅改善の明確な証拠なし |
| foot_flat | 設計ミス（ankle_pitchのみ対象、ankle_roll問題は未解決） |
| ankle_roll | hip_posペナルティと重複（同じhip_rollをペナルティ化） |
| hip_pitch_antiphase | V9で修正版（_v2）に置換 |

### 2. hip_pitch_antiphase_v2（修正版）

V8版の欠陥を修正し、**両脚が動いている場合のみ**逆相関報酬を与える。

```python
def _reward_hip_pitch_antiphase_v2(self):
    """hip_pitch速度の逆相関報酬・修正版（V9追加）"""
    left_vel = self.dof_vel[:, self.left_hip_pitch_idx]
    right_vel = self.dof_vel[:, self.right_hip_pitch_idx]

    # 両脚の速度の大きさ
    left_mag = torch.abs(left_vel)
    right_mag = torch.abs(right_vel)

    # 速度の積
    velocity_product = left_vel * right_vel

    # 両脚が共に動いている閾値（0.1 rad/s以上）
    min_vel_threshold = 0.1
    both_active = (left_mag > min_vel_threshold) & (right_mag > min_vel_threshold)

    # 逆相関時に報酬、同相関時にペナルティ
    antiphase_reward = -torch.tanh(velocity_product / 0.1)

    # 両脚が動いている場合のみ報酬を与える
    reward = antiphase_reward * both_active.float()

    # 移動コマンド時のみ適用
    is_moving = self.commands[:, 0].abs() > 0.05
    return reward * is_moving.float()
```

**V8版との違い**：
- V8版：速度の積が負なら報酬、片脚静止（積=0）でも報酬0（許容）
- V9版：両脚が0.1 rad/s以上で動いている場合のみ報酬を付与

### 3. both_legs_active（新規）

両脚が共に動いていることを直接報酬化する。

```python
def _reward_both_legs_active(self):
    """両脚動作報酬（V9追加）"""
    left_vel_mag = torch.abs(self.dof_vel[:, self.left_hip_pitch_idx])
    right_vel_mag = torch.abs(self.dof_vel[:, self.right_hip_pitch_idx])

    # 両脚の速度の最小値を報酬（片方が0なら報酬0）
    min_vel = torch.min(left_vel_mag, right_vel_mag)

    # 速度を適度にスケーリング
    reward = torch.tanh(min_vel / 0.5)

    # 移動コマンド時のみ適用
    is_moving = self.commands[:, 0].abs() > 0.05
    return reward * is_moving.float()
```

**設計原理**：
- `min(left, right)`を報酬化することで、片脚だけ動かす戦略を明示的に抑制
- 片脚が静止していると報酬が0になる

### 4. 対称性・Yaw追従の強化

| 報酬 | V8値 | V9値 | 強化理由 |
|------|------|------|---------|
| symmetry | 0.3 | 1.0 | 左右非対称動作の抑制（V8でsymmetry報酬値が0.165と低かった） |
| tracking_ang_vel | 0.5 | 1.0 | Yawドリフト+113°への対策 |

### 5. Yaw速度コマンド訓練

サーベイ（exp007_unitree_rl_gym_survey.md セクション7.4.1）の推奨に従い、訓練中にYaw速度コマンドを変動させる。

```python
"ang_vel_range": [-0.2, 0.2],  # V8: [0, 0]
```

これにより、ポリシーがYaw制御を学習し、旋回動作に対するロバスト性が向上する。

### V9報酬スケール一覧

```python
reward_scales = {
    # 主報酬
    "tracking_lin_vel": 1.5,
    "tracking_ang_vel": 1.0,   # ★強化
    # 歩行品質報酬
    "feet_air_time": 1.5,
    "contact": 0.3,
    "alive": 0.03,
    "single_foot_contact": 1.0,
    "symmetry": 1.0,           # ★強化
    "hip_pitch_antiphase_v2": 0.8,  # ★新規（修正版）
    "both_legs_active": 0.5,   # ★新規
    # 安定性ペナルティ
    "lin_vel_z": -2.0,
    "ang_vel_xy": -0.1,
    "orientation": -1.0,
    "base_height": -5.0,
    # 歩行品質ペナルティ
    "feet_swing_height": -5.0,
    "contact_no_vel": -0.1,
    "hip_pos": -0.5,
    "velocity_deficit": -2.0,
    # エネルギー効率ペナルティ
    "torques": -1e-5,
    "action_rate": -0.005,
    "dof_acc": -2.5e-7,
    "dof_vel": -0.005,
}
# 報酬項目数: 21
```

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_unitree_v9.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-unitree-v9 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 110.80 |
| エピソード長 | 1001（最大） |
| 収束ステップ | 約300（step 300で99.5、以降は漸増） |
| 収束判定 | Last 50 steps std = 0.38（まだ変動中） |

**報酬推移（Quarter-wise）:**

| ステップ範囲 | 平均報酬 | 増分 |
|-------------|---------|------|
| 0-125 | 42.56 | - |
| 125-250 | 79.31 | +36.75 |
| 250-375 | 100.11 | +20.79 |
| 375-500 | 108.85 | +8.74 |

### 評価結果

| 指標 | V8値 | V9値 | 変化 | 評価 |
|------|------|------|------|------|
| X速度 | 0.108 m/s | **0.092 m/s** | -15% | △ やや低下 |
| hip_pitch相関 | -0.382 | **+0.605** | - | ❌ **悪化**（同期方向へ） |
| hip_pitch L/R ratio | 1.5倍 | **0.91倍** | - | ✅ **大幅改善** |
| Yawドリフト | +113° | **-30.74°** | -73% | ✅ **大幅改善** |
| Roll std | 4.41° | **2.48°** | -44% | ✅ 改善 |
| 単足接地率 | 74.0% | **97.2%** | +23% | ✅ **大幅改善** |

### 姿勢・安定性

| 指標 | 値 |
|------|-----|
| Roll | mean=-0.13°, std=2.48° |
| Pitch | mean=-1.20°, std=1.65° |
| Yaw drift | -30.74°（10秒間） |
| Base height | 0.237 m (std=0.010) |

### 関節動作分析

| 関節 | L range (rad) | R range (rad) | L vel std (rad/s) | R vel std (rad/s) |
|------|---------------|---------------|-------------------|-------------------|
| hip_pitch | **0.361** | **0.396** | **1.038** | **1.186** |
| hip_roll | 0.241 | 0.225 | 0.634 | 0.566 |
| knee | 0.668 | 0.628 | 1.530 | 1.538 |
| ankle_pitch | 0.249 | 0.374 | 0.979 | 0.980 |
| ankle_roll | 0.371 | 0.325 | 1.140 | 1.033 |

**重要な観察**:
- **hip_pitch L/R比が1.0に近づいた**（V8: 1.5倍 → V9: 0.91倍）
- **hip_pitch速度stdも均等化**（V8: 2.2倍差 → V9: 1.14倍差）
- 左右対称性が大幅に改善

### 接地パターン

| パターン | ステップ数 | 割合 |
|----------|-----------|------|
| Both feet grounded | 8 | 1.6% |
| Single foot grounded | 486 | **97.2%** |
| Both feet airborne | 6 | 1.2% |

### 目視観察（ユーザーフィードバック）

- **良い点**: 左右の脚が交互に動いている
- **悪い点**:
  - 足先を引きずる様な動き
  - 移動速度が断続的に上下してカクカクする

## 考察と改善案

### 成功点

1. **左右対称性の大幅改善**
   - hip_pitch L/R比: 1.5倍 → 0.91倍
   - hip_pitch速度std L/R比: 2.2倍 → 1.14倍
   - symmetry報酬の強化（0.3 → 1.0）が有効に機能

2. **Yawドリフトの大幅改善**
   - +113° → -30.74°（約73%改善）
   - tracking_ang_vel強化（0.5 → 1.0）とYaw訓練が有効
   - 直進性が大幅に向上

3. **単足接地率の大幅改善**
   - 74.0% → 97.2%（+23ポイント）
   - 交互歩行の基本パターンが確立

4. **胴体安定性の改善**
   - Roll std: 4.41° → 2.48°（44%改善）
   - Pitch std: 3.97° → 1.65°（58%改善）

### 課題

1. **hip_pitch相関の悪化（回帰）**
   - V8: -0.382（逆相関）→ V9: +0.605（同期方向）
   - 目視では「交互に動いている」が、速度の相関は正
   - **仮説**: symmetry報酬が強すぎて、「同時に動く」パターンを強化

2. **足先を引きずる動き**
   - サーベイ2.5.4「feet_stumble」ペナルティが未実装
   - 水平方向の接触力が大きい可能性
   - feet_swing_heightの目標高さ（0.03m）が低すぎる可能性

3. **断続的な速度推移（カクカク）**
   - X速度が0.03〜0.15 m/sで変動
   - 滑らかな歩行サイクルが未確立
   - action_rate, dof_accペナルティの調整が必要な可能性

4. **X速度の低下**
   - 0.108 m/s → 0.092 m/s（-15%）
   - 目標0.10 m/sをわずかに下回る

### 根本原因分析

#### 1. symmetry報酬の副作用

symmetry報酬（スケール1.0）は左右の関節角度が同じであることを報酬化する。これにより：
- 左右脚が「同じ動き」をするパターンが強化される
- hip_pitch速度が「同時に正」「同時に負」になりやすい
- 結果として相関が正（同期）方向に

**対策案**: symmetryスケールを1.0 → 0.5に減少、またはhip_pitchを対称性評価から除外

#### 2. feet_swing_height不足

現在の設定：
- swing_height_target: 0.03m
- feet_swing_height penalty: -5.0

サーベイ3.2より、Unitreeでは目標高さ0.08mを使用。BSL-Droidでも0.04〜0.05mに引き上げることで、足上げを促進できる可能性。

#### 3. 足引きずり防止機構の欠如

サーベイ2.5.4「feet_stumble」ペナルティ：
```python
# 水平接触力 > 5 × 垂直接触力 の場合にペナルティ
stumble = norm(contact_xy) > 5 * abs(contact_z)
```
このペナルティがないため、足を引きずる動作が許容されている。

### 次バージョン（V10）への提案

サーベイの知見と今回の結果に基づく設計指針：

#### 優先度1: symmetry報酬の調整

**問題**: symmetry報酬が強すぎてhip_pitchの同期動作を誘発
**対策**: symmetryスケールを減少、またはhip_pitchを除外

```python
# オプションA: スケール減少
"symmetry": 0.5,  # V9: 1.0 → V10: 0.5

# オプションB: hip_pitchを対称性評価から除外（環境クラス修正が必要）
```

#### 優先度2: feet_swing_height目標の引き上げ

**問題**: 足上げ高さが不十分で引きずりが発生
**対策**: swing_height_targetを増加

```python
"swing_height_target": 0.05,  # V9: 0.03 → V10: 0.05
"feet_swing_height": -10.0,   # V9: -5.0 → V10: -10.0（強化）
```

#### 優先度3: feet_stumbleペナルティの追加

**問題**: 足引きずりを明示的に抑制していない
**対策**: サーベイ2.5.4に基づきペナルティを追加

```python
def _reward_feet_stumble(self):
    """足引きずりペナルティ（V10追加）"""
    contacts = self._get_foot_contacts()
    if contacts is None:
        return torch.zeros(...)

    # 水平方向の足速度が大きい接地状態をペナルティ
    vel_xy = torch.sqrt(self.feet_vel[:, :, 0]**2 + self.feet_vel[:, :, 1]**2)
    return torch.sum(vel_xy * contacts.float(), dim=1)
```

**reward_scales追加**:
```python
"feet_stumble": -0.5,  # 新規
```

#### 優先度4: 速度滑らかさの改善

**問題**: 速度推移がカクカクしている
**対策案**:
- action_rate: -0.005 → -0.01（アクション変化を強く抑制）
- または速度変化率ペナルティを追加

#### V10パラメータ案

| パラメータ | V9値 | V10提案値 | 変更理由 |
|-----------|------|----------|---------|
| symmetry | 1.0 | **0.5** | hip_pitch同期を緩和 |
| swing_height_target | 0.03 | **0.05** | 足上げ高さ増加 |
| feet_swing_height | -5.0 | **-10.0** | 足上げペナルティ強化 |
| **feet_stumble** | (なし) | **-0.5** | 【新規】足引きずり抑制 |
| action_rate | -0.005 | **-0.01** | 滑らかさ向上 |

## まとめ

V9は**左右対称性とYawドリフトの大幅改善**に成功した：
- hip_pitch L/R比: 1.5倍 → 0.91倍（ほぼ対称）
- Yawドリフト: +113° → -30.74°（73%改善）
- 単足接地率: 74.0% → 97.2%（交互歩行確立）
- Roll/Pitch std: 大幅改善

しかし、新たな課題が発生：
1. **hip_pitch相関の悪化**: -0.382 → +0.605（symmetry報酬の副作用）
2. **足先引きずり**: feet_swing_height不足、stumbleペナルティ欠如
3. **断続的な速度**: 滑らかな歩行サイクル未確立

V10では：
- symmetry報酬を緩和（1.0 → 0.5）してhip_pitch逆相関を維持
- feet_swing_height目標を引き上げ（0.03 → 0.05m）
- feet_stumbleペナルティを新規追加して引きずりを抑制
- action_rateペナルティを強化して滑らかさを改善

を実施し、「対称的かつ逆相関」の交互歩行と、滑らかな歩容を目指す。

## 備考

- 学習ログ：`rl_ws/logs/droid-walking-unitree-v9/`
- 新規報酬関数の実装：`biped_walking/envs/droid_env_unitree.py`
- 参照：[exp007_report_v8.md](exp007_report_v8.md) 「次バージョンへの提案」「V1-V8振り返り分析」セクション
- 参照：[exp007_unitree_rl_gym_survey.md](exp007_unitree_rl_gym_survey.md)
