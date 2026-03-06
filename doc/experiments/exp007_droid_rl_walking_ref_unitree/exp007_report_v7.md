# V7: 交互歩行強化と歩幅拡大

## 概要

V6は静止ポリシーを回避し前進歩行を達成したが、「小刻みな足の動き」「足首傾斜（爪先立ち）」「両脚同期」という課題が残った。V7では新しい報酬関数（alternating_gait, foot_flat, step_length）を追加し、これらの課題を解決する。

## 前バージョンからの改善事項

V6レポート（exp007_report_v6.md）の「次バージョンへの提案」に基づく改善：

| パラメータ | V6値 | V7値 | 変更理由 |
|-----------|------|------|---------|
| **lin_vel_x_range** | [0.10, 0.15] | **[0.15, 0.25]** | 目標速度を上げて歩幅拡大 |
| **air_time_offset** | 0.25 | **0.20** | より早い段階で滞空報酬 |
| **contact** | 0.2 | **0.3** | 接地フェーズ整合性強化 |
| **single_foot_contact** | 0.8 | **1.0** | 交互歩行強化 |
| **alternating_gait** | (なし) | **0.5** | 【新規】hip_pitch逆位相報酬 |
| **foot_flat** | (なし) | **-3.0** | 【新規】足裏水平ペナルティ |
| **step_length** | (なし) | **0.3** | 【新規】歩幅報酬 |

## 設計詳細

### 新規報酬関数

#### 1. alternating_gait（交互歩行報酬）

hip_pitchの逆位相を報酬化し、交互歩行を直接誘導する。

```python
def _reward_alternating_gait(self):
    """hip_pitch逆位相報酬"""
    left_hp = dof_pos[:, left_hip_pitch_idx] - default[left_hip_pitch_idx]
    right_hp = dof_pos[:, right_hip_pitch_idx] - default[right_hip_pitch_idx]
    # 逆位相時は和が0に近い
    phase_sum = torch.abs(left_hp + right_hp)
    return torch.exp(-phase_sum / 0.3)
```

**設計原理**:
- 理想的な交互歩行では、左脚が前に出るとき右脚は後ろにある
- hip_pitch偏差の和が0に近いほど高報酬

#### 2. foot_flat（足裏水平報酬）

接地時のankle_pitch偏差をペナルティ化し、爪先立ちを抑制する。

```python
def _reward_foot_flat(self):
    """接地時のankle偏差ペナルティ"""
    contacts = self._get_foot_contacts()
    left_dev = abs(dof_pos[:, left_ankle] - default[left_ankle])
    right_dev = abs(dof_pos[:, right_ankle] - default[right_ankle])
    # 接地中の足のみ対象
    return (left_dev² * left_contact) + (right_dev² * right_contact)
```

**設計原理**:
- V6の「爪先だけで接地」問題を解決
- スイング相では制約なし（足上げの自由度を確保）

#### 3. step_length（歩幅報酬）

足間距離を報酬化し、大股歩行を促進する。

```python
def _reward_step_length(self):
    """足間距離報酬"""
    left_x = feet_pos[:, 0, 0]  # 左足X座標
    right_x = feet_pos[:, 1, 0]  # 右足X座標
    distance = abs(left_x - right_x)
    return distance * is_moving
```

**設計原理**:
- 小刻み歩行を抑制し、大きな歩幅を促進
- 移動コマンド時のみ適用

### V7報酬スケール

```python
reward_cfg = {
    "tracking_sigma": 0.10,
    "base_height_target": 0.20,
    "swing_height_target": 0.03,
    "gait_frequency": 0.8,
    "contact_threshold": 0.08,
    "air_time_offset": 0.20,  # ★短縮
    "reward_scales": {
        # 主報酬
        "tracking_lin_vel": 1.5,
        "tracking_ang_vel": 0.5,
        # 歩行品質報酬
        "feet_air_time": 1.5,
        "contact": 0.3,           # ★強化
        "alive": 0.03,
        "single_foot_contact": 1.0,  # ★強化
        "symmetry": 0.3,
        "alternating_gait": 0.5,  # ★新規
        "step_length": 0.3,       # ★新規
        # ペナルティ
        "lin_vel_z": -2.0,
        "ang_vel_xy": -0.05,
        "orientation": -0.5,
        "base_height": -5.0,
        "feet_swing_height": -5.0,
        "contact_no_vel": -0.1,
        "hip_pos": -0.5,
        "velocity_deficit": -2.0,
        "foot_flat": -3.0,        # ★新規
        "torques": -1e-5,
        "action_rate": -0.005,
        "dof_acc": -2.5e-7,
        "dof_vel": -0.005,
    },
}
```

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_unitree_v7.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-unitree-v7 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 約65（推定） |
| エピソード長 | 1001（最大） |
| 収束ステップ | 約200 |

### 評価結果

| 指標 | V6値 | V7値 | 変化 | 評価 |
|------|------|------|------|------|
| X速度 | 0.074 m/s | **0.214 m/s** | +189% | ✅ 目標0.15m/s達成 |
| hip_pitch相関 | +0.781 | **+0.828** | +6% | ❌ **悪化**（より同期） |
| Roll std | 1.09° | **6.21°** | +470% | ❌ **大幅悪化**（胴体傾き） |
| Pitch mean | 0.80° | **5.60°** | +600% | ⚠️ 前傾姿勢 |
| Yaw drift | +2.06° | **-7.56°** | - | ⚠️ ドリフト増加 |
| 単足接地率 | 97.6% | **95.6%** | -2% | ○ 維持 |

### 姿勢・安定性

| 指標 | 値 |
|------|-----|
| Roll | mean=3.36°, **std=6.21°** |
| Pitch | **mean=5.60°**, std=2.77° |
| Yaw drift | -7.56°（10秒間） |
| Base height | 0.210 m (std=0.016) |

### 関節動作分析

| 関節 | L range (rad) | R range (rad) | L vel std (rad/s) | R vel std (rad/s) |
|------|---------------|---------------|-------------------|-------------------|
| hip_pitch | 0.421 | 0.398 | 0.723 | 0.689 |
| hip_roll | 0.289 | 0.312 | 0.456 | 0.478 |
| knee | 0.512 | 0.489 | 0.634 | 0.601 |
| ankle_pitch | 0.267 | 0.245 | 0.589 | 0.567 |
| ankle_roll | **0.489** | **0.512** | **0.823** | **0.798** |

### 接地パターン

| パターン | ステップ数 | 割合 |
|----------|-----------|------|
| Both feet grounded | 12 | 2.4% |
| Single foot grounded | 478 | **95.6%** |
| Both feet airborne | 10 | 2.0% |

## 考察と改善案

### 成功点

1. **速度目標の達成**: 0.214 m/sで目標0.15 m/sを大幅超過（+43%）
2. **前進能力の維持**: V6から引き続き前進歩行を実現
3. **単足接地率の維持**: 95.6%で高い片足接地率を維持

### 課題（目視観察と一致）

1. **両脚同期の悪化**: hip_pitch相関が0.781→0.828と**6%悪化**
   - 目視観察：「前足と後ろ足がまた固定されている」
   - alternating_gait報酬が機能していない

2. **胴体傾斜（Roll）の悪化**: std 1.09°→6.21°と**470%増加**
   - 目視観察：「周期的に胴体が大きく傾く」
   - 速度を上げた結果、安定性が低下

3. **足首傾斜問題の継続**: ankle_roll range 0.489-0.512 rad（V6と同等）
   - 目視観察：「足首が傾いて爪先だけで接地して歩こうとしている」
   - foot_flat報酬が機能していない

4. **小刻み歩行の継続**:
   - 目視観察：「小刻みな足の動き」
   - step_length報酬が歩幅拡大に寄与していない

### 根本原因分析

#### 1. alternating_gait報酬の設計ミス

現在の実装：
```python
phase_sum = torch.abs(left_hp + right_hp)
return torch.exp(-phase_sum / 0.3)
```

**問題点**：
- 「hip_pitch偏差の和が0」を報酬化しているが、これは「両脚が同じ位置（両方デフォルト）」でも達成可能
- **両脚が微小振動で同期している**場合、偏差の和は常に小さく、高報酬を獲得
- サーベイ7.2.2「Periodic Reward Composition」の知見：フェーズを明示的に管理し、SS phase（片脚スタンス）とDS phase（両脚スタンス）を区別する必要がある

#### 2. foot_flat報酬の設計ミス

現在の実装：
```python
left_penalty = left_ankle_dev * contacts[:, 0].float()
right_penalty = right_ankle_dev * contacts[:, 1].float()
return torch.square(left_penalty) + torch.square(right_penalty)
```

**問題点**：
- ankle_pitch（ピッチ方向）のみを対象としているが、問題はankle_roll（ロール方向）の傾斜
- V6レポートでもankle_roll rangeがL=0.454, R=0.527 radと報告されていた
- **ankle_rollに対するペナルティが欠如**している

#### 3. step_length報酬の有効性

現在の実装：
```python
foot_distance_x = torch.abs(left_foot_x - right_foot_x)
return foot_distance_x * is_moving.float()
```

**問題点**：
- 足間距離の絶対値を報酬化しているが、両脚が同期して前後に振動する場合でも、足間距離は一定程度確保される
- **動的な歩幅（ストライド長）**を報酬化するべき

#### 4. 速度目標上昇による安定性低下

lin_vel_x_rangeを[0.10, 0.15]→[0.15, 0.25]に変更したことで：
- Roll stdが1.09°→6.21°と大幅悪化
- **速度追従と安定性のトレードオフ**が顕在化
- サーベイ7.3.1「Curriculum Learning」の知見：速度を段階的に上げるべき

### 次バージョン（V8）への提案

サーベイの知見を踏まえた設計指針：

#### 優先度1: 交互歩行の正しい誘導

**対策**: 歩行フェーズ（leg_phase）を使用した報酬設計（サーベイ7.2.1 Periodic Reward Composition）

```python
def _reward_alternating_gait_v2(self):
    """歩行フェーズベースの交互歩行報酬"""
    # 左右の脚がスタンス/スイングで逆位相であることを報酬化
    left_stance = self.leg_phase[:, 0] < 0.55
    right_stance = self.leg_phase[:, 1] < 0.55

    # 左右が異なるフェーズにあることを報酬（XOR）
    different_phase = left_stance ^ right_stance
    return different_phase.float()
```

または**hip_pitch速度の逆相関**を報酬化：

```python
def _reward_hip_pitch_antiphase(self):
    """hip_pitch速度の逆相関報酬"""
    left_vel = self.dof_vel[:, self.left_hip_pitch_idx]
    right_vel = self.dof_vel[:, self.right_hip_pitch_idx]
    # 速度が逆符号であれば報酬（積が負）
    return -torch.sign(left_vel * right_vel)
```

#### 優先度2: ankle_rollペナルティの追加

```python
def _reward_ankle_roll(self):
    """足首ロール角ペナルティ"""
    left_roll = torch.abs(self.dof_pos[:, self.left_ankle_roll_idx])
    right_roll = torch.abs(self.dof_pos[:, self.right_ankle_roll_idx])
    return torch.square(left_roll) + torch.square(right_roll)

# 推奨スケール: -2.0
```

#### 優先度3: 速度カリキュラムの導入

サーベイ7.3.1より、速度を段階的に上げるカリキュラム学習：

```python
# Phase 1: 低速で安定歩行を学習（0-200 iterations）
lin_vel_x_range = [0.10, 0.15]

# Phase 2: 中速に拡大（200-400 iterations）
if iteration > 200 and tracking_performance > 0.8:
    lin_vel_x_range = [0.12, 0.20]

# Phase 3: 高速に拡大（400+ iterations）
if iteration > 400 and tracking_performance > 0.8:
    lin_vel_x_range = [0.15, 0.25]
```

#### 優先度4: 胴体安定性の強化

```python
# orientation報酬の強化
"orientation": -0.5 → -1.0

# ang_vel_xy報酬の強化
"ang_vel_xy": -0.05 → -0.1
```

### V8パラメータ案

| パラメータ | V7値 | V8提案値 | 変更理由 |
|-----------|------|----------|---------|
| lin_vel_x_range | [0.15, 0.25] | **[0.10, 0.15]** | V6レベルに戻し安定性優先 |
| alternating_gait | 0.5 | 削除 | 機能していない |
| **hip_pitch_antiphase** | (なし) | **0.8** | 【新規】速度逆相関 |
| **ankle_roll** | (なし) | **-2.0** | 【新規】ankle_rollペナルティ |
| foot_flat | -3.0 | **-5.0** | 強化（ankle_pitchも） |
| orientation | -0.5 | **-1.0** | 胴体安定性強化 |
| ang_vel_xy | -0.05 | **-0.1** | 胴体安定性強化 |

### 長期的改善案（サーベイより）

1. **正弦波参照軌道の導入**（サーベイ7.3.2）：周期的な歩行パターンを強制的に誘導
2. **等変ネットワークアーキテクチャ**（サーベイ7.2.3 MS-PPO）：対称性を構造的に保証
3. **Yaw速度コマンド訓練**（サーベイ7.4.1）：意図的にYaw速度を変動させて修正能力を獲得

## まとめ

V7は速度目標（0.214 m/s）を達成したが、3つの新規報酬関数（alternating_gait, foot_flat, step_length）は**設計ミスにより意図した効果を発揮しなかった**：

1. **alternating_gait**: hip_pitch偏差の和ではなく、速度の逆相関を報酬化すべき
2. **foot_flat**: ankle_pitchだけでなくankle_rollもペナルティ対象に含めるべき
3. **step_length**: 静的な足間距離ではなく、動的なストライド長を報酬化すべき

さらに、速度目標の上昇により胴体安定性が大幅に悪化（Roll std 470%増加）。

V8では：
- 速度目標をV6レベルに戻し安定性を優先
- hip_pitch速度の逆相関を報酬化
- ankle_rollペナルティを追加
- 胴体安定性報酬を強化

を実施する。

## 備考

- 学習ログ：`rl_ws/logs/droid-walking-unitree-v7/`
- 新規報酬関数の実装：`biped_walking/envs/droid_env_unitree.py`
- 参照：[exp007_unitree_rl_gym_survey.md](exp007_unitree_rl_gym_survey.md)
