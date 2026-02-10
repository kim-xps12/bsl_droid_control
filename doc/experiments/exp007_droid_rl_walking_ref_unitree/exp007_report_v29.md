# V29: swing_duration報酬導入 + 歩行周期延長 + 歩幅強化

## 概要

V29では、V28で特定した「タップダンス根本原因」を解決するため、報酬計算方式の抜本的変更を行う:

1. **`swing_duration`報酬の導入**（新規）: 「空中にいる間ずっと報酬」方式
2. **`feet_air_time`報酬の削除**: first_contact構造によるタップダンス促進を排除
3. **`gait_frequency`の減少**: 1.2Hz→0.9Hz（歩行周期延長で「ゆったり」歩行）
4. **`step_length`報酬の強化**: 0.5→0.8（「大股」歩行を促進）

目標: 「一般的な歩容、大股でゆったりとした歩き方」の実現

## 前バージョンからの改善事項

### V28の評価結果

| 指標 | V27値 | V28値 | 変化 |
|------|-------|-------|------|
| X速度 | 0.217 m/s | 0.214 m/s | -1%微減 |
| 片足接地率 | 91.4% | 87.8% | -3.6%低下 |
| hip_pitch相関 | -0.931 | -0.951 | 2%改善 |
| タップダンス | あり | **継続** | 改善なし |
| 内股軌道 | あり | **継続** | 改善なし |
| feet_air_time報酬 | -0.0141 | **-0.0427** | 悪化（負のまま） |

### V28で観測された課題

1. **タップダンス継続**:
   - feet_air_time = -0.0427（負の値）は滞空時間がair_time_offset=0.25秒より短いことを示す
   - air_time_offset延長（0.15→0.25）でも解消せず

2. **根本原因の特定**:
   - `feet_air_time`報酬の`first_contact`構造がタップダンスの根本原因
   - 接地時にのみ報酬を付与するため、同じ足で複数回接地すると複数回報酬計算が行われる
   - パラメータ調整では解消不可能な構造的問題

3. **内股軌道継続**:
   - hip_pos = -0.0292（ペナルティ継続）

### 改善策（V29）

| パラメータ | V28値 | V29値 | 変更理由 |
|-----------|-------|-------|---------|
| feet_air_time | 1.5 | **0** | first_contact構造を無効化 |
| swing_duration | - | **1.0** | 新規追加、空中報酬方式 |
| gait_frequency | 1.2 | **0.9** | 歩行周期延長（ゆったり） |
| step_length | 0.5 | **0.8** | 歩幅報酬強化（大股） |
| 報酬項目数 | 19 | **19** | 変更なし（置換） |

## 設計詳細

### swing_duration報酬の設計原理

**V28の問題分析**:

```python
# 現在の実装（first_contact構造）
rew_air_time = (self.feet_air_time - self.air_time_offset) * self._first_contact.float()
```

- `first_contact`は接地開始時にのみTrueになる
- 同じ足で複数回接地すると複数回報酬計算が行われる
- air_time_offsetをいくら延長しても、「接地→短い滞空→接地」のパターンでは報酬計算が複数回発生
- 結果として、タップダンスは構造的に解消されない

**改善策（swing_duration報酬）**:

```python
def _reward_swing_duration(self):
    """空中にいる間ずっと報酬を付与"""
    # air_time_offsetを超えた分のみ報酬
    reward_per_foot = torch.clamp(self.feet_air_time - self.air_time_offset, min=0.0)
    reward = torch.sum(reward_per_foot, dim=1) * is_moving
    return reward
```

**設計上の利点**:
- 空中にいる間毎ステップ報酬を付与
- air_time_offset未満の滞空では報酬なし（clampで0に）
- タップダンス（短い滞空→接地→短い滞空→接地）ではair_time_offsetを超える時間が短いため報酬が小さい
- 長い滞空を維持するほど報酬が大きくなる

### gait_frequency減少の根拠

**目標**: 「ゆったり」歩行

- V28: gait_frequency = 1.2Hz（周期約0.83秒）
- V29: gait_frequency = 0.9Hz（周期約1.1秒）

**期待効果**:
- 歩行サイクルが長くなり、各ステップの滞空時間が延長
- 急がずゆったりとした歩容になる
- swing_duration報酬との相乗効果で長い滞空を促進

### step_length報酬強化の根拠

**目標**: 「大股」歩行

- V28: step_length = 0.5
- V29: step_length = 0.8（60%増加）

**期待効果**:
- 歩幅を大きくするインセンティブが強まる
- gait_frequency減少と合わせて、「大股でゆったり」の歩容を実現

### 報酬構成（V29）

| カテゴリ | 報酬項目 | V28スケール | V29スケール | 変更 |
|---------|---------|-------------|-------------|------|
| **主報酬** | tracking_lin_vel | 1.5 | 1.5 | - |
| | tracking_ang_vel | 0.5 | 0.5 | - |
| **歩行品質報酬** | feet_air_time | 1.5 | **0** | 削除 |
| | swing_duration | - | **1.0** | 新規追加 |
| | contact | 0.4 | 0.4 | - |
| | single_foot_contact | 0.3 | 0.3 | - |
| | step_length | 0.5 | **0.8** | 強化 |
| **安定性ペナルティ** | lin_vel_z | -2.0 | -2.0 | - |
| | ang_vel_xy | -0.05 | -0.05 | - |
| | orientation | -0.5 | -0.5 | - |
| | base_height | -5.0 | -5.0 | - |
| **歩行品質ペナルティ** | feet_swing_height | -8.0 | -8.0 | - |
| | contact_no_vel | -0.1 | -0.1 | - |
| | hip_pos | -0.8 | -0.8 | - |
| | velocity_deficit | -0.5 | -0.5 | - |
| | ankle_pitch_range | -0.3 | -0.3 | - |
| **関節制限** | dof_vel_limits | -0.3 | -0.3 | - |
| **エネルギー効率** | torques | -1e-5 | -1e-5 | - |
| | action_rate | -0.005 | -0.005 | - |
| | dof_acc | -1.0e-7 | -1.0e-7 | - |
| **内股対策** | swing_foot_lateral_velocity | -0.5 | -0.5 | - |

**報酬項目数**: 19項目（V28と同一、feet_air_time→swing_durationに置換）

**追加パラメータ変更**:
- `gait_frequency`: 1.2 → 0.9 Hz
- `air_time_offset`: 0.25秒（維持）

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_unitree_v29.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-unitree-v29 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 54.90 |
| エピソード長 | 1001（最大、転倒なし） |
| 収束ステップ | ~250（std=0.06で収束） |

### 評価結果

| 指標 | V28値 | V29値 | 変化 |
|------|-------|-------|------|
| X速度 | 0.214 m/s | 0.223 m/s | +4%改善 |
| Y速度 | 0.003 m/s | 0.002 m/s | - |
| 片足接地率 | 87.8% | **75.8%** | **-12%悪化** |
| 両足接地率 | - | 22.8% | - |
| 両足浮遊率 | - | 1.4% | - |
| hip_pitch相関 | -0.951 | **-0.971** | **2%改善** |
| 胴体高さ | 0.256±0.009 m | 0.226±0.012 m | -12%低下 |
| Yawドリフト | - | +3.82° | - |
| Total DOF range | 3.934 rad | 5.257 rad | +34%増加 |
| 内股軌道 | あり | **改善傾向** | hip_pos改善 |
| タップダンス | あり | **悪化** | 片足接地率12%低下 |

### 関節可動域分析

| 関節 | 左脚 [min, max] (range) | 右脚 [min, max] (range) |
|------|------------------------|------------------------|
| hip_yaw | [-0.040, 0.128] (0.168) | [-0.121, 0.122] (0.243) |
| hip_roll | [-0.177, 0.000] (0.177) | [-0.036, 0.174] (0.210) |
| hip_pitch | [0.888, 1.574] (0.687) | [0.628, 1.526] (0.898) |
| knee_pitch | [-2.046, -1.563] (0.483) | [-2.088, -1.580] (0.507) |
| ankle_pitch | [0.036, 0.959] (0.922) | [0.250, 1.212] (0.962) |

### 報酬項目の最終値

| 報酬項目 | V28値 | V29値 | 変化 |
|---------|-------|-------|------|
| tracking_lin_vel | 1.1140 | 1.0801 | -3% |
| tracking_ang_vel | - | 0.3550 | - |
| swing_duration | - | **0.0000** | **新規（効果なし）** |
| feet_air_time | -0.0427 | 0.0000 | 削除 |
| contact | 0.5500 | 0.5068 | -8% |
| single_foot_contact | - | 0.1721 | - |
| step_length | 0.0169 | 0.0400 | **+137%改善** |
| hip_pos | -0.0292 | **-0.0122** | **+58%改善** |
| lin_vel_z | - | -0.0349 | - |
| ang_vel_xy | - | -0.0215 | - |
| orientation | - | -0.0022 | - |
| base_height | - | -0.0027 | - |
| feet_swing_height | - | -0.0050 | - |
| swing_foot_lateral_velocity | - | -0.0035 | - |

## 考察と改善案

### 成功点

1. **内股軌道の改善**:
   - hip_posペナルティが-0.0292→-0.0122に改善（+58%）
   - ユーザー目視確認でも「遊脚の軌道が水平面上でカーブを描く問題は改善の兆し」と報告
   - swing_foot_lateral_velocity報酬の効果が継続

2. **hip_pitch相関の継続的改善**:
   - -0.951→-0.971（+2%）で過去最高値
   - V27で達成したエネルギーペナルティ緩和の効果が維持されている

3. **歩幅（step_length）の改善**:
   - 0.0169→0.0400（+137%）で大幅改善
   - step_lengthスケール強化（0.5→0.8）が効果的に機能

4. **DOF可動範囲の増加**:
   - 3.934 rad→5.257 rad（+34%）
   - 「大股」歩行に向けた関節可動域の拡大が確認された

### 課題

1. **swing_durationが完全に機能していない**:
   - swing_duration = **0.0000**（報酬値がゼロ）
   - これは**遊脚が一度も0.25秒以上空中にいなかった**ことを意味する
   - feet_air_timeを削除した結果、空中時間を促進するインセンティブが**完全に消失**

2. **タップダンスの悪化**:
   - 片足接地率: 87.8%→**75.8%**（-12%の大幅悪化）
   - 両足接地率: 22.8%（増加）
   - ユーザー報告:「タン，タン，タン」と足踏みを挟みながら歩いている

3. **根本原因の分析**:

   **仮説**: swing_durationとfeet_air_timeの設計差異が問題

   | 特性 | feet_air_time（V28） | swing_duration（V29） |
   |------|---------------------|----------------------|
   | 報酬タイミング | 接地時に一度 | 空中にいる間毎ステップ |
   | リセット条件 | 接地時に0にリセット | 接地時に0にリセット |
   | 閾値 | air_time_offset=0.25秒 | air_time_offset=0.25秒 |

   **問題点**:
   - V28のfeet_air_timeは負の値（-0.0427）だったが、**何らかの学習シグナルは存在**していた
   - V29のswing_durationは完全に0であり、**学習シグナルが全く存在しない**
   - 「0.25秒以上の空中時間」という閾値がBSL-Droidの自然な歩行パターンでは達成困難
   - gait_frequency=0.9Hz（周期1.1秒）でも、実際の空中時間は0.25秒未満

   **参考**: exp007_unitree_rl_gym_survey.md セクション7.3.3より、Unitree向けの0.5秒オフセットはBSL-Droidには不適切で、0.3秒を推奨。V29の0.25秒でも達成されていないことから、さらなる引き下げまたは構造的変更が必要。

### 次バージョンへの提案

**V30設計案（優先度順）**:

#### 提案A: air_time_offsetの大幅引き下げ（推奨）

```python
# V30: より達成可能なair_time_offsetを設定
air_time_offset = 0.10  # 0.25秒 → 0.10秒
swing_duration_scale = 2.0  # 1.0 → 2.0（強化）
```

**根拠**:
- 現状のロボットは0.25秒の空中時間を達成できていない
- 0.10秒に下げることで、より小さな空中時間でも報酬を獲得可能に
- 徐々に長い空中時間へ誘導するカリキュラム的アプローチ

#### 提案B: スイング位相中の接地ペナルティ追加

```python
def _reward_swing_contact_penalty(self):
    """スイング位相中に接地した場合のペナルティ"""
    is_moving = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
    penalty = torch.zeros(self.num_envs, device=gs.device)
    for i in range(2):
        is_swing_phase = self.leg_phase[:, i] >= 0.55
        is_contact = self.contact_state[:, i] > 0.5
        # スイング位相中に接地 = ペナルティ
        penalty += (is_swing_phase & is_contact).float()
    return penalty * is_moving

# reward_scales
"swing_contact_penalty": -0.5  # 新規追加
```

**根拠**:
- タップダンスを直接抑制するペナルティ
- swing_durationと併用して、「空中維持」と「接地抑制」の両面から誘導

#### 提案C: feet_air_timeの復活（保守的選択）

```python
# V30: swing_durationを削除し、feet_air_timeを復活
feet_air_time_scale = 1.5  # V28と同じ
swing_duration_scale = 0.0  # 削除
air_time_offset = 0.15  # 0.25秒 → 0.15秒に引き下げ
```

**根拠**:
- V28のfeet_air_time=-0.0427は負だが、何らかの学習シグナルがあった
- swing_duration導入がむしろ状況を悪化させた
- first_contact構造の問題はあるが、完全に機能しないよりはマシ

#### 提案D: 「連続空中時間」の新規実装

```python
def _reward_continuous_air_time(self):
    """スイング位相開始からの連続空中時間を報酬"""
    # swing_phase中の累積空中時間（接地でもリセットしない）
    # 次のstance_phaseでリセット
    ...
```

**根拠**:
- 現在のfeet_air_timeは接地するたびにリセットされる
- タップダンス（中断あり）でも、スイング位相全体での空中時間を評価
- 構造的な改善だが、実装が複雑

#### 優先度まとめ

| 提案 | 実装難易度 | 期待効果 | 推奨優先度 |
|------|-----------|---------|-----------|
| A: air_time_offset引き下げ | 低 | 中～高 | **1位（推奨）** |
| B: swing_contact_penalty | 中 | 高 | 2位 |
| C: feet_air_time復活 | 低 | 低～中 | 3位（保守的） |
| D: 連続空中時間 | 高 | 高 | 4位（要検討） |

**V30での同時変更（A+B推奨）**:

```python
# reward_scales変更
"swing_duration": 2.0,  # 1.0 → 2.0（強化）
"swing_contact_penalty": -0.5,  # 新規

# command_cfg変更
"air_time_offset": 0.10,  # 0.25 → 0.10

# gait_frequencyは維持（0.9Hz）
# step_lengthも維持（0.8）
```

## まとめ

V29では、V28で特定した「タップダンス根本原因（first_contact構造）」を解決するため、以下の変更を行った:

1. **swing_duration報酬の導入**: 「空中にいる間ずっと報酬」方式で長い滞空を促進
2. **feet_air_time報酬の削除**: first_contact構造によるタップダンス促進を排除
3. **gait_frequency減少**: 1.2Hz→0.9Hzで歩行周期を延長（ゆったり）
4. **step_length強化**: 0.5→0.8で歩幅を大きく（大股）

**設計根拠**:
- V28のfeet_air_time報酬はfirst_contact構造により、タップダンスを構造的に促進していた
- パラメータ調整では解消不可能な問題であり、報酬計算方式の変更が必要
- swing_duration報酬は毎ステップ報酬を付与するため、長い滞空が明確に有利になる
- gait_frequencyとstep_lengthの調整で「大股でゆったり」の歩容を促進

**結果**:

| 項目 | 状況 | 詳細 |
|------|------|------|
| タップダンス | **悪化** | 片足接地率87.8%→75.8%（-12%） |
| 内股軌道 | **改善** | hip_posペナルティ+58%改善、目視でも改善確認 |
| hip_pitch相関 | **改善** | -0.951→-0.971（過去最高） |
| 歩幅 | **改善** | step_length報酬+137%増加 |
| swing_duration | **機能せず** | 報酬値=0.0000（air_time_offset未達成） |

**重要な発見**:
- swing_duration報酬は**完全に機能していない**（報酬値0.0000）
- これはロボットが0.25秒以上の空中時間を一度も達成していないことを意味する
- feet_air_timeを削除した結果、空中時間を促進するインセンティブが**完全に消失**
- first_contact構造を排除したが、代替のswing_durationが機能しなかったためタップダンスが悪化

**教訓**:
- 報酬構造の変更は、閾値（air_time_offset）の再検討も同時に行う必要がある
- 0.25秒というair_time_offsetはBSL-Droidの現状の歩行パターンでは達成困難
- V30では、air_time_offsetの大幅引き下げ（0.25→0.10秒）とswing_contact_penaltyの追加を推奨

## 備考

- V28で発見した「first_contact構造がタップダンスの根本原因」という知見が本バージョンの設計基盤
- swing_duration報酬は既存のfeet_air_timeを「置換」する形で導入（報酬項目数は維持）
- gait_frequency減少とstep_length強化は「大股でゆったり」の目標に直接対応
- エネルギーペナルティ（action_rate, dof_acc）はV27-V28から維持し、hip_pitch相関改善の成果を保持

### 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-unitree-v29 --no-viewer --duration 10
```

### 訓練ログ分析コマンド

```bash
cd rl_ws
uv run python scripts/analyze_training_log.py droid-walking-unitree-v29
uv run python scripts/show_reward_components.py droid-walking-unitree-v29
```

### 参考文献

- exp007_unitree_rl_gym_survey.md セクション7.3.3: feet_air_timeオフセット調整の指針
- exp007_unitree_rl_gym_survey.md セクション8.2: 片足接地報酬（Single Foot Contact Reward）の有効性
- exp007_report_v28.md: first_contact構造の問題分析
