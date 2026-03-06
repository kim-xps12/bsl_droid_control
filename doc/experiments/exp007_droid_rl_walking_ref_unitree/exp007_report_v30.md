# V30: air_time_offset引き下げ + swing_contact_penalty導入

## 概要

V30では、V29で**完全に機能しなかった**`swing_duration`報酬を修正し、タップダンス問題を直接抑制する。

1. **`air_time_offset`の引き下げ**: 0.25秒→0.10秒（達成可能な閾値に）
2. **`swing_duration`報酬の強化**: 1.0→2.0
3. **`swing_contact_penalty`の導入**（新規）: スイング位相中の接地を直接ペナルティ化

目標: V29で改善した指標（hip_pos、hip_pitch相関、step_length）を維持しつつ、タップダンスを解消する。

## 前バージョンからの改善事項

### V29の評価結果

| 指標 | V28値 | V29値 | 変化 |
|------|-------|-------|------|
| X速度 | 0.214 m/s | 0.223 m/s | +4%改善 |
| 片足接地率 | 87.8% | **75.8%** | **-12%悪化** |
| hip_pitch相関 | -0.951 | -0.971 | 2%改善 |
| hip_pos | -0.0292 | -0.0122 | +58%改善 |
| step_length | 0.0169 | 0.0400 | +137%改善 |
| swing_duration報酬 | - | **0.0000** | **完全に機能せず** |
| タップダンス | あり | **悪化** | 「タン、タン、タン」と足踏み |

### V29で観測された課題

1. **swing_duration報酬が完全に機能していない**:
   - swing_duration = **0.0000**（報酬値がゼロ）
   - これは**遊脚が一度も0.25秒以上空中にいなかった**ことを意味する
   - feet_air_timeを削除した結果、空中時間を促進するインセンティブが**完全に消失**

2. **タップダンスの悪化**:
   - 片足接地率: 87.8%→**75.8%**（-12%の大幅悪化）
   - ユーザー報告:「タン，タン，タン」と足踏みを挟みながら歩いている

3. **根本原因**:
   - `air_time_offset=0.25`秒がBSL-Droidの自然な歩行パターンでは達成困難
   - V29のswing_durationは完全に0であり、**学習シグナルが全く存在しない**

### 改善策（V30）

V29レポートの「次バージョンへの提案」セクションの**提案A+B**を採用。

| パラメータ | V29値 | V30値 | 変更理由 |
|-----------|-------|-------|---------|
| air_time_offset | 0.25 | **0.10** | 達成可能な閾値に引き下げ |
| swing_duration | 1.0 | **2.0** | 空中報酬を強化 |
| swing_contact_penalty | - | **-0.5** | 新規追加、スイング中接地を直接抑制 |
| 報酬項目数 | 19 | **20** | +1（swing_contact_penalty追加） |

## 設計詳細

### air_time_offset引き下げの根拠

**問題分析**:
- V29のswing_duration報酬値が0.0000であった
- これはBSL-Droidが一度も0.25秒以上の空中時間を達成できなかったことを示す
- gait_frequency=0.9Hz（周期1.1秒）でも、実際の空中時間は0.25秒未満

**対策**:
- air_time_offset: 0.25秒 → 0.10秒
- より達成可能な閾値を設定し、swing_duration報酬が実際に機能するようにする
- 0.10秒（100ms）は2-3歩行サイクルで達成可能な見込み

### swing_duration強化の根拠

**問題分析**:
- V29ではswing_duration=1.0だったが、air_time_offsetが高すぎて報酬が発生しなかった
- air_time_offsetを引き下げた上で、空中報酬を強化してより強い学習シグナルを提供

**対策**:
- swing_duration: 1.0 → 2.0（2倍に強化）
- 長い滞空時間をより強く報酬し、タップダンスを抑制

### swing_contact_penalty導入の根拠

**問題分析**:
- swing_duration報酬だけでは「空中にいることの報酬」のみで「接地しないことの抑制」がない
- タップダンスを直接抑制するペナルティが必要

**対策**:
```python
def _reward_swing_contact_penalty(self):
    """スイング位相中の接地ペナルティ（V30追加）"""
    if self.contact_state is None:
        return torch.zeros(self.num_envs, device=gs.device)

    is_moving = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
    penalty = torch.zeros(self.num_envs, device=gs.device)

    for i in range(2):  # 両足
        # スイング位相の判定（位相が0.55以上）
        is_swing_phase = self.leg_phase[:, i] >= 0.55
        # 接地状態の判定
        is_contact = self.contact_state[:, i] > 0.5
        # スイング位相中に接地 = ペナルティ
        penalty += (is_swing_phase & is_contact).float()

    return penalty * is_moving
```

**設計上の利点**:
- swing_durationと併用し、「空中維持」と「接地抑制」の両面から誘導
- スイング位相（leg_phase >= 0.55）中に接地した場合にペナルティ
- 移動コマンド時のみ適用（静止時は影響なし）

### 報酬構成（V30）

| カテゴリ | 報酬項目 | V29スケール | V30スケール | 変更 |
|---------|---------|-------------|-------------|------|
| **主報酬** | tracking_lin_vel | 1.5 | 1.5 | - |
| | tracking_ang_vel | 0.5 | 0.5 | - |
| **歩行品質報酬** | feet_air_time | 0 | 0 | - |
| | swing_duration | 1.0 | **2.0** | 強化 |
| | swing_contact_penalty | - | **-0.5** | 新規追加 |
| | contact | 0.4 | 0.4 | - |
| | single_foot_contact | 0.3 | 0.3 | - |
| | step_length | 0.8 | 0.8 | - |
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

**報酬項目数**: 20項目（V29から+1、swing_contact_penalty追加）

**追加パラメータ変更**:
- `air_time_offset`: 0.25秒 → 0.10秒

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_unitree_v30.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-unitree-v30 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 39.30 |
| 最大報酬 | 46.72（Step 229） |
| エピソード長 | 884.6 steps（17.69秒） |
| 収束状況 | まだ変動中（std=1.34 > 0.1） |

**報酬推移**:
- 初期（Step 0-125）: 23.40
- 中期（Step 125-250）: 42.84
- 後期（Step 250-375）: 41.74
- 最終（Step 375-500）: 40.22

### 評価結果

| 指標 | V29値 | V30値 | 変化 |
|------|-------|-------|------|
| X速度 | 0.223 m/s | 0.170 m/s | -24%低下 |
| 片足接地率 | 75.8% | **95.4%** | **+26%大幅改善** |
| 両足接地率 | - | 3.0% | - |
| 両足浮遊率 | - | 1.6% | - |
| hip_pitch相関 | -0.971 | **-0.571** | **悪化（交互歩行の乱れ）** |
| hip_pos | -0.0122 | -0.0775 | 悪化 |
| swing_duration | 0.0000 | **0.1091** | **機能確認** |
| swing_contact_penalty | - | -0.0395 | 機能中 |
| step_length | 0.0400 | 0.0781 | +95%改善 |
| Yawドリフト | - | +11.76° | - |
| Base height | - | 0.254m（std=0.009m） | - |
| DOF range sum | - | 5.493 rad | - |

### 報酬コンポーネント分析

| 報酬項目 | 値 | 備考 |
|---------|-----|------|
| tracking_lin_vel | 1.0563 | 主報酬、機能中 |
| tracking_ang_vel | 0.2171 | 機能中 |
| contact | 0.5496 | 機能中 |
| single_foot_contact | 0.2196 | 機能中 |
| swing_duration | **0.1091** | **V29の0.0000から改善** |
| step_length | 0.0781 | 機能中 |
| swing_contact_penalty | -0.0395 | 新規、機能中 |
| hip_pos | -0.0775 | V29より悪化 |
| ang_vel_xy | -0.0787 | - |
| action_rate | -0.0655 | - |
| lin_vel_z | -0.0473 | - |
| swing_foot_lateral_velocity | -0.0417 | - |
| feet_swing_height | -0.0269 | - |
| base_height | -0.0131 | - |
| orientation | -0.0125 | - |
| dof_acc | -0.0047 | - |
| contact_no_vel | -0.0036 | - |
| velocity_deficit | -0.0027 | - |
| ankle_pitch_range | -0.0016 | - |
| torques | -0.0001 | - |
| dof_vel_limits | 0.0000 | - |

### 関節動作分析

| 関節 | 左脚範囲(rad) | 右脚範囲(rad) |
|------|-------------|-------------|
| hip_yaw | 0.265 | 0.271 |
| hip_roll | 0.400 | 0.396 |
| hip_pitch | 0.566 | 0.658 |
| knee_pitch | 0.726 | 0.819 |
| ankle_pitch | 0.626 | 0.766 |

### ユーザー目視観察

> 「遊脚が地面を1回叩いてから前側に着地するようになった。前回までは『タン、タン、タン』と複数回の足踏みを挟みながら歩いていた状態だったので少しマシになっている。」

## 考察と改善案

### 成功点

1. **swing_duration報酬が機能するようになった**
   - V29: 0.0000 → V30: 0.1091
   - `air_time_offset`を0.25秒から0.10秒に引き下げたことが奏功
   - BSL-Droidの物理的制約に適合した閾値設定の重要性が確認された

2. **タップダンス問題が大幅改善**
   - 片足接地率: 75.8% → **95.4%**（+26%）
   - ユーザー目視でも「複数回の足踏み」から「1回叩いて着地」に改善
   - `swing_contact_penalty`によるスイング中接地の直接抑制が有効

3. **step_length報酬が向上**
   - 0.0400 → 0.0781（+95%改善）
   - 歩幅が拡大傾向にあることを示す

### 課題

1. **hip_pitch相関の大幅悪化**
   - V29: -0.971 → V30: **-0.571**
   - 左右脚の交互動作が崩れている
   - **根本原因**: swing_contact_penaltyがスイング中の接地を抑制した結果、「足を上げ続ける」動作が優先され、自然な交互歩行パターンが乱れた可能性

2. **X速度の低下**
   - 0.223 m/s → 0.170 m/s（-24%）
   - 目標速度（0.15 m/s）は達成しているが、V29より低下
   - 片足接地率の向上とトレードオフの関係か

3. **hip_posペナルティの悪化**
   - -0.0122 → -0.0775
   - 股関節の開きが増加（より「がに股」傾向）

4. **Yawドリフト**
   - +11.76°のYawドリフト
   - 直進性に課題

5. **目標歩容との乖離**
   - 目標: 「大股でゆったりとした一般的な歩容」
   - 現状: 「1回タップしてから着地」というまだ不自然なパターン
   - 真の遊脚フェーズ（完全に空中）を維持できていない

### 次バージョンへの提案

#### 提案A: hip_pitch交互動作の強化（優先度: 高）

**問題**: hip_pitch相関が-0.571と悪化し、交互歩行が崩れている

**対策**: `single_foot_contact`報酬の強化
- サーベイ(Section 8.2)によると、片足接地報酬は「ホッピングではなく歩行」を誘導する最も信頼性の高い方法
- 現在の0.3から0.5に強化することで、交互歩行パターンを促進

```python
"single_foot_contact": 0.3 → 0.5
```

#### 提案B: gait_frequencyの引き上げ（優先度: 中）

**問題**: 現在のgait_frequency=0.9Hzでは、1歩行サイクルが長すぎて自然な交互動作が難しい可能性

**対策**: gait_frequencyを0.9Hz → 1.0Hzに引き上げ
- より速い歩行周波数で交互動作を強制
- サーベイ(Section 7.2)では周波数調整が交互歩行に有効

```python
gait_frequency: 0.9 → 1.0 Hz
```

#### 提案C: swing_contact_penaltyの緩和（優先度: 中）

**問題**: swing_contact_penalty=-0.5が強すぎて、hip_pitch相関を悪化させた可能性

**対策**: -0.5 → -0.3に緩和
- 多少のタップを許容しつつ、交互歩行を優先

```python
"swing_contact_penalty": -0.5 → -0.3
```

#### 提案D: Yawドリフト対策（優先度: 低）

**問題**: +11.76°のYawドリフト

**対策案**:
1. `tracking_ang_vel`を0.5 → 0.8に強化（サーベイ7.4.1）
2. または、hip_yawの逆位相制御ペナルティ追加（サーベイ7.4.2）

#### 推奨する次バージョン(V31)の変更

| パラメータ | V30値 | V31提案値 | 理由 |
|-----------|-------|-----------|------|
| single_foot_contact | 0.3 | **0.5** | 交互歩行強化 |
| gait_frequency | 0.9 Hz | **1.0 Hz** | 歩行周波数引き上げ |
| swing_contact_penalty | -0.5 | **-0.3** | 緩和 |

**変更戦略**: 3項目同時変更は相互作用が予測困難なため、以下の優先順位で段階的に実施を推奨
1. まず提案Aのみ（single_foot_contact強化）
2. 効果が限定的な場合、提案B（gait_frequency）を追加
3. 提案Cは提案A/Bの結果を見て判断

## まとめ

V30では、V29で特定した「air_time_offsetが高すぎてswing_duration報酬が機能しない」問題に対処した。

### 達成された効果

| 目標 | 結果 | 判定 |
|------|------|------|
| swing_duration報酬 > 0.0 | **0.1091** | ✅ 達成 |
| 片足接地率 > 80% | **95.4%** | ✅ 大幅達成 |
| hip_pitch相関 < -0.9 | **-0.571** | ❌ 未達成（悪化） |
| タップダンス改善 | 複数回→1回 | △ 部分的改善 |

### 結論

**成功点**:
- `swing_duration`報酬が機能し（0.0000→0.1091）、BSL-Droid向けの`air_time_offset=0.10秒`が有効であることを確認
- タップダンス問題は大幅改善（片足接地率75.8%→95.4%）
- `swing_contact_penalty`によるスイング中接地抑制が効果的

**新たな課題**:
- hip_pitch相関の悪化（-0.971→-0.571）により、交互歩行パターンが崩れた
- これは`swing_contact_penalty`が「足を上げ続ける」動作を優先させた副作用と推測
- 目標とする「大股でゆったりとした一般的な歩容」にはまだ乖離がある

### V31への方針

`single_foot_contact`報酬の強化（0.3→0.5）を優先的に試行し、交互歩行パターンの回復を目指す。`swing_contact_penalty`は-0.3に緩和して、タップダンス抑制と交互歩行のバランスを取る。

## 備考

- V29レポートの「次バージョンへの提案」セクションの**提案A+B**を採用
- 報酬項目数は19→20に増加（swing_contact_penalty追加）
- 報酬項目追加ルール（exp007_reward_design.md Section 1）に基づき、1項目のみ追加
- gait_frequencyとstep_lengthはV29から維持（0.9Hz、0.8）

### 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-unitree-v30 --no-viewer --duration 10
```

### 訓練ログ分析コマンド

```bash
cd rl_ws
uv run python scripts/analyze_training_log.py droid-walking-unitree-v30
uv run python scripts/show_reward_components.py droid-walking-unitree-v30
```

### 参考文献

- exp007_report_v29.md: V29の結果と次バージョンへの提案（提案A+B）
- exp007_unitree_rl_gym_survey.md: Unitree RL Gymの報酬設計
