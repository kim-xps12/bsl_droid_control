# V8: 交互歩行報酬の修正と胴体安定性強化

## 概要

V7は速度目標（0.214 m/s）を達成したが、新規報酬関数（alternating_gait, foot_flat, step_length）の設計ミスにより意図した効果が得られなかった。特にhip_pitch相関が悪化（0.781→0.828）し、胴体傾斜（Roll std）も470%増加した。V8では報酬設計を修正し、動的な交互歩行と胴体安定性の両立を目指す。

## 前バージョンからの改善事項

V7レポート（exp007_report_v7.md）の「次バージョンへの提案」に基づく改善：

### V7の根本原因分析

1. **alternating_gait報酬の設計ミス**
   - 「hip_pitch位置の和が0」を報酬化したが、これは両脚が静止（デフォルト位置）でも達成可能
   - 結果：両脚が微小振動で同期し、高報酬を獲得

2. **foot_flat報酬の不完全性**
   - ankle_pitch（ピッチ方向）のみを対象としていたが、問題はankle_roll（ロール方向）の傾斜
   - 結果：足首傾斜問題が継続

3. **速度目標上昇による安定性低下**
   - lin_vel_x_range [0.15, 0.25]が高すぎ、Roll std 470%悪化

### V8パラメータ変更

| パラメータ | V7値 | V8値 | 変更理由 |
|-----------|------|------|---------|
| **lin_vel_x_range** | [0.15, 0.25] | **[0.10, 0.15]** | V6レベルに戻し安定性優先 |
| **alternating_gait** | 0.5 | **削除** | 設計ミス、機能していない |
| **hip_pitch_antiphase** | (なし) | **0.8** | 【新規】速度逆相関報酬 |
| **ankle_roll** | (なし) | **-2.0** | 【新規】足首ロールペナルティ |
| **foot_flat** | -3.0 | **-5.0** | 強化（ankle_pitchペナルティ） |
| **orientation** | -0.5 | **-1.0** | 胴体姿勢ペナルティ強化 |
| **ang_vel_xy** | -0.05 | **-0.1** | 胴体角速度ペナルティ強化 |

## 設計詳細

### 新規報酬関数

#### 1. hip_pitch_antiphase（速度逆相関報酬）

V7のalternating_gait報酬は「位置の和」を報酬化したが、これは静的な状態でも達成可能だった。V8では**速度の逆相関**を報酬化することで、動的な交互歩行を直接誘導する。

```python
def _reward_hip_pitch_antiphase(self):
    """hip_pitch速度の逆相関報酬（V8追加）"""
    left_vel = self.dof_vel[:, self.left_hip_pitch_idx]
    right_vel = self.dof_vel[:, self.right_hip_pitch_idx]

    # 速度の積が負（逆符号）なら報酬
    velocity_product = left_vel * right_vel

    # 移動コマンド時のみ適用
    is_moving = self.commands[:, 0].abs() > 0.05

    # tanh関数でスケーリングして滑らかな報酬に
    reward = -torch.tanh(velocity_product / 0.1)

    return reward * is_moving.float()
```

**設計原理**:
- 理想的な交互歩行では、左脚が前に振れるとき右脚は後ろに振れる
- hip_pitch速度の符号が逆（積が負）であれば報酬
- 両脚が同期している（積が正）場合は報酬なし
- tanh関数で滑らかなグラデーションを実現

#### 2. ankle_roll（足首ロールペナルティ）

V7のfoot_flat報酬はankle_pitchのみを対象としていた。V8ではankle_roll（BSL-Droidでは存在しないためhip_rollで代替）の偏差もペナルティ化。

```python
def _reward_ankle_roll(self):
    """足首ロール角ペナルティ（V8追加）"""
    # BSL-Droid Simplifiedにはankle_rollがないため、hip_rollで代替
    left_roll = self.dof_pos[:, self.left_hip_roll_idx]
    right_roll = self.dof_pos[:, self.right_hip_roll_idx]

    return torch.square(left_roll) + torch.square(right_roll)
```

**設計原理**:
- 足が内側/外側に傾くことを防止
- hip_rollがデフォルト（0度）から大きく逸脱するとペナルティ

### V8報酬スケール

```python
reward_cfg = {
    "tracking_sigma": 0.10,
    "base_height_target": 0.20,
    "swing_height_target": 0.03,
    "gait_frequency": 0.8,
    "contact_threshold": 0.08,
    "air_time_offset": 0.20,
    "reward_scales": {
        # 主報酬
        "tracking_lin_vel": 1.5,
        "tracking_ang_vel": 0.5,
        # 歩行品質報酬
        "feet_air_time": 1.5,
        "contact": 0.3,
        "alive": 0.03,
        "single_foot_contact": 1.0,
        "symmetry": 0.3,
        "hip_pitch_antiphase": 0.8,  # ★新規（alternating_gaitの代替）
        "step_length": 0.3,
        # 安定性ペナルティ ★強化
        "lin_vel_z": -2.0,
        "ang_vel_xy": -0.1,           # ★強化
        "orientation": -1.0,          # ★強化
        "base_height": -5.0,
        # 歩行品質ペナルティ
        "feet_swing_height": -5.0,
        "contact_no_vel": -0.1,
        "hip_pos": -0.5,
        "velocity_deficit": -2.0,
        "foot_flat": -5.0,            # ★強化
        "ankle_roll": -2.0,           # ★新規
        # エネルギー効率ペナルティ
        "torques": -1e-5,
        "action_rate": -0.005,
        "dof_acc": -2.5e-7,
        "dof_vel": -0.005,
    },
}
```

### V8の設計方針

1. **速度目標の保守化**: 高速歩行（0.15-0.25 m/s）を目指すのではなく、まず安定した低速歩行（0.10-0.15 m/s）を確立
2. **動的報酬への転換**: 位置ベースの報酬（和が0）から速度ベースの報酬（逆相関）へ
3. **胴体安定性の優先**: orientation, ang_vel_xyペナルティを強化し、Roll振動を抑制

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_unitree_v8.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-unitree-v8 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 71.41 |
| エピソード長 | 1001（最大） |
| 収束ステップ | 約150（step 150で62、以降は漸増） |
| 収束判定 | Last 50 steps std = 0.061（ほぼ収束） |

**報酬推移（Quarter-wise）:**

| ステップ範囲 | 平均報酬 | 増分 |
|-------------|---------|------|
| 0-125 | 30.12 | - |
| 125-250 | 64.91 | +34.79 |
| 250-375 | 69.95 | +5.04 |
| 375-500 | 71.10 | +1.16 |

### 評価結果

| 指標 | V7値 | V8値 | 変化 | 評価 |
|------|------|------|------|------|
| X速度 | 0.214 m/s | **0.108 m/s** | -49% | ✅ 目標0.10m/s達成 |
| hip_pitch相関 | +0.828 | **-0.382** | - | ✅ **大幅改善**（逆相関へ） |
| Roll std | 6.21° | **4.41°** | -29% | ○ 改善 |
| 単足接地率 | 95.6% | **74.0%** | -21.6% | ⚠️ 低下 |
| Yaw drift | -7.56° | **+113.09°** | - | ❌ **大幅悪化**（旋回） |

### 姿勢・安定性

| 指標 | 値 |
|------|-----|
| Roll | mean=-1.03°, std=4.41° |
| Pitch | mean=-0.71°, std=3.97° |
| Yaw drift | **+113.09°**（10秒間で約1回転） |
| Base height | 0.211 m (std=0.015) |

### 関節動作分析

| 関節 | L range (rad) | R range (rad) | L vel std (rad/s) | R vel std (rad/s) |
|------|---------------|---------------|-------------------|-------------------|
| hip_pitch | **0.237** | **0.358** | **0.521** | **1.137** |
| hip_roll | 0.107 | 0.135 | 0.379 | 0.502 |
| knee | 0.575 | 0.569 | 1.495 | 1.808 |
| ankle_pitch | 0.508 | 0.436 | 0.885 | 1.429 |
| ankle_roll | 0.285 | 0.402 | 0.920 | 2.023 |

**重要な観察**:
- **右脚hip_pitchの可動範囲が左脚の1.5倍**（0.358 vs 0.237）
- **右脚hip_pitchの速度stdが左脚の2.2倍**（1.137 vs 0.521）
- 左右非対称な動作が顕著

### 接地パターン

| パターン | ステップ数 | 割合 |
|----------|-----------|------|
| Both feet grounded | 124 | 24.8% |
| Single foot grounded | 370 | **74.0%** |
| Both feet airborne | 6 | 1.2% |

### 報酬コンポーネント分析（最終ステップ）

| 報酬項目 | 値 | 備考 |
|----------|-----|------|
| tracking_lin_vel | 0.837 | 主報酬、高い |
| hip_pitch_antiphase | **0.454** | 高報酬獲得 |
| single_foot_contact | 0.430 | 中程度 |
| symmetry | **0.165** | 低い（非対称） |
| tracking_ang_vel | 0.135 | Yaw追従 |
| contact | 0.180 | 歩行フェーズ |

## 考察と改善案

### 成功点

1. **hip_pitch相関の劇的改善**: +0.828（同期）→ -0.382（逆相関）
   - hip_pitch_antiphase報酬が意図通り機能
   - 「交互に動く」という動的パターンを学習

2. **胴体安定性の改善**: Roll std 6.21° → 4.41°（-29%）
   - orientation, ang_vel_xyペナルティ強化が有効

3. **速度追従**: X速度 0.108 m/s（目標0.10 m/s）を達成

### 課題（目視観察と一致）

1. **左右非対称な脚動作**
   - 目視観察：「左脚が全く動かない」「右脚の動きだけよくなった」
   - 定量データ：右脚hip_pitch range = 0.358 rad、左脚 = 0.237 rad（**1.5倍の差**）
   - 定量データ：右脚hip_pitch vel std = 1.137 rad/s、左脚 = 0.521 rad/s（**2.2倍の差**）

2. **大きなYawドリフト**
   - 目視観察：「その場で旋回して前進はしない」
   - 定量データ：+113.09°（10秒間で約1回転）
   - V7の-7.56°から**15倍悪化**

3. **単足接地率の低下**
   - V7: 95.6% → V8: 74.0%（-21.6ポイント）
   - 両足接地が24.8%に増加

### 根本原因分析

#### 1. hip_pitch_antiphase報酬の設計欠陥

現在の実装：
```python
velocity_product = left_vel * right_vel
reward = -torch.tanh(velocity_product / 0.1)
```

**問題点**：
- 「速度の積が負」を報酬化しているが、**片方が静止していても成立**
- 例：左脚速度=0、右脚速度=±1の場合、積=0でtanh(0)=0、報酬=0（ペナルティなし）
- ポリシーは「右脚だけ動かし、左脚は静止」という局所最適を発見
- **両脚が等しく動くことを保証していない**

#### 2. symmetry報酬の不十分さ

- symmetry報酬値: 0.165（低い）
- symmetryスケール: 0.3（相対的に弱い）
- hip_pitch_antiphase: 0.8と比較して、対称性への動機が弱い

#### 3. Yawドリフトの物理的原因

- 右脚のみが大きく動くと、その反作用でYaw方向に角運動量が発生
- tracking_ang_vel報酬（Yaw速度追従）のスケールが0.5と弱く、修正できない
- サーベイ7.4.1より：「意図的にYaw速度コマンドを変動させて訓練」が有効

### max_iterationsについて

**結論: max_iterationsを増やしても改善しない**

理由：
- 訓練は既に収束（Last 50 steps std = 0.061）
- 報酬は71.41で安定しており、これ以上の上昇は見込めない
- 問題は収束先の局所最適が「左右非対称な動作」であること
- **報酬設計の修正が必要**であり、訓練時間の延長では解決しない

### 次バージョン（V9）への提案

サーベイの知見と今回の失敗分析に基づく設計指針：

#### 優先度1: 両脚動作の強制（対称性強化）

**対策A: symmetry報酬の大幅強化**
```python
# V8: 0.3 → V9: 1.0以上
"symmetry": 1.0,
```

**対策B: 両脚最小動作報酬の追加**
```python
def _reward_both_legs_active(self):
    """両脚が共に動いていることを報酬化（V9追加）"""
    left_vel_mag = torch.abs(self.dof_vel[:, self.left_hip_pitch_idx])
    right_vel_mag = torch.abs(self.dof_vel[:, self.right_hip_pitch_idx])

    # 両脚の速度の最小値を報酬（片方が0なら報酬0）
    min_vel = torch.min(left_vel_mag, right_vel_mag)

    # 移動コマンド時のみ
    is_moving = self.commands[:, 0].abs() > 0.05
    return min_vel * is_moving.float()
```

**設計原理**: min(left, right)を報酬化することで、片方だけ動かす戦略を抑制

#### 優先度2: Yawドリフト抑制

**対策A: tracking_ang_vel強化**
```python
# V8: 0.5 → V9: 1.0以上
"tracking_ang_vel": 1.0,
```

**対策B: Yaw速度コマンド訓練**（サーベイ7.4.1）
```python
# 訓練中にYaw速度コマンドを変動させる
"ang_vel_range": [-0.3, 0.3],  # V8: [0, 0]
```

#### 優先度3: hip_pitch_antiphase報酬の修正

現在の設計では片脚静止でも許容されてしまう。修正案：

```python
def _reward_hip_pitch_antiphase_v2(self):
    """hip_pitch速度の逆相関報酬（V9修正版）"""
    left_vel = self.dof_vel[:, self.left_hip_pitch_idx]
    right_vel = self.dof_vel[:, self.right_hip_pitch_idx]

    # 両脚の速度の大きさ
    left_mag = torch.abs(left_vel)
    right_mag = torch.abs(right_vel)

    # 逆相関条件: 積が負
    antiphase = (left_vel * right_vel) < 0

    # 両脚が共に動いている場合のみ報酬
    both_active = (left_mag > 0.1) & (right_mag > 0.1)

    reward = antiphase.float() * both_active.float()

    is_moving = self.commands[:, 0].abs() > 0.05
    return reward * is_moving.float()
```

### V9パラメータ案

| パラメータ | V8値 | V9提案値 | 変更理由 |
|-----------|------|----------|---------|
| symmetry | 0.3 | **1.0** | 対称性強化 |
| tracking_ang_vel | 0.5 | **1.0** | Yawドリフト抑制 |
| **both_legs_active** | (なし) | **0.5** | 【新規】両脚動作強制 |
| hip_pitch_antiphase | 0.8 | **修正版** | 片脚静止を許さない |
| ang_vel_range | [0, 0] | **[-0.2, 0.2]** | Yaw訓練 |

---

## V1-V8振り返り分析

V8終了時点でexp007全体を振り返り、**報酬設計の複雑化による破綻リスク**を評価する。

### バージョン別成功/失敗パターン

| Version | X速度 | hip_pitch相関 | 主な変更 | 結果 |
|---------|-------|---------------|----------|------|
| V1 | 0.01 m/s | +0.954 | Unitreeベース実装 | ❌ 静止ポリシー |
| V2 | 0.004 m/s | - | 重み調整、目標速度低下 | ❌ 静止継続 |
| **V3** | **0.246 m/s** | +0.886 | velocity_deficit, single_foot_contact追加 | ✅ **静止回避成功** |
| **V4** | **0.192 m/s** | +0.792 | 接地検出修正（threshold 0.08） | ✅ **機能改善** |
| V5 | 0.003 m/s | +0.653 | 目標速度低下、dof_vel強化、symmetry追加 | ❌ **静止回帰** |
| V6 | 0.074 m/s | +0.781 | tracking_sigma 0.25→0.10 | △ 部分回復 |
| V7 | 0.214 m/s | +0.828 | alternating_gait, foot_flat, step_length追加 | △ 速度OK、交互歩行悪化 |
| V8 | 0.108 m/s | -0.382 | hip_pitch_antiphase追加 | △ 相関改善、**左右非対称・Yaw113°** |

### 報酬項目数の肥大化

| Version | 報酬項目数 | 新規追加 |
|---------|-----------|----------|
| V1 | 14 | 基本セット（Unitreeベース） |
| V3 | 16 | velocity_deficit, single_foot_contact |
| V5 | 18 | symmetry, air_time_offset |
| V7 | **21** | alternating_gait, foot_flat, step_length |
| V8 | **22** | hip_pitch_antiphase, ankle_roll |

**問題点**：報酬項目が14→22（57%増加）に肥大化し、相互作用の予測が困難になっている。

### サーベイ適用の問題

Unitreeサーベイの推奨値と実際の適用を比較：

| 項目 | サーベイ推奨 | 我々の適用 | 乖離 |
|------|-------------|-----------|------|
| tracking_lin_vel | 1.0 | 1.5 | +50% |
| orientation | -0.5〜-1.0 | -0.5→-1.0 | 妥当 |
| base_height | -5.0〜-10.0 | -5.0 | 妥当 |
| feet_swing_height | -5.0〜-20.0 | -5.0 | 妥当 |
| alive | 0.1〜0.15 | 0.03 | 減少（妥当） |
| **追加報酬** | **なし** | **8項目追加** | **大幅乖離** |

サーベイで推奨されていない報酬を8項目追加しており、これが複雑化の主因。

### 設計ミスの蓄積

| Version | 追加した報酬 | 設計ミス |
|---------|-------------|---------|
| V7 | alternating_gait | 「位置の和が0」→両脚静止でも達成可能 |
| V7 | foot_flat | ankle_pitchのみ対象→ankle_roll問題は未解決 |
| V8 | hip_pitch_antiphase | 速度の積が負→**片脚静止でも報酬0（ペナルティなし）** |

V7, V8の新規報酬は3つとも設計欠陥を含んでいた。

### V3-V4が成功した理由

V3-V4が相対的に成功した理由：

1. **最小限の変更**：velocity_deficit + single_foot_contact の2項目のみ追加
2. **明確な目的**：「静止を罰する」「片足接地を報酬化」という単純な設計
3. **Unitreeベースをほぼ維持**：基本構造を崩していない

### 破綻リスクの評価

**結論：パラメータの弄りすぎによる破綻方向に進んでいる**

根拠：
1. **報酬項目の肥大化**：14→22項目（57%増加）
2. **設計ミスの蓄積**：V7, V8で追加した4つの報酬のうち3つが欠陥あり
3. **相互作用の複雑化**：新報酬が既存報酬の効果を打ち消している可能性
4. **性能の不安定化**：V3で達成した0.246 m/sがV5で0.003 m/sに急落

### V9以降への提言

**オプションA：V4ベースにロールバック**

V4の成功要素のみを維持し、複雑な報酬を削除：

```python
# V4ベース + 検証済み報酬のみ
reward_scales = {
    # 主報酬（Unitreeベース）
    "tracking_lin_vel": 1.0,      # サーベイ推奨値に戻す
    "tracking_ang_vel": 0.5,
    "feet_air_time": 1.0,
    "contact": 0.2,
    "alive": 0.03,
    # V3-V4で有効だった追加報酬
    "single_foot_contact": 0.8,   # ✅ 機能確認済み
    "velocity_deficit": -0.5,     # ✅ 静止回避に有効
    # Unitreeベースのペナルティ（サーベイ推奨値）
    "lin_vel_z": -2.0,
    "ang_vel_xy": -0.05,
    "orientation": -0.5,
    "base_height": -5.0,
    "feet_swing_height": -5.0,
    "contact_no_vel": -0.1,
    "hip_pos": -0.5,
    "torques": -1e-5,
    "action_rate": -0.01,
    "dof_acc": -2.5e-7,
}
# 報酬項目数: 16（V3-V4レベル）
```

**オプションB：1つずつ慎重に追加**

V4ベースから、**1回に1つの変更のみ**追加し、効果を検証してから次の変更を行う：

1. V9a：V4ベース + tracking_sigma=0.10のみ
2. V9b：V9aが成功したら + symmetry=0.3
3. V9c：V9bが成功したら + 正しく設計したhip_pitch_antiphase

**推奨アクション**：
- サーベイ知見は「参考」として慎重に適用（そのまま適用しない）
- 新規報酬は追加前に設計の妥当性を検証
- 複数パラメータの同時変更を避ける

---

## まとめ

V8は**hip_pitch相関を+0.828から-0.382に改善**し、交互歩行の基本的なパターンを獲得した。しかし、**hip_pitch_antiphase報酬の設計欠陥**により「右脚だけ動かす」局所最適に収束し、以下の問題が発生：

1. **左右非対称**: 左脚がほぼ動かない（右脚の半分以下の可動範囲）
2. **大きなYawドリフト**: +113°（V7の15倍悪化）、その場で旋回
3. **前進しない**: 旋回のため実質的な前進が限定的

**max_iterationsを増やしても改善しない**（既に収束済み、std=0.061）。

V9では：
- **symmetry報酬を3倍以上に強化**（0.3 → 1.0）
- **両脚動作報酬（both_legs_active）を新規追加**
- **tracking_ang_velを2倍に強化**（0.5 → 1.0）
- **hip_pitch_antiphase報酬を修正**（両脚動作を必須条件に）

を実施し、対称的な交互歩行を目指す。

## 備考

- 学習ログ：`rl_ws/logs/droid-walking-unitree-v8/`
- 新規報酬関数の実装：`biped_walking/envs/droid_env_unitree.py`
- 参照：[exp007_unitree_rl_gym_survey.md](exp007_unitree_rl_gym_survey.md)
- 参照：[exp007_report_v7.md](exp007_report_v7.md) 「次バージョン（V8）への提案」セクション
