# EXP006: BSL-Droid 足先空間（Task-Space）による強化学習歩容獲得実験

## 概要

BSL-Droid Simplified二脚ロボットモデルに対して、足先空間（Task-Space）アプローチによる強化学習歩容獲得を行う。exp004のPhase 19以降で確立されたFK/IK実装を活用し、関節空間（10 DOF）ではなく足先位置（4 DOF: 左右XZ）で行動空間を定義することで、探索効率の向上を目指す。

## 実験の進め方

実験手順・コマンド・ルールは [exp006_rules.md](exp006_rules.md) を参照。

## 背景

### 関節空間アプローチの限界（exp004 V1〜V21）

exp004のV1〜V21では関節空間（10 DOF）で行動空間を定義していたが、以下の限界が観察された：

- **胴体高さ維持の失敗**: 関節角度の組み合わせが膨大で、高さ維持と前進の両立が困難
- **小刻み歩行**: 歩幅が小さく、大股歩行への移行が困難
- **同期歩行**: 両脚が同時に動き、交互歩行が発生しない問題

### 足先空間アプローチへの移行

これらの問題を解決するため、V22以降では足先空間（Task-Space）アプローチに移行した：

- **行動空間の次元削減**: 10 DOF → 4 DOF（左右足先のXZ座標）
- **FK/IK変換**: 足先位置から関節角度を逆運動学（IK）で計算
- **物理的制約の暗黙的適用**: IKにより到達可能な姿勢のみが生成される

## ロボット仕様

### BSL-Droid Simplified

| 項目 | 値 |
|------|-----|
| 脚数 | 2脚 |
| DOF | 10 (片脚5関節×2) |
| 関節構成 | hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch |
| 膝構造 | 逆関節（負角度で後方屈曲、Unitree/ANYmal規約） |
| 胴体サイズ | 0.14m(D) × 0.18m(W) × 0.16m(H) |
| 総質量 | 約5.8kg |
| 脚長 | 約0.31m (thigh: 0.11m + shank: 0.12m + foot: 0.08m) |

### 運動学パラメータ

| パラメータ | 値 [m] | 説明 |
|-----------|--------|------|
| `thigh_length` | 0.11 | 大腿部長さ |
| `shank_length` | 0.12 | 下腿部長さ |
| `foot_height` | 0.035 | 足部高さ |
| `ankle_offset_x` | 0.02 | 足首から足先中心へのオフセット |

**到達範囲**: 
- 最大: 0.225m (`thigh + shank - 0.005`)
- 最小: 0.015m (`|thigh - shank| + 0.005`)

---

## Phase 1: FK/IK実装とV22環境構築

### 1.1 概要

足先空間アプローチの基盤となる運動学モジュール `DroidKinematics` の設計・実装・検証について記述する。

### 1.2 運動学モジュール設計

#### 1.2.1 ファイル構成

```
rl_ws/biped_walking/envs/
├── droid_kinematics.py         # FK/IK実装
├── droid_env_taskspace.py      # V22-V23環境（Residual Policy）
├── droid_env_taskspace_e2e.py  # V24環境（End-to-End）
└── droid_env_taskspace_e2e_v25.py  # V25環境（報酬バランス改善版）

rl_ws/scripts/
└── test_kinematics.py          # FK/IK検証スクリプト
```

#### 1.2.2 座標系定義

BSL-Droid Simplifiedの座標系：
- **X軸**: 前方（正）
- **Y軸**: 左方（正）
- **Z軸**: 上方（正）

#### 1.2.3 脚構造（5 DOF）

```
hip_yaw_joint   (Z軸回転)
  └─ hip_roll_joint  (X軸回転)
       └─ hip_pitch_joint (Y軸回転)
            └─ knee_pitch_joint (Y軸回転、逆関節：負で後方屈曲)
                 └─ ankle_pitch_joint (Y軸回転)
```

### 1.3 順運動学（FK）実装

FK は hip_pitch起点からの足先位置を計算する。

**XZ平面（サジタル面）での計算**:
```python
# 膝位置（大腿先端）
knee_x = thigh_length * sin(hip_pitch)
knee_z = -thigh_length * cos(hip_pitch)

# 足首位置（下腿先端）
cumulative_pitch = hip_pitch + knee_pitch
ankle_x = knee_x + shank_length * sin(cumulative_pitch)
ankle_z = knee_z - shank_length * cos(cumulative_pitch)

# 足先位置
foot_x = ankle_x + ankle_offset_x
foot_z = ankle_z - foot_height
```

### 1.4 逆運動学（IK）実装

標準的な2リンク平面IK（余弦定理）による解法:

```python
# 膝角度（余弦定理）
cos_knee_inner = (L1**2 + L2**2 - d**2) / (2*L1*L2)
knee_inner = acos(cos_knee_inner)
knee_pitch = -(π - knee_inner)  # 逆関節

# 股関節ピッチ
sin_beta = L2 * sin(knee_inner) / d
hip_pitch = alpha + beta

# 足首角度（地面平行維持）
ankle_pitch = -(hip_pitch + knee_pitch)
```

### 1.5 FK-IK整合性検証結果

```
=== DroidKinematics FK/IK 整合性検証 ===
Test 1: デフォルト足先位置 [PASS]
Test 2: FK→IK→FK ラウンドトリップ [PASS]
Test 3: バッチ処理の検証 [PASS]
Test 4: 3D IK検証（hip_yawによるY座標対応）[PASS]
Test 5: 左右対称性の検証 [PASS]
=== 全テスト PASSED ===
```

---

## Phase 2: V22 - Residual Policy Learning（失敗）

### 2.1 設計概要

V22は足先空間で探索するResidual Policyアプローチを採用した初のバージョン：

```python
# 行動空間: 4次元（足先XZ残差）
num_actions = 4  # [left_dx, left_dz, right_dx, right_dz]
action_scale = 0.02  # 2cm

# ベース軌道（正弦波パターン）
gait_frequency = 2.0   # Hz
swing_height = 0.03    # m
stride_length = 0.06   # m
```

### 2.2 評価結果

```
=== V22 Evaluation Statistics ===
X移動距離: -0.013 m  ← ほぼ前進せず
hip_pitch相関: 0.219  ← 弱い同期
接地率: 0%           ← 常に両足浮遊
```

### 2.3 失敗原因分析

**致命的問題: 足が地面に届いていない**

```
base_height = 0.35m（初期胴体高さ）
default_foot_z = -0.182m（股関節フレームでの足先Z座標）
→ 足先の絶対Z座標 = 0.35 - 0.182 = 0.168m

地面はZ=0なので、足は16.8cm浮いている。
action_scale = 0.02m（残差の最大振幅）
→ action=-1でも足先Z = 0.148m（まだ14.8cm浮いている）
```

**Residual Policy Learningの前提条件違反**:
- RPLは「良いが不完全なベースコントローラ」の上に残差を学習するアプローチ
- V22のベース軌道は「不完全」どころか「完全に機能しない」状態だった

---

## Phase 3: V23 - Residual Policy改善版

### 3.1 実装変更点

| パラメータ | V22 | V23 | 理由 |
|-----------|-----|-----|------|
| `action_scale` | 0.02m | 0.03m | 探索範囲の拡大 |
| `stride_length` | 0.06m | 0.08m | 歩幅拡大で前進効率向上 |
| `swing_height` | 0.03m | 0.035m | 足の持ち上げ高さ増加 |
| `hip_pitch_alternation` | - | 2.0 | 交互歩行促進（新規追加） |

### 3.2 結果

V23も同様にベース軌道の根本的問題を解決できず、前進運動は発生しなかった。

---

## Phase 4: V24 - Task-Space End-to-End（Residual廃止）

### 4.1 設計方針

RPLアプローチを放棄し、End-to-End（直接足先位置を学習）に移行。

#### 4.1.1 主な変更点

| 項目 | V22-V23 (Residual) | V24 (End-to-End) |
|------|----------------|------------------|
| 環境クラス | `DroidEnvTaskSpace` | `DroidEnvTaskSpaceE2E` |
| ベース軌道 | 正弦波スイング | **廃止** |
| gait_phase | あり（観測に含む） | **廃止** |
| 行動の意味 | デフォルト+残差 | **デフォルト+直接オフセット** |
| action_scale | 0.02m（2cm） | **0.05m（5cm）** |
| base_init_pos[2] | 0.35m | **0.19m**（足が地面に着く高さ） |
| 観測次元 | 39 | **37**（gait_phase削除） |

#### 4.1.2 初期高さ問題の解決

```
デフォルト足先Z（股関節フレーム）: -0.182m
足が地面に着くためのbase_height = 0.182m
1cm余裕を持たせて: base_init_pos[2] = 0.19m

検証:
  足先絶対Z = 0.19 - 0.182 = 0.008m（地面から0.8cm）
  action_scale = 0.05m で十分な可動範囲
```

### 4.2 V24訓練結果（500 iteration）

```
=== V24 Training Progress (500 iterations) ===

Mean Reward:
  iter    0:     8.39
  iter  100:   500.85
  iter  200:  2304.10
  iter  350:  3876.85  ← ピーク
  iter  499:  3039.67

Summary: Reward: 8.39 → 3039.67（大幅向上）
```

### 4.3 V24評価結果（model_499.pt）

```
=== V24 Evaluation Statistics (10秒間) ===
X移動距離: -0.061 m（後退）
合計移動距離: 0.065 m（ほぼ静止）

胴体高さ:
  Mean: 0.278 m（初期: 0.208 m → 上昇後安定）

DOF velocity RMS: 0.935 rad/s（低い→動きがほとんどない）

接地パターン:
  Both feet airborne: 500 steps (100.0%)  ← 完全に浮いている
```

### 4.4 ユーザー目視評価

**良い点**: なし

**悪い点**:
- 最初に跳ねるような動作で立ち上がる
- その後完全に静止
- 前進しない
- 姿勢が前傾したまま

### 4.5 V24失敗の根本原因分析

**現象**: 報酬は3000超まで上昇したが、評価では静止。

**原因**: **「静止」が最適解**

```
V24報酬構成（1ステップあたりの最大値概算）:
  alive: 0.5                  ← 静止でも獲得
  base_height: 2.0            ← 静止でも獲得
  tracking_lin_vel: 2.0       ← 前進しないと獲得できない
  
問題: alive + base_height (2.5) > tracking_lin_vel (2.0)
→ 静止していても高報酬を獲得できてしまう
```

### 4.6 V24からの教訓

1. **報酬バランスの重要性**: 主報酬（前進）を圧倒しない補助報酬設計が必要
2. **静止回避**: `alive`や`base_height`だけでは静止が最適解になりうる
3. **接地の重要性**: 足が地面に着いていないと歩行学習は不可能

---

## Phase 5: V25 - 報酬バランス改善版

### 5.1 設計方針

V24からの報酬バランス修正に特化。

#### 5.1.1 先行研究の知見（exp006_reward_design_survey.mdより）

> **Balance is Critical**: 報酬バランスが最重要。主報酬を圧倒しないペナルティ設計（全般）
> 
> 推奨: 主報酬 > ペナルティ合計（タスク達成を優先）
> 危険: 主報酬 < ペナルティ合計（何もしないことが最適解に）

#### 5.1.2 報酬スケール変更

| 報酬項 | V24 | V25 | 変更理由 |
|--------|-----|-----|---------|
| `tracking_lin_vel` | 2.0 | **5.0** | 前進を強力に報酬化（主報酬強化） |
| `forward_progress` | - | **3.0** | X方向移動を直接報酬化（新規） |
| `alive` | 0.5 | **0.1** | 静止の利益を大幅削減 |
| `base_height` | 2.0 | **0.5** | 静止の利益を大幅削減 |
| `foot_height_diff` | 1.5 | **2.0** | 交互歩行誘導強化 |
| `hip_pitch_alternation` | 0.5 | **1.0** | 交互歩行誘導強化 |

#### 5.1.3 新規報酬: forward_progress

速度追従（`tracking_lin_vel`）は「目標速度に近いか」を評価するが、`forward_progress`は「どれだけ前に進んだか」を直接報酬化：

```python
def _reward_forward_progress(self):
    """前進距離報酬"""
    delta_x = self.base_pos[:, 0] - self.last_base_pos_x
    return torch.clamp(delta_x, min=0.0) / self.dt  # 速度換算
```

#### 5.1.4 foot_height_diffの改善

V24では「目標差（2cm）に近いほど報酬」だったが、これは静止（差=0）でも一定の報酬を得られてしまう。

V25では「差が大きいほど報酬」に変更：

```python
# V24（問題あり）
target_diff = 0.02
return torch.exp(-torch.abs(height_diff - target_diff) / 0.01)

# V25（改善）
return height_diff / 0.05  # 差が大きいほど報酬、静止は報酬0
```

### 5.2 報酬バランス検証

```
前進した場合（速度0.2m/s）:
  tracking_lin_vel: 5.0 × exp(-small_error) ≈ 4.5
  forward_progress: 3.0 × 0.2 = 0.6
  hip_pitch_alternation: 1.0 × (difference) ≈ 0.5
  foot_height_diff: 2.0 × (difference) ≈ 0.4
  alive: 0.1
  base_height: 0.5 × (height_match) ≈ 0.4
  計: 約6.5/step

静止した場合:
  tracking_lin_vel: ≈ 3.0（目標0.2との誤差が大）
  forward_progress: 0
  hip_pitch_alternation: 0
  foot_height_diff: 0
  alive: 0.1
  base_height: 0.5
  計: 約3.6/step

→ 前進 (6.5) > 静止 (3.6) となり、前進が有利
```

### 5.3 ファイル構成

```
rl_ws/
├── biped_walking/
│   ├── envs/
│   │   ├── droid_env_taskspace_e2e.py      # V24用（変更なし）
│   │   └── droid_env_taskspace_e2e_v25.py  # V25用
│   └── train/
│       ├── droid_train_v24.py              # V24用（変更なし）
│       └── droid_train_v25.py              # V25用
└── logs/
    └── droid-walking-v25/                  # 訓練ログ出力先
```

### 5.4 実行方法

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_v25.py --max_iterations 500
```

### 5.5 動作検証（3イテレーション）

```
Learning iteration 0/3: 初期化完了
Learning iteration 1/3: Mean reward: 119.31, Episode length: 36.06
Learning iteration 2/3: Mean reward: 42.94, Episode length: 31.27
```

エラーなく実行完了。V25の本格的なトレーニング準備完了。

### 5.6 期待される結果

1. **前進運動の発生**: 報酬バランス改善により静止が最適解でなくなる
2. **交互歩行の促進**: `foot_height_diff`と`hip_pitch_alternation`の強化
3. **V24からの大幅改善**: 前進距離が0mから有意な値へ

### 5.7 V25評価結果（500 iteration完了後）

#### 5.7.1 訓練ログ分析

```
=== V25 Training Log Analysis (500 iterations) ===

報酬推移:
  最初: 53.24
  最終: 665.10
  最大: 736.08 (step 476)
  最小: -8.66 (step 3)

Quarter平均:
  Q1: 194.22, Q2: 327.73, Q3: 418.07, Q4: 554.86
  → 報酬は継続的に上昇

エピソード長:
  最初: 14.16 steps
  最終: 74.36 steps
  → 生存時間も大幅に向上
```

報酬は約12倍に増加し、エピソード長も5倍以上に改善。V24より大幅な進歩。

#### 5.7.2 評価結果（model_499.pt, 10秒間）

```
=== V25 Evaluation Statistics ===

移動距離:
  X: 0.102 m（目標: 2.0m @ 0.2m/s）→ 達成率5%
  Y: 0.112 m（横方向ドリフト）
  Total: 0.152 m

平均速度:
  X: 0.065 m/s（目標: 0.200 m/s）→ 32.5%達成
  Y: 0.140 m/s（横方向への流れ）

姿勢:
  Roll:  mean=-8.90°, std=4.53°（左傾斜）
  Pitch: mean=-6.42°, std=11.82°（不安定な前後揺れ）
  Yaw:   drift=-43.52°（大きな回転ドリフト）

胴体高さ:
  Mean: 0.238 m（初期0.19mより上昇）
  Std: 0.0163 m（変動大）

関節動作:
  hip_pitch相関: -0.363（弱い交互性あり、-1.0が理想）
  DOF velocity RMS: 2.639 rad/s

接地パターン（致命的問題）:
  Both feet grounded:   0 steps (0.0%)
  Single foot grounded: 0 steps (0.0%)
  Both feet airborne: 500 steps (100.0%) ← 常に浮遊
```

#### 5.7.3 目視評価（ユーザー報告）

**良い点**: なし

**悪い点**:
- 脚を八の字に開いたまま静止しようと震える
- そのまま倒れる

#### 5.7.4 V25失敗の根本原因分析

**症状1: 両足が常に浮遊**

```
接地検出閾値: contact_threshold = 0.025m（2.5cm）
実際の足先Z座標: 常に閾値以上 → 100%浮遊判定

原因仮説:
1. IKで計算した足先位置が、実際のシミュレーション結果と乖離
2. hip_rollが異常に大きい値（L: 1.592rad, R: 2.749rad）
   → 脚が大きく外側に開き、足先が浮く
```

**症状2: 脚が八の字に開く（hip_roll異常）**

```
正常なhip_roll: ±0.1 rad程度（±5.7°）
V25のhip_roll:
  Left:  0.000 〜 1.592 rad（0° 〜 91°）
  Right: -1.745 〜 1.004 rad（-100° 〜 57°）

→ IKがhip_rollを0固定で計算しているにも関わらず、
  実際の関節角度が大きく変動している

原因: V25のIKは hip_roll=0 を前提としているが、
     PDコントローラが追従できていない可能性
```

**症状3: Yaw drift（-43.5°）**

```
原因:
- 左右非対称な脚の動き
- Left knee negative ratio: 8%
- Right knee negative ratio: 97.6%
→ 左右で全く異なる関節動作をしている

これは_compute_joint_commands()内のインデックスマッピングか、
motor_dof_idxの順序に問題がある可能性
```

**症状4: 震え（高周波振動）**

```
先行研究より:
> Without joint angle priors, reward shaping alone never produced proper rolling.
> When angular velocity reward dominated, it rocked in place to maximize spin.

V25では:
- foot_height_diff: 2.0
- hip_pitch_alternation: 1.0
→ 「足の高さを変えよ」「股関節を交互に動かせ」という報酬が
  高周波の微小振動で満たされてしまう（exploitingの典型例）
```

#### 5.7.5 V25失敗の技術的原因まとめ

| 症状 | 原因 | 影響 |
|------|------|------|
| 八の字開脚 | hip_rollがIK=0だがPDで追従失敗 | 足が浮く、接地不可 |
| 震え | foot_height_diff等の報酬を微振動でexploit | エネルギー消費、不安定 |
| Yaw drift | 左右非対称な関節動作 | まっすぐ進めない |
| 接地0% | 上記複合 | 推進力なし |

---

## Phase 6: V26以降への改善指針

### 6.1 先行研究サーベイからの知見

#### 6.1.1 成功事例の共通点

| 研究 | フレームワーク | 行動空間 | 重要な報酬設計 |
|------|---------------|---------|--------------|
| [Cassie RL (UC Berkeley)](https://hybrid-robotics.berkeley.edu/publications/IJRR2024_Cassie_RL_Versatile_Locomotion.pdf) | PPO + 双歴史アーキテクチャ | 関節位置 | style-reward（モーション参照） |
| [Humanoid-Gym](https://arxiv.org/html/2404.05695v2) | PPO + IsaacGym | 関節位置 | gait reward（周期的脚運動） |
| [TumblerNet](https://www.nature.com/articles/s44182-025-00043-2) | PPO | 関節位置 | CoM/CoP追跡 |
| [Legged Gym](https://github.com/leggedrobotics/legged_gym) | PPO + IsaacGym | 関節位置 | 地形カリキュラム |

**共通点**:
1. **関節空間制御**: 足先空間ではなく関節角度を直接出力
2. **周期的歩容報酬**: gait phaseに連動した報酬
3. **カリキュラム学習**: 徐々に難易度を上げる
4. **参照モーション**: デモンストレーションや物理的に妥当な基準軌道

#### 6.1.2 失敗パターンの共通点

| パターン | 症状 | 原因 | 対策 |
|---------|------|------|------|
| Reward Hacking | 高周波振動 | 報酬を微動作でexploit | action_rate penalty強化 |
| 静止が最適 | 動かない | alive > tracking_vel | 報酬バランス調整 |
| IK特異点 | 脚が伸びきり | 可動範囲外 | 到達範囲クリップ |
| 左右非対称 | Yaw drift | マッピングエラー | インデックス検証 |

### 6.2 V26への具体的改善提案

#### 6.2.1 案A: 関節空間に戻る（推奨）

**理由**:
- 足先空間（Task-Space）の成功事例が少ない
- IK+PD追従の精度問題を回避
- 先行研究の大部分が関節空間を採用

```python
# V26: 関節空間に戻る
num_actions = 10  # 全関節角度オフセット
action_scale = 0.1  # rad（約5.7°）
```

#### 6.2.2 案B: 足先空間を維持（改善版）

足先空間を維持する場合、以下の修正が必須：

1. **hip_roll問題の修正**
```python
# IKでhip_rollを0固定ではなく、現在値を維持
def inverse_kinematics(self, foot_pos, current_hip_roll):
    # hip_rollは現在の値を保持
    hip_roll = current_hip_roll
    ...
```

2. **震え対策**
```python
reward_cfg = {
    "action_rate": -0.1,  # 大幅強化（V25: -0.01）
    "dof_acc": -1e-5,     # 大幅強化（V25: -1e-7）
}
```

3. **周期的歩容報酬の追加**
```python
def _reward_gait_cycle(self):
    """周期的な脚運動を報酬化（高周波振動を抑制）"""
    t = self.episode_length_buf * self.dt
    target_phase = t * gait_frequency * 2 * math.pi

    # 左脚はsin、右脚は-sin（逆位相）
    left_target_z = swing_height * torch.sin(target_phase)
    right_target_z = swing_height * torch.sin(target_phase + math.pi)

    left_error = torch.abs(self.current_foot_pos[:, 1] - left_target_z)
    right_error = torch.abs(self.current_foot_pos[:, 3] - right_target_z)

    return torch.exp(-(left_error + right_error) / 0.01)
```

#### 6.2.3 案C: カリキュラム学習

[Curricular Hindsight RL](https://www.nature.com/articles/s41598-024-79292-4)の知見：

```python
# Stage 1: 立位維持のみ
reward_scales_stage1 = {
    "base_height": 5.0,
    "orientation": -2.0,
    "alive": 1.0,
}

# Stage 2: 低速歩行
reward_scales_stage2 = {
    **reward_scales_stage1,
    "tracking_lin_vel": 3.0,
    "foot_height_diff": 1.0,
}

# Stage 3: 通常速度
reward_scales_stage3 = {
    **reward_scales_stage2,
    "tracking_lin_vel": 5.0,
    "forward_progress": 3.0,
}
```

### 6.3 優先度順の改善ステップ

1. **[高] 関節インデックスの検証**
   - `_setup_motor_dofs()`と`_compute_joint_commands()`のマッピングが一致しているか確認

2. **[高] action_rate/dof_acc ペナルティ強化**
   - 高周波振動（震え）を抑制

3. **[中] 周期的歩容報酬の追加**
   - gait_phase連動で自然な歩行リズムを誘導

4. **[中] カリキュラム学習**
   - 立位維持 → 低速歩行 → 通常歩行の段階的学習

5. **[低] 関節空間への回帰検討**
   - 足先空間の改善が困難な場合

---

## Phase 7: V26 - 足先空間改善版の実装

### 7.1 設計方針

V25の失敗分析（Phase 5.7）に基づき、**案B: 足先空間を維持した改善版**を採用。

#### 7.1.1 V25の主要な失敗原因と対策

| 失敗原因 | V25での症状 | V26での対策 |
|---------|------------|------------|
| hip_roll異常 | 脚が八の字に開く（0〜91°） | 行動空間に hip_roll を追加（明示的制御） |
| 高周波振動（震え） | 報酬を微振動でexploit | action_rate/dof_acc ペナルティ大幅強化 |
| 周期性欠如 | 自然な歩行リズムがない | gait_phase復活 + gait_cycle報酬追加 |
| 開脚抑制なし | hip_rollが無制限に変動 | hip_roll_penalty追加 |

### 7.2 V26の主要変更点

#### 7.2.1 行動空間の拡張（4次元 → 6次元）

```python
# V25: 4次元（足先XZのみ）
actions = [left_dx, left_dz, right_dx, right_dz]

# V26: 6次元（足先XZ + hip_roll）
actions = [left_dx, left_dz, right_dx, right_dz, left_hip_roll, right_hip_roll]
```

**理由**: V25ではIKが`hip_roll=0`を前提としていたが、PDコントローラが追従できず脚が開いてしまった。V26ではhip_rollを行動空間に含め、ポリシーが直接制御できるようにした。

#### 7.2.2 観測空間の変更（37次元 → 41次元）

```python
# V26観測空間（41次元）
obs = [
    base_ang_vel,           # 3
    projected_gravity,      # 3
    commands,               # 3
    dof_pos - default,      # 10
    dof_vel,                # 10
    actions,                # 6（V25: 4）
    gait_phase (sin/cos),   # 2（V25: なし）
    current_foot_pos,       # 4
]
```

#### 7.2.3 報酬設計の改善

| 報酬項 | V25 | V26 | 変更理由 |
|--------|-----|-----|---------|
| `action_rate` | -0.01 | **-0.1** | 震え対策（10倍強化） |
| `dof_acc` | -1e-7 | **-1e-5** | 震え対策（100倍強化） |
| `gait_cycle` | なし | **2.0** | 周期的歩容誘導（新規） |
| `hip_roll_penalty` | なし | **-1.0** | 開脚抑制（新規） |
| `foot_height_diff` | 2.0 | **1.0** | gait_cycleと役割分担 |
| `hip_pitch_alternation` | 1.0 | **0.5** | gait_cycleと役割分担 |

#### 7.2.4 歩容パラメータの調整

| パラメータ | V25 | V26 | 変更理由 |
|-----------|-----|-----|---------|
| `gait_frequency` | 2.0 Hz | **1.5 Hz** | よりゆっくりした歩行で安定化 |
| `目標速度範囲` | 0.2-0.5 m/s | **0.15-0.3 m/s** | 低速から開始 |
| `swing_height` | 0.03m | 0.03m | 変更なし |

### 7.3 新規報酬関数

#### 7.3.1 _reward_gait_cycle（周期的歩容報酬）

```python
def _reward_gait_cycle(self):
    """周期的歩容報酬（V26新規）

    gait_phaseに同期した足の高さを報酬化。
    高周波振動を抑制し、自然な歩行リズムを誘導。
    """
    # 左脚: sin(phase)で上下
    # 右脚: sin(phase + π)で逆位相
    left_target_z = self.default_foot_z + self.swing_height * torch.sin(self.gait_phase)
    right_target_z = self.default_foot_z + self.swing_height * torch.sin(self.gait_phase + math.pi)

    left_z = self.current_foot_pos[:, 1]
    right_z = self.current_foot_pos[:, 3]

    left_error = torch.abs(left_z - left_target_z)
    right_error = torch.abs(right_z - right_target_z)

    return torch.exp(-(left_error + right_error) / 0.02)
```

**設計意図**: V25の`foot_height_diff`は「左右の高さに差があればよい」という報酬だったため、高周波振動でexploitされた。V26の`gait_cycle`は「特定の周期で交互に動かせ」という報酬であり、振動では達成できない。

#### 7.3.2 _reward_hip_roll_penalty（開脚抑制ペナルティ）

```python
def _reward_hip_roll_penalty(self):
    """hip_rollが大きくなりすぎることへのペナルティ（V26新規）"""
    left_hip_roll = self.dof_pos[:, self.left_hip_roll_idx]
    right_hip_roll = self.dof_pos[:, self.right_hip_roll_idx]
    # 目標は0付近
    return torch.square(left_hip_roll) + torch.square(right_hip_roll)
```

**設計意図**: V25では脚が91°まで開いていた。V26ではhip_rollを行動空間に含めると同時に、0付近に維持するペナルティを追加。

### 7.4 ファイル構成

```
rl_ws/
├── biped_walking/
│   ├── envs/
│   │   ├── droid_env_taskspace_e2e_v25.py  # V25環境
│   │   └── droid_env_taskspace_e2e_v26.py  # V26環境（新規）
│   ├── train/
│   │   ├── droid_train_v25.py              # V25訓練
│   │   └── droid_train_v26.py              # V26訓練（新規）
│   └── biped_eval.py                       # 評価スクリプト（V26対応追加）
└── logs/
    └── droid-walking-v26/                  # 訓練ログ出力先
```

### 7.5 動作検証（3イテレーション）

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_v26.py --max_iterations 3 --num_envs 256
```

```
=== V26 動作検証結果 ===

Actor MLP: Sequential(
  (0): Linear(in_features=41, out_features=256, bias=True)  # 観測41次元
  ...
  (6): Linear(in_features=128, out_features=6, bias=True)   # 行動6次元
)

Learning iteration 0/3:
  Mean total reward: 15.72
  Mean episode length: 17.00 steps

Learning iteration 1/3:
  Mean total reward: 11.31
  Mean episode length: 36.63 steps  ← エピソード長が2倍以上に向上

Learning iteration 2/3:
  Mean total reward: -110.26
  Mean episode length: 30.91 steps
```

**観察**:
- 観測次元41、行動次元6が正しく設定されている
- エピソード長が17→36ステップに向上（V25初期の14ステップより改善）
- 報酬が負になっている → ペナルティが強すぎる可能性あり

### 7.6 本格訓練の実行方法

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_v26.py --max_iterations 500
```

### 7.7 評価方法

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-v26 --no-viewer --duration 10
```

### 7.8 V26で検証すべきポイント

1. **hip_roll異常の解消**: 脚が八の字に開かないか
2. **震えの抑制**: 高周波振動が減少したか
3. **周期的歩容**: gait_phaseに同期した脚運動が発生するか
4. **接地率の改善**: 0%から有意な値への向上
5. **前進距離の改善**: V25の0.102mから向上するか

### 7.9 想定される課題と次のステップ

#### 7.9.1 報酬バランスの微調整

動作検証で報酬が負になっているため、以下の調整が必要な可能性：

```python
# 調整案
"action_rate": -0.05,     # -0.1 → -0.05（緩和）
"dof_acc": -5e-6,         # -1e-5 → -5e-6（緩和）
"hip_roll_penalty": -0.5, # -1.0 → -0.5（緩和）
```

#### 7.9.2 V27以降の検討事項

V26で改善が不十分な場合：
1. **案A採用**: 関節空間に戻る（10次元行動空間）
2. **カリキュラム学習**: Stage 1で立位維持のみ → Stage 2で低速歩行
3. **参照モーション導入**: style-rewardによる自然な歩容誘導

---

## 参照ドキュメント

- [exp006_rules.md](exp006_rules.md) - 実験ルール・手順・コマンド
- [exp006_reward_design_survey.md](exp006_reward_design_survey.md) - 報酬設計の先行研究サーベイ
- exp004_droid_rl_walking.md - 関節空間アプローチの実験記録（V1〜V21）

## 外部参考文献

- [Reinforcement Learning for Versatile, Dynamic, and Robust Bipedal Locomotion Control (UC Berkeley)](https://journals.sagepub.com/doi/full/10.1177/02783649241285161)
- [Learning agility and adaptive legged locomotion via curricular hindsight RL](https://www.nature.com/articles/s41598-024-79292-4)
- [Humanoid-Gym: Zero-Shot Sim2Real Transfer](https://arxiv.org/html/2404.05695v2)
- [Legged Gym (Isaac Gym環境)](https://github.com/leggedrobotics/legged_gym)
- [Deep RL for Robotic Bipedal Locomotion Survey](https://arxiv.org/html/2404.17070v5)
