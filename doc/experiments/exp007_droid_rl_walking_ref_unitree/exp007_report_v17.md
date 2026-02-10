# V17: 「ゆったり大股で歩く」の実現

## 概要

V16は数値的指標では改善を示したが（DOF range sum +48%、knee_pitch可動域 2-3倍）、目視評価では「細かい振動」「片脚固定」問題が継続した。根本原因分析により、**左右hip_pitch可動域の非対称**（L=0.524 vs R=0.197 rad、約2.7倍差）がこれらの問題の主因であることが判明した。

V17では、学術サーベイに基づく**階層的報酬設計**と**hip_pitch可動域の対称性報酬**の導入により、「ゆったり大股で歩く」を実現する。

## 前バージョンからの改善事項

V16レポート（exp007_report_v16.md）の「次バージョンへの提案」セクションに基づいて改善を実施。

### V16の問題分析

| 問題 | データ | 根本原因 |
|------|--------|----------|
| 左右非対称 | hip_pitch可動域 L=0.524 vs R=0.197 rad（2.7倍差） | symmetry報酬がhip_pitchを除外 |
| 接地検出失敗 | feet_air_time=0, single_foot_contact=0, 100%空中判定 | contact_threshold=0.025mが不適切 |
| Yawドリフト悪化 | V15: -4.92° → V16: +13.57° | 非対称推進力、tracking_ang_vel不足 |
| 振動継続 | 目視で細かい振動 | dof_vel=-0.005では不十分 |

### V17での変更点（項目数維持: 18項目）

| パラメータ | V16値 | V17値 | 変更理由 |
|-----------|-------|-------|---------|
| **symmetry_range** | (なし) | **0.5** | hip_pitch可動域の左右対称性（最重要）★追加 |
| **step_length** | 0.5 | **削除** | 対称性確保で自然と歩幅が出る、V16で効果なし |
| **tracking_ang_vel** | 0.5 | **0.8** | Yawドリフト対策 |
| **contact_threshold** | 0.025 m | **0.035 m** | 接地検出改善 |
| **dof_vel** | -0.005 | **-0.01** | 振動抑制強化 |

**設計方針**: step_lengthをsymmetry_rangeに置き換えることで、項目数を18項目に維持（推奨範囲15-17に近い）。

## 設計詳細

### 1. 「ゆったり大股で歩く」の数学的定式化

「ゆったり大股で歩く」を以下の3要素に分解して定式化した：

| 要素 | 定義 | 測定指標 | 報酬設計 |
|------|------|----------|----------|
| **ゆったり** | 低周波数・滑らかな動作 | gait_frequency ≤ 1.0Hz, 低dof_vel | dof_vel=-0.01 |
| **大股** | 大きなストライド長 | hip_pitch可動域 > 0.3 rad | symmetry_rangeで自然に実現 |
| **対称性** | 左右均等な動作 | hip_pitch可動域比 < 1.5x | symmetry_range=0.5 |

**考察**: step_lengthを明示的に報酬化するより、symmetry_rangeで左右対称性を確保することで、自然と両脚が同程度の可動域で動き、結果として大股歩行が実現される。V16ではstep_length=0.5を追加したが左右非対称（L/R=2.7x）は解消されなかった。

### 2. 階層的報酬設計（学術サーベイに基づく）

学術論文（STRIDE arXiv:2502.04692、Leveraging Symmetry IROS 2024）の知見に基づき、報酬を5階層に整理した：

```
R_total = R_tier1 + R_tier2 + R_tier3 + R_tier4 + R_tier5

R_tier1（主報酬）     = 1.5 * tracking_lin_vel + 0.8 * tracking_ang_vel
R_tier2（歩行品質）   = 1.5 * feet_air_time + 0.5 * symmetry_range + 0.3 * single_foot_contact + 0.2 * contact
R_tier3（安定性）     = Σ(stability_penalties)
R_tier4（動作品質）   = Σ(motion_quality_penalties)
R_tier5（エネルギー） = Σ(energy_penalties)
```

### 3. 新報酬関数: symmetry_range

既存の`_reward_symmetry`はhip_pitchを除外しているため、V16の問題に対処できない。新たに`_reward_symmetry_range`を追加し、hip_pitch可動域の対称性を評価する：

```python
def _reward_symmetry_range(self):
    """hip_pitch可動域の左右対称性報酬（V17追加）"""
    left_hip_pitch = self.dof_pos[:, self.left_hip_pitch_idx]
    right_hip_pitch = self.dof_pos[:, self.right_hip_pitch_idx]

    # デフォルトからの偏差（振幅の指標）
    left_deviation = torch.abs(left_hip_pitch - self.default_dof_pos[self.left_hip_pitch_idx])
    right_deviation = torch.abs(right_hip_pitch - self.default_dof_pos[self.right_hip_pitch_idx])

    # 偏差の差をペナルティ（対称なら0）
    deviation_diff = torch.abs(left_deviation - right_deviation)

    # 指数型報酬（対称時に最大1.0）
    sigma = 0.1  # rad
    return torch.exp(-deviation_diff / sigma)
```

**設計意図**:
- 位相ではなく「振幅」の対称性を評価
- 左右が同程度の可動域で動くことを促進
- 片脚だけ大きく動く局所最適を回避

### 4. 報酬バランス検証

静止ポリシー回避のため、正報酬合計 > ペナルティ合計を確認：

**正の報酬（理想状態）:**
| 報酬項目 | 重み | 最大値 | 期待値 |
|----------|------|--------|--------|
| tracking_lin_vel | 1.5 | 1.0 | 1.5 |
| tracking_ang_vel | 0.8 | 1.0 | 0.8 |
| feet_air_time | 1.5 | ~0.2 | 0.3 |
| symmetry_range | 0.5 | 1.0 | 0.5 |
| single_foot_contact | 0.3 | 1.0 | 0.3 |
| contact | 0.2 | ~2.0 | 0.4 |
| **合計** | | | **~3.8** |

**負のペナルティ（通常歩行）:**
| カテゴリ | 期待値 |
|----------|--------|
| 安定性ペナルティ | ~-0.5 |
| 動作品質ペナルティ | ~-0.3 |
| エネルギー効率 | ~-0.1 |
| **合計** | **~-0.9** |

**純報酬: ~2.9（正、動作を奨励）** ✓

### 5. 報酬設計（18項目、V16と同数）

```python
reward_scales = {
    # === Tier 1: 主報酬 ===
    "tracking_lin_vel": 1.5,
    "tracking_ang_vel": 0.8,      # V16 0.5 → V17 0.8

    # === Tier 2: 歩行品質報酬 ===
    "feet_air_time": 1.5,
    "contact": 0.2,
    "single_foot_contact": 0.3,
    "symmetry_range": 0.5,        # V17 新規（step_lengthと入替）
    # step_length: 削除 - symmetry_rangeで対称性確保すれば自然と歩幅が出る

    # === Tier 3: 安定性ペナルティ ===
    "lin_vel_z": -2.0,
    "ang_vel_xy": -0.05,
    "orientation": -0.5,
    "base_height": -5.0,

    # === Tier 4: 動作品質ペナルティ ===
    "feet_swing_height": -5.0,
    "contact_no_vel": -0.1,
    "hip_pos": -0.5,
    "velocity_deficit": -0.5,

    # === Tier 5: エネルギー効率ペナルティ ===
    "torques": -1e-5,
    "action_rate": -0.005,
    "dof_acc": -2.5e-7,
    "dof_vel": -0.01,             # V16 -0.005 → V17 -0.01
}
```

### 6. その他の設定

```python
reward_cfg = {
    "tracking_sigma": 0.25,
    "base_height_target": 0.20,
    "swing_height_target": 0.04,
    "gait_frequency": 1.0,
    "contact_threshold": 0.035,   # V16 0.025 → V17 0.035
    "air_time_offset": 0.3,
}

command_cfg = {
    "lin_vel_x_range": [0.15, 0.25],
    "lin_vel_y_range": [0, 0],
    "ang_vel_range": [0, 0],
}
```

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_unitree_v17.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-unitree-v17 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 52.19 |
| エピソード長 | 994.9（最大1001） |
| 収束ステップ | ~150 |

**報酬推移（Quarter-wise）:**
- Steps 0-125: 27.22
- Steps 125-250: 51.22 (+24.00)
- Steps 250-375: 52.50 (+1.28)
- Steps 375-500: 52.44 (-0.06) ← 収束

**報酬成分（最終ステップ）:**

| 報酬項目 | 値 | 評価 |
|---------|-----|------|
| tracking_lin_vel | 0.8326 | ⚠️ 高報酬だが実速度は0.006 m/s |
| tracking_ang_vel | 0.5051 | ✓ |
| symmetry_range | 0.3130 | ⚠️ 静止でも達成可能 |
| contact | 0.1202 | ✓ |
| feet_air_time | 0.0000 | ❌ 接地検出失敗 |
| single_foot_contact | 0.0000 | ❌ 接地検出失敗 |
| velocity_deficit | -0.0133 | ✓ 低速ペナルティ機能 |

### 評価結果

| 指標 | V16値 | V17目標 | V17結果 | 達成 |
|------|-------|---------|---------|------|
| X速度 | 0.185 m/s | 0.15-0.20 m/s | **0.006 m/s** | ❌ |
| hip_pitch可動域 L | 0.524 rad | 0.3-0.5 rad | **0.020 rad** | ❌ |
| hip_pitch可動域 R | 0.197 rad | 0.3-0.5 rad | **0.020 rad** | ❌ |
| L/R比 | 2.7x | < 1.5x | **1.0x** | ✓（静止による） |
| Yawドリフト(10s) | +13.57° | < 5° | **+8.84°** | ❌ |
| feet_air_time報酬 | 0.0 | > 0.1 | **0.0** | ❌ |
| single_foot_contact報酬 | 0.0 | > 0.1 | **0.0** | ❌ |
| 目視評価 | 振動・片脚固定 | ゆったり大股 | **静止の局所解** | ❌ |

**詳細評価データ（10秒間）:**

| カテゴリ | 指標 | 値 |
|---------|------|-----|
| 移動 | 総移動距離 | 0.057 m |
| 移動 | X移動距離 | 0.052 m |
| 姿勢 | Roll mean | -3.53° |
| 姿勢 | Pitch mean | 10.70° |
| 姿勢 | Base height mean | 0.235 m |
| 関節 | DOF range sum | 2.006 rad |
| 関節 | hip_pitch相関 | +0.496（同期傾向） |
| 接地 | 両足接地率 | 0.0% |
| 接地 | 片足接地率 | 0.0% |
| 接地 | 両足空中率 | **100.0%** |

**関節可動域詳細（rad）:**

| 関節 | L min | L max | L range | R min | R max | R range |
|------|-------|-------|---------|-------|-------|---------|
| hip_yaw | -0.082 | -0.000 | 0.082 | -0.008 | 0.030 | 0.038 |
| hip_roll | -0.041 | 0.040 | 0.081 | -0.048 | 0.060 | 0.108 |
| hip_pitch | 1.030 | 1.050 | **0.020** | 1.033 | 1.053 | **0.020** |
| knee_pitch | -1.827 | -1.652 | 0.175 | -1.807 | -1.363 | 0.444 |
| ankle_pitch | 0.418 | 0.785 | 0.367 | 0.115 | 0.785 | 0.670 |

## 考察と改善案

### 成功点

- **転倒回避**: エピソード長994.9/1001でほぼ最大、転倒なしで20秒間姿勢維持
- **L/R対称性達成**: hip_pitch L/R比が2.7x→1.0xに改善（ただし静止状態による）
- **高報酬獲得**: 最終報酬52.19で安定収束

### 課題

#### 1. 静止の局所解（最重要）

**データによる根拠:**
- X速度: 0.006 m/s（目標の4%）
- 総移動距離: 0.057 m（10秒でたった5.7cm）
- hip_pitch可動域: L=0.020 rad, R=0.020 rad（ほぼ不動）
- DOF range sum: 2.006 rad（V16: 2.724 rad から低下）

**原因分析:**

| 優先度 | 原因 | 根拠 |
|--------|------|------|
| **1** | **tracking_lin_vel報酬の設計問題** | 報酬0.8326なのに実速度0.006 m/s。ガウシアン報酬（σ=0.25）は速度0でも `exp(-0.15²/0.25²)≈0.70` を獲得可能 |
| **2** | **symmetry_range報酬の設計問題** | 静止状態（両脚がデフォルト位置）でも最大報酬1.0を獲得可能。「動いている状態での対称性」を評価していない |
| **3** | **step_length削除の影響** | 大股歩行の直接的動機が消失。symmetry_rangeは対称性のみで歩幅を促進しない |
| **4** | **dof_velペナルティ強化** | -0.005→-0.01で動作自体が抑制された可能性 |

#### 2. 接地検出の継続失敗

- feet_air_time報酬: 0.0（V16と同じ）
- single_foot_contact報酬: 0.0（V16と同じ）
- 両足空中率: 100%

**原因**: contact_threshold=0.035mへの調整が不十分。実際の接地高さとの乖離が大きい。

#### 3. サーベイ知見との照合

**exp007_unitree_rl_gym_survey.md Section 8.2:**
> "single_foot_contact報酬が最も信頼性が高く、チューニング不要で歩行を実現する方法"

→ V17ではsingle_foot_contact=0.3を使用しているが、接地検出失敗で機能していない。

**exp007_unitree_rl_gym_survey.md Section 7.2.3:**
> "対称性を『報酬shaping』で誘導する従来手法はソフト制約に過ぎず、完全な対称性は保証されない"

→ symmetry_rangeは静止でも達成可能なソフト制約に過ぎない。

**exp007_unitree_rl_gym_survey.md Section 8.5:**
> "カリキュラム学習による局所最適回避: 高速歩行から開始して徐々に目標速度を下げる"

→ V17では固定速度（0.15-0.25 m/s）で訓練。静止が最適解として学習されやすい。

### 次バージョン（V19）への提案

#### 優先度1（必須）: 報酬設計の根本的見直し

**A. tracking_lin_vel報酬の修正:**

問題: ガウシアン報酬は速度0でも高報酬を獲得可能

```python
# 現在（V17）: 静止でも報酬獲得可能
lin_vel_error = torch.sum(torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1)
reward = torch.exp(-lin_vel_error / self.reward_cfg["tracking_sigma"])
# 速度0でcmd=0.15の場合: exp(-0.15²/0.25²) ≈ 0.70

# 提案A: 最低速度要件を追加
min_vel = 0.05  # m/s
vel_magnitude = torch.norm(self.base_lin_vel[:, :2], dim=1)
moving = (vel_magnitude > min_vel).float()
reward = torch.exp(-lin_vel_error / self.reward_cfg["tracking_sigma"]) * moving

# 提案B: velocity_deficitペナルティの強化（-0.5 → -2.0）
"velocity_deficit": -2.0,  # 静止への強いペナルティ
```

**B. symmetry_rangeを「動的対称性」に変更:**

問題: 静止状態でも最大報酬を獲得可能

```python
def _reward_active_symmetry_range(self):
    """動いている状態での対称性のみ報酬（V19への提案）"""
    left_hip = self.dof_pos[:, self.left_hip_pitch_idx]
    right_hip = self.dof_pos[:, self.right_hip_pitch_idx]

    # 両脚が動いているか確認
    left_vel = torch.abs(self.dof_vel[:, self.left_hip_pitch_idx])
    right_vel = torch.abs(self.dof_vel[:, self.right_hip_pitch_idx])
    both_active = ((left_vel > 0.1) & (right_vel > 0.1)).float()

    # 可動域の対称性（既存ロジック）
    left_dev = torch.abs(left_hip - self.default_dof_pos[self.left_hip_pitch_idx])
    right_dev = torch.abs(right_hip - self.default_dof_pos[self.right_hip_pitch_idx])
    symmetry = torch.exp(-torch.abs(left_dev - right_dev) / 0.1)

    # 動いている時のみ報酬
    return symmetry * both_active
```

**C. step_lengthの復活:**

V16で削除したstep_lengthを復活させ、大股歩行の動機を回復:

```python
"step_length": 0.3,  # 歩幅報酬を復活（symmetry_rangeと併用）
```

#### 優先度2（推奨）: 接地検出の修正

```python
# 現在: contact_threshold = 0.035 m（不十分）
# 提案: より緩い閾値 + 力ベース検出の併用
"contact_threshold": 0.05,  # 高さベース閾値を緩和

# または力ベース検出の追加
def _compute_contacts(self):
    # 高さベース（既存）
    height_contact = self.foot_heights < self.contact_threshold
    # 力ベース（追加）
    force_contact = self.contact_forces[:, self.feet_indices, 2] > 1.0
    return height_contact | force_contact
```

#### 優先度3（オプション）: カリキュラム学習の導入

サーベイSection 8.5に基づき、高速から開始して徐々に減速:

```python
# 初期: 高速歩行（静止回避）
command_cfg = {
    "lin_vel_x_range": [0.3, 0.5],  # 高速スタート
}

# 訓練進行に応じて目標速度を下げる
if iteration > 250:
    command_cfg["lin_vel_x_range"] = [0.15, 0.25]  # 目標速度
```

#### 優先度4（オプション）: dof_velペナルティの緩和

```python
"dof_vel": -0.005,  # V16レベルに戻す（-0.01は過剰抑制の可能性）
```

### V19設計の推奨パラメータ変更

| パラメータ | V17値 | V19提案値 | 変更理由 |
|-----------|-------|-----------|---------|
| velocity_deficit | -0.5 | **-2.0** | 静止への強いペナルティ |
| symmetry_range | 0.5 | **削除** | active_symmetry_rangeに置換 |
| active_symmetry_range | なし | **0.5** | 動的対称性のみ報酬 |
| step_length | 削除 | **0.3** | 歩幅動機の復活 |
| dof_vel | -0.01 | **-0.005** | 過剰抑制の回避 |
| contact_threshold | 0.035 | **0.05** | 接地検出改善 |
| tracking_sigma | 0.25 | **0.15** | 速度追従の厳格化 |

## まとめ

### 実施内容

V17では、V16の根本原因分析に基づき、以下の改善を実施した：

1. **symmetry_range報酬の新規追加**: hip_pitch可動域の左右対称性を明示的に報酬化
2. **階層的報酬設計**: 学術サーベイに基づく5階層構造の整理
3. **接地検出改善**: contact_threshold緩和（0.025→0.035m）
4. **Yawドリフト対策**: tracking_ang_vel強化（0.5→0.8）
5. **振動抑制強化**: dof_vel強化（-0.005→-0.01）

### 結果

**V17は「静止の局所解」に陥り、失敗した。**

- X速度: 0.006 m/s（目標の4%）
- 10秒間の移動距離: 5.7cm
- hip_pitch可動域: 0.020 rad（ほぼ不動）

### 失敗の根本原因

1. **tracking_lin_vel報酬の設計問題**: ガウシアン報酬（σ=0.25）は速度0でも約70%の報酬を獲得可能
2. **symmetry_range報酬の設計問題**: 静止状態でも最大報酬を獲得可能
3. **step_length削除**: 大股歩行の動機が消失
4. **dof_vel強化**: 動作自体が抑制された可能性

### 教訓

- 報酬設計では「望ましくない状態（静止）でも報酬獲得可能か」を必ず検証すべき
- 対称性報酬は「動いている状態での対称性」を評価する必要がある
- 複数の報酬変更を同時に行うと、どの変更が問題かの特定が困難

### 次ステップ

V19(V18は別で実装済み)では以下を実装する：
1. velocity_deficitペナルティ強化（-0.5→-2.0）
2. active_symmetry_range（動的対称性のみ報酬）への変更
3. step_lengthの復活
4. 接地検出閾値の追加緩和

## 備考

- 学習スクリプト：`rl_ws/biped_walking/train/droid_train_unitree_v17.py`
- 環境クラス：`rl_ws/biped_walking/envs/droid_env_unitree.py`（`_reward_symmetry_range`追加）
- 参照：[exp007_report_v16.md](exp007_report_v16.md)「次バージョンへの提案」

### 学術的参考文献

- **STRIDE** (arXiv:2502.04692): 階層的報酬構造
- **Leveraging Symmetry in RL-based Legged Locomotion Control** (IROS 2024, arXiv:2403.17320): 対称性強制
- **Revisiting Reward Design and Evaluation for Robust Humanoid Standing and Walking** (arXiv:2404.19173): 報酬設計のベストプラクティス
