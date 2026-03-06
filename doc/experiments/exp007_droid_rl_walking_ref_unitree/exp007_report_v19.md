# V19: 歩幅報酬（step_length）の追加

## 概要

V17（静止の局所解で失敗）とV18（歩行実現、ストライド不足）の両方の知見を統合し、「ゆったり大股で歩く」を目指す。V18の成功要素（歩行実現、実機パラメータ適合）を維持しつつ、**step_length報酬のみを追加**してストライド改善を図る。

hip_pitch_range報酬は次バージョン（V20）で追加予定。一度に複数の報酬を追加すると効果の評価が困難になるため、段階的に追加する。

## 前バージョンからの改善事項

### V17とV18の比較分析

| 項目 | V17 | V18 | V19方針 |
|------|-----|-----|---------|
| ベース | V16（改善系） | V3（静止対策済み） | **V18をベースに採用** |
| X速度 | 0.006 m/s（失敗） | 0.248 m/s（成功） | V18の成功を維持 |
| hip_pitch可動域 | 0.020 rad（静止） | 0.368-0.432 rad（不足） | **0.5+ radを目標** |
| 特徴的報酬 | symmetry_range | dof_vel_limits | 両方の知見を統合 |
| 静止対策 | なし（失敗原因） | single_foot_contact, velocity_deficit | V18を維持 |
| 実機適合 | なし | dof_vel_limits | **V18を維持** |

### V17の失敗原因分析と対策

| 優先度 | 原因 | 根拠 | V19での対策 |
|--------|------|------|-------------|
| **1** | **tracking_lin_vel報酬の設計問題** | ガウシアン報酬（σ=0.25）は速度0でも約70%を獲得可能 | velocity_deficitを維持（直接対策済み） |
| **2** | **symmetry_range報酬の設計問題** | 静止状態でも最大報酬1.0を獲得可能 | **使用しない**（active_symmetry_rangeも見送り） |
| **3** | **step_length削除の影響** | 大股歩行の直接的動機が消失 | **step_lengthを復活** |
| **4** | **dof_velペナルティ強化** | -0.005→-0.01で動作自体が抑制 | V18の設定（dof_velなし）を維持 |

**教訓**: V17では「望ましくない状態（静止）でも報酬獲得可能」な報酬設計が失敗の原因だった。対称性報酬は「動いている状態での対称性」を評価する必要があるが、設計が複雑になるため、V19ではシンプルに「動くこと自体を報酬化」する方針を採用。

### V18の課題分析と対策

| 課題 | データ | 原因 | V19での対策 |
|------|--------|------|-------------|
| **ストライド不足** | hip_pitch L=0.368, R=0.432 rad | V15-V17の改善（step_length等）が未反映 | **step_length, hip_pitch_range追加** |
| **足上げ幅不足** | 目視：細かい足の動き | swing_height_target=0.03mが低い | **0.03→0.05mに引き上げ** |
| **接地検出失敗** | feet_air_time=0, single_foot_contact=0 | contact_threshold=0.025mが不適切 | **0.025→0.05mに緩和** |

**V18の成功要因（維持項目）**:
- 静止ポリシー対策: single_foot_contact=0.3, velocity_deficit=-0.5
- 実機パラメータ適合: dof_vel_limits=-0.3（44 rad/s制限）
- 安定した収束: std=0.0089で非常に安定

### V19での変更点（V18からの差分）

| パラメータ | V18値 | V19値 | 変更理由 |
|-----------|-------|-------|---------|
| **step_length** | なし | **0.5** | 歩幅報酬追加（V17失敗原因の直接対策） |
| **swing_height_target** | 0.03 | **0.05** | 足上げ高さ増加 |
| **feet_swing_height** | -5.0 | **-8.0** | 遊脚高さ目標追従ペナルティ強化（足上げ不足を抑制） |
| **contact_threshold** | 0.025 | **0.05** | 接地検出改善（全バージョン共通課題） |
| **gait_frequency** | 1.5 | **1.2** | やや遅くして大股歩行を促進 |
| **lin_vel_x_range** | [0.2, 0.3] | **[0.15, 0.25]** | 目標速度をやや低く（大股歩行促進） |
| **air_time_offset** | 0.5 | **0.3** | 小型ロボット向けに調整 |

**次バージョン（V20）で追加予定:**
- hip_pitch_range: hip_pitch可動域を明示的に報酬化

## 設計詳細

### 1. step_length報酬の活用

V7で実装済みの`_reward_step_length`を有効化する。この報酬は前後の足間距離を報酬化し、大股歩行を促進する。

```python
def _reward_step_length(self):
    """歩幅報酬（V7追加）

    前後の足の距離が大きいことを報酬化し、大股歩行を促進。
    """
    # 前後方向（X軸）の足間距離
    left_foot_x = self.feet_pos[:, 0, 0]
    right_foot_x = self.feet_pos[:, 1, 0]
    foot_distance_x = torch.abs(left_foot_x - right_foot_x)

    # 移動コマンド時のみ報酬
    is_moving = self.commands[:, 0].abs() > 0.05
    reward = foot_distance_x * is_moving.float()

    return reward
```

**V17での失敗原因との関連**:
- V17ではstep_lengthを削除し、symmetry_rangeに置き換えた
- その結果、大股歩行の動機が消失し、静止局所解に陥った
- V19ではstep_lengthを復活させ、歩幅促進の動機を回復

### 2. 報酬設計（17項目）

```python
reward_scales = {
    # === 主報酬 ===
    "tracking_lin_vel": 1.5,          # 線速度追従（V3強化）
    "tracking_ang_vel": 0.5,          # 角速度追従

    # === 歩行品質報酬 ===
    "feet_air_time": 1.0,             # 滞空時間報酬
    "contact": 0.2,                   # 接地フェーズ整合性
    "single_foot_contact": 0.3,       # 片足接地報酬（V3追加、静止対策）
    "step_length": 0.5,               # 【V19追加】歩幅報酬

    # === 安定性ペナルティ ===
    "lin_vel_z": -2.0,                # Z軸速度ペナルティ
    "ang_vel_xy": -0.05,              # XY角速度ペナルティ
    "orientation": -0.5,              # 姿勢ペナルティ
    "base_height": -5.0,              # 高さ維持

    # === 歩行品質ペナルティ ===
    "feet_swing_height": -8.0,        # 【V18: -5.0 → V19: -8.0】遊脚高さ目標追従（足上げ不足抑制強化）
    "contact_no_vel": -0.1,           # 接地時足速度
    "hip_pos": -0.5,                  # 股関節位置（開脚抑制）
    "velocity_deficit": -0.5,         # 速度未達ペナルティ（V3追加、静止対策）

    # === 関節速度制限 ===
    "dof_vel_limits": -0.3,           # 実機パラメータ超過ペナルティ（V18追加）

    # === エネルギー効率ペナルティ ===
    "torques": -1e-5,                 # トルクペナルティ
    "action_rate": -0.01,             # アクション変化率
    "dof_acc": -2.5e-7,               # 関節加速度
}
```

**項目数**: 17項目（V18から1項目追加）
- V18から削除: なし
- V18に追加: step_length（1項目追加）
- 変更: feet_swing_height（-5.0 → -8.0）

**V20で追加予定**: hip_pitch_range（可動域促進）

### 3. 報酬バランス検証

**正の報酬（理想状態）:**

| 報酬項目 | 重み | 最大値 | 期待値 |
|----------|------|--------|--------|
| tracking_lin_vel | 1.5 | 1.0 | 1.5 |
| tracking_ang_vel | 0.5 | 1.0 | 0.5 |
| feet_air_time | 1.0 | ~0.2 | 0.2 |
| contact | 0.2 | ~2.0 | 0.4 |
| single_foot_contact | 0.3 | 1.0 | 0.3 |
| step_length | 0.5 | ~0.1 | 0.05 |
| **合計** | | | **~2.95** |

**負のペナルティ（通常歩行）:**

| カテゴリ | 期待値 |
|----------|--------|
| 安定性ペナルティ | ~-0.5 |
| 歩行品質ペナルティ | ~-0.4 |
| 実機適合ペナルティ | ~0 |
| エネルギー効率 | ~-0.1 |
| **合計** | **~-1.0** |

**純報酬: ~1.95（正、動作を奨励）** ✓

### 4. その他の設定

```python
reward_cfg = {
    "tracking_sigma": 0.25,           # V18と同じ（厳格化はしない）
    "base_height_target": 0.20,       # 目標胴体高さ
    "swing_height_target": 0.05,      # 【V18: 0.03 → V19: 0.05】遊脚目標高さ
    "gait_frequency": 1.2,            # 【V18: 1.5 → V19: 1.2】歩行周波数（やや遅く）
    "contact_threshold": 0.05,        # 【V18: 0.025 → V19: 0.05】接地判定閾値
    "air_time_offset": 0.3,           # 【V18: 0.5 → V19: 0.3】滞空時間オフセット
    "dof_vel_limits": 44.0,           # 実機パラメータ維持（RS-02仕様）
    "soft_dof_vel_limit": 0.9,        # ソフトリミット係数（90%）
}

command_cfg = {
    "lin_vel_x_range": [0.15, 0.25],  # 【V18: [0.2, 0.3] → V19: [0.15, 0.25]】
    "lin_vel_y_range": [0, 0],
    "ang_vel_range": [0, 0],
}
```

**変更理由**:
- `swing_height_target`: V18の0.03mでは足上げが不十分だったため、0.05mに引き上げ
- `gait_frequency`: 1.5 Hz → 1.2 Hz でゆったり歩行を促進
- `contact_threshold`: 0.025m → 0.05m で接地検出の信頼性向上
- `air_time_offset`: 0.5s → 0.3s で小型ロボット（BSL-Droid）に適合
- `lin_vel_x_range`: [0.15, 0.25] で「ゆったり」歩行を実現

### 5. サーベイ知見との照合

**exp007_unitree_rl_gym_survey.md Section 7.3.2:**
> "Sinusoidal Reference Trajectory + Bayesian Optimization"

→ 基準軌道を与えることで大きなストライドを誘導できる。V19では基準軌道ではなく、`hip_pitch_range`報酬で同様の効果を狙う。

**exp007_unitree_rl_gym_survey.md Section 8.2:**
> "single_foot_contact報酬が最も信頼性が高く、チューニング不要で歩行を実現する方法"

→ V19では`single_foot_contact=0.3`を維持（V18で効果を確認済み）。

**exp007_unitree_rl_gym_survey.md Section 7.6.1:**
> "feet_air_timeオフセットの調整: 0.5→0.3秒"

→ V19で`air_time_offset=0.3`に変更（小型ロボット向け）。

## 期待される効果

| 指標 | V17値 | V18値 | V19目標 | 達成基準 |
|------|-------|-------|---------|----------|
| X速度 | 0.006 m/s | 0.248 m/s | 0.15-0.25 m/s | 目標範囲内 |
| hip_pitch可動域 L | 0.020 rad | 0.368 rad | **0.5+ rad** | 50%以上の増加 |
| hip_pitch可動域 R | 0.020 rad | 0.432 rad | **0.5+ rad** | 50%以上の増加 |
| 足上げ高さ | - | 細かい動き | **明確な足上げ** | 目視で確認 |
| 接地検出 | 0.0 | 0.0 | **>0** | feet_air_time, single_foot_contact > 0 |
| 実機適合 | - | 達成 | 維持 | dof_vel_limits ≈ 0 |
| Yawドリフト | +8.84° | +1.37° | <5° | 5°以内 |

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_unitree_v19.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-unitree-v19 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 41.81 |
| エピソード長 | 1001（フルエピソード、20秒） |
| 収束ステップ | ~200（std < 0.1） |
| 最終std | 0.061（収束） |

**四半期ごとの報酬推移:**
| 期間 | 平均報酬 | 変化 |
|------|----------|------|
| Steps 0-125 | 20.94 | - |
| Steps 125-250 | 39.23 | +18.29 |
| Steps 250-375 | 41.26 | +2.02 |
| Steps 375-500 | 41.66 | +0.40 |

### 評価結果

| 指標 | V18値 | V19目標 | V19結果 | 達成 |
|------|-------|---------|---------|------|
| X速度 | 0.248 m/s | 0.15-0.25 m/s | **0.185 m/s** | ✓ |
| 移動距離（10秒） | 2.476 m | - | **1.847 m** | - |
| hip_pitch可動域 L | 0.368 rad | 0.5+ rad | **0.531 rad** | ✓ |
| hip_pitch可動域 R | 0.432 rad | 0.5+ rad | **0.469 rad** | △ |
| L-R hip_pitch相関 | - | < 0 | **-0.859** | ✓（交互） |
| DOF range sum | 2.809 rad | > 3.0 | **3.533 rad** | ✓ |
| feet_air_time報酬 | 0.0 | > 0 | **0.0** | ❌ |
| single_foot_contact報酬 | 0.0 | > 0 | **0.0** | ❌ |
| step_length報酬 | - | > 0 | **0.0257** | ✓（新規） |
| dof_vel_limits報酬 | 0.0 | ≈ 0 | **0.0** | ✓ |
| Yawドリフト | +1.37° | < 5° | **+3.95°** | ✓ |
| Base高さ | - | 0.20 m | **0.224 m** | △（高め） |
| 接地パターン | - | 交互接地 | **100%両足空中** | ❌ |
| 目視評価 | 細かい足の動き | ゆったり大股 | **すり足状態** | ❌ |

### 報酬項目比較（V18 vs V19）

| 報酬項目 | V18 | V19 | 変化 |
|---------|-----|-----|------|
| tracking_lin_vel | 0.6054 | 0.4844 | -20%（目標速度低下のため妥当） |
| tracking_ang_vel | 0.1965 | 0.1559 | -21% |
| **feet_air_time** | **0.0000** | **0.0000** | **変化なし（問題継続）** |
| **single_foot_contact** | **0.0000** | **0.0000** | **変化なし（問題継続）** |
| contact | 0.0750 | 0.0603 | -20% |
| **step_length** | N/A | **0.0257** | ✓ V19で追加、機能 |
| feet_swing_height | -0.0062 | -0.0019 | 改善 |
| velocity_deficit | -0.0003 | -0.0005 | ほぼ同等 |

## 考察と改善案

### 成功点

1. **hip_pitch可動域の増加**: L=0.531 rad（目標達成）、R=0.469 rad（ほぼ達成）
2. **交互歩行の実現**: L-R相関-0.859で良好な交互動作
3. **DOF range sumの増加**: 2.809→3.533 rad（+26%）
4. **step_length報酬が機能**: 0.0257で歩幅動機付けに貢献
5. **安定した収束**: std=0.061で非常に安定
6. **Yawドリフト抑制**: +3.95°で良好

### 失敗点と根本原因

**問題**: 「すり足状態」が再発（前足と後足が固定）

**根本原因**: 接地検出の完全な失敗
- `feet_air_time = 0.0`（V18から変化なし）
- `single_foot_contact = 0.0`（V18から変化なし）
- 接地パターン: **100%両足空中と判定**

V19で`contact_threshold`を0.025m→0.05mに緩和したが、効果なし。

**原因分析**:
1. **Base高さが目標より高い**: 0.224m > 0.20m（目標）
2. **足のZ位置が常にcontact_thresholdより高い**: 高さベースの接地検出が機能していない
3. **足を上げる動機の消失**: feet_air_time=0、single_foot_contact=0のため、足を持ち上げるインセンティブがない
4. **step_lengthの限界**: 前後の足距離は報酬化するが、足を「上げる」動機にはならない

### exp007_unitree_rl_gym_survey.md知見との照合

**Section 6.1「接地検出の信頼性」**:
> 高さベースの接地検出は環境に依存し、閾値設定が困難。力ベースの検出への移行を検討。

→ V19では高さベース検出の限界に直面。力ベース検出への移行が必要。

**Section 7.4.1「swing_height_target追従」**:
> 目標を直接追従するreward設計の検討。ペナルティ形式では動かない解に収束しやすい。

→ V19の`feet_swing_height`はペナルティ形式（-8.0）だが、接地検出が機能しないため効果なし。

**Section 8.2「静止対策としてのsingle_foot_contact」**:
> single_foot_contact報酬は接地検出に依存。検出が失敗すると報酬が0になり、効果を発揮しない。

→ V19ではsingle_foot_contact=0で効果を発揮できていない。

### V20への改善提案

**優先度1（必須）: 接地検出の根本修正**

| 方策 | 内容 | 期待効果 |
|------|------|---------|
| **A. 力ベース検出への移行** | `contact_threshold`を高さから接触力に変更 | 接地検出の信頼性向上 |
| **B. 高さ閾値の大幅緩和** | 0.05m → 0.10m（base_height 0.224mに対応） | 即座に適用可能 |
| **C. 動的閾値** | base_heightに連動した閾値設定 | 環境変化に対応 |

**推奨**: A（力ベース）が根本解決だが、B（閾値緩和）を先に試行して効果を確認。

**優先度2: 足を上げる直接的動機**

| 方策 | 内容 | 期待効果 |
|------|------|---------|
| **feet_clearance報酬（正）** | 遊脚の高さを正の報酬で評価 | ペナルティ形式より直接的 |
| **swing_phase_height報酬** | スイングフェーズ中の足の高さを明示的に報酬化 | 足上げの強い動機 |

**優先度3: hip_pitch_range報酬（当初V20予定）**

V19で可動域は増加したが、接地検出問題により「すり足」に。接地検出修正後に追加。

### V20パラメータ変更案

```python
# 優先度1: 接地検出の修正
contact_threshold = 0.10  # 0.05 → 0.10m（大幅緩和）

# 優先度2: 足上げ報酬の追加
reward_scales = {
    ...
    "feet_clearance": 0.5,      # 【新規】遊脚高さの正報酬
    "step_length": 0.8,         # 0.5 → 0.8（強化）
    ...
}
```

## まとめ

### 実施内容

V19では、V17（静止の局所解で失敗）とV18（歩行実現、ストライド不足）の両方の知見を統合した：

1. **V18の成功要素を維持**
   - 静止ポリシー対策: single_foot_contact=0.3, velocity_deficit=-0.5
   - 実機パラメータ適合: dof_vel_limits=-0.3

2. **V17の失敗を回避**
   - symmetry_range報酬は使用しない（静止でも報酬獲得可能のため）
   - 代わりに「動くこと自体を報酬化」する方針を採用

3. **ストライド・足上げ幅の改善**
   - step_length報酬追加（歩幅促進）
   - swing_height_target引き上げ（0.03→0.05m）
   - feet_swing_height目標追従強化（-5.0→-8.0、足上げ不足を抑制）

4. **接地検出の改善試行**
   - contact_threshold緩和（0.025→0.05m）
   - air_time_offset調整（0.5→0.3s）

### 実際の結果

**成功した点:**
- hip_pitch可動域: L=0.531 rad（目標達成）、R=0.469 rad（ほぼ達成）
- 交互歩行: L-R相関-0.859（良好）
- DOF range sum: 3.533 rad（V18比+26%）
- step_length報酬が機能（0.0257）

**失敗した点:**
- **接地検出の失敗が継続**: feet_air_time=0, single_foot_contact=0
- **目視評価**: 「すり足状態」再発（前足と後足の固定）
- 接地パターン: 100%両足空中と誤判定

### 根本原因と次バージョンへの課題

**問題**: 高さベースの接地検出が機能していない
- Base高さ0.224mに対して、contact_threshold=0.05mでは不十分
- 足が常に閾値より高い位置にあり、「空中」と判定される

**V20での対策**:
1. **優先度1**: contact_thresholdの大幅緩和（0.05→0.10m）または力ベース検出への移行
2. **優先度2**: feet_clearance報酬（正の報酬形式）で足上げを直接促進
3. **優先度3**: hip_pitch_range報酬（接地検出修正後）

## 備考

- 学習スクリプト: `rl_ws/biped_walking/train/droid_train_unitree_v19.py`
- 環境クラス: `rl_ws/biped_walking/envs/droid_env_unitree.py`（`_reward_hip_pitch_range`追加）
- 参照: [exp007_report_v17.md](exp007_report_v17.md), [exp007_report_v18.md](exp007_report_v18.md)

### 学術的参考文献

- **STRIDE** (arXiv:2502.04692): 階層的報酬構造
- **Leveraging Symmetry in RL-based Legged Locomotion Control** (IROS 2024, arXiv:2403.17320): 対称性強制
- **Revisiting Reward Design and Evaluation for Robust Humanoid Standing and Walking** (arXiv:2404.19173): 報酬設計のベストプラクティス
- **RobStride RS-02仕様**: `ros2_ws/src/robstride_hardware/include/robstride_hardware/robstride_driver.hpp`
