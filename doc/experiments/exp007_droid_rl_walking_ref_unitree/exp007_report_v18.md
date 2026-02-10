# V18: RobStride 02実機パラメータ適合（関節角速度制限）

## 概要

V3の報酬設計をベースに、RobStride RS-02モータの実機パラメータ（最大角速度 ±44 rad/s）に適合した動作を学習させることを目的とする。シミュレーション環境で速度制限を超える動作を学習すると、実機展開時に速度飽和による動作不安定・性能劣化が発生する。本バージョンでは関節角速度のソフトリミット報酬を導入し、実機パラメータ内での動作獲得を目指す。

## 前バージョンからの改善事項

V3は穏やかな高さペナルティと膝正角度維持により、交互歩行の萌芽が見られたバージョンである。本バージョンではV3の報酬設計を維持しつつ、実機展開を見据えた改善を行う。

| パラメータ | V3値 | V18値 | 変更理由 |
|-----------|------|-------|---------|
| `dof_vel_limits` | 設定なし | 44.0 rad/s | RobStride RS-02実機仕様 |
| `soft_dof_vel_limit` | 設定なし | 0.9 (90%) | 制限の90%でペナルティ開始 |
| `reward_scales["dof_vel_limits"]` | 設定なし | -0.3 | 速度制限超過ペナルティ |

### 実機パラメータの根拠

RobStride RS-02の仕様（出典: `ros2_ws/src/robstride_hardware/include/robstride_hardware/robstride_driver.hpp`）:

```cpp
namespace scale {
  constexpr double POSITION = 4.0 * 3.14159265358979323846;  // ±4π rad
  constexpr double VELOCITY = 44.0;   // ±44 rad/s (RS-02 spec)
  constexpr double TORQUE = 17.0;     // ±17 Nm (RS-02 spec)
  constexpr double KP = 500.0;        // 0-500 Nm/rad (RS-02 spec)
  constexpr double KD = 5.0;          // 0-5 Nm/(rad/s) (RS-02 spec)
}
```

## 設計詳細

### 新規報酬関数: `_reward_dof_vel_limits`

関節角速度のソフトリミット報酬を実装。Unitree RL Gymの実装を参考にした。

**実装（`rl_ws/biped_walking/envs/droid_env_unitree.py` に追加）**:

```python
def _reward_dof_vel_limits(self):
    """関節角速度のソフトリミット報酬（V18追加）

    【設計原理】
    実機（RobStride RS-02）の最大角速度は ±44 rad/s。
    シミュレーションでこの制限を超える動作を学習すると、実機展開時に
    速度飽和による動作不安定・性能劣化が発生する。

    この報酬はsoft_dof_vel_limit（デフォルト90%）を超えた角速度に
    ペナルティを与えることで、実機パラメータ内での動作を促す。

    【参考文献】
    - ETH Zurich Legged Gym:
      https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot.py
    - RobStride RS-02仕様:
      ros2_ws/src/robstride_hardware/include/robstride_hardware/robstride_driver.hpp

    【実装】
    - dof_vel_limits: 実機の最大速度（44 rad/s）
    - soft_dof_vel_limit: 制限の何%で報酬を開始するか（0.9 = 90%）
    - ペナルティは制限超過分を0-1の範囲にクリップして合計
    """
    dof_vel_limits = self.reward_cfg.get("dof_vel_limits", 44.0)
    soft_limit_factor = self.reward_cfg.get("soft_dof_vel_limit", 0.9)

    out_of_limits = (torch.abs(self.dof_vel) - dof_vel_limits * soft_limit_factor).clip(min=0.0, max=1.0)

    return torch.sum(out_of_limits, dim=1)
```

**設計意図**:

1. **実機パラメータ遵守**: シミュレーションで実機の物理制約を学習
2. **ソフトリミット**: 90%（39.6 rad/s）を超えたら徐々にペナルティ
3. **スケーラビリティ**: 他のモータ仕様にも容易に対応可能

### 報酬スケール設計（V3からの継承）

V3の報酬設計をそのまま維持し、`dof_vel_limits`のみを追加。

| 報酬カテゴリ | 報酬関数 | スケール | 目的 |
|-------------|---------|---------|------|
| **主報酬** | `tracking_lin_vel` | 1.5 | 線速度追従（V3強化） |
| | `tracking_ang_vel` | 0.5 | 角速度追従 |
| **歩行品質** | `feet_air_time` | 1.0 | 滞空時間報酬 |
| | `contact` | 0.2 | 接地フェーズ整合性 |
| | `single_foot_contact` | 0.3 | 片足接地報酬（V3追加） |
| **安定性** | `lin_vel_z` | -2.0 | Z軸速度ペナルティ |
| | `ang_vel_xy` | -0.05 | XY角速度ペナルティ |
| | `orientation` | -0.5 | 姿勢ペナルティ |
| | `base_height` | -5.0 | 高さ維持 |
| **歩行品質ペナルティ** | `feet_swing_height` | -5.0 | 遊脚高さ |
| | `contact_no_vel` | -0.1 | 接地時足速度 |
| | `hip_pos` | -0.5 | 股関節位置（開脚抑制） |
| | `velocity_deficit` | -0.5 | 速度未達ペナルティ（V3追加） |
| **関節速度制限** | **`dof_vel_limits`** | **-0.3** | **速度制限超過抑制（V18新規）** |
| **エネルギー効率** | `torques` | -1e-5 | トルクペナルティ |
| | `action_rate` | -0.01 | アクション変化率 |
| | `dof_acc` | -2.5e-7 | 関節加速度 |

### ハイパーパラメータ（V3から変更なし）

| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `num_envs` | 4096 | 並列環境数 |
| `max_iterations` | 500 | 最大イテレーション |
| `episode_length_s` | 20.0 | エピソード長（秒） |
| `dt` | 0.02 | 制御周期（50Hz） |
| `action_scale` | 0.4 | アクションスケール |
| `kp` | 35.0 | 位置ゲイン |
| `kd` | 2.0 | 速度ゲイン |
| `base_height_target` | 0.20 | 目標高さ（m） |
| `gait_frequency` | 1.5 | 歩容周波数（Hz） |
| `lin_vel_x_range` | [0.2, 0.3] | 目標前進速度（m/s） |

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_unitree_v18.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-unitree-v18 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 40.11 |
| エピソード長 | 1001（最大） |
| 収束ステップ | ~100 |
| 収束判定 | std=0.0089 < 0.01（安定収束） |

**報酬推移（Quarter-wise）:**
- Steps 0-125: 20.11
- Steps 125-250: 38.68 (+18.57)
- Steps 250-375: 39.90 (+1.22)
- Steps 375-500: 40.07 (+0.17) ← 収束

**報酬成分（最終ステップ）:**

| 報酬項目 | 値 | 評価 |
|---------|-----|------|
| tracking_lin_vel | 0.6054 | ✓ 実速度0.248 m/sと整合 |
| tracking_ang_vel | 0.1965 | ✓ |
| dof_vel_limits | 0.0000 | ✓ **速度制限内で動作** |
| contact | 0.0750 | ✓ |
| feet_air_time | 0.0000 | ❌ 接地検出失敗 |
| single_foot_contact | 0.0000 | ❌ 接地検出失敗 |
| velocity_deficit | -0.0003 | ✓ 速度達成 |

### 評価結果

| 指標 | V3参考値 | V18目標 | V18結果 | 達成 |
|------|----------|---------|---------|------|
| X速度 | 0.151 m/s | 0.2 m/s | **0.248 m/s** | ✓ 124% |
| Y速度 | - | ~0 m/s | **0.004 m/s** | ✓ |
| hip_pitch相関 | -0.17 | < 0 | **-0.059** | ✓ 交互歩行 |
| DOF range sum | - | > 2.0 | **2.809 rad** | ✓ |
| Yawドリフト(10s) | - | < 5° | **+1.37°** | ✓ |
| 目視評価 | - | - | 細かい足の動き | △ |

**詳細評価データ（10秒間）:**

| カテゴリ | 指標 | 値 |
|---------|------|-----|
| 移動 | 総移動距離 | 2.476 m |
| 移動 | X移動距離 | 2.475 m |
| 姿勢 | Roll mean | +2.79° |
| 姿勢 | Pitch mean | +0.83° |
| 姿勢 | Base height mean | 0.221 m |
| 関節 | DOF velocity RMS | 0.891 rad/s |
| 接地 | 両足接地率 | 0.0% |
| 接地 | 片足接地率 | 0.0% |
| 接地 | 両足空中率 | **100.0%** |

**関節可動域詳細（rad）:**

| 関節 | L min | L max | L range | R min | R max | R range |
|------|-------|-------|---------|-------|-------|---------|
| hip_yaw | -0.086 | 0.049 | 0.135 | -0.126 | 0.078 | 0.204 |
| hip_roll | -0.133 | 0.003 | 0.136 | -0.028 | 0.180 | 0.208 |
| hip_pitch | 1.047 | 1.415 | **0.368** | 1.047 | 1.480 | **0.432** |
| knee_pitch | -1.958 | -1.697 | 0.261 | -2.075 | -1.711 | 0.365 |
| ankle_pitch | 0.460 | 0.785 | 0.325 | 0.410 | 0.785 | 0.375 |

**関節角速度（実機制限との比較）:**

| 指標 | 値 | 制限 | マージン |
|------|-----|------|---------|
| DOF velocity RMS | 0.891 rad/s | 44 rad/s | 十分 |
| dof_vel_limits報酬 | 0.0000 | - | **制限内** |

## 考察と改善案

### 成功点

1. **歩行の実現**
   - X速度: 0.248 m/s（目標0.2 m/sの124%達成）
   - 10秒間で2.476 m移動（V17: 0.057m → 43倍改善）
   - エピソード長1001で転倒なし

2. **交互歩行の達成**
   - hip_pitch相関: -0.059（交互歩行傾向）
   - DOF range sum: 2.809 rad（良好な動作範囲）

3. **実機パラメータ適合**
   - dof_vel_limits報酬: 0.0000（**速度制限内で動作**）
   - DOF velocity RMS: 0.891 rad/s（制限44 rad/sに対して十分なマージン）

4. **姿勢安定性**
   - Yawドリフト: +1.37°（非常に小さい）
   - Roll/Pitch偏差: 小さい（それぞれ2.79°, 0.83°）

### 課題

#### 1. 足の動き（ストライドと上げ幅）が小さい

**ユーザー目視評価**: 「細かい足の動きで進む歩容」

**データによる根拠:**
- hip_pitch可動域: L=0.368 rad, R=0.432 rad
- V16の hip_pitch可動域（L=0.524 rad）と比較して小さい
- 「ゆったり大股」には不十分

**原因分析:**
| 優先度 | 原因 | 根拠 |
|--------|------|------|
| **1** | **V3ベースのため「ゆったり大股」改善が未反映** | V15-V17の改善（step_length, symmetry等）が含まれていない |
| **2** | **feet_swing_height報酬の不足** | -5.0で足上げを促進しているが、目標高さが低い可能性 |
| **3** | **dof_vel_limits報酬の影響** | 速度制限が動作を抑制した可能性（ただしRMS=0.891で制限に達していない） |

#### 2. 接地検出の継続失敗

- feet_air_time報酬: 0.0（V3, V16, V17と同じ問題）
- single_foot_contact報酬: 0.0
- 両足空中率: 100%

**原因**: contact_threshold設定が実際の接地高さと乖離。この問題は全バージョンで共通。

### サーベイ知見との照合

**exp007_unitree_rl_gym_survey.md Section 7.5 (Foot Clearance):**
> "Barrier-Based Style Rewards, Height-Based Contact Detection"

→ 足上げ高さの明示的な報酬化が必要。現在の`feet_swing_height=-5.0`は不十分。

**exp007_unitree_rl_gym_survey.md Section 7.3.2:**
> "Sinusoidal Reference Trajectory + Bayesian Optimization"

→ 基準軌道を与えることで、より大きなストライドを誘導できる可能性。

### 次バージョン（V19）への提案

#### 優先度1（必須）: ストライドと足上げ幅の改善

**A. step_length報酬の追加（V15-V17の知見を統合）:**

```python
"step_length": 0.5,  # V16で使用された歩幅報酬を追加
```

**B. feet_swing_height目標の引き上げ:**

```python
# 現在: swing_height_target = 0.04 m
# 提案: swing_height_target = 0.06 m（1.5倍）
"swing_height_target": 0.06,
"feet_swing_height": -8.0,  # ペナルティ強化
```

**C. hip_pitch可動域報酬の追加:**

```python
def _reward_hip_pitch_range(self):
    """hip_pitch可動域の大きさを報酬（大股歩行促進）"""
    left_hip = self.dof_pos[:, self.left_hip_pitch_idx]
    right_hip = self.dof_pos[:, self.right_hip_pitch_idx]

    # デフォルトからの偏差（可動域の大きさ）
    left_range = torch.abs(left_hip - self.default_dof_pos[self.left_hip_pitch_idx])
    right_range = torch.abs(right_hip - self.default_dof_pos[self.right_hip_pitch_idx])

    # 大きな可動域を報酬
    return torch.mean(torch.stack([left_range, right_range]), dim=0)
```

#### 優先度2（推奨）: 接地検出の修正

```python
# contact_threshold: 0.025 → 0.05 m
"contact_threshold": 0.05,
```

#### 優先度3（オプション）: V15-V17系統の知見統合

V18（V3ベース）にV15-V17で得られた知見を統合:

| V15-V17の知見 | V19への適用 |
|--------------|-------------|
| symmetry報酬 | 位相ではなく「動的対称性」として追加 |
| velocity_deficit強化 | -0.5 → -1.0 |
| tracking_sigma縮小 | 0.25 → 0.2（速度追従厳格化） |

### V19設計の推奨パラメータ変更

| パラメータ | V18値 | V19提案値 | 変更理由 |
|-----------|-------|-----------|---------|
| step_length | なし | **0.5** | 歩幅報酬追加 |
| swing_height_target | 0.04 | **0.06** | 足上げ高さ増加 |
| feet_swing_height | -5.0 | **-8.0** | 足上げペナルティ強化 |
| hip_pitch_range | なし | **0.3** | 可動域報酬追加 |
| contact_threshold | 0.025 | **0.05** | 接地検出改善 |

## まとめ

### 実施内容

V18はV3の報酬設計をベースに、RobStride RS-02の実機パラメータ（最大角速度 ±44 rad/s）を考慮した関節角速度ソフトリミット報酬を導入したバージョンである。

**実装完了事項**:
- `_reward_dof_vel_limits`報酬関数を`droid_env_unitree.py`に追加
- V3の報酬設計を維持しつつ、速度制限報酬のみを追加
- 実機パラメータを`reward_cfg`に明示的に設定（`dof_vel_limits=44.0`, `soft_dof_vel_limit=0.9`）

### 結果

**V18は歩行を実現し、実機パラメータ内での動作を達成した。**

| 指標 | 結果 | 評価 |
|------|------|------|
| X速度 | 0.248 m/s | ✓ 目標の124% |
| 10秒間移動距離 | 2.476 m | ✓ V17の43倍 |
| hip_pitch相関 | -0.059 | ✓ 交互歩行 |
| dof_vel_limits報酬 | 0.0000 | ✓ **速度制限内** |
| 目視評価 | 細かい足の動き | △ ストライド不足 |

### 成功要因

1. **V3の報酬設計の継承**: 静止回避機構（single_foot_contact, velocity_deficit）が機能
2. **実機パラメータ適合**: 速度制限内で歩行を学習
3. **安定した収束**: std=0.0089で非常に安定

### 残課題

1. **ストライドと足上げ幅が小さい**: hip_pitch可動域0.368-0.432 rad（「大股」には不十分）
2. **接地検出の継続失敗**: feet_air_time=0, single_foot_contact=0（全バージョン共通）
3. **V15-V17の知見が未統合**: 「ゆったり大股」の改善が含まれていない

### 次ステップ

V19では以下を実装する：
1. step_length報酬の追加（歩幅促進）
2. swing_height_target引き上げ（0.04→0.06m）
3. 接地検出閾値の緩和（contact_threshold: 0.025→0.05m）
4. V15-V17の知見（対称性報酬等）の統合検討

### 実機展開への貢献

- ✅ シミュレーション段階で実機制約（±44 rad/s）を学習
- ✅ 速度飽和による動作不安定を事前に防止
- ✅ 実機での安全性向上（過度な速度指令を回避）

## 備考

### 参考実装

- **ETH Zurich Legged Gym**: `ref/unitree_rl_gym/legged_gym/envs/base/legged_robot.py`
  - `_reward_dof_vel_limits()`の実装を参考
  - ソフトリミット係数（0.9）の選定根拠

- **RobStride Hardware**: `ros2_ws/src/robstride_hardware/`
  - 実機パラメータの出典
  - `robstride_driver.hpp`のスケールファクター定義

### 関連ドキュメント

- [exp007_rules.md](exp007_rules.md): 実験ルール・手順
- [exp007_unitree_rl_gym_survey.md](exp007_unitree_rl_gym_survey.md): Unitree RL Gym報酬設計調査
- [exp007_report_v3.md](exp007_report_v3.md): V3レポート（ベースバージョン）
