## Phase 4: V4 - 交互歩行・大股歩行への改善

### 4.1 V4設計概要

V3の「両脚同期歩行」問題に対して、接地検出の修正と交互歩行報酬の強化を実装。

#### 4.1.1 接地検出問題の特定

V3評価時に足先Z座標を実測した結果:
```
foot Z座標の実測値: 0.063 ~ 0.071 m（約6-7cm）
contact_threshold:   0.025 m（2.5cm）

→ 足先Z座標が閾値の2.5倍以上高い
→ 接地判定が常にFalse
→ single_foot_contact報酬が機能しない
```

**原因**: `foot_link`のZ座標は足リンクの原点位置（足首付近）を示しており、足裏ではない。

#### 4.1.2 V4パラメータ変更一覧

| パラメータ | V3値 | V4値 | 変更理由 |
|-----------|------|------|---------|
| **contact_threshold** | 0.025 m | **0.08 m** | 接地検出を修正（実測値に基づく） |
| **single_foot_contact** | 0.3 | **0.8** | 交互歩行誘導を大幅強化 |
| **gait_frequency** | 1.5 Hz | **1.0 Hz** | 歩幅拡大のため周波数低下 |
| **action_rate** | -0.01 | **-0.005** | 大きな関節動作を許容 |
| **feet_air_time** | 1.0 | **1.5** | 滞空時間報酬を強化 |
| **alive** | 0.0 | **0.03** | 小量復活（安定性補助） |
| **dof_vel** | - | **-0.001（新規）** | 関節速度ペナルティ（高速振動抑制） |

### 4.2 期待される効果

1. **接地検出の修正**: `contact_threshold=0.08m`で`single_foot_contact`が正常に機能
2. **交互歩行の実現**: `single_foot_contact=0.8`で片足接地を強く誘導
3. **歩幅の拡大**: `gait_frequency=1.0Hz` + `action_rate`緩和で大きな動作
4. **動作のスムーズ化**: `dof_vel`追加で高速振動を抑制

### 4.3 実行方法

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_unitree_v4.py --max_iterations 500
```

### 4.4 本格訓練結果（500イテレーション）

#### 4.4.1 学習曲線

```
Train/mean_reward:
  Step    0:     0.57
  Step   50:    18.67
  Step  100:    33.80
  Step  150:    41.63
  Step  200:    48.51
  Step  250:    53.36
  Step  300:    55.09
  Step  350:    55.55
  Step  400:    56.11
  Step  450:    56.49
  Step  499:    56.85
  → 収束傾向（last 50 steps std: 0.128）

Train/mean_episode_length:
  Step  100:   981 steps
  Step  200: 1,001 steps（最大値に到達）
  → V3同様、エピソード終了なしで20秒間生存
```

#### 4.4.2 評価結果（10秒間ヘッドレス評価）

| 指標 | V4結果 | V3結果 | 変化 | 評価 |
|------|--------|--------|------|------|
| X移動距離 | 2.43 m | 2.45 m | -0.8% | ✅ 維持 |
| 平均X速度 | 0.244 m/s | 0.246 m/s | -0.8% | ✅ 維持 |
| 胴体高さ | 0.262 m | 0.227 m | +15% | ✅ 高め安定 |
| Roll | mean=1.70°, std=3.37° | ±0.93° | - | ⚠️ 揺れ増加 |
| Pitch | mean=1.36°, std=1.03° | ±0.44° | - | ⚠️ 揺れ増加 |
| Yaw drift | +1.77° | -8.47° | 大幅改善 | ✅ 改善 |
| hip_pitch相関 | **+0.792** | +0.886 | -10.6% | ⚠️ やや改善 |
| DOF range sum | 5.42 rad | 2.43 rad | +123% | ✅ 動き増加 |
| Single foot contact | **98.4%** | 0% | - | ✅ 接地検出機能 |

#### 4.4.3 個別報酬項目（最終ステップ）

```
【主報酬】
tracking_lin_vel:      1.2591  ← V3: 0.7865から大幅改善
tracking_ang_vel:      0.4131  ← V3: 0.2555から改善
contact:               0.3027  ← V3: 0.0975から改善
single_foot_contact:   0.6901  ← V3: 0.0000から機能開始 ✅

【問題点】
feet_air_time:        -0.0275  ← ⚠️ 負の報酬（オフセット0.5秒が長すぎる可能性）
hip_pos:              -0.0523  ← V3: -0.0133から悪化（開脚傾向）

【ペナルティ】
dof_vel:              -0.0279  ← 新規追加、足首振動のペナルティ
ang_vel_xy:           -0.0216  ← V3: -0.0105から悪化
action_rate:          -0.0081
```

#### 4.4.4 歩行パターン分析

**接地パターン**:
```
Both feet grounded:     0.6%
Single foot grounded:  98.4%  ← V3: 0%から大幅改善！
Both feet airborne:     1.0%
```

**関節可動域**:
```
hip_pitch: L=0.182 rad (10.4°), R=0.221 rad (12.7°)  ← V3: 7-9°から改善
hip_roll:  L=0.321 rad, R=0.435 rad                  ← 左右非対称
knee:      L=0.625 rad, R=0.357 rad                  ← 左右非対称
ankle:     L=0.841 rad, R=1.235 rad                  ← 左右非対称、高速振動
```

**関節速度std**:
```
ankle_roll: L=2.733, R=3.574 rad/s  ← ⚠️ 非常に高い（振動）
ankle_pitch: L=1.673, R=1.835 rad/s
knee: L=1.459, R=1.097 rad/s
hip_pitch: L=0.571, R=0.795 rad/s
```

#### 4.4.5 V4結果の分析

**成功点**:
1. **接地検出の修正成功**: single foot contact 98.4%（V3: 0%）
2. **single_foot_contact報酬の機能**: 0.6901（V3: 0.0）
3. **Yawドリフトの改善**: +1.77°（V3: -8.47°）
4. **関節可動域の増加**: hip_pitch 10-13°（V3: 7-9°）

**問題点**:
1. **左右非対称な動き**:
   - hip_roll: L=0.321, R=0.435 rad
   - ankle_roll: L=0.841, R=1.235 rad
   - 原因: 報酬設計に対称性制約がない

2. **足首の高速振動**:
   - ankle_roll velocity std: 2.7-3.6 rad/s（非常に高い）
   - dof_vel=-0.001では不十分
   - 「周期的に片足を大きく上げ、それ以外は足首振動で推力を得る」パターン

3. **hip_pitch相関がまだ正**:
   - +0.792（目標: 負）
   - 真の交互歩行には至っていない

4. **feet_air_time報酬が負**:
   - オフセット0.5秒が1.0Hz歩行（遊脚期約0.45秒）に対して長すぎる
   - 滞空時間報酬が機能していない

### 4.5 V5への改善案

#### 4.5.1 問題の原因分析

| 問題 | 原因 | 証拠 |
|------|------|------|
| 左右非対称 | 報酬に対称性制約がない | hip_roll/ankle_roll可動域の左右差 |
| 足首振動 | dof_vel=-0.001が不十分、目標速度が高すぎる | ankle velocity std 2.7-3.6 rad/s |
| 交互歩行未達成 | 速度追従が「足首振動」で達成可能 | hip_pitch相関+0.792 |
| feet_air_time負 | オフセット0.5秒がBSL-Droid向けに長い | 1.0Hz歩行で遊脚期約0.45秒 |

#### 4.5.2 先行研究からの知見（survey参照）

**目標速度について**:
- BSL-Droid（身長約35cm）で0.2-0.3 m/sは身長比0.57-0.86
- NAO（58cm）: 0.1-0.3 m/s（身長比0.17-0.52）
- Darwin-OP（45cm）: 0.1-0.2 m/s（身長比0.22-0.44）
- **BSL-Droidは相対的に高速すぎる可能性**

**feet_air_timeオフセット**:
- survey セクション7.3.3より、Unitreeのオフセット0.5秒はG1/H1（大型ロボット）向け
- BSL-Droidの1.0Hz歩行では0.3秒程度が適切

**対称性報酬**:
- survey セクション7.4.3より、左右脚の対称性を明示的に報酬化することでYawドリフトや非対称動作を抑制可能

#### 4.5.3 V5パラメータ変更案

| パラメータ | V4値 | V5案 | 変更理由 |
|-----------|------|------|---------|
| **lin_vel_x_range** | [0.2, 0.3] | **[0.10, 0.15]** | 目標速度低下で足首振動依存を軽減 |
| **dof_vel** | -0.001 | **-0.005** | 足首振動を強く抑制 |
| **feet_air_time offset** | 0.5秒 | **0.25秒** | BSL-Droidサイズに適合 |
| **symmetry（新規）** | - | **0.3** | 左右対称性を報酬化 |
| **gait_frequency** | 1.0 Hz | **0.8 Hz** | さらに歩幅拡大 |

**注意**: V2で目標速度低下が静止ポリシーを誘発したが、V4/V5では以下の対策が機能:
- single_foot_contact=0.8（片足接地を強く誘導）
- velocity_deficit=-0.5（速度未達ペナルティ）
- alive=0.03（控えめ）

#### 4.5.4 新規報酬関数: symmetry（対称性報酬）

```python
def _reward_symmetry(self):
    """左右脚の対称性報酬

    hip_pitch, hip_roll, knee, ankleの左右差をペナルティ化
    """
    # 左右脚の関節角度差
    left_joints = self.dof_pos[:, :5]   # 左脚5関節
    right_joints = self.dof_pos[:, 5:]  # 右脚5関節

    # hip_pitchは逆位相が理想（交互歩行）なので除外または別処理
    # hip_roll, knee, ankleは同位相が理想
    symmetry_error = torch.sum(torch.square(
        left_joints[:, [1, 3, 4]] - right_joints[:, [1, 3, 4]]  # roll, knee, ankle
    ), dim=1)

    return torch.exp(-symmetry_error / 0.5)  # ガウシアン型報酬
```

---

