## Phase 2: V2 - 重み・目標速度調整による改善

### 2.1 V1課題の原因分析

#### 2.1.1 両脚同期問題（hip_pitch相関0.954）

**原因仮説**:
1. `contact`報酬のスケール（0.2）が低すぎて、交互歩行へのインセンティブが弱い
2. `feet_air_time`報酬が0付近で機能していない（接地検出問題に起因）
3. 歩行周波数1.5Hzが速すぎて、交互動作を学習する前に両脚同期で安定してしまう

**対策案**:
- `contact`報酬を0.2→0.5に増加（交互歩行の重要度を上げる）
- 歩行周波数を1.5Hz→1.0Hzに低下（ゆっくりした歩行で交互動作を学習しやすくする）

#### 2.1.2 歩幅過小（DOF可動範囲2.290rad）

**原因仮説**:
1. 目標速度0.2-0.3m/sに対して小刻みな歩行が最適解になっている
2. `action_rate`ペナルティ（-0.01）が大きな関節動作を抑制
3. 目標速度が速すぎて「速く小刻み」が「ゆっくり大股」より高報酬

**対策案**:
- 目標速度を0.2-0.3m/s→0.10-0.15m/sに低下（大股歩行を誘導）
- `action_rate`を-0.01→-0.005に緩和（大きな関節動作を許容）

#### 2.1.3 足上げ不足

**原因仮説**:
1. `feet_swing_height`ペナルティ（-5.0）が緩すぎて目標高さ0.03mへの誘導が弱い
2. 小刻み歩行では足を大きく上げる必要がない

**対策案**:
- `feet_swing_height`を-5.0→-10.0に増加（目標高さへの誘導を強化）
- `swing_height_target`を0.03m→0.04mに増加（より高い足上げを目標に）

#### 2.1.4 Yawドリフト（-12.33°）

**原因仮説**:
1. 両脚同期歩行により左右の推力バランスが崩れている
2. `tracking_ang_vel`（0.5）がYaw速度0への追従を十分に報酬化していない

**対策案**:
- 両脚同期問題の解決がYawドリフトも改善する可能性
- `tracking_ang_vel`を0.5→0.8に増加（直進性の重要度を上げる）

### 2.2 V2設計案

#### 2.2.1 パラメータ変更一覧

| パラメータ | V1値 | V2値 | 変更理由 |
|-----------|------|------|---------|
| **目標速度** | 0.2-0.3 m/s | 0.10-0.15 m/s | 大股・ゆっくり歩行を誘導 |
| **歩行周波数** | 1.5 Hz | 1.0 Hz | 交互歩行を学習しやすく |
| `contact` | 0.2 | 0.5 | 交互歩行の重要度を上げる |
| `tracking_ang_vel` | 0.5 | 0.8 | 直進性の重要度を上げる |
| `feet_swing_height` | -5.0 | -10.0 | 足上げ目標への誘導強化 |
| `swing_height_target` | 0.03 m | 0.04 m | より高い足上げを目標に |
| `action_rate` | -0.01 | -0.005 | 大きな関節動作を許容 |

#### 2.2.2 期待される効果

1. **大股歩行**: 目標速度低下＋歩行周波数低下により、1歩あたりの移動距離を増加
2. **交互歩行**: `contact`報酬増加により、左右脚の交互接地を強く誘導
3. **足上げ増加**: `feet_swing_height`増加と目標高さ増加により、可愛い足上げを実現
4. **直進性向上**: 交互歩行の実現とtracking_ang_vel増加により、Yawドリフトを軽減

#### 2.2.3 リスクと対策

| リスク | 対策 |
|--------|------|
| 歩行周波数低下で転倒増加 | alive報酬を0.1→0.15に増加して安定性を維持 |
| 足上げ目標高すぎてつまずき | swing_height_targetを段階的に調整（0.03→0.04→0.05） |
| 速度低下で進まなくなる | tracking_lin_vel報酬は1.0を維持して速度追従を確保 |

### 2.3 ファイル構成

```
rl_ws/
├── biped_walking/
│   ├── envs/
│   │   ├── droid_env_unitree_v1.py     # V1環境
│   │   └── droid_env_unitree_v2.py     # V2環境（新規作成）
│   └── train/
│       ├── droid_train_unitree_v1.py   # V1訓練スクリプト
│       └── droid_train_unitree_v2.py   # V2訓練スクリプト（新規作成）
└── logs/
    └── droid-walking-unitree-v2/       # V2訓練ログ出力先
```

### 2.4 実行方法

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_unitree_v2.py --max_iterations 500
```

### 2.5 実装詳細

#### 2.5.1 環境ファイル（droid_env_unitree_v2.py）

V1からの変更点は最小限に抑え、クラス名とdocstringのみを変更した。報酬パラメータは訓練スクリプト側で設定されるため、環境ファイル自体の報酬関数ロジックはV1と同一である。

```python
class DroidEnvUnitreeV2:
    """BSL-Droid Simplified二脚ロボットのGenesis強化学習環境（Unitree参考版V2）"""
```

#### 2.5.2 訓練スクリプト（droid_train_unitree_v2.py）

V1からの変更箇所を以下に詳述する。

**reward_cfg の変更:**

```python
reward_cfg = {
    "tracking_sigma": 0.25,         # 速度追従のガウシアン幅（変更なし）
    "base_height_target": 0.20,     # 目標胴体高さ（変更なし）
    "swing_height_target": 0.04,    # V1: 0.03m → V2: 0.04m
    "gait_frequency": 1.0,          # V1: 1.5Hz → V2: 1.0Hz
    "contact_threshold": 0.025,     # 接地判定閾値（変更なし）

    "reward_scales": {
        # 主報酬
        "tracking_lin_vel": 1.0,    # 変更なし
        "tracking_ang_vel": 0.8,    # V1: 0.5 → V2: 0.8

        # 歩行品質報酬
        "feet_air_time": 1.0,       # 変更なし
        "contact": 0.5,             # V1: 0.2 → V2: 0.5
        "alive": 0.15,              # V1: 0.1 → V2: 0.15

        # 安定性ペナルティ（変更なし）
        "lin_vel_z": -2.0,
        "ang_vel_xy": -0.05,
        "orientation": -0.5,
        "base_height": -5.0,

        # 歩行品質ペナルティ
        "feet_swing_height": -10.0, # V1: -5.0 → V2: -10.0
        "contact_no_vel": -0.1,     # 変更なし
        "hip_pos": -0.5,            # 変更なし

        # エネルギー効率ペナルティ
        "torques": -1e-5,           # 変更なし
        "action_rate": -0.005,      # V1: -0.01 → V2: -0.005
        "dof_acc": -2.5e-7,         # 変更なし
    },
}
```

**command_cfg の変更:**

```python
command_cfg = {
    "num_commands": 3,
    "lin_vel_x_range": [0.10, 0.15],  # V1: [0.2, 0.3] → V2: [0.10, 0.15]
    "lin_vel_y_range": [0, 0],
    "ang_vel_range": [0, 0],
}
```

#### 2.5.3 動作検証結果（3イテレーション）

実装後、3イテレーションの動作検証を実施した。

```
Learning iteration 0/3:
  Mean total reward: 0.29
  Mean episode length: 21.64 steps
  報酬内訳:
    tracking_lin_vel: 0.0078
    tracking_ang_vel: 0.0037
    contact: 0.0045
    alive: 0.0015
    lin_vel_z: -0.0080
    feet_swing_height: -0.0003

Learning iteration 1/3:
  Mean total reward: 0.33
  Mean episode length: 26.00 steps
  報酬内訳:
    tracking_lin_vel: 0.0194
    tracking_ang_vel: 0.0085
    contact: 0.0118
    alive: 0.0041
    lin_vel_z: -0.0129
    feet_swing_height: -0.0012

Learning iteration 2/3:
  Mean total reward: 0.49
  Mean episode length: 31.87 steps
  報酬内訳:
    tracking_lin_vel: 0.0201
    tracking_ang_vel: 0.0098
    contact: 0.0138
    alive: 0.0048
    lin_vel_z: -0.0120
    feet_swing_height: -0.0019
```

**観察:**
- 観測空間50次元、行動空間10次元が正しく設定されている
- 報酬が0.29→0.49に上昇（約70%改善）
- エピソード長が21→31ステップに改善（約47%向上）
- contact報酬が増加傾向（0.0045→0.0138）、接地フェーズ整合性の学習が進行
- feet_swing_heightペナルティも増加傾向、足上げが活発化している兆候
- エラーなく実行完了

### 2.6 本格訓練結果（500イテレーション）

#### 学習曲線分析

```
Train/mean_reward:
  Step    0:     0.38
  Step   50:    22.30
  Step  100:    37.61
  Step  150:    43.61
  Step  200:    45.36
  Step  250:    45.72
  Step  300:    45.86
  Step  350:    45.92
  Step  400:    45.91
  Step  450:    45.90
  Step  499:    45.85

Train/mean_episode_length:
  Step   50:   739 steps
  Step  100:  1001 steps (最大エピソード長に到達)
```

**V1との比較:**

| 指標 | V1 (Final) | V2 (Final) | 変化 |
|------|-----------|-----------|------|
| Mean reward | 32.57 | 45.85 | **+40.8%** |
| Episode length | 1001 | 1001 | 同等 |
| Value loss (final) | 0.000014 | 0.000036 | 増加 |

報酬値はV1より40%以上高いが、これは報酬スケールの変更（alive: 0.1→0.15, contact: 0.2→0.5等）による影響も含まれる。

#### 個別報酬項目の比較（Final Step）

| 報酬項目 | V1 | V2 | 変化 | 解釈 |
|---------|-----|-----|------|------|
| tracking_lin_vel | 0.597 | **0.706** | +18% | 速度追従が改善（ただし目標速度が低下） |
| tracking_ang_vel | 0.301 | **0.597** | +98% | 直進性が大幅改善 |
| contact | 0.113 | **0.345** | +206% | 接地フェーズ整合性が大幅向上 |
| alive | 0.063 | **0.113** | +79% | 安定性向上 |
| feet_air_time | 0.000 | 0.000 | - | **両者とも滞空時間報酬を獲得できず** |
| orientation | -0.001 | **-0.012** | 悪化 | Pitch傾斜が増加 |
| action_rate | -0.011 | -0.001 | 改善 | 動作が滑らかに（＝動かなくなった） |

### 2.7 評価結果

#### 定量評価（10秒間、ヘッドレスモード）

```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 0.030 m
  Y: -0.010 m
  Total: 0.031 m

Average velocity:
  X: 0.004 m/s (target: 0.100)
  Y: -0.001 m/s

Base height:
  Mean: 0.190 m
  Std: 0.0157 m

Orientation (deg):
  Roll:  mean=  1.37, std= 0.25
  Pitch: mean= 10.58, std= 1.77
  Yaw:   start=  0.00, end= -1.55, drift= -1.55

DOF velocity RMS: 0.216 rad/s
Action RMS: 0.856

Left-Right hip_pitch correlation: -0.162
  (-1.0 = perfect alternating, +1.0 = perfect synchronized)

Contact Pattern:
  Both feet grounded:       0 steps (  0.0%)
  Single foot grounded:     0 steps (  0.0%)
  Both feet airborne:     500 steps (100.0%)

Total DOF range sum: 2.734 rad
```

#### V1との比較

| 指標 | V1 | V2 | 判定 |
|------|-----|-----|------|
| X移動距離 | 0.487 m | **0.030 m** | **大幅悪化** |
| 平均X速度 | 0.046 m/s | **0.004 m/s** | **大幅悪化** |
| 目標達成率 | 23% | **4%** | **大幅悪化** |
| hip_pitch相関 | 0.954 | **-0.162** | 改善（同期→分離） |
| Yawドリフト | -12.33° | **-1.55°** | **大幅改善** |
| DOF range sum | 2.290 rad | 2.734 rad | やや改善 |
| Pitch傾斜 | 1.9° | **10.6°** | 悪化 |

### 2.8 V2結果の考察

#### 2.8.1 観察された問題：「静止ポリシー」の学習

V2は**「静止して倒れない」**という局所最適解に収束した。

**根本原因の分析:**

1. **報酬バランスの崩壊**
   - V2で`alive`報酬（0.1→0.15）、`contact`報酬（0.2→0.5）を増加
   - これらは**静止状態でも獲得可能な報酬**
   - 結果：動かずに倒れないだけで高報酬を獲得

2. **速度追従報酬の相対的低下**
   - 目標速度を0.2-0.3→0.10-0.15 m/sに低下
   - 低速では速度追従報酬の勾配が緩やか
   - ポリシーが速度追従を優先しなくなった

3. **feet_air_time報酬の機能不全**
   - V1/V2ともに`feet_air_time`報酬が0.000
   - 滞空時間報酬が機能していない可能性
   - 交互歩行への誘導が不十分

4. **歩行周波数低下の影響**
   - 1.5→1.0Hzへの低下
   - 遊脚期待時間が長くなり、静止の方が「位相整合」しやすい

#### 2.8.2 先行研究からの知見

[Revisiting Reward Design and Evaluation for Robust Humanoid Standing and Walking (arXiv 2024)](https://arxiv.org/html/2404.19173v1) によると：

> "Curriculum Learning (CL) has been used successfully in multiple previous works to overcome the problem of local minima, where **an RL agent being trained for locomotion may learn to stand in place and not make any motion to avoid falling**, if presented with extremely challenging tasks in the initial stages of training."

この「静止ポリシー」問題は二脚歩行RL研究における既知の課題である。

**推奨される対策:**

1. **Single Foot Contact Reward（片足接地報酬）**
   - 移動コマンド時に「片足のみ接地」状態を報酬化
   - 静止（両足接地）を明示的に抑制

2. **速度追従と静止の報酬分離**
   - 移動コマンド時と静止コマンド時で報酬関数を切り替え
   - 移動コマンド時は`alive`報酬を減らすか削除

3. **負の速度ペナルティ**
   - 目標速度を達成できない場合にペナルティを付与
   - 静止が「安全」でなくなる

4. **カリキュラム学習**
   - 最初は高い目標速度で「動く」ことを学習
   - 徐々に速度範囲を調整

#### 2.8.3 V3への設計指針

V2の失敗から、以下の設計変更が必要と考えられる：

1. **alive報酬の削除または大幅減少**（0.15→0.0または0.05）
2. **片足接地報酬の導入**（移動コマンド時のみ）
3. **目標速度の復元**（0.10-0.15→0.2-0.3 m/s）
4. **速度未達ペナルティの追加**
5. **feet_air_time報酬の検証と修正**

### 2.9 V3設計案（予備）

V2の失敗分析と先行研究サーベイに基づき、V3の設計案を以下に提案する。

#### 2.9.1 報酬スケールの変更案

```python
reward_cfg = {
    # 基本パラメータ（V1ベースに戻す）
    "tracking_sigma": 0.25,
    "base_height_target": 0.20,
    "swing_height_target": 0.03,     # V2: 0.04 → V3: 0.03（V1に戻す）
    "gait_frequency": 1.5,           # V2: 1.0 → V3: 1.5（V1に戻す）
    "contact_threshold": 0.025,

    "reward_scales": {
        # 【主報酬】速度追従を最優先
        "tracking_lin_vel": 1.5,     # V2: 1.0 → V3: 1.5（増加）
        "tracking_ang_vel": 0.5,     # V2: 0.8 → V3: 0.5（V1に戻す）

        # 【歩行品質報酬】
        "feet_air_time": 1.0,
        "contact": 0.2,              # V2: 0.5 → V3: 0.2（V1に戻す）
        "alive": 0.0,                # V2: 0.15 → V3: 0.0（**削除**）
        "single_foot_contact": 0.3,  # **新規追加**

        # 【安定性ペナルティ】
        "lin_vel_z": -2.0,
        "ang_vel_xy": -0.05,
        "orientation": -0.5,
        "base_height": -5.0,

        # 【歩行品質ペナルティ】
        "feet_swing_height": -5.0,   # V2: -10.0 → V3: -5.0（V1に戻す）
        "contact_no_vel": -0.1,
        "hip_pos": -0.5,
        "velocity_deficit": -0.5,    # **新規追加**: 目標速度未達ペナルティ

        # 【エネルギー効率ペナルティ】
        "torques": -1e-5,
        "action_rate": -0.01,        # V2: -0.005 → V3: -0.01（V1に戻す）
        "dof_acc": -2.5e-7,
    },
}

command_cfg = {
    "num_commands": 3,
    "lin_vel_x_range": [0.2, 0.3],   # V2: [0.10, 0.15] → V3: [0.2, 0.3]（V1に戻す）
    "lin_vel_y_range": [0, 0],
    "ang_vel_range": [0, 0],
}
```

#### 2.9.2 新規報酬関数の定義

**片足接地報酬（single_foot_contact）:**

```python
def _reward_single_foot_contact(self):
    """片足接地報酬（移動コマンド時のみ）

    移動コマンドが与えられている場合、片足のみ接地している状態を報酬化。
    これにより「両足で静止」という局所最適を回避する。
    """
    # 移動コマンドかどうかを判定
    is_moving = self.commands[:, 0].abs() > 0.05

    # 接地状態
    left_contact = self.feet_pos[:, 0, 2] < self.contact_threshold
    right_contact = self.feet_pos[:, 1, 2] < self.contact_threshold

    # 片足のみ接地 (XOR)
    single_contact = left_contact ^ right_contact

    # 移動コマンド時のみ報酬、静止コマンド時は1.0
    reward = torch.where(is_moving, single_contact.float(), torch.ones_like(single_contact.float()))
    return reward
```

**速度未達ペナルティ（velocity_deficit）:**

```python
def _reward_velocity_deficit(self):
    """目標速度未達ペナルティ

    目標速度を下回っている場合にペナルティを付与。
    「動かない」局所最適を回避する。
    """
    # 目標X速度 - 実際のX速度（正の値 = 目標未達）
    deficit = torch.clamp(self.commands[:, 0] - self.base_lin_vel[:, 0], min=0)
    return deficit ** 2
```

#### 2.9.3 V3の期待される効果

| 変更 | 期待効果 |
|------|---------|
| alive報酬の削除 | 「静止」の報酬価値を低下、動く動機を強化 |
| single_foot_contact報酬の追加 | 交互歩行を明示的に誘導 |
| velocity_deficit報酬の追加 | 速度目標未達時にペナルティ、動く動機を強化 |
| 目標速度をV1に復元 | 速度追従報酬の勾配を回復 |
| 歩行周波数をV1に復元 | 遊脚期を短縮、静止より歩行が有利に |

#### 2.9.4 代替案：カリキュラム学習の導入

より確実に「動く」ことを学習させるため、カリキュラム学習を検討：

```python
# Phase 1 (0-200 iter): 高速歩行を優先
command_cfg["lin_vel_x_range"] = [0.3, 0.5]
reward_cfg["reward_scales"]["alive"] = 0.0
reward_cfg["reward_scales"]["tracking_lin_vel"] = 2.0

# Phase 2 (200-400 iter): 速度範囲を拡大
command_cfg["lin_vel_x_range"] = [0.15, 0.4]
reward_cfg["reward_scales"]["alive"] = 0.05
reward_cfg["reward_scales"]["tracking_lin_vel"] = 1.5

# Phase 3 (400-500 iter): 最終調整
command_cfg["lin_vel_x_range"] = [0.1, 0.3]
reward_cfg["reward_scales"]["alive"] = 0.1
reward_cfg["reward_scales"]["tracking_lin_vel"] = 1.0
```

---

