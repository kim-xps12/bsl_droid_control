## Phase 3: V3 - 静止ポリシー問題への対策

### 3.1 V3設計概要

V2の「静止ポリシー」問題に対して、先行研究サーベイ（[exp007_unitree_rl_gym_survey.md](exp007_unitree_rl_gym_survey.md) セクション7-8）に基づく対策を実装した。

#### 3.1.1 V2失敗の原因分析（再掲）

| 変更内容 | V2での意図 | 実際の影響 |
|---------|----------|-----------|
| alive: 0.1→0.15 | 安定性向上 | **静止の報酬増加** |
| contact: 0.2→0.5 | 交互歩行強化 | 静止でも位相整合可能 |
| lin_vel_x: 0.2-0.3→0.10-0.15 | 大股歩行誘導 | **速度追従の動機低下** |
| gait_frequency: 1.5→1.0 Hz | ゆっくり歩行 | 遊脚期が長く、静止が有利 |

#### 3.1.2 V3パラメータ変更一覧

| パラメータ | V2値 | V3値 | 変更理由 |
|-----------|------|------|---------|
| **alive** | 0.15 | **0.0（削除）** | 静止の報酬価値を除去 |
| **single_foot_contact** | - | **0.3（新規）** | 片足接地報酬の導入 |
| **velocity_deficit** | - | **-0.5（新規）** | 速度未達ペナルティの導入 |
| **tracking_lin_vel** | 1.0 | **1.5** | 動く動機を強化 |
| **目標速度** | 0.10-0.15 m/s | **0.2-0.3 m/s** | V1に復元、速度追従の勾配を回復 |
| **gait_frequency** | 1.0 Hz | **1.5 Hz** | V1に復元、遊脚期を短縮 |
| contact | 0.5 | 0.2 | V1に復元 |
| feet_swing_height | -10.0 | -5.0 | V1に復元 |
| swing_height_target | 0.04 m | 0.03 m | V1に復元 |
| tracking_ang_vel | 0.8 | 0.5 | V1に復元 |
| action_rate | -0.005 | -0.01 | V1に復元 |

### 3.2 新規報酬関数

#### 3.2.1 片足接地報酬（single_foot_contact）

**参考文献**: [Revisiting Reward Design and Evaluation for Robust Humanoid Standing and Walking (arXiv 2024)](https://arxiv.org/html/2404.19173v1)

```python
def _reward_single_foot_contact(self):
    """片足接地報酬（移動コマンド時のみ）

    移動コマンドが与えられている場合、片足のみ接地している状態を報酬化。
    これにより「両足で静止」という局所最適を回避する。
    """
    # 移動コマンドかどうかを判定（X速度コマンド > 0.05 m/s）
    is_moving_command = self.commands[:, 0].abs() > 0.05

    # 接地状態を取得（Z座標ベース）
    left_contact = self.feet_pos[:, 0, 2] < self.contact_threshold
    right_contact = self.feet_pos[:, 1, 2] < self.contact_threshold

    # 片足のみ接地 = XOR
    single_contact = left_contact ^ right_contact

    # 移動コマンド時のみ報酬、静止コマンド時は1.0
    reward = torch.where(is_moving_command, single_contact.float(), 1.0)
    return reward
```

#### 3.2.2 速度未達ペナルティ（velocity_deficit）

```python
def _reward_velocity_deficit(self):
    """目標速度未達ペナルティ

    目標速度を下回っている場合にペナルティを付与。
    静止ポリシーでは deficit = (target - 0)² > 0 となりペナルティ。
    """
    deficit = torch.clamp(self.commands[:, 0] - self.base_lin_vel[:, 0], min=0)
    return deficit ** 2
```

### 3.3 ファイル構成

```
rl_ws/
├── biped_walking/
│   ├── envs/
│   │   └── droid_env_unitree_v3.py     # V3環境（静止ポリシー対策版）
│   └── train/
│       └── droid_train_unitree_v3.py   # V3訓練スクリプト
└── logs/
    └── droid-walking-unitree-v3/       # V3訓練ログ出力先
```

### 3.4 実行方法

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_unitree_v3.py --max_iterations 500
```

### 3.5 動作検証結果（3イテレーション）

環境構築後、3イテレーションの動作検証を実施：

```
Learning iteration 0/3:
  Mean total reward: 0.38
  Mean episode length: 24.00 steps
  報酬内訳:
    tracking_lin_vel: 0.0142
    tracking_ang_vel: 0.0031
    single_foot_contact: 0.0035
    velocity_deficit: -0.0004

Learning iteration 1/3:
  Mean total reward: 0.02
  Mean episode length: 27.92 steps
  報酬内訳:
    tracking_lin_vel: 0.0278
    tracking_ang_vel: 0.0058
    single_foot_contact: 0.0050
    velocity_deficit: -0.0017

Learning iteration 2/3:
  Mean total reward: 0.12
  Mean episode length: 33.35 steps
  報酬内訳:
    tracking_lin_vel: 0.0243
    tracking_ang_vel: 0.0064
    single_foot_contact: 0.0004
    velocity_deficit: -0.0033
```

**観察**:
- 観測空間50次元、行動空間10次元、報酬項目数16が正しく設定されている
- 新規報酬項目（single_foot_contact, velocity_deficit）が正常に計算されている
- エピソード長が24→33ステップに改善（約38%向上）
- velocity_deficitが増加傾向 → 速度未達が検出されている
- エラーなく実行完了

### 3.6 本格訓練結果（500イテレーション）

#### 3.6.1 学習曲線

```
Train/mean_reward:
  Step    0:     0.36
  Step   50:    19.57
  Step  100:    32.41
  Step  150:    37.54
  Step  200:    39.25
  Step  499:    40.08
  → 収束（last 50 steps std: 0.013）

Train/mean_episode_length:
  Step   50:   835 steps
  Step  100: 1,001 steps（最大値に到達）
  → エピソード終了なしで20秒間生存
```

#### 3.6.2 評価結果（10秒間ヘッドレス評価）

| 指標 | V3結果 | V2結果 | V1目標 | 評価 |
|------|--------|--------|--------|------|
| X移動距離 | **2.45 m** | 0.04 m | - | ✅ 大幅改善 |
| 平均X速度 | **0.246 m/s** | 0.004 m/s | 0.2 m/s | ✅ 目標達成 |
| 胴体高さ | 0.227 m | - | 0.20 m | ✅ 安定 |
| Roll | ±0.93° | - | - | ✅ 安定 |
| Pitch | ±0.44° | - | - | ✅ 安定 |
| Yaw drift | -8.47° | - | 0° | ⚠️ やや大きい |
| hip_pitch相関 | **+0.886** | - | -1.0 | ❌ 同期歩行 |
| DOF range sum | 2.43 rad | 0.09 rad | >2.0 | ✅ 動きあり |

#### 3.6.3 個別報酬項目（最終ステップ）

```
【主報酬】
tracking_lin_vel:      0.7865  ← 高い速度追従
tracking_ang_vel:      0.2555  ← 良好
contact:               0.0975  ← フェーズ整合あり

【問題点】
single_foot_contact:   0.0000  ← ❌ 機能していない
feet_air_time:         0.0000  ← ❌ 滞空時間なし

【ペナルティ】
hip_pos:              -0.0133  ← 最大ペナルティ
ang_vel_xy:           -0.0105
action_rate:          -0.0092
velocity_deficit:     -0.0004  ← 小さい（速度達成）
```

#### 3.6.4 歩行パターン分析

**接地パターン**:
```
Both feet grounded:     0.0%
Single foot grounded:   0.0%
Both feet airborne:   100.0%  ← 接地検出の問題
```

**関節可動域**:
```
hip_pitch: L=0.135 rad (7.7°), R=0.159 rad (9.1°)  ← 小さい
knee:      L=0.305 rad, R=0.311 rad               ← 適度
```

#### 3.6.5 V3結果の分析

**成功点**:
1. **静止ポリシーの回避に成功**: V2の0.004 m/s → V3の0.246 m/s
2. **安定した歩行**: エピソード長1001ステップ（最大値）
3. **速度追従**: 目標0.2 m/sに対して0.246 m/s（123%達成）

**問題点**:
1. **交互歩行の失敗**: hip_pitch相関 +0.886（両脚同期）
   - 理想は -1.0（完全交互）
   - 原因: `single_foot_contact` 報酬が 0.0（機能していない）

2. **小刻みな歩行**: hip_pitch可動域が約8°と小さい
   - 原因: 高周波で同期的に動いている

3. **接地検出の問題**: 100%が「両足空中」と判定
   - `contact_threshold = 0.025 m` が不適切か、またはZ座標参照の問題

#### 3.6.6 V4への改善案

| 項目 | V3値 | V4案 | 理由 |
|------|------|------|------|
| **contact_threshold** | 0.025 m | **0.03-0.04 m** | 接地検出を緩和 |
| **single_foot_contact** | 0.3 | **0.5-1.0** | 交互歩行誘導を強化 |
| **gait_frequency** | 1.5 Hz | **1.0-1.25 Hz** | 歩幅拡大のため周波数低下 |
| **action_rate** | -0.01 | **-0.005** | 大きな関節動作を許容 |
| **feet_air_time** | 1.0 | **1.5** | 滞空時間報酬を強化 |
| **alive** | 0.0 | **0.03** | 小量復活（安定性補助） |

---

