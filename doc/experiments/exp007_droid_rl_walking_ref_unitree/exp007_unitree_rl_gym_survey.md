# Unitree RL Gym 報酬設計サーベイ

**作成日**: 2026-02-02
**関連実験**: exp007_droid_rl_walking_ref_unitree
**目的**: Unitree RL Gym（unitree_rl_gym）の報酬設計を調査し、BSL-Droid Simplifiedへの適用指針を明確化する。

> **Note**: 本ドキュメントは調査結果のみを記載する。実験結果の分析・考察は [exp007_droid_rl_walking_ref_unitree.md](exp007_droid_rl_walking_ref_unitree.md) を参照。

---

## 1. Unitree RL Gym 概要

### 1.1 リポジトリ情報

- **リポジトリ**: https://github.com/unitreerobotics/unitree_rl_gym
- **ライセンス**: BSD-3-Clause
- **対応ロボット**: Go2（四脚）、H1/H1_2（二脚ヒューマノイド）、G1（二脚ヒューマノイド）
- **シミュレータ**: Isaac Gym（訓練）、Mujoco（Sim2Sim検証）

### 1.2 アーキテクチャ

```
unitree_rl_gym/
├── legged_gym/                    # メイン学習フレームワーク
│   ├── envs/
│   │   ├── base/
│   │   │   ├── legged_robot.py    # 基底環境クラス（報酬関数定義）
│   │   │   └── legged_robot_config.py  # 基底設定クラス
│   │   ├── g1/
│   │   │   ├── g1_env.py          # G1固有の環境実装
│   │   │   └── g1_config.py       # G1固有の設定
│   │   └── h1/
│   │       ├── h1_env.py          # H1固有の環境実装
│   │       └── h1_config.py       # H1固有の設定
│   └── ...
├── deploy/
│   ├── deploy_mujoco/             # Mujoco検証
│   └── deploy_real/               # 実機デプロイ
└── resources/robots/              # ロボット定義ファイル
```

### 1.3 ワークフロー

```
Train (Isaac Gym) → Play (可視化) → Sim2Sim (Mujoco) → Sim2Real (実機)
```

---

## 2. 基底クラスの報酬関数（legged_robot.py）

### 2.1 速度追従報酬

#### 2.1.1 tracking_lin_vel（線速度追従）

```python
def _reward_tracking_lin_vel(self):
    """
    線速度コマンド追従報酬
    指数関数（ガウシアン）で誤差を評価
    """
    lin_vel_error = torch.sum(
        torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]),
        dim=1
    )
    return torch.exp(-lin_vel_error / self.cfg.rewards.tracking_sigma)
```

- **パラメータ**: `tracking_sigma`（誤差の許容幅）
- **デフォルトスケール**: 1.0
- **特徴**: 指数関数により、誤差が大きいと急激に報酬が減少

#### 2.1.2 tracking_ang_vel（角速度追従）

```python
def _reward_tracking_ang_vel(self):
    """
    角速度コマンド追従報酬（Yaw軸のみ）
    """
    ang_vel_error = torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2])
    return torch.exp(-ang_vel_error / self.cfg.rewards.tracking_sigma)
```

- **デフォルトスケール**: 0.5
- **特徴**: Yaw軸（旋回）のみを評価

### 2.2 安定性ペナルティ

#### 2.2.1 lin_vel_z（Z軸速度ペナルティ）

```python
def _reward_lin_vel_z(self):
    """
    上下動を抑制するペナルティ
    """
    return torch.square(self.base_lin_vel[:, 2])
```

- **デフォルトスケール**: -2.0
- **目的**: 跳ねるような動作を抑制

#### 2.2.2 ang_vel_xy（XY角速度ペナルティ）

```python
def _reward_ang_vel_xy(self):
    """
    Roll/Pitch方向の角速度を抑制
    """
    return torch.sum(torch.square(self.base_ang_vel[:, :2]), dim=1)
```

- **デフォルトスケール**: -0.05
- **目的**: 胴体の揺れを抑制

#### 2.2.3 orientation（姿勢ペナルティ）

```python
def _reward_orientation(self):
    """
    胴体が水平を保つよう促進
    projected_gravityのXY成分を使用
    """
    return torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1)
```

- **デフォルトスケール**: -1.0（G1/H1）, 0.0（Go2デフォルト）
- **目的**: 転倒防止、直立姿勢維持

#### 2.2.4 base_height（高さ維持）

```python
def _reward_base_height(self):
    """
    目標高さからの偏差をペナルティ
    """
    base_height = torch.mean(
        self.root_states[:, 2].unsqueeze(1) - self.measured_heights,
        dim=1
    )
    return torch.square(base_height - self.cfg.rewards.base_height_target)
```

- **デフォルトスケール**: -10.0（G1/H1）
- **パラメータ**:
  - G1: `base_height_target = 0.78`
  - H1: `base_height_target = 1.05`
- **目的**: 膝を曲げすぎたり伸ばしすぎたりを防止

### 2.3 エネルギー効率ペナルティ

#### 2.3.1 torques（トルクペナルティ）

```python
def _reward_torques(self):
    """
    トルク出力の二乗和をペナルティ
    """
    return torch.sum(torch.square(self.torques), dim=1)
```

- **デフォルトスケール**: -0.00001
- **目的**: エネルギー効率改善

#### 2.3.2 dof_vel（関節速度ペナルティ）

```python
def _reward_dof_vel(self):
    """
    関節速度の二乗和をペナルティ
    """
    return torch.sum(torch.square(self.dof_vel), dim=1)
```

- **デフォルトスケール**: 0.0（デフォルト無効）
- **目的**: 急激な動作抑制

#### 2.3.3 dof_acc（関節加速度ペナルティ）

```python
def _reward_dof_acc(self):
    """
    関節加速度の二乗和をペナルティ
    """
    return torch.sum(
        torch.square((self.last_dof_vel - self.dof_vel) / self.dt),
        dim=1
    )
```

- **デフォルトスケール**: -2.5e-7
- **目的**: 滑らかな動作誘導

#### 2.3.4 action_rate（アクション変化率ペナルティ）

```python
def _reward_action_rate(self):
    """
    連続するアクション間の変化をペナルティ
    """
    return torch.sum(torch.square(self.last_actions - self.actions), dim=1)
```

- **デフォルトスケール**: -0.01
- **目的**: 高周波振動抑制

### 2.4 歩行品質報酬

#### 2.4.1 feet_air_time（滞空時間報酬）

```python
def _reward_feet_air_time(self):
    """
    長いステップの歩行を報酬
    接地直後に滞空時間に比例した報酬を付与
    """
    contact = self.contact_forces[:, self.feet_indices, 2] > 1.
    contact_filt = torch.logical_or(contact, self.last_contacts)
    self.last_contacts = contact
    first_contact = (self.feet_air_time > 0.) * contact_filt
    self.feet_air_time += self.dt
    rew_airTime = torch.sum(
        (self.feet_air_time - 0.5) * first_contact,
        dim=1
    )
    rew_airTime *= (
        torch.norm(self.commands[:, :2], dim=1) > 0.1
    )  # ゼロ指令時は報酬なし
    self.feet_air_time *= ~contact_filt
    return rew_airTime
```

- **デフォルトスケール**: 1.0
- **オフセット**: 0.5秒（0.5秒以上の滞空で報酬）
- **目的**: 大きなストライドを促進

#### 2.4.2 stand_still（静止報酬）

```python
def _reward_stand_still(self):
    """
    ゼロ指令時は関節をデフォルト位置に維持
    """
    joint_error = torch.sum(
        torch.abs(self.dof_pos - self.default_dof_pos),
        dim=1
    )
    return joint_error * (
        torch.norm(self.commands[:, :2], dim=1) < 0.1
    )
```

- **デフォルトスケール**: 0.0（デフォルト無効）
- **目的**: 停止指令時の安定姿勢

### 2.5 安全性ペナルティ

#### 2.5.1 collision（衝突ペナルティ）

```python
def _reward_collision(self):
    """
    禁止された接触点との衝突をペナルティ
    """
    return torch.sum(
        1. * (torch.norm(
            self.contact_forces[:, self.penalised_contact_indices, :],
            dim=-1
        ) > 0.1),
        dim=1
    )
```

- **デフォルトスケール**: -1.0
- **閾値**: 0.1 N
- **目的**: 脚以外の接触を防止

#### 2.5.2 termination（終了ペナルティ）

```python
def _reward_termination(self):
    """
    タイムアウト以外の終了にペナルティ
    """
    return self.reset_buf * ~self.time_out_buf
```

- **デフォルトスケール**: 0.0（デフォルト無効）
- **目的**: 転倒等の早期終了を抑制

#### 2.5.3 dof_pos_limits（関節位置限界ペナルティ）

```python
def _reward_dof_pos_limits(self):
    """
    関節が可動域限界に近づくことをペナルティ
    """
    out_of_limits = -(self.dof_pos - self.dof_pos_limits[:, 0]).clip(max=0.)
    out_of_limits += (self.dof_pos - self.dof_pos_limits[:, 1]).clip(min=0.)
    return torch.sum(out_of_limits, dim=1)
```

- **デフォルトスケール**: 0.0（デフォルト無効）
- **目的**: 関節限界への衝突防止

#### 2.5.4 feet_stumble（つまずきペナルティ）

```python
def _reward_stumble(self):
    """
    足が垂直面に衝突することをペナルティ
    （水平成分 > 5 × 垂直成分の場合）
    """
    return torch.any(
        torch.norm(
            self.contact_forces[:, self.feet_indices, :2],
            dim=2
        ) > 5 * torch.abs(
            self.contact_forces[:, self.feet_indices, 2]
        ),
        dim=1
    )
```

- **デフォルトスケール**: 0.0（デフォルト無効）
- **目的**: 足を引きずる動作を防止

---

## 3. 二脚ヒューマノイド固有の報酬（G1/H1）

### 3.1 contact（接地フェーズ報酬）

```python
def _reward_contact(self):
    """
    脚の接地状態と歩行フェーズの整合性を報酬

    - leg_phase < 0.55: スタンス相（接地期待）
    - leg_phase >= 0.55: スイング相（非接地期待）
    """
    res = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
    for i in range(2):
        is_stance = self.leg_phase[:, i] < 0.55
        contact = self.contact_forces[:, self.feet_indices[i], 2] > 1
        res += ~(contact ^ is_stance)  # XORの否定：一致で報酬
    return res
```

- **スケール**: 0.18（G1）
- **閾値**: 接触力 > 1 N
- **スタンス/スイング境界**: phase = 0.55
- **目的**: 自然な歩行フェーズの学習

### 3.2 feet_swing_height（遊脚高さ報酬）

```python
def _reward_feet_swing_height(self):
    """
    スイング中の足の高さを制御
    接地していない足のみを対象
    """
    contact = self.contact_forces[:, self.feet_indices, 2] > 1
    pos_error = torch.square(self.feet_pos[:, :, 2] - 0.08)  # 目標高さ8cm
    return torch.sum(pos_error * ~contact, dim=1)
```

- **スケール**: -20.0（G1/H1）
- **目標高さ**: 0.08 m
- **目的**: つまずき防止、適切な足上げ

### 3.3 contact_no_vel（接地時速度ペナルティ）

```python
def _reward_contact_no_vel(self):
    """
    接地中の足の速度をペナルティ
    """
    contact = self.contact_forces[:, self.feet_indices, 2] > 1
    return torch.sum(
        torch.square(self.feet_vel[:, :, :2]) * contact.unsqueeze(-1),
        dim=(1, 2)
    )
```

- **スケール**: -0.2（H1）
- **目的**: 接地時の足滑りを防止

### 3.4 hip_pos（股関節位置ペナルティ）

```python
def _reward_hip_pos(self):
    """
    股関節（hip_yaw, hip_roll）の過度な変位をペナルティ
    """
    return torch.sum(
        torch.square(self.dof_pos[:, [1, 2, 7, 8]]),  # G1のインデックス
        dim=1
    )
```

- **対象関節**: hip_yaw, hip_roll（左右各2）
- **目的**: 脚が大きく開くことを防止

### 3.5 alive（生存報酬）

```python
def _reward_alive(self):
    """
    生存報酬（固定値）
    """
    return torch.ones(self.num_envs, dtype=torch.float, device=self.device)
```

- **スケール**: 0.15（G1/H1）
- **目的**: 継続的な行動を奨励

---

## 4. デフォルト報酬スケール一覧

### 4.1 基底クラスのデフォルト値

| 報酬項目 | デフォルト値 | 種別 |
|---------|------------|------|
| `tracking_lin_vel` | 1.0 | 報酬 |
| `tracking_ang_vel` | 0.5 | 報酬 |
| `feet_air_time` | 1.0 | 報酬 |
| `lin_vel_z` | -2.0 | ペナルティ |
| `ang_vel_xy` | -0.05 | ペナルティ |
| `orientation` | 0.0 | ペナルティ |
| `torques` | -0.00001 | ペナルティ |
| `dof_vel` | 0.0 | ペナルティ |
| `dof_acc` | -2.5e-7 | ペナルティ |
| `base_height` | 0.0 | ペナルティ |
| `action_rate` | -0.01 | ペナルティ |
| `collision` | -1.0 | ペナルティ |
| `termination` | 0.0 | ペナルティ |
| `stand_still` | 0.0 | ペナルティ |
| `feet_stumble` | 0.0 | ペナルティ |

### 4.2 G1（二脚ヒューマノイド）の設定

| 報酬項目 | G1値 | 基底差分 |
|---------|------|---------|
| `tracking_lin_vel` | 1.0 | - |
| `tracking_ang_vel` | 0.5 | - |
| `orientation` | **-1.0** | 有効化 |
| `base_height` | **-10.0** | 有効化 |
| `feet_swing_height` | **-20.0** | 新規 |
| `contact` | **0.18** | 新規 |
| `alive` | **0.15** | 新規 |

### 4.3 H1（二脚ヒューマノイド）の設定

| 報酬項目 | H1値 | 基底差分 |
|---------|------|---------|
| `tracking_lin_vel` | 1.0 | - |
| `tracking_ang_vel` | 0.5 | - |
| `orientation` | **-1.0** | 有効化 |
| `base_height` | **-10.0** | 有効化 |
| `feet_swing_height` | **-20.0** | 新規 |
| `contact_no_vel` | **-0.2** | 新規 |
| `alive` | **0.15** | 新規 |
| `dof_pos_limits` | **-5.0** | 有効化 |

---

## 5. 重要な設定パラメータ

### 5.1 報酬関連

```python
class rewards:
    only_positive_rewards = True  # 負の総報酬をゼロにクリップ
    tracking_sigma = 0.25         # 速度追従のガウシアン幅
    base_height_target = 0.78     # 目標胴体高さ（G1）
    max_contact_force = 100       # 最大許容接触力
```

### 5.2 歩行フェーズ管理

```python
# leg_phase: [0, 1) の周期的な位相
# < 0.55: スタンス相（接地）
# >= 0.55: スイング相（遊脚）

# 左右の位相差
left_phase = gait_phase
right_phase = (gait_phase + 0.5) % 1.0  # 180°位相差
```

### 5.3 接触検出

```python
contact_threshold = 1.0  # N（接触力の閾値）
feet_indices = [left_foot_idx, right_foot_idx]  # 足先リンクのインデックス
```

---

## 6. BSL-Droid Simplifiedへの適用指針

### 6.1 ロボット仕様の対比

| 項目 | G1/H1 | BSL-Droid Simplified |
|------|-------|---------------------|
| 脚DOF | 12（片脚6） | 10（片脚5） |
| 膝構造 | 通常 | 逆関節 |
| 胴体高さ | 0.78-1.05 m | 0.19 m |
| 質量 | 30-50 kg | 5.8 kg |

### 6.2 推奨する報酬設計

#### 6.2.1 主報酬（正のスケール）

| 報酬項目 | 推奨スケール | 理由 |
|---------|------------|------|
| `tracking_lin_vel` | 1.0 | Unitreeと同等 |
| `tracking_ang_vel` | 0.5 | Unitreeと同等 |
| `contact` | 0.2 | 歩行フェーズ学習 |
| `alive` | 0.1 | 控えめに設定 |

#### 6.2.2 ペナルティ（負のスケール）

| 報酬項目 | 推奨スケール | 理由 |
|---------|------------|------|
| `lin_vel_z` | -2.0 | Unitreeと同等 |
| `ang_vel_xy` | -0.05 | Unitreeと同等 |
| `orientation` | -0.5 | Unitreeより緩和（小型ロボット） |
| `base_height` | -5.0 | Unitreeより緩和 |
| `feet_swing_height` | -5.0 | Unitreeより大幅緩和（-20.0は過剰） |
| `action_rate` | -0.01 | Unitreeと同等 |
| `dof_acc` | -2.5e-7 | Unitreeと同等 |

### 6.3 要調整パラメータ

| パラメータ | Unitree値 | BSL-Droid推奨 | 理由 |
|-----------|----------|--------------|------|
| `base_height_target` | 0.78 m | 0.18-0.20 m | 脚長の違い |
| `feet_swing_height` target | 0.08 m | 0.03-0.04 m | ロボットサイズに比例 |
| `tracking_sigma` | 0.25 | 0.25 | そのまま適用 |
| stance/swing境界 | 0.55 | 0.55 | そのまま適用 |

### 6.4 実装上の注意点

1. **接触力検出**: Genesis物理エンジンでの接触力取得方法を確認する必要がある
2. **足先位置取得**: `feet_pos`の取得方法がIsaac Gymと異なる可能性
3. **歩行フェーズ管理**: `leg_phase`の更新ロジックを実装する必要がある
4. **インデックスマッピング**: BSL-Droidの関節インデックスに合わせて`hip_pos`ペナルティを調整

---

## 7. V1/V2課題に対する追加調査（2026-02-02追記）

V1/V2の実験で観察された課題に対して、改善策の設計指針となる先行研究・事例を調査した。

### 7.1 V1/V2で観察された課題

| 課題 | V1結果 | V2対策 | 残存課題 |
|------|--------|--------|---------|
| **両脚同期** | hip_pitch相関0.954 | contact報酬0.2→0.5、歩行周波数1.5→1.0Hz | 交互歩行の確立 |
| **歩幅過小** | DOF可動範囲2.290rad | 目標速度低下、action_rate緩和 | 大股歩行の誘導 |
| **足上げ不足** | feet_swing_height報酬低 | -5.0→-10.0、目標0.03→0.04m | 適切なfoot clearance |
| **Yawドリフト** | -12.33° | tracking_ang_vel 0.5→0.8 | 直進性の確保 |

### 7.2 交互歩行誘導に関する先行研究

#### 7.2.1 Periodic Reward Composition（周期報酬合成）

**出典**: [LearningHumanoidWalking (GitHub)](https://github.com/rohanpsingh/LearningHumanoidWalking)、IEEE ICRA 2021

歩行サイクルを以下のフェーズに分割し、各フェーズで期待される状態を報酬化する手法：

```
1サイクル = [DS phase] → [SS phase (L stance)] → [DS phase] → [SS phase (R stance)]

DS phase: 両足接地期待
SS phase (L): 左足接地、右足遊脚期待
SS phase (R): 右足接地、左足遊脚期待
```

**BSL-Droidへの適用可能性**:
- V1/V2の`contact`報酬はSS phaseの整合性のみを評価
- DS phaseを明示的に報酬化することで、両脚同期（常に両足接地）を防止できる可能性

#### 7.2.2 Dynamic Stimulus Signals（動的刺激信号）

**出典**: [Adaptive Gait Acquisition through Learning Dynamic Stimulus Instinct (MDPI, 2024)](https://www.mdpi.com/2313-7673/9/6/310)

固定周波数ではなく、動的に調整される刺激周波数を導入する手法。歩行周波数を環境・状態に応じて変化させることで、適応的な歩容を獲得する。

**BSL-Droidへの適用可能性**:
- V2では歩行周波数を1.5→1.0Hzに固定で変更
- 将来的には周波数を観測に含め、ポリシーが適応的に調整する設計も検討可能

#### 7.2.3 Symmetry Equivariant Policy（対称等変ポリシー）

**出典**: [MS-PPO: Morphological-Symmetry-Equivariant Policy (arXiv, 2024)](https://arxiv.org/pdf/2512.00727)、[Coordinated Humanoid Robot Locomotion (arXiv, 2025)](https://arxiv.org/html/2508.01247v1)

ロボットの形態学的対称性をネットワークアーキテクチャに組み込む手法。左右対称な入力に対して左右対称な出力を保証する。

**主な知見**:
- 対称性を「報酬shaping」で誘導する従来手法は**ソフト制約**に過ぎず、完全な対称性は保証されない
- **等変アーキテクチャ**を使用することで、追加のハイパーパラメータなしに自然な対称動作を獲得
- Unitree Go2、Xiaomi CyberDog2で検証済み

**BSL-Droidへの適用可能性**:
- 現在のMLPアーキテクチャを等変GNNに置き換えることで、報酬設計の複雑さを軽減できる可能性
- ただし、アーキテクチャ変更は大幅な実装変更を伴う

### 7.3 歩幅拡大・大股歩行に関する先行研究

#### 7.3.1 Curriculum Learning for Velocity Commands（速度コマンドのカリキュラム学習）

**出典**: [Booster Gym (arXiv, 2025)](https://arxiv.org/html/2506.15132v1)、[Legged Gym (GitHub)](https://github.com/leggedrobotics/legged_gym)

速度コマンドの範囲を段階的に拡大するカリキュラム学習：

```python
# 追跡性能が閾値を超えたら、速度コマンド範囲を拡大
if tracking_performance > threshold:
    lin_vel_x_range = [
        max(lin_vel_x_range[0] - delta, min_limit),
        min(lin_vel_x_range[1] + delta, max_limit)
    ]
```

**BSL-Droidへの適用可能性**:
- V2では固定範囲0.10-0.15m/sを使用
- カリキュラム学習を導入し、低速で安定した歩行を学習後、徐々に速度範囲を拡大する戦略が有効

#### 7.3.2 Sinusoidal Reference Trajectory + Bayesian Optimization

**出典**: [Reinforcement Learning of Bipedal Walking Using a Simple Reference Motion (MDPI, 2024)](https://www.mdpi.com/2076-3417/14/5/1803)

3つの正弦波で構成される簡易参照軌道を使用し、ベイズ最適化でパラメータを自動調整する手法：

```python
# 簡易参照軌道（3つの正弦波）
ref_hip_pitch = A1 * sin(2π * f * t + φ1)
ref_knee = A2 * sin(2π * f * t + φ2)
ref_ankle = A3 * sin(2π * f * t + φ3)

# パラメータ（A1, A2, A3, f, φ1, φ2, φ3）をベイズ最適化で決定
```

**主な知見**:
- 参照軌道を「模倣目標」ではなく「アクションのバイアス」として使用
- 実機で1.9cm/s、ストライド3.8cmの歩行を達成

**BSL-Droidへの適用可能性**:
- 現在は参照軌道なしの純粋なRL
- 正弦波参照軌道を導入することで、周期的な歩行パターンの学習を促進できる可能性
- ベイズ最適化で振幅・位相を自動調整し、BSL-Droidに適した歩幅を発見

#### 7.3.3 feet_air_timeオフセットの調整

**出典**: Unitree RL Gym

現在のfeet_air_time報酬は0.5秒オフセットを使用しているが、これはUnitree G1/H1（大型ロボット）向けの設定。

**調整案**:
- BSL-Droidの歩行周波数1.0Hz（周期1.0秒）では、遊脚期は約0.45秒
- オフセットを0.5→0.3秒に変更することで、より小さい滞空時間でも報酬を獲得可能に

### 7.4 Yawドリフト抑制に関する先行研究

#### 7.4.1 Lateral/Yaw Velocity Correction

**出典**: [Real-world humanoid locomotion with reinforcement learning (Science Robotics, 2024)](https://www.science.org/doi/10.1126/scirobotics.adi9579)

直線歩行を実現するために、Yaw角と横方向ドリフトの両方を修正する必要がある：

```
直線歩行の要件:
1. Yaw角速度コマンド（z軸周り）への応答能力
2. 横方向速度コマンド（Y軸）への応答能力
3. PDコントローラで直線歩行を維持
```

**訓練手法**:
- 単独の横方向/Yaw速度コマンドでの訓練
- 両者を組み合わせたPDコントローラでの直線歩行訓練

**BSL-Droidへの適用可能性**:
- 現在はYaw速度コマンド0（旋回なし）で訓練
- **意図的にYaw速度コマンドを変動させて訓練**することで、Yawドリフト修正能力を獲得

#### 7.4.2 Angular Momentum Balancing（角運動量バランシング）

**出典**: [Achieving Stable High-Speed Locomotion (arXiv, 2024)](https://arxiv.org/html/2409.16611)

高速歩行時のYawドリフトは、遊脚の角運動量不均衡が原因：

```
問題: 遊脚のスイングがYaw方向の角運動量を生成
解決: 腕のスイングで角運動量をカウンターバランス
```

**BSL-Droidへの適用可能性**:
- BSL-Droid Simplifiedには腕がないため、この手法は直接適用不可
- 代替案: **hip_yawの逆位相制御**でYaw角運動量をバランス

#### 7.4.3 Symmetric Gait Reward

**出典**: [Leveraging Symmetry in RL-based Legged Locomotion Control (IROS 2024)](https://equirob2024.github.io/papers/SymRob-2024_paper_7.pdf)

歩容の対称性を報酬化する手法：

```python
def symmetric_gait_reward():
    # 左右の脚の位相差
    phase_diff = abs(left_phase - (right_phase + 0.5) % 1.0)
    # 左右の足間距離
    foot_distance = norm(left_foot_pos - right_foot_pos)
    return symmetry_score(phase_diff, foot_distance)
```

**BSL-Droidへの適用可能性**:
- 現在の`contact`報酬は位相整合性のみを評価
- **左右脚の対称性を明示的に報酬化**することで、Yawドリフトの原因となる非対称動作を抑制

### 7.5 Foot Clearance（足上げ高さ）に関する先行研究

#### 7.5.1 Barrier-Based Style Rewards

**出典**: [A Learning Framework for Diverse Legged Robot Locomotion Using Barrier-Based Style Rewards (arXiv, 2024)](https://arxiv.org/html/2409.15780v1)

対数バリア関数を使用した「スタイル報酬」で、foot clearanceを制御：

```python
def barrier_style_reward(foot_z, target_z, margin):
    """
    緩和対数バリア関数による足上げ高さ報酬
    - target_z: 目標高さ
    - margin: 許容マージン
    """
    error = abs(foot_z - target_z)
    if error < margin:
        return 1.0  # マージン内は最大報酬
    else:
        return -log(error / margin)  # マージン外は対数減衰
```

**特徴**:
- 二乗誤差よりも柔軟な制約
- 厳密な制約ではなく「好み」として足上げ高さを誘導

**BSL-Droidへの適用可能性**:
- 現在のfeet_swing_heightは二乗誤差ペナルティ
- バリア関数に変更することで、目標近傍での過剰なペナルティを軽減

#### 7.5.2 Height-Based Contact Detection

**出典**: [Booster Gym (arXiv, 2025)](https://arxiv.org/html/2506.15132v1)

Isaac Gymの簡略化された衝突推定の制約により、接触力ではなく**足と地面の高さ差**で接地判定：

```python
contact = (foot_z - ground_z) < threshold  # 高さベースの接地判定
```

**BSL-Droidへの適用可能性**:
- V1/V2では既にZ座標ベースの接地判定を使用（contact_threshold = 0.025m）
- この設計は先行研究と一致しており、Genesis環境での適切なアプローチ

### 7.6 改善策の優先順位と設計指針

調査結果に基づき、V3以降で検討すべき改善策を優先順位付けする。

#### 7.6.1 短期的改善（V3で検討）

| 改善策 | 対象課題 | 実装難易度 | 期待効果 |
|--------|---------|-----------|---------|
| **対称性報酬の追加** | Yawドリフト | 低 | 中 |
| **feet_air_timeオフセット調整** | 歩幅過小 | 低 | 低〜中 |
| **DS phase報酬の追加** | 両脚同期 | 中 | 高 |
| **速度コマンドカリキュラム** | 歩幅過小 | 中 | 中〜高 |

#### 7.6.2 中期的改善（V4以降で検討）

| 改善策 | 対象課題 | 実装難易度 | 期待効果 |
|--------|---------|-----------|---------|
| **正弦波参照軌道の導入** | 交互歩行、歩幅 | 中〜高 | 高 |
| **Yaw速度コマンド訓練** | Yawドリフト | 中 | 高 |
| **バリア関数型スタイル報酬** | 足上げ不足 | 中 | 中 |

#### 7.6.3 長期的改善（アーキテクチャ変更）

| 改善策 | 対象課題 | 実装難易度 | 期待効果 |
|--------|---------|-----------|---------|
| **等変ネットワークアーキテクチャ** | 全課題 | 高 | 非常に高 |
| **動的刺激周波数ポリシー** | 適応的歩行 | 高 | 高 |

### 7.7 V3設計案（予備）

調査結果に基づくV3の暫定設計案：

```python
# V3: 対称性報酬とDS phase報酬の追加
reward_scales = {
    # V2から継続
    "tracking_lin_vel": 1.0,
    "tracking_ang_vel": 0.8,
    "contact": 0.5,
    # ...

    # 新規追加（対称性報酬）
    "symmetry": 0.3,          # 左右脚の対称性
    "double_support": 0.2,    # DS phaseでの両足接地

    # 調整
    "feet_air_time": 1.0,     # オフセットを0.5→0.3に変更
}

# 速度コマンドカリキュラム（オプション）
command_curriculum = {
    "initial_range": [0.05, 0.10],
    "final_range": [0.15, 0.25],
    "threshold": 0.8,  # 追跡性能の閾値
}
```

---

## 8. V2「静止ポリシー」問題に対する追加調査（2026-02-02追記）

V2実験で観察された「静止ポリシー」問題（ロボットが動かずに立ち続ける局所最適解に収束）に対して、先行研究から得られた知見を整理する。

### 8.1 「静止ポリシー」問題の定義

[Deep RL for Bipedal Locomotion Survey (arXiv 2024)](https://arxiv.org/html/2404.17070v5) より：

> "Curriculum Learning (CL) has been used successfully in multiple previous works to overcome the problem of local minima, where **an RL agent being trained for locomotion may learn to stand in place and not make any motion to avoid falling**, if presented with extremely challenging tasks in the initial stages of training."

これは二脚歩行RL研究における既知の問題であり、以下の条件で発生しやすい：

1. **alive報酬が相対的に高い**: 動かずに生存するだけで報酬を獲得
2. **速度追従報酬の勾配が緩い**: 低速目標では動く動機が弱い
3. **ペナルティが支配的**: 動くとペナルティが発生し、動かない方が安全

### 8.2 Single Foot Contact Reward（片足接地報酬）

[Revisiting Reward Design and Evaluation for Robust Humanoid Standing and Walking (arXiv 2024)](https://arxiv.org/html/2404.19173v1) より：

> "The most reliable and unconstrained way to produce walking instead of hopping is via the **single foot contact reward**, which does not require tuning. For non-standing commands, the single foot contact component provides a **reward of 1 at each time step where only one foot is in contact with the ground**."

**実装方法:**

```python
def _reward_single_foot_contact(self):
    """片足接地報酬（移動コマンド時のみ）"""
    # 移動コマンドかどうかを判定
    is_moving_command = self.commands[:, 0].abs() > 0.05  # X速度コマンド

    # 接地状態を取得
    left_contact = self.feet_pos[:, 0, 2] < self.contact_threshold
    right_contact = self.feet_pos[:, 1, 2] < self.contact_threshold

    # 片足のみ接地 = XOR
    single_contact = left_contact ^ right_contact

    # 移動コマンド時のみ報酬
    reward = torch.where(is_moving_command, single_contact.float(), torch.ones_like(single_contact.float()))
    return reward
```

**特徴:**
- 移動コマンド時に「片足のみ接地」を報酬化
- 静止コマンド時は常に1.0（両足接地でもペナルティなし）
- 両足同時接地（静止）を抑制しつつ、回復動作は許容

### 8.3 報酬と制約の分離

[Not Only Rewards But Also Constraints: Applications on Legged Robot Locomotion (arXiv 2024)](https://arxiv.org/html/2308.12517v2) より：

> "Outstanding controllers with natural motion style and high task performance are developed through extensive reward engineering, which is a highly laborious process. We propose a **novel reinforcement learning framework** for training neural network controllers consisting of **both rewards and constraints**."

報酬のみで行動を誘導するのではなく、制約（Constraints）を明示的に定義する手法が提案されている：

- **報酬**: 主タスク（速度追従）の達成度
- **制約**: 安全条件（転倒回避、関節限界）

これにより、「動かない」という局所最適を回避しつつ、安全性を保証できる。

### 8.4 Reward-Oriented Gains via Embodied Regulation (ROGER)

[Gain Tuning Is Not What You Need: Reward Gain Adaptation for Constrained Locomotion Learning (arXiv 2024)](https://arxiv.org/html/2510.10759v1) より：

> "ROGER adapts **reward-weighting gains online** based on penalties received throughout the embodied interaction process. The ratio between the positive reward (primary reward) and negative reward (penalty) gains is **automatically reduced** as the learning approaches the constraint thresholds."

報酬スケールを学習中に自動調整する手法。V2の問題（`alive`報酬が支配的になった）を回避するために有効な可能性がある。

### 8.5 カリキュラム学習による局所最適回避

[Learning agility and adaptive legged locomotion via curricular hindsight reinforcement learning (Scientific Reports 2024)](https://www.nature.com/articles/s41598-024-79292-4) より：

> "Curricular Hindsight Reinforcement Learning (CHRL) learns an end-to-end tracking controller that achieves powerful agility and adaptation for legged robots. The two key components are a **novel automatic curriculum strategy on task difficulty** and a Hindsight Experience Replay strategy."

**カリキュラム設計案:**

```python
# Phase 1: 高速歩行の学習（動く動機を強化）
lin_vel_x_range = [0.3, 0.5]  # 高速から開始
alive_scale = 0.0             # alive報酬なし

# Phase 2: 速度範囲の拡大
lin_vel_x_range = [0.1, 0.5]
alive_scale = 0.05            # 少量のalive報酬を追加

# Phase 3: 低速歩行の精度向上
lin_vel_x_range = [0.05, 0.3]
alive_scale = 0.1
```

### 8.6 V2失敗の原因分析

V2では以下の変更が「静止ポリシー」を誘発した：

| 変更内容 | 意図 | 実際の影響 |
|---------|------|-----------|
| alive: 0.1→0.15 | 安定性向上 | **静止の報酬増加** |
| contact: 0.2→0.5 | 交互歩行強化 | 静止でも位相整合可能 |
| lin_vel_x: 0.2-0.3→0.10-0.15 | 大股歩行誘導 | **速度追従の動機低下** |
| gait_frequency: 1.5→1.0 Hz | ゆっくり歩行 | 遊脚期が長く、静止が有利 |

**教訓:**
- `alive`報酬は「動かない」局所最適を誘発する主要因
- 速度目標を下げると、速度追従報酬の勾配が緩くなり、動く動機が低下
- 複数のパラメータを同時に変更すると、相互作用が予測困難

### 8.7 V3設計への提言

先行研究に基づくV3の設計指針：

1. **alive報酬の削除**（または大幅減少: 0.15→0.0）
2. **片足接地報酬の導入**（scale: 0.3-0.5）
3. **目標速度の復元**（0.10-0.15→0.2-0.3 m/s）
4. **速度未達ペナルティの追加**
   ```python
   def _reward_velocity_deficit(self):
       """目標速度未達ペナルティ"""
       vel_deficit = torch.clamp(self.commands[:, 0] - self.base_lin_vel[:, 0], min=0)
       return vel_deficit ** 2
   ```
5. **feet_air_time報酬の検証**（現在機能していない可能性）

---

## 9. 参考文献

### 9.1 Unitree RL Gym関連

1. Unitree RL Gym: https://github.com/unitreerobotics/unitree_rl_gym
2. Legged Gym (ETH): https://github.com/leggedrobotics/legged_gym
3. rsl_rl: https://github.com/leggedrobotics/rsl_rl
4. Isaac Gym: https://developer.nvidia.com/isaac-gym

### 9.2 交互歩行・対称性に関する研究

5. LearningHumanoidWalking: https://github.com/rohanpsingh/LearningHumanoidWalking
6. MS-PPO (arXiv 2024): https://arxiv.org/pdf/2512.00727
7. Coordinated Humanoid Robot Locomotion (arXiv 2025): https://arxiv.org/html/2508.01247v1
8. Leveraging Symmetry in RL (IROS 2024): https://equirob2024.github.io/papers/SymRob-2024_paper_7.pdf
9. Adaptive Gait Acquisition (MDPI 2024): https://www.mdpi.com/2313-7673/9/6/310

### 9.3 歩幅・カリキュラム学習に関する研究

10. Booster Gym (arXiv 2025): https://arxiv.org/html/2506.15132v1
11. RL with Simple Reference Motion (MDPI 2024): https://www.mdpi.com/2076-3417/14/5/1803
12. Bayesian Optimization for Cassie (ICRA 2022): https://hybrid-robotics.berkeley.edu/publications/ICRA2022_Bayesian-Optimization_Cassie-Walking.pdf

### 9.4 Yawドリフト・安定性に関する研究

13. Real-world humanoid locomotion (Science Robotics 2024): https://www.science.org/doi/10.1126/scirobotics.adi9579
14. Stable High-Speed Locomotion (arXiv 2024): https://arxiv.org/html/2409.16611
15. RL for Versatile Bipedal Locomotion (IJRR 2024): https://hybrid-robotics.berkeley.edu/publications/IJRR2024_Cassie_RL_Versatile_Locomotion.pdf

### 9.5 Foot Clearance・スタイル報酬に関する研究

16. Barrier-Based Style Rewards (arXiv 2024): https://arxiv.org/html/2409.15780v1
17. Deep RL for Bipedal Locomotion Survey (arXiv 2024): https://arxiv.org/html/2404.17070v5

### 9.6 静止ポリシー問題・報酬設計に関する研究（V2失敗分析用）

18. Revisiting Reward Design and Evaluation for Robust Humanoid Standing and Walking (arXiv 2024): https://arxiv.org/html/2404.19173v1
19. Not Only Rewards But Also Constraints: Applications on Legged Robot Locomotion (arXiv 2024): https://arxiv.org/html/2308.12517v2
20. Gain Tuning Is Not What You Need: Reward Gain Adaptation (arXiv 2024): https://arxiv.org/html/2510.10759v1
21. Learning agility via curricular hindsight RL (Scientific Reports 2024): https://www.nature.com/articles/s41598-024-79292-4
22. Reward Hacking in Reinforcement Learning (Lil'Log 2024): https://lilianweng.github.io/posts/2024-11-28-reward-hacking/

---

## 関連リンク

- [exp007_droid_rl_walking_ref_unitree.md](exp007_droid_rl_walking_ref_unitree.md) - 実験結果
- [exp007_rules.md](exp007_rules.md) - 実験ルール
- [exp006_reward_design_survey.md](../exp006_droid_rl_walking_taskspace/exp006_reward_design_survey.md) - 足先空間の先行研究サーベイ
