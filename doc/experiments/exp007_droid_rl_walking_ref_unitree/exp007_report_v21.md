# V21: Z座標閾値ベース接地検出の検証

## 概要

V20レポートの「V21への提案」セクションにある**実験A**を実施する。V20で導入したGenesis Contact Sensorを使用せず、適切なZ座標閾値設定で接地検出が正しく機能するかを検証する。

この実験は、Contact Sensorが使用できない環境でのフォールバック実装の有効性を確認し、他のプロジェクトへの知見共有に役立てることを目的とする。

## 前バージョンからの改善事項

### V20の成果と提案

V20では、Genesis Contact Sensorの導入により接地検出が正常に機能するようになった：

| 指標 | V19値 | V20結果 |
|------|-------|---------|
| 片足接地率 | 0% | 91.4% |
| single_foot_contact報酬 | 0.0 | 0.206 |
| contact報酬 | 0.060 | 0.240 |
| X速度 | 0.185 m/s | 0.204 m/s |

V20レポートでは、Z座標閾値ベースの接地検出が失敗していた原因を詳細に分析し、修正案として「実験A: Z座標閾値の修正」を提案した。

### Z座標閾値ベース接地検出の問題分析

**旧アルゴリズム**:
```python
link_pos = self.robot.get_links_pos()
feet_z = link_pos[:, self.feet_indices, 2]
return feet_z < self.contact_threshold  # 0.05m
```

**問題の根本原因**:
1. `get_links_pos()` が返す位置は**リンク原点（ankle joint位置）**であり、足裏ではない
2. URDFの衝突形状を分析すると、足裏底面はリンク原点から約0.08m下に位置
3. foot_linkのZ座標（完全接地時でも約0.08m）は閾値（0.05m）を下回ることが不可能
4. 結果として、常に「空中」と誤判定されていた

### V21での変更

| パラメータ | V20値 | V21値 | 変更理由 |
|-----------|-------|-------|---------|
| `use_contact_sensor` | True（暗黙） | **False** | Z座標ベース検出の検証 |
| `contact_threshold` | 0.05m | **0.10m** | 足裏オフセット（約0.08m）を考慮 |
| 報酬設計 | - | V20と同一 | 接地検出方式変更の効果を純粋に検証 |

## 設計詳細

### 1. 接地判定閾値の修正

**修正の根拠**:

| 状態 | foot_link Z座標 | 足裏底面 Z座標 | 接地判定 |
|------|-----------------|---------------|---------|
| 初期姿勢 | 約0.095m | 約0.015m | 非接地 |
| 完全接地時 | 約0.08m | 約0m | **接地** |

- 完全接地時のfoot_link Z座標は約0.08m
- 閾値を0.10mに設定することで、`0.08m < 0.10m` → True（接地）と正しく判定

### 2. 環境クラスの変更

`droid_env_unitree.py` に `use_contact_sensor` オプションを追加：

```python
# V21追加: Contact Sensorの使用を制御するオプション
if env_cfg.get("use_contact_sensor", True):
    for foot_name in self._feet_names_for_sensor:
        # Contact Sensorを初期化
        ...
```

これにより、トレーニングスクリプト側で接地検出方式を選択可能になった。

### 3. 報酬設計（V20と同一、17項目）

```python
reward_scales = {
    # === 主報酬 ===
    "tracking_lin_vel": 1.5,
    "tracking_ang_vel": 0.5,

    # === 歩行品質報酬 ===
    "feet_air_time": 1.0,
    "contact": 0.2,
    "single_foot_contact": 0.3,
    "step_length": 0.5,

    # === 安定性ペナルティ ===
    "lin_vel_z": -2.0,
    "ang_vel_xy": -0.05,
    "orientation": -0.5,
    "base_height": -5.0,

    # === 歩行品質ペナルティ ===
    "feet_swing_height": -8.0,
    "contact_no_vel": -0.1,
    "hip_pos": -0.5,
    "velocity_deficit": -0.5,

    # === 関節速度制限 ===
    "dof_vel_limits": -0.3,

    # === エネルギー効率ペナルティ ===
    "torques": -1e-5,
    "action_rate": -0.01,
    "dof_acc": -2.5e-7,
}
```

### 4. その他の設定（V20と同一）

```python
reward_cfg = {
    "tracking_sigma": 0.25,
    "base_height_target": 0.20,
    "swing_height_target": 0.05,
    "gait_frequency": 1.2,
    "contact_threshold": 0.10,  # V21変更: 0.05 → 0.10
    "air_time_offset": 0.3,
    "dof_vel_limits": 44.0,
    "soft_dof_vel_limit": 0.9,
}

command_cfg = {
    "lin_vel_x_range": [0.15, 0.25],
    "lin_vel_y_range": [0, 0],
    "ang_vel_range": [0, 0],
}
```

## 期待される効果

| 指標 | V20値（Contact Sensor） | V21目標（Z座標閾値0.10） |
|------|------------------------|------------------------|
| 接地検出 | 正常動作 | 正常動作 |
| 片足接地率 | 91.4% | > 50% |
| feet_air_time報酬 | -0.033 | > 0 |
| single_foot_contact報酬 | 0.206 | > 0 |
| X速度 | 0.204 m/s | 0.15-0.25 m/s |

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_unitree_v21.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-unitree-v21 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | model_499.pt |
| エピソード長 | 500 iterations |
| 収束ステップ | 500 |

### 評価結果

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-unitree-v21 --no-viewer --duration 10
```

**移動性能**:
| 指標 | V20値 | V21値 | 評価 |
|------|-------|-------|------|
| X速度 | 0.204 m/s | 0.185 m/s | ✗ 微減 |
| 移動距離(10秒) | - | 1.848 m | - |
| Yawドリフト | - | +7.93° | 微小 |

**接地パターン（重大問題）**:
| 指標 | V20値 | V21値 | 評価 |
|------|-------|-------|------|
| 片足接地率 | 91.4% | **0.2%** | ✗✗ 大幅劣化 |
| 両足接地率 | 6.4% | **98.8%** | ✗✗ 大幅劣化 |
| 両足空中率 | 2.2% | 1.0% | - |
| hip_pitch相関 | -0.550 | -0.714 | ✓ 改善 |

**関節可動域**:
| 関節 | 左脚 (rad) | 右脚 (rad) |
|------|-----------|-----------|
| hip_yaw | 0.215 | 0.197 |
| hip_roll | 0.148 | 0.232 |
| hip_pitch | 0.355 | 0.542 |
| knee_pitch | 0.262 | 0.223 |
| ankle_pitch | 0.549 | 0.718 |

**評価ログ詳細**:
```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 1.842 m
  Y: 0.142 m
  Total: 1.848 m

Average velocity:
  X: 0.185 m/s (target: 0.150)
  Y: -0.002 m/s

Base height:
  Mean: 0.229 m
  Std: 0.0103 m

Orientation (deg):
  Roll:  mean=  2.68, std= 1.16
  Pitch: mean= -2.39, std= 0.96
  Yaw:   start=  0.00, end=  7.93, drift= +7.93

DOF velocity RMS: 0.851 rad/s
Action RMS: 1.156

=== Contact Pattern Analysis ===
Both feet grounded:     494 steps ( 98.8%)
Single foot grounded:     1 steps (  0.2%)
Both feet airborne:       5 steps (  1.0%)

Left-Right hip_pitch correlation: -0.714
  (-1.0 = perfect alternating, +1.0 = perfect synchronized)
```

## 考察と改善案

### 成功点

1. **hip_pitch相関が改善**: -0.550 → -0.714（理想値 -1.0 に近づいた）
   - 交互歩行パターン自体は形成されつつある
2. **速度追従は維持**: 目標0.15 m/s に対し0.185 m/s を達成
3. **姿勢は安定**: roll/pitch共に5°以内

### 課題（重大）

**V21は「剣道のすり足」状態に退行した**

| 症状 | 数値 | 影響 |
|------|------|------|
| 両足接地率98.8% | 正常歩行では~50%が理想 | 足を上げずに擦って移動 |
| 片足接地率0.2% | V20は91.4%だった | 交互接地が失われた |

### 根本原因分析

#### 足先Z座標の実測データ

V21ポリシーの評価時に足先（foot_link）のZ座標を計測した結果：

```
contact_threshold: 0.1

Step | foot_L_Z | foot_R_Z | threshold | L < thres | R < thres
----------------------------------------------------------------------
   0 |   0.1751 |   0.1751 |    0.1000 | False     | False
  20 |   0.0685 |   0.0652 |    0.1000 | True      | True
  40 |   0.0725 |   0.0673 |    0.1000 | True      | True
  60 |   0.0643 |   0.0681 |    0.1000 | True      | True
  80 |   0.0714 |   0.0639 |    0.1000 | True      | True
 100 |   0.0660 |   0.0717 |    0.1000 | True      | True
 ...

======================================================================
足先Z座標の範囲:
  Left:  min=0.0571, max=0.1751, range=0.1180
  Right: min=0.0590, max=0.1751, range=0.1161
```

**重要な発見**:
- 足先Z座標の**最大値は0.175m**（閾値0.10mを超えている）
- サンプリング時の典型値は**0.06-0.07m**（閾値以下）
- 足は上がっているが、**閾値0.10mを超えている時間が非常に短い**

#### URDF足形状の分析と最適閾値の計算

`rl_ws/assets/bsl_droid_simplified.urdf` の足形状を分析：

```
ankle_pitch_joint (foot_link原点)
        ↓
        │  ← Z = 0 (foot_link原点)
        │
        │  -0.0175m (box中心)
        ├──────────  足本体 (box: 0.09 x 0.06 x 0.035m)
        │  -0.035m
        │
        ◯  -0.035m (cylinder中心)
       ╱╲  radius = 0.03m
      ╱  ╲
     ────── -0.065m (足裏底面 = 接地面)
```

**foot_link のcollision形状**（URDF L323-329, L481-487）:
```xml
<collision>
  <!-- 接地用の半円形状 -->
  <origin rpy="0 1.5707963267948966 0" xyz="0.02 0 -0.035"/>
  <geometry>
    <cylinder length="0.09" radius="0.03"/>
  </geometry>
</collision>
```

**計算結果**:

| 項目 | 値 |
|------|-----|
| foot_link原点から足裏底面まで | **0.065m** |
| collision形状 | cylinder (radius=0.03m, center Z=-0.035m) |
| 足裏底面のZ座標（foot_link基準） | -0.035 - 0.03 = **-0.065m** |

**完全接地時のfoot_link Z座標**:
```
foot_link Z = 地面Z(0) + 足裏オフセット(0.065m) = 0.065m
```

**実測値との比較**:
- 実測: 完全接地時のfoot_link Z座標 ≈ 0.057-0.065m
- 計算: 0.065m
- **一致！**

#### 閾値0.10mが高すぎる問題

| 閾値 | 完全接地(0.065m) | 足1cm上げ(0.075m) | 足2cm上げ(0.085m) | 足3.5cm上げ(0.10m) |
|------|-----------------|------------------|------------------|-------------------|
| 0.10m | 接地 ✓ | **接地（誤判定）** | **接地（誤判定）** | 非接地 ✓ |
| 0.07m | 接地 ✓ | 非接地 ✓ | 非接地 ✓ | 非接地 ✓ |
| 0.068m | 接地 ✓ | 非接地 ✓ | 非接地 ✓ | 非接地 ✓ |

**閾値0.10mの問題点**:
- 足を**3.5cm以上**上げないと「非接地」と判定されない
- V21ポリシーは足を3.5cm以上上げる時間が非常に短い
- 結果として、ほとんどのタイムステップで両足接地と判定される

**URDFに基づく最適閾値**: **0.068m**（完全接地時 + 3mmマージン）

#### 因果関係の整理

```
V21の問題のフロー:
1. 訓練開始時、ポリシーは足をほとんど上げない（ランダム初期化）
2. foot_link Z座標が 0.065〜0.10m の範囲に留まる
3. 閾値0.10mでは「0.10m未満 = 接地」なので、両足とも「接地」と判定
4. single_foot_contact報酬 = 0（両足接地では報酬なし）
5. feet_air_time報酬 = 0（接地→接地では滞空時間が計測されない）
6. ポリシーは「足を上げても報酬が増えない」と学習
7. 足上げを学習する動機がなくなり、すり足に収束

V20（成功例）のフロー:
1. 訓練開始時、ポリシーは足をほとんど上げない
2. Contact Sensorは物理的接触を検出
3. 足が少しでも浮けば「非接地」と正しく判定
4. 片足が浮いた瞬間にsingle_foot_contact報酬 = 1
5. ポリシーは「足を上げると報酬が増える」ことを学習
6. 足上げが強化され、正常な歩行パターンに収束
```

#### Contact Sensorとの挙動差

| 検出方式 | 非接地の条件 | 特性 |
|---------|------------|------|
| Contact Sensor (V20) | 物理的に地面から離れた瞬間 | 足が1mmでも浮けば非接地 |
| Z座標閾値 (V21) | foot_link Z > 0.10m | 足を約3.5cm上げないと非接地にならない |

Contact Sensorは「足が物理的に地面に触れているか」を判定するのに対し、Z座標閾値は「足の高さが閾値以上か」を判定する。**閾値が高すぎると、足が上がっているのに「接地」と誤判定される**。

### 次バージョンへの提案

#### 提案A: feet_swing_heightの強化（優先度: 高）

**目的**: 足を高く上げるインセンティブを強化

| パラメータ | V21値 | V22推奨値 | 変更理由 |
|-----------|-------|----------|---------|
| feet_swing_height | -8.0 | **-20.0** | Unitree G1/H1と同等 |
| swing_height_target | 0.05m | **0.08m** | 足上げ高さ目標を増加 |

#### 提案B: Z座標閾値の調整（優先度: 中）

**目的**: 閾値を下げて接地判定を厳しくする

| パラメータ | V21値 | V22推奨値 | 変更理由 |
|-----------|-------|----------|---------|
| contact_threshold | 0.10m | **0.085m** | 足上げ時に非接地と判定されやすくする |

**注意**: 閾値を下げすぎると完全接地時でも「非接地」と誤判定される危険あり。

#### 提案C: Contact Sensorへの回帰（優先度: 高）

**V20で成功したContact Sensorを再導入することを推奨**

Z座標閾値ベースの接地検出は、足を十分に上げるポリシーが前提となる。しかし、足上げ動作自体を学習させる段階では、Contact Sensorの方が適切。

```python
# V22推奨設定
env_cfg = {
    "use_contact_sensor": True,  # Contact Sensorに戻す
}
reward_cfg = {
    "feet_swing_height": -20.0,  # 足上げを強化
    "swing_height_target": 0.08,
}
```

#### 提案D: single_foot_contact報酬の強化

| パラメータ | V21値 | V22推奨値 | 変更理由 |
|-----------|-------|----------|---------|
| single_foot_contact | 0.3 | **0.5-1.0** | 片足接地を強く報酬 |

### サーベイからの追加知見

[exp007_unitree_rl_gym_survey.md](exp007_unitree_rl_gym_survey.md) セクション7.5.2より:

> **Height-Based Contact Detection**: Isaac Gymの簡略化された衝突推定の制約により、接触力ではなく**足と地面の高さ差**で接地判定

この手法はBooster Gym (arXiv 2025)でも採用されているが、**足上げ動作が確立された後**のフォールバック手法として使用されている。

### 結論

**Z座標閾値ベースの接地検出は、現段階では不適切**

V21の実験により、以下のことが判明した:

1. Z座標閾値方式は「足が十分に上がる」ことを前提とする
2. V21ポリシーは足を十分に上げていないため、常に両足接地と誤判定される
3. Contact Sensorは物理的接触を判定するため、足上げが不十分でも正しく動作する

**V22では以下を推奨**:
1. Contact Sensorを再導入（`use_contact_sensor=True`）
2. feet_swing_height強化（-20.0）とswing_height_target増加（0.08m）
3. 足上げが確立された後に、Z座標閾値方式の再検証を検討

## まとめ

V21では、V20レポートで提案された「実験A: Z座標閾値の修正」を実施した。

**主な変更点**:
1. `use_contact_sensor` オプションを環境クラスに追加し、Contact Sensorの使用を制御可能に
2. `contact_threshold` を 0.05m → 0.10m に修正（足裏オフセットを考慮）
3. トレーニングスクリプトで `use_contact_sensor=False` を設定し、Z座標ベースの接地検出を使用

**実験結果**:

| 項目 | V20（Contact Sensor） | V21（Z座標閾値0.10m） | 評価 |
|------|---------------------|---------------------|------|
| 片足接地率 | 91.4% | **0.2%** | ✗✗ 大幅劣化 |
| 両足接地率 | 6.4% | **98.8%** | ✗✗ 大幅劣化 |
| X速度 | 0.204 m/s | 0.185 m/s | ✗ 微減 |
| hip_pitch相関 | -0.550 | -0.714 | ✓ 改善 |
| 歩行形態 | 正常な交互接地 | **剣道のすり足** | ✗✗ |

**結論**:
Z座標閾値ベースの接地検出は、**足が十分に上がるポリシーが前提条件**となる。V21では足上げが不十分なため、閾値0.10mでは常に両足接地と誤判定された。

Contact Sensorは物理的接触を判定するため、足上げが確立されていない段階でも正しく動作する。**V22ではContact Sensorを再導入し、feet_swing_heightの強化を推奨**する。

**教訓**:
- Z座標閾値方式はフォールバック手法としては有効だが、足上げ動作が確立された後に使用すべき
- 接地検出方式の変更は、歩行パターンに大きな影響を与える
- Contact Sensor無しの環境では、feet_swing_heightを強化して足上げを先に確立させる必要がある

## 備考

- 学習スクリプト: `rl_ws/biped_walking/train/droid_train_unitree_v21.py`
- 環境クラス: `rl_ws/biped_walking/envs/droid_env_unitree.py`（`use_contact_sensor`オプション追加）
- 参照: [exp007_report_v20.md](exp007_report_v20.md)（V21への提案 - 実験A）

### V20との比較ポイント

| 項目 | V20（Contact Sensor） | V21（Z座標閾値） |
|------|---------------------|-----------------|
| 接地検出方式 | 物理エンジンの接触判定 | 足リンクZ座標の閾値判定 |
| 実装複雑度 | センサー初期化が必要 | 閾値設定のみ |
| 汎用性 | Genesis依存 | 任意の物理エンジン |
| 信頼性 | 高（物理ベース） | 中（閾値依存） |
