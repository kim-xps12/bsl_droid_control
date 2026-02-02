# 足先空間（Task-Space）強化学習による歩容獲得に関する先行研究サーベイ

**作成日**: 2026-01-31  
**関連実験**: exp006_droid_rl_walking_taskspace  
**目的**: BSL-Droid Simplified（10 DOF二脚ロボット）の足先空間（Task-Space）歩行制御において、報酬設計に関する先行研究を調査し、設計指針を明確化する。

> **Note**: 本ドキュメントは調査結果のみを記載する。実験結果の分析・考察は [exp006_droid_rl_walking_taskspace.md](exp006_droid_rl_walking_taskspace.md) を参照。

------

## 1. 足先空間（Task-Space）を探索対象とした歩容獲得

### 1.1 概要：関節空間 vs 足先空間

強化学習による歩行制御において、**行動空間（Action Space）の選択**は学習効率と歩行品質に大きく影響する。

| 行動空間 | 説明 | 長所 | 短所 |
|----------|------|------|------|
| **関節空間（Joint-Space）** | 各関節角度/トルクを直接出力 | シンプル、実機直結 | 高次元、協調困難 |
| **足先空間（Task-Space）** | 足先位置/速度を出力、IKで関節角に変換 | 低次元、直感的 | IK必要、特異点 |
| **階層型（Hierarchical）** | 高レベル（足先）＋低レベル（関節）の2層 | 柔軟、転移学習に有利 | 複雑、学習難 |

### 1.2 足先空間アプローチの理論的背景

#### 1.2.1 次元削減効果

BSL-Droid Simplified（10 DOF）の場合：
- **関節空間**: 10次元（各関節角度）
- **足先空間**: 6次元（左足xyz + 右足xyz）または 4次元（2D平面歩行）

探索空間が小さくなることで、学習効率が向上する可能性がある。

#### 1.2.2 タスク整合性

歩行の本質は「足先をどこに置くか」であり、関節角度は手段に過ぎない：

```
タスク目標: 足先を (x, y, z) に配置
  ↓ 逆運動学（IK）
関節角度: [hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch]
```

足先空間で探索することで、タスクに直結した学習が可能になる。

### 1.3 主要先行研究

#### 1.3.1 Humanoid Locomotion with Task-Space Control (ETH Zurich)

**研究**: "Learning Humanoid Locomotion with Transformers"  
**URL**: https://arxiv.org/abs/2303.03381

##### アプローチ

- Transformerベースのポリシーで足先軌道を生成
- 低レベルPD制御で関節角度追従
- 足先位置はベース座標系での相対位置

```python
# 行動空間の構成
action = {
    "left_foot_pos": [x, y, z],   # 左足先位置（ベース座標系）
    "right_foot_pos": [x, y, z],  # 右足先位置（ベース座標系）
    "torso_orientation": [roll, pitch, yaw],  # 胴体姿勢（オプション）
}
```

##### 成果

- 実機（Digit）での歩行成功
- 複雑な地形への適応
- 長距離歩行の安定性

##### 本実験への示唆

- **足先位置の表現**: ワールド座標系ではなくベース座標系での相対位置が安定
- **IK不要アプローチ**: 足先位置から直接PD制御へのマッピングも可能

---

#### 1.3.2 Residual Policy Learning (RPL)

**論文**: "Residual Policy Learning"  
**URL**: https://arxiv.org/abs/1812.06298

##### コンセプト

基本ポリシー（IK軌道など）に残差を加える：

```
action = base_policy(state) + residual_policy(state)
```

##### 二脚歩行への応用

```python
# 基本軌道: 正弦波ベースの足先軌道
def base_foot_trajectory(phase):
    x = stride_length * phase
    z = swing_height * sin(pi * phase)
    return [x, 0, z]

# 残差ポリシー: 外乱や地形適応のための補正
residual = policy(observation)
final_foot_pos = base_trajectory + residual
```

##### 利点

1. **学習効率**: ゼロから学習するより速い
2. **安全性**: 基本軌道が最低限の歩行を保証
3. **解釈性**: 残差を分析することで学習内容を理解

##### 成功の必要条件

| 条件 | 説明 |
|------|------|
| ベース軌道の最低限の機能 | 基本タスク（歩行）を概ね達成 |
| 残差の役割 | 微調整・ロバスト化 |
| 残差の範囲 | 小さい探索空間 |

**重要**: ベース軌道が全く機能しない場合、RPLは失敗する。

---

#### 1.3.3 DreamWaQ (KAIST, 2024)

**論文**: "DreamWaQ: Learning Robust Quadrupedal Locomotion With Implicit Terrain Imagination via Deep Reinforcement Learning"  
**URL**: https://arxiv.org/abs/2301.10602

##### アプローチ

- 暗黙的な地形推定と足先配置の学習
- ワールドモデルによる将来予測
- 4脚だが2脚への応用可能

##### 足先空間での探索

```python
# 行動空間（4脚の場合）
action_dim = 12  # 4足 × 3次元（x, y, z）

# 各足の目標位置はヒップ座標系で定義
foot_targets = action.reshape(4, 3)
```

##### 本実験への示唆

- **座標系の選択**: ヒップ座標系での足先位置表現が安定
- **正規化**: 可動範囲に対する正規化が重要

---

#### 1.3.4 Hierarchical RL for Legged Locomotion

**論文**: "Learning Agile Robotic Locomotion Skills by Imitating Animals" (Google, 2020)  
**URL**: https://arxiv.org/abs/2004.00784

##### 階層構造

```
High-Level Policy (10Hz)
  │
  │ 足先軌道指令
  ↓
Low-Level Controller (100Hz)
  │
  │ 関節トルク
  ↓
Robot
```

##### 利点

- **周波数分離**: 高レベルは粗い計画、低レベルは細かい制御
- **転移学習**: 高レベルポリシーは異なるロボットに転移可能
- **解釈性**: 高レベルの出力は人間が理解しやすい

##### 本実験への適用案

| レベル | 周波数 | 出力 | 学習方法 |
|--------|--------|------|---------|
| High-Level | 10-20Hz | 足先目標位置 | RL |
| Low-Level | 200Hz | 関節角度指令 | IK + PD |

---

### 1.4 逆運動学（IK）の実装オプション

足先空間アプローチでは、足先位置から関節角度への変換（IK）が必要。

#### 1.4.1 解析的IK

BSL-Droid Simplifiedの脚構造（5 DOF）は、幾何学的に解ける：

```python
def analytical_ik(foot_pos, hip_pos):
    """
    BSL-Droid Simplified の解析的IK
    
    脚構造: hip_yaw → hip_roll → hip_pitch → knee_pitch → ankle_pitch
    """
    # hip_yaw: 足先のXY平面での方向
    hip_yaw = atan2(foot_pos.y - hip_pos.y, foot_pos.x - hip_pos.x)
    
    # 2リンク平面IK
    L1 = thigh_length  # 大腿
    L2 = shank_length  # 下腿
    
    # 足先までの距離
    d = sqrt(dx**2 + dz**2)
    
    # 膝角度（余弦定理）
    cos_knee = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    knee_pitch = acos(clamp(cos_knee, -1, 1))
    
    # 股関節ピッチ
    hip_pitch = atan2(dx, -dz) - asin(L2 * sin(knee_pitch) / d)
    
    # 足首角度（地面に対して平行を維持）
    ankle_pitch = -(hip_pitch + knee_pitch)
    
    return [hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch]
```

#### 1.4.2 数値的IK（Jacobianベース）

より一般的だが計算コストが高い：

```python
def jacobian_ik(foot_pos_target, current_joint_angles, max_iter=10):
    """Jacobianベースの反復IK"""
    q = current_joint_angles.copy()
    
    for _ in range(max_iter):
        current_foot_pos = forward_kinematics(q)
        error = foot_pos_target - current_foot_pos
        
        if norm(error) < tolerance:
            break
        
        J = compute_jacobian(q)
        dq = pinv(J) @ error  # 疑似逆行列
        q += alpha * dq
    
    return q
```

### 1.5 BSL-Droid Simplifiedへの適用設計案

#### 1.5.1 設計案A: End-to-End足先空間

```python
# 行動空間
action_dim = 6  # 左足(x,y,z) + 右足(x,y,z)

# 観測空間
observation = {
    "base_orientation": quat,       # 胴体姿勢
    "base_angular_vel": vec3,       # 角速度
    "command_velocity": vec3,       # 速度指令
    "current_foot_pos": vec6,       # 現在の足先位置
    "gait_phase": scalar,           # 歩行位相（オプション）
}

# 行動 → 関節角度変換
foot_targets = action.reshape(2, 3)
joint_targets = [analytical_ik(foot_targets[i], hip_pos[i]) for i in [0, 1]]
```

##### 利点
- 低次元探索空間（6D vs 10D）
- 足先位置という直感的な行動

##### 課題
- IKの特異点処理
- 可動範囲制約の表現

---

#### 1.5.2 設計案B: Residual Policy + 基本軌道

```python
# 基本軌道（正弦波ベース）
def base_trajectory(phase, command_vel):
    stride = command_vel.x * stride_scale
    height = swing_height
    
    # スイング相の足
    swing_z = height * sin(pi * phase)
    swing_x = stride * (phase - 0.5)
    
    return {
        "swing_foot": [swing_x, 0, swing_z],
        "stance_foot": [0, 0, 0],
    }

# 残差ポリシー
residual = policy(observation)  # 6D出力
final_foot_targets = base_trajectory + residual
```

##### 利点
- 学習効率が高い（基本歩行パターンが保証される）
- 安全な探索（残差は小さい範囲に制限可能）

##### 課題
- 基本軌道の設計が必要
- **基本軌道が機能しないとRPLは失敗する**

---

#### 1.5.3 設計案C: 階層型RL

```python
# High-Level Policy (20Hz)
class HighLevelPolicy:
    """足先軌道を生成"""
    def __init__(self):
        self.action_dim = 6  # 足先位置
        self.observation_dim = 20  # 状態 + コマンド
    
    def forward(self, obs):
        return self.net(obs)  # 足先目標位置

# Low-Level Controller (200Hz)
class LowLevelController:
    """IK + PD制御"""
    def __init__(self):
        self.kp = 50.0
        self.kd = 2.0
    
    def forward(self, foot_targets, current_state):
        joint_targets = analytical_ik(foot_targets)
        torques = self.kp * (joint_targets - q) - self.kd * dq
        return torques
```

##### 利点
- 周波数分離による安定性
- 高レベルポリシーの転移学習が容易
- 低レベルはIK+PDで高速・安定

##### 課題
- 2つのコンポーネントの統合
- 高低レベル間のインターフェース設計

### 1.6 足先空間アプローチの比較まとめ

| 設計案 | 探索次元 | 学習効率 | 実装複雑度 | 推奨度 |
|--------|---------|---------|-----------|--------|
| A: End-to-End | 6D | 中 | 低 | ○ |
| B: Residual | 6D | 高（条件付き） | 中 | △ |
| C: 階層型 | 6D (HL) | 高 | 高 | ○ |

### 1.7 実装時の注意点

#### 1.7.1 座標系の選択

| 座標系 | 説明 | 利点 | 欠点 |
|--------|------|------|------|
| ワールド座標系 | 絶対位置 | 直感的 | 胴体姿勢に依存 |
| ベース座標系 | 胴体相対 | 姿勢不変 | 変換が必要 |
| **ヒップ座標系** | 股関節相対 | IKに直結 | 推奨 |

#### 1.7.2 可動範囲の正規化

```python
# 足先位置の正規化
foot_pos_normalized = {
    "x": (foot_x - x_min) / (x_max - x_min) * 2 - 1,  # [-1, 1]
    "y": (foot_y - y_min) / (y_max - y_min) * 2 - 1,
    "z": (foot_z - z_min) / (z_max - z_min) * 2 - 1,
}
```

#### 1.7.3 特異点の回避

膝が完全に伸びた状態（特異点）を避ける：

```python
def safe_ik(foot_pos):
    # 到達可能範囲をチェック
    d = norm(foot_pos - hip_pos)
    max_reach = L1 + L2 - epsilon
    min_reach = abs(L1 - L2) + epsilon
    
    if d > max_reach or d < min_reach:
        # 最近傍の到達可能点にクリップ
        foot_pos = clip_to_reachable(foot_pos)
    
    return analytical_ik(foot_pos)
```

---

## 2. BSL-Droid Simplified 足先空間実装の詳細設計

### 2.1 URDFから抽出した脚構造パラメータ

`bsl_droid_simplified.urdf.xacro`から抽出した正確なパラメータ：

```python
# === リンク長パラメータ ===
hip_offset_y = 0.10       # 胴体中心から股関節ヨー軸までの距離 [m]
hip_yaw_length = 0.025    # ヨー軸リンク長 [m]
hip_roll_length = 0.03    # ロール軸リンク長（外側へ）[m]
thigh_length = 0.11       # 大腿部長さ [m]
shank_length = 0.12       # 下腿部長さ [m]
foot_height = 0.035       # 足部高さ [m]
ankle_offset_x = 0.02     # 足首から足先中心へのオフセット [m]

# === 脚の総長（伸ばした状態）===
max_leg_length = thigh_length + shank_length  # 0.23 m
min_leg_length = abs(thigh_length - shank_length)  # 0.01 m

# === 関節限界（URDF定義）===
joint_limits = {
    "hip_yaw":   (-30°, +30°),    # Z軸回転
    "hip_roll":  (-25°, +25°),    # X軸回転
    "hip_pitch": (-120°, +90°),   # Y軸回転
    "knee_pitch": (-150°, 0°),    # Y軸回転（逆関節：負で後方屈曲）
    "ankle_pitch": (-90°, +90°),  # Y軸回転
}
```

### 2.2 関節構成と座標系

#### 2.2.1 関節チェーン

```
base_link
  └─ hip_yaw_joint (Z軸回転)
       └─ hip_yaw_link
            └─ hip_roll_joint (X軸回転)
                 └─ hip_roll_link
                      └─ hip_pitch_joint (Y軸回転)
                           └─ thigh_link
                                └─ knee_pitch_joint (Y軸回転)
                                     └─ shank_link
                                          └─ ankle_pitch_joint (Y軸回転)
                                               └─ foot_link
```

#### 2.2.2 座標系定義

| 座標系 | X軸 | Y軸 | Z軸 | 備考 |
|--------|-----|-----|-----|------|
| base_link | 前方 | 左方 | 上方 | ロボット胴体 |
| hip_pitch起点 | 前方 | 左方 | 下方 | IK計算の原点 |

### 2.3 解析的IKの可能性分析

#### 2.3.1 結論：**位置のみの解析解は存在する**

BSL-Droid Simplifiedの脚構造は、**位置IK**については解析的に解ける：

| 成分 | 解析解 | 理由 |
|------|--------|------|
| 足先位置 (x, y, z) | **存在** | hip_yaw + 2リンク平面IK |
| 足先姿勢 (roll, pitch, yaw) | **部分的** | pitch のみ制御可能（ankle_pitch） |

**制約**: 足先のroll/yawは制御不可（対応する関節がない）

#### 2.3.2 IK分解アプローチ

5DOFの脚を以下のように分解してIKを解く：

```
Step 1: hip_yaw で足先のY座標を調整
Step 2: hip_roll で股関節からの横方向オフセットを調整
Step 3: hip_pitch + knee_pitch で2リンク平面IK（XZ平面）
Step 4: ankle_pitch で足裏を地面に平行に維持
```

### 2.4 解析的IKの数学的導出

#### 2.4.1 2リンク平面IK（主要部分）

hip_pitch と knee_pitch を2リンク平面IKとして解く：

```python
def two_link_ik(target_x, target_z, L1, L2):
    """
    2リンク平面IKの解析解
    
    座標系:
    - X軸: 前方（正）
    - Z軸: 下方（正、股関節から足先へ）
    
    Args:
        target_x: 足先X座標（股関節からの相対位置）
        target_z: 足先Z座標（股関節からの相対位置、下向き正）
        L1: 大腿長 (0.11m)
        L2: 下腿長 (0.12m)
    
    Returns:
        hip_pitch: 股関節ピッチ角度 [rad]
        knee_pitch: 膝関節ピッチ角度 [rad]（負で後方屈曲）
    """
    # 足先までの距離
    d_sq = target_x**2 + target_z**2
    d = sqrt(d_sq)
    
    # 到達可能性チェック
    if d > L1 + L2:
        d = L1 + L2 - 0.001
    if d < abs(L1 - L2):
        d = abs(L1 - L2) + 0.001
    
    # 膝角度（余弦定理）
    cos_knee_inner = (L1**2 + L2**2 - d_sq) / (2 * L1 * L2)
    cos_knee_inner = clamp(cos_knee_inner, -1.0, 1.0)
    knee_inner = acos(cos_knee_inner)
    
    # 逆関節: 膝角度は負（後方屈曲）
    knee_pitch = -(math.pi - knee_inner)  # 常に負
    
    # 股関節ピッチ角度
    alpha = atan2(target_x, target_z)  # 前方が正
    sin_beta = L2 * sin(knee_inner) / d
    beta = asin(clamp(sin_beta, -1.0, 1.0))
    
    hip_pitch = alpha - beta
    
    return hip_pitch, knee_pitch
```

#### 2.4.2 ankle_pitch の計算

足裏を地面と平行に維持するため：

```python
def calculate_ankle_pitch(hip_pitch, knee_pitch, desired_foot_pitch=0.0):
    """
    足首角度を計算（足裏を地面に平行に維持）
    """
    cumulative_pitch = hip_pitch + knee_pitch
    ankle_pitch = desired_foot_pitch - cumulative_pitch
    return ankle_pitch
```

### 2.5 足先空間実装に必要な要素一覧

#### 2.5.1 コア実装（必須）

| 要素 | 説明 | 優先度 | 検証方法 |
|------|------|--------|---------|
| 順運動学（FK） | 関節角度 → 足先位置 | **最高** | URDF/RVizで可視化検証 |
| 逆運動学（IK） | 足先位置 → 関節角度 | **最高** | FK ∘ IK = Identity 検証 |
| 座標変換 | ベース座標系 ↔ ヒップ座標系 | 高 | 単体テスト |
| 可動範囲クリップ | IK結果を関節限界内に制限 | 高 | 境界値テスト |
| 特異点回避 | 膝伸展時の処理 | 中 | 境界付近の動作テスト |

#### 2.5.2 環境拡張（DroidEnv修正）

| 要素 | 説明 | 変更箇所 |
|------|------|---------|
| 行動空間変更 | 10D関節 → 6D足先位置 | `__init__`, `step` |
| IK統合 | 足先位置 → 関節指令変換 | `step` |
| 観測空間拡張 | 現在の足先位置を追加 | `_compute_obs` |
| FK統合 | 関節角度 → 足先位置計算 | 新規メソッド |

### 2.6 検証手順

#### 2.6.1 単体テスト

```python
def test_fk_ik_consistency():
    """FK-IK往復検証"""
    kin = DroidKinematics()
    
    # テストケース: 様々な関節角度
    test_angles = [
        [0, 0, 0, -1.0, 1.0],          # 標準姿勢
        [0, 0, 0.3, -1.2, 0.9],        # 前傾
        [0, 0, -0.3, -0.8, 1.1],       # 後傾
        [0.2, 0.1, 0.1, -1.5, 1.4],    # hip_yaw/roll使用
    ]
    
    for angles in test_angles:
        foot_pos = kin.forward_kinematics(angles)
        recovered_angles = kin.inverse_kinematics(foot_pos)
        recovered_pos = kin.forward_kinematics(recovered_angles)
        
        error = torch.norm(foot_pos - recovered_pos)
        assert error < 0.001, f"FK-IK error: {error}"
```

---

## 3. 報酬設計の指針

### 3.1 報酬バランスの重要性

**最重要原則**: 主報酬がペナルティ合計を上回ること

```
主報酬（前進） > ペナルティ合計
```

- 主報酬がペナルティを下回ると「動かない」が最適解になる
- 静止でも獲得できる報酬（alive, base_height等）は控えめに設定

### 3.2 接地検出なしでの交互歩行誘導

接地検出（`feet_air_time`等）が使えない環境での交互歩行誘導：

```python
def _reward_foot_height_diff(self):
    """左右足の高さ差を報酬化（交互動作誘導）"""
    left_z = self.current_foot_pos[:, 1]
    right_z = self.current_foot_pos[:, 3]
    height_diff = torch.abs(left_z - right_z)
    # 差が大きいほど報酬（静止は報酬0）
    return height_diff / 0.05

def _reward_forward_progress(self):
    """前進距離を直接報酬化"""
    delta_x = self.base_pos[:, 0] - self.last_base_pos_x
    return torch.clamp(delta_x, min=0.0) / self.dt
```

### 3.3 推奨報酬構成

| 報酬項 | スケール | 説明 |
|--------|---------|------|
| `tracking_lin_vel` | 5.0 | 主報酬（速度追従） |
| `forward_progress` | 3.0 | 前進距離の直接報酬 |
| `foot_height_diff` | 2.0 | 交互歩行誘導 |
| `hip_pitch_alternation` | 1.0 | 股関節交互動作 |
| `base_height` | 0.5 | 高さ維持（控えめ） |
| `alive` | 0.1 | 生存報酬（控えめ） |
| `orientation` | -1.0 | 姿勢ペナルティ |
| `termination` | -100.0 | 転倒ペナルティ |

---

## 4. Residual Policy用ベース軌道の設計手法

### 4.1 調査の背景と問題提起

exp004のV22/V23で採用した「股関節フレーム基準の正弦波足先軌道」には根本的な問題がある：

```python
# 現在のV22/V23の実装（問題あり）
left_x = default_foot_x + stride_length * 0.5 * sin(phase)
left_z = default_foot_z + swing_height * clamp(sin(phase), min=0)
```

**問題点**：
1. **座標系の誤り**: 股関節フレーム基準では、胴体が動いても足先指令が変わらない
2. **接地タイミング無視**: スタンス相（接地中）も軌道が動き続ける
3. **前進メカニズム欠如**: 足を前後に振るだけでは推進力が生まれない

### 4.2 歩行の基本原理：スイング相とスタンス相

#### 4.2.1 二脚歩行の位相構造

歩行は2つの位相の繰り返しで構成される：

| 位相 | 説明 | 足の状態 |
|------|------|---------|
| **スイング相 (Swing)** | 足を持ち上げて前に振る | 空中 |
| **スタンス相 (Stance)** | 足で地面を蹴り、胴体を推進 | 接地 |

**重要**: 左右の脚は180°位相差で動作し、常にどちらかがスタンス相にある。

#### 4.2.2 前進メカニズム

歩行で前進するメカニズムは以下の通り：

1. スタンス脚が地面を後方に蹴る（反作用で胴体が前進）
2. スイング脚が空中を前方に振る（次の接地点を確保）
3. スイング脚が接地し、スタンス脚になる
4. 役割交代

**股関節フレーム基準の問題**: 胴体が前進しても足先指令が変わらないため、スタンス相で足が「地面を蹴る」動作にならない。

### 4.3 先行研究における軌道生成手法

#### 4.3.1 手法の分類

| 手法 | 座標系 | 位相同期 | 代表研究 | 学習との相性 |
|------|--------|---------|---------|-------------|
| **LIPM + 足配置** | ワールド | あり | Cassie | ◎ 高レベルRL |
| **CPG + IK** | ヒップ相対 | あり | Endo et al. | ○ パラメータ学習 |
| **Bézier曲線** | ヒップ相対 | あり | MIT Cheetah | ○ 軌道最適化 |
| **動作模倣** | ワールド | なし | DeepMimic | ◎ End-to-End |
| **End-to-End RL** | なし | なし | Legged Gym | ◎ 軌道不要 |

#### 4.3.2 LIPMベース軌道生成（Cassie等）

Linear Inverted Pendulum Model (LIPM) を用いたアプローチ：

```
High-Level Controller (LIPM)
  │
  │ CoM軌道 + 足配置位置
  ↓
Low-Level Controller (WBC/IK)
  │
  │ 関節トルク/位置
  ↓
Robot
```

**LIPM軌道生成の原理**：
```python
# LIPM dynamics: CoM位置 x, 速度 v
# x_dot = v
# v_dot = omega^2 * (x - p)  # p: 支持脚位置
# omega = sqrt(g / z_c)

def lipm_trajectory(x0, v0, p_stance, T_step, omega):
    """LIPM軌道を解析的に計算"""
    t = np.linspace(0, T_step, 100)
    x = (x0 - p_stance) * np.cosh(omega * t) + v0 / omega * np.sinh(omega * t) + p_stance
    v = (x0 - p_stance) * omega * np.sinh(omega * t) + v0 * np.cosh(omega * t)
    return x, v

def compute_next_foot_placement(x_apex, v_apex, v_desired, omega):
    """Capture Point理論に基づく次の足配置"""
    capture_point = x_apex + v_apex / omega
    return capture_point - v_desired / omega
```

**本実験への示唆**：
- 高レベルのRLがLIPMの足配置を学習し、低レベルがIKで追従
- ワールド座標系での足配置により、自然な前進が可能
- ただし、実装が複雑（階層制御が必要）

#### 4.3.3 CPGベース軌道生成

Central Pattern Generator (CPG) を用いた生物模倣アプローチ：

```python
# CPG oscillator (Matsuoka model)
class CPGOscillator:
    """単一ニューロンCPGモデル"""
    def __init__(self, tau_r=0.5, tau_a=0.25, w=2.5, beta=2.5):
        self.tau_r = tau_r
        self.tau_a = tau_a
        self.w = w
        self.beta = beta
        self.u = 0
        self.v = 0

    def step(self, s_input, dt):
        y = max(0, self.u)
        du = (-self.u - self.w * self.v - self.beta * y + s_input) / self.tau_r
        dv = (-self.v + y) / self.tau_a
        self.u += du * dt
        self.v += dv * dt
        return y

# 左右結合による交互歩行パターン
class BipedCPG:
    def __init__(self):
        self.left = CPGOscillator()
        self.right = CPGOscillator()
        self.coupling = -1.0  # 抑制結合で逆位相

    def step(self, command_vel, dt):
        left_input = command_vel - self.coupling * self.right.y
        right_input = command_vel - self.coupling * self.left.y
        return self.left.step(left_input, dt), self.right.step(right_input, dt)
```

**特徴**：
- 自律的に交互パターンを生成（位相ロック）
- センサフィードバックでリズムを調整可能（entrainment）
- パラメータ学習でスタイル変更可能

#### 4.3.4 Bézier曲線軌道

MIT Cheetahで採用されたスイング軌道生成：

```python
def bezier_swing_trajectory(phase, p_start, p_end, swing_height, control_points=12):
    """
    Bézier曲線によるスイング軌道生成

    Args:
        phase: スイング位相 [0, 1]
        p_start: スイング開始位置 (x, z)
        p_end: スイング終了位置 (x, z)
        swing_height: 足上げ高さ
    """
    # X座標の制御点（線形に近い）
    cx = np.linspace(p_start[0], p_end[0], control_points)

    # Z座標の制御点（台形状）
    cz = np.array([
        p_start[1],                           # 開始高さ
        p_start[1],                           # 低く維持
        p_start[1] + swing_height * 0.5,      # 上昇開始
        p_start[1] + swing_height,            # 最高点
        p_start[1] + swing_height,            # 最高点維持
        p_start[1] + swing_height,
        p_start[1] + swing_height * 0.8,      # 下降開始
        p_start[1] + swing_height * 0.5,
        p_start[1] + swing_height * 0.2,
        p_end[1] + 0.01,                      # 着地直前
        p_end[1],                             # 着地
        p_end[1],                             # 着地維持
    ])

    return bezier_eval(phase, cx), bezier_eval(phase, cz)
```

**利点**：
- 滑らかな軌道（連続的な加速度）
- 着地時の衝撃を最小化（速度0で接地）
- 制御点で形状を直感的に調整可能

#### 4.3.5 階層型RL（Task-Space Actions）

UC Berkeleyの研究による足先空間でのRL：

```python
# 高レベルポリシー: 足先セットポイントを出力
class HighLevelPolicy:
    """
    出力: 足先目標位置（ヒップ座標系）
    """
    def forward(self, obs):
        return self.actor(obs)  # 6D出力

# 低レベル制御: 逆動力学で関節制御
class InverseDynamicsController:
    """
    足先目標位置 → 関節トルク
    """
    def forward(self, foot_targets, robot_state):
        J = compute_jacobian(robot_state)
        f_task = self.kp * (foot_targets - current_foot_pos) - self.kd * foot_vel
        tau = J.T @ f_task
        return tau
```

**重要な知見**（Kumar et al., 2021）：
> "Many of the most successful and capable bipedal systems choose to control actions in task space by controlling foot placements, ground reaction forces (GRFs), swing foot motions, or center-of-mass (CoM) motion to maintain balance and stability."

### 4.4 現在のベース軌道の問題点（詳細分析）

#### 4.4.1 問題点の詳細

| 問題 | 説明 | 影響 |
|------|------|------|
| **座標系** | 股関節フレーム基準 | 胴体前進に追従しない |
| **スタンス相** | 軌道が動き続ける | 足が滑る/持ち上がる |
| **接地タイミング** | 考慮されていない | 足上げと接地がランダム |
| **位相同期** | 時間ベース | 実際の接地と非同期 |

#### 4.4.2 なぜ歩けないか（力学的説明）

```
現在の動作:
1. 左足が前に出る（股関節フレームで）
2. 右足が後ろに出る（股関節フレームで）
3. 両足とも地面を蹴らない（股関節基準なので）
4. 胴体は静止したまま
5. 結果: 足だけが振れて、前進しない

本来あるべき動作:
1. 左足（スタンス）が地面を後方に蹴る
2. 反作用で胴体が前進
3. 右足（スイング）が空中を前方に振る
4. 右足が接地し、スタンスに移行
5. 役割交代で継続
```

### 4.5 正しいベース軌道設計の指針

#### 4.5.1 選択肢の比較

| アプローチ | 実装難易度 | 期待効果 | 推奨度 |
|-----------|-----------|---------|--------|
| **A: ワールド座標系軌道** | 高 | 高 | △ |
| **B: 接地同期スイング軌道** | 中 | 中〜高 | ○ |
| **C: ベース軌道廃止（End-to-End）** | 低 | 高 | **◎** |
| **D: CPGベース** | 高 | 中 | △ |

#### 4.5.2 推奨: アプローチC（ベース軌道廃止）

先行研究の知見から、**ベース軌道を廃止してEnd-to-End RLで足先位置を学習**するのが最も効果的：

```python
# 提案: ベース軌道なし、足先位置を直接出力
def _compute_joint_commands(self, actions):
    """
    actions: [left_dx, left_dz, right_dx, right_dz]
    ベース軌道なし、行動を直接足先位置オフセットとして使用
    """
    action_scale = 0.05  # 5cm（探索範囲拡大）

    # デフォルト位置からのオフセット
    left_x = self.default_foot_x + actions[:, 0] * action_scale
    left_z = self.default_foot_z + actions[:, 1] * action_scale
    right_x = self.default_foot_x + actions[:, 2] * action_scale
    right_z = self.default_foot_z + actions[:, 3] * action_scale

    # IKで関節角度に変換
    ...
```

**利点**：
1. 実装がシンプル
2. 学習に自由度を与える
3. 事前知識の注入ミスを防ぐ
4. 先行研究（Legged Gym等）で実績あり

#### 4.5.3 代替案: アプローチB（接地同期スイング軌道）

ベース軌道を維持する場合の改善案：

```python
def _get_base_trajectory_improved(self):
    """改善版: 接地同期スイング軌道"""
    phase = self.gait_phase  # [0, 2π)

    # 左脚のスイング位相（0〜πがスイング、π〜2πがスタンス）
    left_in_swing = (phase < math.pi)
    left_swing_phase = phase / math.pi  # [0, 1] during swing

    # スイング軌道（Bézier曲線）
    def swing_z(t):
        """スイング時のZ軌道（0→max→0）"""
        return self.swing_height * 4 * t * (1 - t)  # 放物線近似

    def swing_x(t):
        """スイング時のX軌道（後→前）"""
        return self.stride_length * (t - 0.5)

    # スタンス時は固定（地面上）
    left_x = torch.where(left_in_swing,
                         self.default_foot_x + swing_x(left_swing_phase),
                         self.default_foot_x - self.stride_length * 0.5)  # 後方固定
    left_z = torch.where(left_in_swing,
                         self.default_foot_z + swing_z(left_swing_phase),
                         self.default_foot_z)  # 地面上

    return left_x, left_z, right_x, right_z
```

**ただし注意**: この改善でも、スタンス脚が「地面を蹴る」動作にはならない。股関節フレーム基準の限界。

### 4.6 結論と推奨

#### 4.6.1 V22/V23の失敗原因

1. **根本原因**: 股関節フレーム基準の正弦波軌道では前進メカニズムが成立しない
2. **補助原因**: スタンス相とスイング相の区別がない

#### 4.6.2 推奨アプローチ（優先順）

1. **End-to-End足先空間**（推奨）
   - ベース軌道を廃止
   - 行動を直接足先位置オフセットに
   - 報酬（tracking_lin_vel, foot_height_diff等）のみで歩行を誘導

2. **階層型RL**（将来的）
   - 高レベル: 足配置位置を学習
   - 低レベル: IK + PDで追従
   - LIPM等のモデルベース要素を統合

#### 4.6.3 実装上の注意

- 足先位置の可動範囲を適切に設定（到達可能範囲内）
- action_scaleを十分に大きく（5cm以上）
- 接地促進のためbase_height報酬を適切に設定

---

## 参考文献

1. Radosavovic, I., et al. (2023). "Learning Humanoid Locomotion with Transformers." arXiv:2303.03381.
2. Silver, T., et al. (2018). "Residual Policy Learning." arXiv:1812.06298.
3. Nahrendra, I.M.A., et al. (2024). "DreamWaQ: Learning Robust Quadrupedal Locomotion With Implicit Terrain Imagination." ICRA 2024.
4. Peng, X.B., et al. (2020). "Learning Agile Robotic Locomotion Skills by Imitating Animals." RSS 2020.
5. Rudin, N., et al. (2021). "Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning." CoRL 2021.
6. Paredes, V., et al. (2022). "Resolved Motion Control for 3D Underactuated Bipedal Walking using LIPM." RSS.
7. Endo, G., et al. (2008). "Learning CPG-based Biped Locomotion with a Policy Gradient Method." IJRR.
8. Kim, S., et al. (2019). "Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control." IROS.
9. Kajita, S., et al. (2001). "The 3D Linear Inverted Pendulum Mode." IROS.
10. Kumar, A., et al. (2021). "Learning Task Space Actions for Bipedal Locomotion." arXiv:2011.04741.
11. Li, Z., et al. (2024). "Reinforcement Learning for Versatile, Dynamic, and Robust Bipedal Locomotion Control." IJRR.

---

## 関連リンク

- [Deep Reinforcement Learning for Robotic Bipedal Locomotion: A Brief Survey](https://arxiv.org/html/2404.17070v5)
- [Reinforcement Learning for Versatile, Dynamic, and Robust Bipedal Locomotion Control](https://journals.sagepub.com/doi/full/10.1177/02783649241285161)
- [Learning Task Space Actions for Bipedal Locomotion](https://arxiv.org/pdf/2011.04741)
- [Learning Bipedal Walking On Planned Footsteps For Humanoid Robots](https://ar5iv.labs.arxiv.org/html/2207.12644)
- [Foot trajectory as a key factor for diverse gait patterns in quadruped robot locomotion](https://www.nature.com/articles/s41598-024-84060-5)
- [FootTrajectoryPlanner (GitHub)](https://github.com/pat92fr/FootTrajectoryPlanner)
