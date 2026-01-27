# 技術仕様書 - biped_gait_control

## 1. 概要

本パッケージは逆関節（Digitigrade/鳥脚型）二脚ロボットの歩容パターン生成を行います。

### 1.1 座標系

```
      +Z (上)
       |
       |
       +------ +X (前)
      /
     /
   +Y (左)
```

- **原点**: 地面中央 (0, 0, 0)
- **X軸**: 前後方向（正が前方）
- **Y軸**: 左右方向（正が左、負が右）
- **Z軸**: 上下方向（正が上）

### 1.2 関節構成

片脚5自由度 × 2脚 = 10自由度

| 関節名 (URDF) | 略称 | 回転軸 | 説明 |
|--------------|------|--------|------|
| `left_hip_yaw_joint` | j11 | Z軸 | 左腰ヨー |
| `left_hip_roll_joint` | j12 | X軸 | 左股関節ロール |
| `left_hip_pitch_joint` | j13 | Y軸 | 左股関節ピッチ |
| `left_knee_pitch_joint` | j14 | Y軸 | 左膝ピッチ |
| `left_ankle_pitch_joint` | j15 | Y軸 | 左足首ピッチ |
| `right_hip_yaw_joint` | j21 | Z軸 | 右腰ヨー |
| `right_hip_roll_joint` | j22 | X軸 | 右股関節ロール |
| `right_hip_pitch_joint` | j23 | Y軸 | 右股関節ピッチ |
| `right_knee_pitch_joint` | j24 | Y軸 | 右膝ピッチ |
| `right_ankle_pitch_joint` | j25 | Y軸 | 右足首ピッチ |

## 2. 運動学

### 2.1 リンク長パラメータ

| パラメータ | 値 [m] | 説明 |
|-----------|--------|------|
| `hip_width` | 0.16 | 左右股関節間距離 |
| `hip_yaw` | 0.03 | ヨー軸リンク長 |
| `hip_roll` | 0.02 | ロール軸リンク長 |
| `thigh` | 0.18 | 大腿部長さ |
| `shank` | 0.20 | 下腿部長さ |
| `foot` | 0.10 | 足部長さ |

### 2.2 逆運動学（2リンク）

股関節ピッチ軸から足首位置への2リンク逆運動学を使用します。

**入力**: 目標足首位置 (target_x, target_z)
**出力**: (hip_pitch, knee_pitch) [degrees]

```python
# 目標までの距離
d = sqrt(target_x² + target_z²)

# 余弦定理で膝角度を計算
cos_knee = (l1² + l2² - d²) / (2 * l1 * l2)
knee_inner_angle = arccos(cos_knee)

# 逆関節: 膝が前方に曲がる = 負の角度
knee_pitch = -(π - knee_inner_angle)

# 股関節ピッチを計算
theta_target = arctan2(target_x, -target_z)
theta_correction = arctan2(l2 * sin(-knee_pitch), l1 + l2 * cos(-knee_pitch))
hip_pitch = theta_target + theta_correction
```

### 2.3 足首角度の計算

足部を地面と平行に保つための足首角度を計算します。

**URDF座標系での定義**:
- 足首角度 0度: 足部は既に後方（-X方向）に配置
- 足部を水平に保つには、脚のピッチ角を打ち消す

```python
ankle_pitch = -(hip_pitch + knee_pitch)
```

## 3. 軌道生成

### 3.1 歩行周期

1歩行周期は以下の2相で構成されます：

| 相 | 位相 | 動作 |
|---|------|------|
| 接地相 (Stance) | 0.0 - 0.5 | 地面を蹴る（後→前） |
| 遊脚相 (Swing) | 0.5 - 1.0 | 足を持ち上げて戻る（前→後） |

左右の脚は位相が0.5（180度）ずれています。

### 3.2 軌道パターン

**接地相**: 直線軌道
```python
x = half_step * (2 * progress - 1)  # -half_step → +half_step
z = 0  # 地面に接地
```

**遊脚相**: 半楕円軌道
```python
theta = π * progress  # 0 → π
x = half_step * cos(theta)  # +half_step → -half_step
z = step_height * sin(theta)  # 0 → peak → 0
```

### 3.3 軌道の視覚化

```
    Z (上)
    |       遊脚相（半楕円）
    |      ___________
    |     /           \
    |    /             \
    +---+---------------+---→ X (前)
       -half_step  +half_step
        ←  接地相（直線）  →
```

## 4. ROS 2インターフェース

### 4.1 トピック

#### パブリッシュ

| トピック | 型 | QoS | 説明 |
|---------|-----|-----|------|
| `/joint_states` | sensor_msgs/JointState | Reliable, depth=10 | 関節角度 [rad] |
| `/joint_trajectory` | trajectory_msgs/JointTrajectory | Reliable, depth=10 | 軌道コントローラ用 |
| `/foot_markers` | visualization_msgs/MarkerArray | Reliable, depth=10 | 足先マーカー |

### 4.2 JointStateメッセージ構造

```yaml
header:
  stamp: <current_time>
  frame_id: "base_link"
name:
  - left_hip_yaw_joint
  - left_hip_roll_joint
  - left_hip_pitch_joint
  - left_knee_pitch_joint
  - left_ankle_pitch_joint
  - right_hip_yaw_joint
  - right_hip_roll_joint
  - right_hip_pitch_joint
  - right_knee_pitch_joint
  - right_ankle_pitch_joint
position: [0.0, 0.0, <hip_pitch_rad>, <knee_pitch_rad>, <ankle_pitch_rad>, ...]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

## 5. URDF連携

### 5.1 必須条件

`gait_pattern_generator`が出力するジョイント名はURDFのジョイント名と**完全に一致**する必要があります。

```xml
<!-- URDF定義例 -->
<joint name="left_hip_pitch_joint" type="revolute">
  ...
</joint>
```

### 5.2 座標系の整合性

| 項目 | URDF定義 | gait_pattern_generator |
|------|---------|----------------------|
| 足首角度 0度 | 足部が後方(-X)に配置 | `-total_leg_angle` で水平維持 |
| 膝角度の符号 | 負で前方に曲がる | 逆運動学で負値を出力 |
| 足部配置 | 足首関節が足部中心 | 運動学計算に影響なし |

## 6. 制限事項

### 6.1 現在の制限

- ヨー・ロール軸は常に0度（前進歩行のみ）
- ZMP制御・バランス制御は未実装
- 歩行速度の動的変更は再起動が必要

### 6.2 今後の拡張予定

- サービスによる歩行開始/停止制御
- 歩行パラメータの動的変更
- 旋回歩行のサポート
