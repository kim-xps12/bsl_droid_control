# biped_msgs パッケージ モジュール設計書

## 目的

biped_msgs は、BSL-Droid ROS 2 歩行制御システムにおけるカスタムメッセージ定義パッケージである。ament_cmake パッケージとして構築し、安全監視状態の通知（SafetyStatus）およびRLポリシーのデバッグ情報の配信（RLPolicyState）に用いる2種類のメッセージを提供する。

これらのメッセージは、biped_safety、biped_rl_policy など複数のパッケージから参照されるため、独立したパッケージとして切り出し、依存関係の循環を防止する。

## パッケージ構成

```
ros2_ws/src/biped_msgs/
  CMakeLists.txt
  package.xml
  msg/
    SafetyStatus.msg
    RLPolicyState.msg
```

- ビルドシステム: ament_cmake
- メッセージ生成: rosidl_default_generators

## メッセージ定義

### SafetyStatus.msg

安全監視ノード（biped_safety_node）が200Hzで `/safety_status` トピックにpublishするメッセージ。ロボットの安全状態を集約し、他ノードやオペレータが現在の安全状態を把握できるようにする。

#### フィールド定義

| フィールド名 | 型 | 説明 |
|---|---|---|
| `header` | `std_msgs/Header` | タイムスタンプおよびフレームID。安全監視のサンプリング時刻を記録する |
| `is_safe` | `bool` | 総合的な安全状態。`true` のとき全チェック項目が正常、`false` のとき1つ以上のエラーが発生 |
| `warning_flags` | `uint32` | 警告フラグのビットマスク。閾値に近づいているが限界には達していない状態を示す |
| `error_flags` | `uint32` | エラーフラグのビットマスク。閾値を超過し、緊急停止が必要な状態を示す |
| `message` | `string` | 人間が読める形式の状態説明。複数の警告・エラーが同時発生した場合はセミコロン区切りで連結する |

#### warning_flags ビット定義

| ビット位置 | 値（16進） | 名称 | 説明 | 閾値の目安 |
|---|---|---|---|---|
| bit 0 | `0x01` | JOINT_POS_NEAR_LIMIT | 関節角度がURDFリミットに近づいている | URDFリミットの90% |
| bit 1 | `0x02` | JOINT_VEL_NEAR_LIMIT | 関節速度がRS02モータ仕様上限に近づいている | 最大速度の80%（~20 rad/s） |
| bit 2 | `0x04` | HIGH_TILT | Roll/Pitchの傾斜が大きい | 45度以上60度未満 |
| bit 3 | `0x08` | GAMEPAD_DISCONNECTED | ゲームパッドからの入力が途絶している | /joy 未受信が0.5秒以上 |

#### error_flags ビット定義

| ビット位置 | 値（16進） | 名称 | 説明 | 閾値 |
|---|---|---|---|---|
| bit 0 | `0x01` | JOINT_POS_LIMIT_EXCEEDED | 関節角度がURDFリミットを超過した | URDFリミットの100% |
| bit 1 | `0x02` | JOINT_VEL_LIMIT_EXCEEDED | 関節速度がRS02モータ仕様上限を超過した | ~25 rad/s |
| bit 2 | `0x04` | EXCESSIVE_TILT | Roll/Pitchが過度に傾斜し、復帰不能の可能性がある | 60度以上（SR-01準拠） |
| bit 3 | `0x08` | FALL_DETECTED | ベース高さが転倒判定閾値を下回った | 0.1m未満（SR-02準拠） |

#### is_safe の判定ロジック

`is_safe` は `error_flags == 0` のときに `true` となる。`warning_flags` のみがセットされている場合は `is_safe` は `true` のままとなり、オペレータへの注意喚起にとどまる。`error_flags` が1ビットでもセットされた場合は `is_safe` が `false` になり、同時に `/emergency_stop` トピックへの緊急停止信号発行が行われる。

#### message フィールドの例

- 正常時: `"All checks passed"`
- 警告時: `"WARNING: Joint left_knee_pitch position at 92% of limit"`
- エラー時: `"ERROR: Excessive tilt detected (Roll=65.3 deg); ERROR: Joint right_hip_pitch position limit exceeded"`

### RLPolicyState.msg

RLポリシーノード（biped_rl_policy_node）がデバッグ目的で `/rl_policy_state` トピックにpublishするメッセージ。ポリシーの入出力および推論レイテンシを記録し、シミュレータとの挙動比較やパフォーマンスモニタリングに使用する。

本メッセージは通常運用時には高頻度で購読する必要はなく、`ros2 topic echo` やロギングツールによるオンデマンドの確認を想定している。

#### フィールド定義

| フィールド名 | 型 | 説明 |
|---|---|---|
| `header` | `std_msgs/Header` | タイムスタンプ。推論実行時刻を記録する |
| `observation` | `float64[]` | 50次元の観測ベクトル。RLポリシーへの入力そのもの |
| `action` | `float64[]` | 10次元のアクション出力。ポリシーのforward passの直接出力 |
| `target_positions` | `float64[]` | 10次元の目標関節位置 [rad]。`action * action_scale + default_dof_pos` の変換後の値 |
| `inference_time_ms` | `float64` | PyTorchのforward passに要した時間 [ms]。NFR-01（10ms以内）の監視に使用する |

#### observation ベクトルの内訳（50次元）

観測ベクトルは `droid_env_unitree.py:714-730` の構築順序と完全に一致する。

| インデックス | 次元数 | 内容 | スケーリング |
|---|---|---|---|
| 0-2 | 3 | base_lin_vel | obs_scales.lin_vel |
| 3-5 | 3 | base_ang_vel | obs_scales.ang_vel |
| 6-8 | 3 | projected_gravity | なし |
| 9-11 | 3 | commands (vel_x, vel_y, vel_yaw) | commands_scale |
| 12-21 | 10 | dof_pos - default_dof_pos | obs_scales.dof_pos |
| 22-31 | 10 | dof_vel | obs_scales.dof_vel |
| 32-41 | 10 | actions (前回出力) | なし |
| 42-43 | 2 | gait_phase (sin, cos) | なし |
| 44-45 | 2 | leg_phase | なし |
| 46-47 | 2 | feet_pos_z | なし |
| 48-49 | 2 | contact_state | なし |

#### target_positions の関節順序（10次元）

| インデックス | 関節名 |
|---|---|
| 0 | left_hip_yaw_joint |
| 1 | left_hip_roll_joint |
| 2 | left_hip_pitch_joint |
| 3 | left_knee_pitch_joint |
| 4 | left_ankle_pitch_joint |
| 5 | right_hip_yaw_joint |
| 6 | right_hip_roll_joint |
| 7 | right_hip_pitch_joint |
| 8 | right_knee_pitch_joint |
| 9 | right_ankle_pitch_joint |

## 依存関係

### ビルド依存（build_depend）

- `rosidl_default_generators` -- メッセージコード生成
- `std_msgs` -- Header型の参照

### 実行依存（exec_depend）

- `rosidl_default_runtime` -- 生成済みメッセージの実行時ライブラリ

### メンバーグループ依存

- `rosidl_interface_packages` -- ament_indexへのメッセージパッケージ登録

## 使用例

### SafetyStatus の購読（Python）

```python
from biped_msgs.msg import SafetyStatus

def safety_callback(msg: SafetyStatus) -> None:
    if not msg.is_safe:
        self.get_logger().error(f"Safety violation: {msg.message}")
        if msg.error_flags & 0x04:  # EXCESSIVE_TILT
            self.trigger_emergency_stop()
    elif msg.warning_flags != 0:
        self.get_logger().warn(f"Safety warning: {msg.message}")
```

### RLPolicyState の確認（CLI）

```bash
# 推論レイテンシのリアルタイム監視
ros2 topic echo /rl_policy_state --field inference_time_ms

# 観測ベクトルの確認
ros2 topic echo /rl_policy_state --field observation --once
```

### SafetyStatus のビットフラグ判定（Python）

```python
# 複数フラグの同時チェック
JOINT_POS_NEAR_LIMIT = 0x01
HIGH_TILT = 0x04

if msg.warning_flags & (JOINT_POS_NEAR_LIMIT | HIGH_TILT):
    # 関節リミット接近かつ傾斜大 -- 特に注意が必要
    self.get_logger().warn("Multiple warnings active")
```

