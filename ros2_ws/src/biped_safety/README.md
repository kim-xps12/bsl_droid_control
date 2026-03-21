# biped_safety

`biped_safety` はロボットの安全機能を集約するパッケージです。

## 提供ノード

| ノード | 状態 | 役割 |
|---|---|---|
| `biped_joy_safety_node` | ✅ 実装済み | `/joy` を監視し、非常停止ボタンと切断タイムアウトから `/emergency_stop` を publish |
| `biped_safety_node` | 🔄 スタブ | `/joint_states`・IMU 姿勢の安全監視（将来実装） |

## biped_joy_safety_node

- L3 + R3 ボタン同時押しで非常停止をトグル
- 一定時間 `/joy` が来なければ非常停止を強制有効化
- パラメータは通常 `biped_bringup/config/joy_f710.yaml` から与える

## biped_safety_node（未実装）

以下の機能を将来実装予定：

- `/joint_states` 監視（関節可動範囲チェック）
- IMU 姿勢監視（転倒検知）
- `/emergency_stop` の発行
- `biped_msgs/SafetyStatus` の publish

`biped_bringup/config/safety_limits.yaml` は将来実装のための設計値であり、現行コードでは利用されません。

## テレオペ時の役割分担

- `joy_node`: 物理ゲームパッド入力の取得
- `teleop_twist_joy_node`: `/joy` から `/cmd_vel` を生成
- `biped_joy_safety_node`: `/joy` から `/emergency_stop` を生成
