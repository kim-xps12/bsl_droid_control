# biped_bringup

`biped_bringup` は BSL-Droid 歩行系の launch と設定ファイルをまとめるためのパッケージです。実行可能ノードは持たず、既存ノード群の配線だけを担当します。

## 含まれる launch

| launch | 用途 | 構成 |
|---|---|---|
| `genesis_teleop.launch.py` | Genesis 物理シミュレーション上での RL 歩行テレオペ | `robot_state_publisher` + `genesis_sim_node` + `biped_rl_policy_node(sim)` + `joy` 系 |

## 設定ファイル

| ファイル | 役割 |
|---|---|
| `config/joy_f710.yaml` | F710 の軸・ボタン割当と非常停止設定 |
| `config/rl_policy.yaml` | RL 推論ノードの共通パラメータ |
| `config/genesis_sim.yaml` | Genesis 環境ノードの PD・観測構成 |
| `config/safety_limits.yaml` | 将来の `biped_safety_node` 用の設計値 |
| `config/controllers.yaml` | Gazebo / ros2_control 系の比較用設定 |

## 現在の整理

- 実装済みなのは `genesis_teleop` です。
- 実機 RL 制御を一括で起動する launch はまだありません。
- `safety_limits.yaml` は将来の `biped_safety` 実装向けで、現時点では読み込まれていません。

起動コマンドはプロジェクトルートの `README.md` を参照してください。
