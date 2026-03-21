# biped_rl_policy

`biped_rl_policy` は学習済みポリシーの推論を担当するパッケージです。提供ノードは `biped_rl_policy_node` のみです。

## モード

| モード | 実装状態 | 概要 |
|---|---|---|
| `viz` | 実装済み（コード内残存、システムlaunchでは未使用） | `/cmd_vel` から擬似観測を構築し `/joint_states` を publish |
| `sim` | 実装済み | `/policy_obs` を受けて `/policy_actions` を返す純粋な推論ラッパー |
| `control` | 未実装 | 実機用の観測構築・出力経路は未着手 |

## 補足

- `model_path` を与えない場合、推論結果はゼロベクトルになります。
- `viz` モードでは `biped_msgs/RLPolicyState` をまだ publish していません。
- 実際の起動は `biped_bringup/genesis_teleop.launch.py` 経由が前提です。
