# biped_gait_control

`biped_gait_control` は事前設計した軌道の再生（`trajectory_replay`）を提供するパッケージです。

現行の既定パラメータは `bsl_droid_simplified_v2` に合わせた脚長 `thigh=0.11 m`, `shank=0.12 m` を使います。

## エントリポイント

| 実行名 | 実体 | 用途 |
|---|---|---|
| `trajectory_replay` | `biped_gait_control/trajectory_replay_node.py` | 軌道リプレイ |

## launch

| launch | 用途 |
|---|---|
| `trajectory_replay.launch.py` | 軌道リプレイ。`replay_foot.yaml` / `replay_oscillation.yaml` / `replay_waypoint.yaml` を切替可能 |

## 詳細文書

- `doc/README.md`

起動コマンドはプロジェクトルートの `README.md` を参照してください。
