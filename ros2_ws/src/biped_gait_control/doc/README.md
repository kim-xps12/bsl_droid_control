# biped_gait_control ドキュメント

## 概要

このパッケージは、事前設計された脚軌道を再生する `trajectory_replay` ノードを提供します。

現行の既定寸法は `thigh_length=0.11 m`, `shank_length=0.12 m` で、`bsl_droid_simplified_v2` 系モデルに合わせています。

## ノード

### trajectory_replay

- `source_type` に応じて軌道ソースを切替（Strategy Pattern で `compute()` を差し替え）
- 出力は常に `joint_limits.py` でクランプ
- `viz` モードでは `/joint_states` を publish
- `control` モードでは加えて `/forward_position_controller/commands` を publish
- 実機制御そのものは起動しないため、`control` モード時は別途 `robstride_hardware` 側の bringup が必要

### トピック

| トピック | 送信元 | 用途 |
|---|---|---|
| `/joint_states` | `trajectory_replay` | 可視化用 |
| `/forward_position_controller/commands` | `trajectory_replay` (`control` 時) | ros2_control 位置指令 |
| `/emergency_stop` | 外部入力 | `trajectory_replay` の停止トリガ |

## 軌道ソース

| `source_type` | 実装 | 用途 |
|---|---|---|
| `foot` | `foot_trajectory_source.py` | 足軌道 + IK |
| `oscillation` | `single_joint_oscillation_source.py` | 単関節検証 |
| `waypoint` | `waypoint_playback_source.py` | ウェイポイント補間 |

## 設定ファイル

| ファイル | 用途 |
|---|---|
| `replay_foot.yaml` | 足軌道リプレイ |
| `replay_oscillation.yaml` | 単関節振動 |
| `replay_waypoint.yaml` | ウェイポイント再生 |

## 制限事項

- `trajectory_replay` はトルク制御や速度制御を行わない
- 実機と可視化の統合検証は `robstride_hardware` 側の単関節デモを超えていない

起動コマンドはプロジェクトルートの `README.md` を参照してください。
