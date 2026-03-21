# biped_description

`biped_description` は BSL-Droid 系の URDF / xacro、RViz 設定、GUI 補助をまとめるパッケージです。現在は複数のモデルを同居させており、用途ごとに使い分けます。

## モデル一覧

| モデル | 用途 | 主な寸法 |
|---|---|---|
| `biped_digitigrade.urdf.xacro` | 旧来の鳥脚デモと GUI 操作用 | `thigh=0.18`, `shank=0.20` |
| `bsl_droid_simplified.urdf.xacro` | BSL-Droid 簡易モデル V1 の比較 | `thigh=0.11`, `shank=0.12` |
| `bsl_droid_simplified_v2.urdf.xacro` | 現行の可視化・RL・軌道リプレイの既定 | `thigh=0.11`, `shank=0.12` |
| `biped_digitigrade_claude_code.urdf.xacro` | 実験的な比較資産 | `thigh=0.10`, `shank=0.12` |

全モデルとも脚の可動関節名は共通で、左右 5 関節ずつの 10 DoF です。

## launch とモデルの対応

| launch | 既定モデル | 目的 |
|---|---|---|
| `display.launch.py` | `biped_digitigrade` | `joint_state_publisher(_gui)` による標準可視化 |
| `display_custom.launch.py` | `biped_digitigrade` | PyQt ベースのカスタム GUI |
| `display_rviz_only.launch.py` | `bsl_droid_simplified_v2` | 外部 `/joint_states` を流し込む既定の可視化 |
| `display_bsl_droid_simplified.launch.py` | `bsl_droid_simplified` | V1 比較表示 |
| `display_bsl_droid_simplified_v2.launch.py` | `bsl_droid_simplified_v2` | V2 単体表示 |
| `display_claude_code.launch.py` | `biped_digitigrade_claude_code` | 実験用比較表示 |

## 注意点

- `display.launch.py` は `joint_state_publisher_gui` を起動するため、外部ノードが `/joint_states` を publish する用途には向きません。
- `trajectory_replay`、`genesis_teleop` の現行系は `bsl_droid_simplified_v2` を前提にしています。
- `biped_display.rviz` は全モデルで共通利用しています。

## 詳細資料

- `doc/robot_structure.drawio`
- `doc/tf_tree.drawio`
- `doc/node_architecture.drawio`

起動コマンドはプロジェクトルートの `README.md` を参照してください。
