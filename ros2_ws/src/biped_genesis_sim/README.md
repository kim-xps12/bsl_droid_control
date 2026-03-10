# biped_genesis_sim

`biped_genesis_sim` は Genesis 物理エンジンを ROS 2 ノードとして包むパッケージです。提供ノードは `genesis_sim_node` のみです。

## ノードの責務

- `/policy_actions` を受け取って物理ステップを進める
- 50 次元の観測を `/policy_obs` に publish する
- 可視化用に `/joint_states` を publish する
- シミュレーション時間を `/clock` に publish する
- `/cmd_vel` を受けて観測ベクトル中のコマンド成分へ反映する

## 実装上の前提

- `urdf_path` は必須です。
- `show_viewer` を `false` にすればヘッドレス起動できます。
- `biped_bringup/genesis_teleop.launch.py` から利用する前提です。

## 現在の位置づけ

- sim モードの RL 検証では本パッケージが環境側を担当します。
- ros2_control は使いません。
- 実機制御用のノードではありません。
