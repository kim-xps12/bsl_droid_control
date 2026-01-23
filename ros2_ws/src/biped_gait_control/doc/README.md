# biped_gait_control パッケージ

逆関節（鳥脚型）二脚ロボット用の歩容パターン生成パッケージ

## 概要

このパッケージは `ref/example_gait.py` のサンプル歩容モーションをROS 2のベストプラクティスに従って移植したものです。50Hzで関節角度を生成し、`/joint_states` トピックにパブリッシュします。

## パッケージ構成

```
biped_gait_control/
├── biped_gait_control/          # Pythonモジュール
│   ├── __init__.py
│   ├── gait_pattern_generator.py   # メインROS 2ノード
│   ├── kinematics.py               # 運動学計算
│   └── trajectory.py               # 軌道生成
├── config/                      # パラメータ設定ファイル
│   ├── gait_params.yaml            # デフォルト設定
│   ├── gait_params_slow.yaml       # 低速歩行用
│   └── gait_params_fast.yaml       # 高速歩行用
├── launch/                      # Launchファイル
│   ├── gait_control.launch.py           # ノード単体起動
│   ├── gait_control_with_config.launch.py  # YAML設定使用
│   └── gait_visualization.launch.py     # RViz可視化付き
├── doc/                         # ドキュメント
│   ├── README.md                   # 本ドキュメント
│   ├── technical_specification.md  # 技術仕様
│   └── architecture.drawio         # アーキテクチャ図
├── test/                        # テスト
├── package.xml
├── setup.py
└── setup.cfg
```

## クイックスタート

### ビルド

```bash
cd ros2_ws
pixi run colcon build --packages-select biped_gait_control
source install/setup.bash
```

### 起動方法

#### 1. RViz可視化付きで起動（推奨）

```bash
pixi run ros2 launch biped_gait_control gait_visualization.launch.py
```

#### 2. ノード単体で起動

```bash
# RViz + robot_state_publisher を別途起動
pixi run ros2 launch biped_description display_rviz_only.launch.py

# 別ターミナルで歩容生成ノードを起動
pixi run ros2 launch biped_gait_control gait_control.launch.py
```

#### 3. パラメータをカスタマイズして起動

```bash
# コマンドライン引数で指定
pixi run ros2 launch biped_gait_control gait_visualization.launch.py \
    step_frequency:=1.0 \
    step_height:=0.05 \
    step_length:=0.10

# YAMLファイルで指定
pixi run ros2 launch biped_gait_control gait_control_with_config.launch.py \
    config_file:=/path/to/custom_params.yaml
```

## ノード詳細

### gait_pattern_generator

歩容パターンを生成し、関節角度をパブリッシュするメインノード

#### パブリッシュトピック

| トピック | メッセージ型 | 説明 |
|---------|------------|------|
| `/joint_states` | sensor_msgs/JointState | 関節角度（robot_state_publisher用） |
| `/joint_trajectory` | trajectory_msgs/JointTrajectory | 軌道コントローラ用 |
| `/foot_markers` | visualization_msgs/MarkerArray | 足先位置可視化用 |

#### パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|----------|------|
| `publish_rate` | double | 50.0 | パブリッシュレート [Hz] |
| `step_height` | double | 0.04 | 足上げ高さ [m] |
| `step_length` | double | 0.08 | ストライド長 [m] |
| `step_frequency` | double | 0.5 | 歩行周波数 [Hz] |
| `leg_extension_ratio` | double | 0.90 | 脚伸展率（0.0-1.0） |
| `thigh_length` | double | 0.18 | 大腿部長さ [m] |
| `shank_length` | double | 0.20 | 下腿部長さ [m] |
| `enabled` | bool | true | 歩行パターン生成の有効/無効 |

## 関連ドキュメント

- [技術仕様書](technical_specification.md) - 運動学・軌道生成の詳細
- [アーキテクチャ図](architecture.drawio) - システム構成図

## 依存パッケージ

- `rclpy`
- `std_msgs`
- `sensor_msgs`
- `trajectory_msgs`
- `visualization_msgs`
- `numpy`
