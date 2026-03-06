# biped_teleop パッケージ モジュール設計書

## 目的

biped_teleop は、Logitech F710r ゲームパッドからの入力を ROS 2 の速度指令（geometry_msgs/Twist）に変換し、安全機能（緊急停止・切断検出）を提供するパッケージである。ament_python パッケージとして構築する。

速度指令変換には ROS 2 標準パッケージ `teleop_twist_joy` をそのまま利用する。`teleop_twist_joy` はデッドマンスイッチ、軸マッピング、速度スケーリングをパラメータで設定可能であり、本システムの FR-01（ゲームパッド速度指令入力）および SR-04（デッドマンスイッチ）の要件を単体で満たす。

本パッケージが提供する独自ノード `biped_joy_safety_node` は、`teleop_twist_joy` が対応しない安全機能（緊急停止ボタン、ゲームパッド切断検出）を担当する。

### 設計判断: teleop_twist_joy の採用理由

自前の teleop ノードではなく `teleop_twist_joy` を採用した理由:

- **保守コスト低減**: ROS 2 の新ディストリビューションへの追従が不要
- **十分な機能**: デッドマンスイッチ、軸マッピング、速度スケーリング、`linear.y` 対応など、コア機能を全て備える
- **テスト容易性**: teleop のスケーリングロジックと安全機能を独立にテストできる
- **安全性への影響なし**: 安全機能が正しく動くかどうかはノードの統合・分離と独立した問題であり、`/joy` を2ノードで subscribe しても同一メッセージを同タイミングで受け取る

自前ノードで統合する場合のメリット（joy コールバック1つで全処理が完結する見やすさ、ノード数1つ減）は、上記のメリットに比べて実質的な価値が低いと判断した。

## パッケージ構成

```
ros2_ws/src/biped_teleop/
  package.xml
  setup.py
  setup.cfg
  resource/biped_teleop
  biped_teleop/
    __init__.py
    joy_safety_node.py
  test/
    test_joy_safety_node.py
```

- ビルドシステム: ament_python
- エントリポイント: `biped_joy_safety_node = biped_teleop.joy_safety_node:main`

## ノード構成

本パッケージは2つのノードを組み合わせて使用する。

### teleop_twist_joy_node（外部パッケージ）

ROS 2 標準パッケージ `teleop_twist_joy` が提供するノード。`/joy` を購読し `/cmd_vel` を配信する。パラメータ設定のみで本システムの要件を満たす。

#### teleop_twist_joy のパラメータ設定

以下のパラメータを `biped_bringup/config/joy_f710.yaml` で設定する。

| パラメータ名 | 値 | 説明 |
|---|---|---|
| `enable_button` | 4 | デッドマンスイッチ。F710r の LB ボタン（SR-04） |
| `axis_linear.x` | 1 | 左スティック Y（上下）→ 前後速度 |
| `axis_linear.y` | 0 | 左スティック X（左右）→ 横速度 |
| `axis_angular.yaw` | 2 | 右スティック X（左右）→ yaw角速度 |
| `scale_linear.x` | -0.5 | 前後速度のスケール [m/s]。負値で符号反転（注1） |
| `scale_linear.y` | -0.3 | 横速度のスケール [m/s]。負値で符号反転（注1） |
| `scale_angular.yaw` | -1.0 | yaw角速度のスケール [rad/s]。負値で符号反転（注1） |

**注1: 符号反転について**

joy_node はスティック上方向を負値、左方向を負値として報告するが、ロボットの座標系では前進が+X、左移動が+Y、左旋回が+yaw である。`teleop_twist_joy` の `scale_*` パラメータを負値にすることでこの符号反転を実現する。この処理は `biped_eval_gamepad.py:216-218` の実装と等価である。

**注2: デッドゾーンについて**

`teleop_twist_joy` 自体にはデッドゾーン機能がない。デッドゾーン処理は `joy_node` のパラメータ `deadzone`（デフォルト 0.05）で対応する。本システムでは `joy_node` の `deadzone` を 0.08 に設定する。

### biped_joy_safety_node（本パッケージ提供）

`teleop_twist_joy` が対応しない安全機能を担当する軽量ノード。

#### 購読トピック

| トピック名 | 型 | QoS | 説明 |
|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | デフォルト | joy_node からのゲームパッド入力。緊急停止ボタンの状態を監視する |

#### 配信トピック

| トピック名 | 型 | QoS | 説明 |
|---|---|---|---|
| `/emergency_stop` | `std_msgs/Bool` | Reliable, Transient Local | 緊急停止信号。`data=true` で停止を発行する |

#### パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `emergency_stop_button` | `int` | 7 | 緊急停止ボタンのインデックス。F710r の START ボタンに対応する |
| `joy_timeout` | `double` | 0.5 | ゲームパッド切断検出のタイムアウト [秒]。この時間 `/joy` が途絶えると切断と判断する |

## 状態遷移

biped_joy_safety_node は以下の2状態を管理する。デッドマンスイッチによる IDLE/ACTIVE の切替は `teleop_twist_joy` が担当するため、本ノードでは管理しない。

### 状態定義

| 状態 | 説明 | 動作 |
|---|---|---|
| `NORMAL` | 通常動作状態 | `/joy` を監視し、緊急停止ボタンの押下を検出する |
| `E_STOP` | 緊急停止が発行された状態 | `/emergency_stop` に `true` を publish し続ける |

### 遷移条件

| 遷移元 | 遷移先 | トリガー | 副作用 |
|---|---|---|---|
| `NORMAL` | `E_STOP` | START ボタン（emergency_stop_button）が押下された | `/emergency_stop` に `true` を publish する |
| `E_STOP` | `NORMAL` | START ボタンが再度押下された（トグル動作） | `/emergency_stop` に `false` を publish する |

### 設計意図

- `teleop_twist_joy` が IDLE/ACTIVE（デッドマンスイッチ）を管理し、`biped_joy_safety_node` が E_STOP を管理する。責務が明確に分離されている。
- E_STOP 状態では `/emergency_stop` に `true` を Transient Local QoS で publish するため、後から起動したノードも即座に緊急停止状態を認識できる。
- E_STOP からの復帰には START ボタンの明示的な再押下を要求し、不用意な動作再開を防止する（FR-05）。

## ゲームパッド切断検出

`/joy` トピックのメッセージが `joy_timeout`（デフォルト 0.5 秒）以上途絶した場合、ゲームパッドが切断されたと判断する（FR-07）。この検出は ROS 2 タイマーにより実装する。

切断検出時の動作:

1. `/emergency_stop` に `true` を publish する（E_STOP 状態に遷移）
2. ログに警告メッセージを出力する（`WARN` レベル）

`teleop_twist_joy` 側も `/joy` が途絶えればデッドマンスイッチが離された状態と同等になり、ゼロ Twist を publish する。したがって、切断時はゼロ速度指令と緊急停止信号の両方が発行され、ロボットは安全に停止する。

`/joy` メッセージの受信が再開した場合の復帰:

1. 自動的に E_STOP は解除しない（安全側の設計）
2. オペレータが START ボタンを押して明示的に E_STOP を解除する必要がある
3. E_STOP 解除後、LB ボタン（デッドマンスイッチ）を押し直すことで歩行指令が再開する

## エラーハンドリング

### ボタンインデックス範囲外

`/joy` メッセージの `buttons` 配列長がパラメータで指定した `emergency_stop_button` インデックスより短い場合、緊急停止ボタンは「押されていない」として扱う。すなわち、安全側にフォールバックする（E_STOP への遷移は発生しないが、デッドマンスイッチが機能していれば歩行指令は送信されない）。

## 処理フロー

### biped_joy_safety_node のコールバック処理

1. `/joy` コールバックが発火する
2. タイムスタンプを記録する（切断検出用タイマーのリセット）
3. 緊急停止ボタンの状態を確認する
   - 押下が検出された場合（立ち上がりエッジ）:
     - 現在 `NORMAL` なら `E_STOP` に遷移し、`/emergency_stop` に `true` を publish
     - 現在 `E_STOP` なら `NORMAL` に遷移し、`/emergency_stop` に `false` を publish

### 切断検出タイマー処理

1. タイマーが発火する（周期: `joy_timeout` / 2）
2. 最後の `/joy` 受信から `joy_timeout` 秒以上経過しているか確認する
3. 経過していれば:
   - `E_STOP` に遷移し、`/emergency_stop` に `true` を publish
   - `WARN` ログを出力する（1回のみ）

## 依存関係

### ビルド・実行依存

- `rclpy` -- ROS 2 Python クライアントライブラリ
- `sensor_msgs` -- Joy メッセージ型
- `std_msgs` -- Bool メッセージ型（緊急停止用）

### 外部パッケージ依存（実行時）

- `teleop_twist_joy` -- Joy→Twist 変換ノード。`pixi.toml` に `ros-jazzy-teleop-twist-joy` として追加する
- `joy` -- ゲームパッドドライバノード。既存の依存

