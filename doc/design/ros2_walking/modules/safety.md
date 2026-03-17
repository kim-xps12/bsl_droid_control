# biped_safety パッケージ モジュール設計

## 1. 概要

`biped_safety`は、BSL-Droid二脚ロボットの安全機能を集約するament_pythonパッケージである。以下の2つのノードで構成される。

| ノード | 役割 | 実装状態 |
|---|---|---|
| `biped_joy_safety_node` | ゲームパッドからの緊急停止ボタン監視・切断検出 | [実装済み] |
| `biped_safety_node` | リアルタイム安全監視（関節・姿勢・コマンド入力、200Hz） | [スタブ実装] |

両ノードとも `/emergency_stop` トピックに `true` を publish することでロボットを安全に停止させる。

---

## 2. パッケージ構成

```
ros2_ws/src/biped_safety/
  package.xml
  setup.py
  setup.cfg
  resource/biped_safety
  biped_safety/
    __init__.py
    safety_node.py
    joy_safety_node.py
  test/
```

- ビルドシステム: ament_python
- エントリポイント:
  - `biped_safety_node = biped_safety.safety_node:main`
  - `biped_joy_safety_node = biped_safety.joy_safety_node:main`

---

## 3. biped_joy_safety_node — ゲームパッド安全機能

`teleop_twist_joy` が対応しない安全機能を担当する軽量ノード。緊急停止ボタン監視とゲームパッド切断検出を行う。

### 3.1 購読トピック

| トピック名 | 型 | QoS | 説明 |
|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | デフォルト | joy_node からのゲームパッド入力。緊急停止ボタンの状態を監視する |

### 3.2 配信トピック

| トピック名 | 型 | QoS | 説明 |
|---|---|---|---|
| `/emergency_stop` | `std_msgs/Bool` | Reliable, Transient Local | 緊急停止信号。`data=true` で停止を発行する |

### 3.3 パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `emergency_stop_buttons` | `int[]` | [9, 10] | 緊急停止ボタンのインデックス配列。F710r の L3（button 9）+ R3（button 10）の同時押し込みに対応する |
| `joy_timeout` | `double` | 0.5 | ゲームパッド切断検出のタイムアウト [秒]。この時間 `/joy` が途絶えると切断と判断する |

### 3.4 状態遷移

biped_joy_safety_node は以下の2状態を管理する。デッドマンスイッチによる IDLE/ACTIVE の切替は `teleop_twist_joy` が担当するため、本ノードでは管理しない。

#### 状態定義

| 状態 | 説明 | 動作 |
|---|---|---|
| `NORMAL` | 通常動作状態 | `/joy` を監視し、緊急停止ボタンの押下を検出する |
| `E_STOP` | 緊急停止が発行された状態 | `/emergency_stop` に `true` を publish し続ける |

#### 遷移条件

| 遷移元 | 遷移先 | トリガー | 副作用 |
|---|---|---|---|
| `NORMAL` | `E_STOP` | L3+R3 同時押し込み（emergency_stop_buttons）が検出された | `/emergency_stop` に `true` を publish する |
| `E_STOP` | `NORMAL` | L3+R3 同時押し込みが再度検出された（トグル動作） | `/emergency_stop` に `false` を publish する |

#### 設計意図

- `teleop_twist_joy` が IDLE/ACTIVE（デッドマンスイッチ）を管理し、`biped_joy_safety_node` が E_STOP を管理する。責務が明確に分離されている。
- E_STOP 状態では `/emergency_stop` に `true` を Transient Local QoS で publish するため、後から起動したノードも即座に緊急停止状態を認識できる。
- E_STOP からの復帰には L3+R3 の明示的な再同時押し込みを要求し、不用意な動作再開を防止する（FR-05）。

### 3.5 ゲームパッド切断検出

`/joy` トピックのメッセージが `joy_timeout`（デフォルト 0.5 秒）以上途絶した場合、ゲームパッドが切断されたと判断する（FR-07）。この検出は ROS 2 タイマーにより実装する。

切断検出時の動作:

1. `/emergency_stop` に `true` を publish する（E_STOP 状態に遷移）
2. ログに警告メッセージを出力する（`WARN` レベル）

`teleop_twist_joy` 側も `/joy` が途絶えればデッドマンスイッチが離された状態と同等になり、ゼロ Twist を publish する。したがって、切断時はゼロ速度指令と緊急停止信号の両方が発行され、ロボットは安全に停止する。

`/joy` メッセージの受信が再開した場合の復帰:

1. 自動的に E_STOP は解除しない（安全側の設計）
2. オペレータが L3+R3 を同時押し込みして明示的に E_STOP を解除する必要がある
3. E_STOP 解除後、LB ボタン（デッドマンスイッチ）を押し直すことで歩行指令が再開する

### 3.6 エラーハンドリング

#### ボタンインデックス範囲外

`/joy` メッセージの `buttons` 配列長がパラメータで指定した `emergency_stop_button` インデックスより短い場合、緊急停止ボタンは「押されていない」として扱う。すなわち、安全側にフォールバックする（E_STOP への遷移は発生しないが、デッドマンスイッチが機能していれば歩行指令は送信されない）。

### 3.7 処理フロー

#### joy コールバック処理

1. `/joy` コールバックが発火する
2. タイムスタンプを記録する（切断検出用タイマーのリセット）
3. 緊急停止ボタン（L3+R3）の状態を確認する
   - 両ボタンの同時押し込みが検出された場合（立ち上がりエッジ）:
     - 現在 `NORMAL` なら `E_STOP` に遷移し、`/emergency_stop` に `true` を publish
     - 現在 `E_STOP` なら `NORMAL` に遷移し、`/emergency_stop` に `false` を publish

#### 切断検出タイマー処理

1. タイマーが発火する（周期: `joy_timeout` / 2）
2. 最後の `/joy` 受信から `joy_timeout` 秒以上経過しているか確認する
3. 経過していれば:
   - `E_STOP` に遷移し、`/emergency_stop` に `true` を publish
   - `WARN` ログを出力する（1回のみ）

### 3.8 依存関係

#### ビルド・実行依存

- `rclpy` -- ROS 2 Python クライアントライブラリ
- `sensor_msgs` -- Joy メッセージ型
- `std_msgs` -- Bool メッセージ型（緊急停止用）

#### 外部パッケージ依存（実行時）

- `joy` -- ゲームパッドドライバノード。既存の依存

---

## 4. biped_safety_node — リアルタイム安全監視

200Hzの高頻度で関節状態・姿勢・コマンド入力を監視し、安全違反を検出した際に緊急停止信号を発行してロボットの破損や転倒被害を防止する。

> **実装状況: [スタブ実装]**
>
> 現在の `biped_safety_node` はノードクラスの骨格のみで、以下に記載する安全チェックロジックは未実装である。本ドキュメントは将来の実装（Session D / Phase 2）に向けた設計仕様として維持する。

### 4.1 Subscribe トピック

| トピック | メッセージ型 | 周波数 | 用途 |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 200Hz | 関節位置・速度の取得 |
| `/imu/data` | `sensor_msgs/Imu` | 200Hz | IMU姿勢・角速度の取得（Phase 3のみ） |
| `/cmd_vel` | `geometry_msgs/Twist` | ~50Hz | 速度指令の受信監視（ウォッチドッグ用） |

### 4.2 Publish トピック

| トピック | メッセージ型 | 周波数 | 用途 |
|---|---|---|---|
| `/safety_status` | `biped_msgs/SafetyStatus` | 200Hz | 安全状態の連続配信（is_safe, warning_flags, error_flags） |
| `/emergency_stop` | `std_msgs/Bool` | イベント駆動 | 安全違反検出時にtrueを発行 |

`/safety_status`は毎サイクル無条件にpublishする。`/emergency_stop`は安全違反を検出した瞬間のみpublishする（通常時は発行しない）。

### 4.3 安全チェック項目（200Hzサイクル）

biped_safety_nodeは200Hzのタイマーコールバック内で以下のチェックを順次実行する。

#### 4.3.1 関節位置リミットチェック

各関節の現在位置がURDF定義の可動範囲内にあるかを検査する。閾値はURDF limitの100%とする（要件SR-03に準拠）。

biped_description URDFに定義された10関節の可動範囲は以下の通りである。

| 関節 | 下限 [rad] | 上限 [rad] | 角度表記 | 備考 |
|---|---|---|---|---|
| hip_yaw | -pi/4 | +pi/4 | -45度〜+45度 | 旋回軸 |
| hip_roll | -pi/6 | +pi/6 | -30度〜+30度 | 左右傾斜軸 |
| hip_pitch | -pi/2 | +pi/3 | -90度〜+60度 | 前後屈曲軸 |
| knee_pitch | -2pi/3 | 0 | -120度〜0度 | 逆関節。負の角度方向に屈曲する |
| ankle_pitch | -pi/3 | +pi/3 | -60度〜+60度 | 足首屈曲軸 |

上記は左右各脚に同一の範囲が適用される（left_*, right_*の計10関節）。

**判定基準:**
- 関節位置がリミットの80%（`warning_threshold_ratio`）に達した場合 → Warning（`warning_flags`にビットを立てる）
- 関節位置がリミットの100%に達した場合 → Error（`error_flags`にビットを立て、`/emergency_stop`を発行）

#### 4.3.2 関節速度リミットチェック

各関節の角速度がRS02モータ仕様の最大速度を超えていないかを検査する。

- **最大関節速度**: 25.0 rad/s（RS02モータの無負荷最大速度に基づく）
- Warning閾値: 最大速度の80%（20.0 rad/s）
- Error閾値: 最大速度の100%（25.0 rad/s）

関節速度は`/joint_states`メッセージのvelocityフィールドから取得する。velocityフィールドが空の場合は、前回のpositionとの差分とタイムスタンプ差から推定する。

#### 4.3.3 姿勢異常チェック（Phase 3のみ）

IMUデータからRoll角およびPitch角を算出し、過度な傾斜を検出する。要件SR-01に準拠する。

- Warning閾値: Roll/Pitchが45度を超過
- Error閾値: Roll/Pitchが60度（1.047 rad）を超過 → 緊急停止

Phase 1ではIMUが搭載されないため、このチェックは無効化する。`/imu/data`トピックの受信有無により自動的に有効・無効を切り替える。

#### 4.3.4 転倒検出（Phase 3のみ）

IMUデータと運動学計算から推定したベース高さが閾値を下回った場合に転倒と判定する。要件SR-02に準拠する。

- Error閾値: ベース高さ < 0.1m → 緊急停止
- BSL-Droidの立位時ベース高さは約0.35mであり、0.1mは明確な転倒状態を示す

Phase 1では姿勢異常チェックと同様に無効化する。

#### 4.3.5 コマンドウォッチドッグ

`/cmd_vel`トピックの最終受信時刻を監視し、一定時間以上受信がない場合に警告フラグを立てる。

- Warning閾値: 最終受信から1.0秒（`cmd_vel_timeout`）経過
- この警告は緊急停止には至らないが、`/safety_status`のwarning_flagsに反映する
- ゲームパッド切断等のフェイルセーフ検出に寄与する（要件FR-07と連携）

### 4.4 Warning と Error の区別

#### Warning（警告）

- 安全限界に接近している状態を示す
- `SafetyStatus.warning_flags`にビットを立てる
- ロボットは動作を継続する
- オペレータへの注意喚起が目的

#### Error（異常）

- 安全限界を超過した状態を示す
- `SafetyStatus.error_flags`にビットを立てる
- `SafetyStatus.is_safe`をfalseに設定する
- `/emergency_stop`トピックにtrue値をpublishする
- ロボットは即座に停止しなければならない

### 4.5 パラメータ一覧

| パラメータ名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `joint_limits` | dict | URDFから読込 | 各関節の可動範囲（上限・下限）。URDFまたはconfigで指定 |
| `max_joint_velocity` | double | 25.0 | 関節速度の上限 [rad/s]。RS02モータ仕様に基づく |
| `max_tilt_angle` | double | 1.047 | 姿勢異常のError閾値 [rad]。60度に相当 |
| `min_base_height` | double | 0.1 | 転倒検出の閾値 [m] |
| `warning_threshold_ratio` | double | 0.8 | Warning閾値をError閾値に対する比率で指定 |
| `cmd_vel_timeout` | double | 1.0 | コマンドウォッチドッグのタイムアウト [s] |

パラメータはlaunchファイルからYAMLファイル（`biped_bringup/config/safety_limits.yaml`）経由で読み込む。URDFから関節リミットを自動取得する機能も備えるが、設定ファイルでの明示的なオーバーライドを優先する。

### 4.6 SafetyStatusメッセージのフラグ定義

`biped_msgs/SafetyStatus`メッセージのwarning_flagsおよびerror_flagsは以下のビットフィールドで構成される。

```
# Warning flags (uint32)
WARNING_JOINT_POSITION  = 0x01  # 関節位置がリミットの80%に接近
WARNING_JOINT_VELOCITY  = 0x02  # 関節速度がリミットの80%に接近
WARNING_ROLL_PITCH      = 0x04  # Roll/Pitchが45度超過
WARNING_CMD_TIMEOUT     = 0x08  # /cmd_velが1秒以上未受信

# Error flags (uint32)
ERROR_JOINT_LIMIT       = 0x10  # 関節位置がリミット超過
ERROR_JOINT_VELOCITY    = 0x20  # 関節速度がリミット超過
ERROR_FALL_DETECTED     = 0x40  # ベース高さが0.1m未満（転倒）
ERROR_TILT_EXCEEDED     = 0x80  # Roll/Pitchが60度超過
```

### 4.7 実装上の留意事項

- 関節名は`biped_description` URDFに定義された名称（`left_hip_yaw_joint`等）を使用し、関節順序は統一関節インターフェースのALL_JOINTS定義に準拠する
- knee_pitchは逆関節であり、可動範囲が負の値のみ（-2pi/3〜0）である点に注意する。リミットチェックの符号処理を誤らないこと
- `/joint_states`のnameフィールドの順序はpublisherに依存するため、名前ベースで各関節の値を取得すること（インデックスベースのアクセスは禁止）
- 複数のError条件が同時に成立する場合は、全てのerror_flagsビットを立てた上で`/emergency_stop`を1回だけpublishする

---

## 5. 緊急停止の挙動

`/emergency_stop` に `true` が publish される経路は以下の2つである:

| 発行元 | トリガー |
|---|---|
| `biped_joy_safety_node` | ゲームパッドの L3+R3 同時押し込み、またはゲームパッド切断検出 |
| `biped_safety_node` | 関節リミット超過、姿勢異常、転倒検出（センサベース） |

### 5.1 停止動作

`/emergency_stop`にtrue値がpublishされた場合、全制御ノードは以下の動作を行う:

- forward_position_controllerは現在位置を保持する（急なトルクオフは転倒リスクがあるため行わない）
- biped_rl_policy_nodeは新たなアクション出力を停止する
- teleop_twist_joyはデッドマンスイッチにより速度指令の送信を停止する

### 5.2 復帰手順

緊急停止からの復帰は発行元によって異なる:

- **biped_joy_safety_node からの E_STOP**: ゲームパッドの L3+R3 を再度同時押し込みして解除（トグル動作）。解除後、LB ボタン（デッドマンスイッチ）を押し直すことで歩行指令が再開する
- **biped_safety_node からの E_STOP**: リセットサービスの呼び出し。復帰時はbiped_safety_nodeが全チェック項目を再評価し、全項目がWarning以下であることを確認してから`/emergency_stop`にfalse値をpublishする

いずれの場合も自動復帰は行わない。

---

## 6. スケジューリングとリアルタイム性

本セクションは `biped_safety_node` のみに適用される。`biped_joy_safety_node` はイベント駆動の軽量ノードであり、リアルタイムスケジューリングの対象外である。

### 6.1 スレッド優先度

Jetson Orin Nano Super上での実行時、biped_safety_nodeはSCHED_FIFO優先度45で動作する。これは次期ノード設計の性能要件表に準拠した値であり、以下の優先度階層に位置する。

| ノード | SCHED_FIFO優先度 | 周波数 |
|---|---|---|
| aoba_hardware | 50 | 200Hz |
| biped_safety_node | 45 | 200Hz |
| state_estimator | 40 | 200Hz |
| rl_policy等 | SCHED_OTHER | 50-100Hz |

### 6.2 処理時間制約

200Hzタイマーコールバックの1サイクルの処理（全チェック項目の実行とpublish）は1ms以内に完了しなければならない（NFR-03に準拠）。安全違反の検出から`/emergency_stop`信号の発行までの遅延は5ms以内とする。

### 6.3 CPUアフィニティ

Jetson Orin Nano Super（6コア）において、biped_safety_nodeはCore 2に割り当てる。state_estimatorと同一コアに配置し、安全監視に必要な状態データへのキャッシュ局所性を確保する。

---

## 7. Phase別の有効チェック項目

biped_safety_node のチェック項目について、Phase別の有効/無効を示す。biped_joy_safety_node は全Phaseで有効である。

| チェック項目 | Phase 1 (viz) | Phase 2 (安全整備) | Phase 3 (実機) |
|---|---|---|---|
| 関節位置リミット | 有効 | 有効 | 有効 |
| 関節速度リミット | 有効 | 有効 | 有効 |
| 姿勢異常（IMU） | 無効 | 無効 | 有効 |
| 転倒検出（IMU） | 無効 | 無効 | 有効 |
| コマンドウォッチドッグ | 有効 | 有効 | 有効 |

Phase 1およびPhase 2ではIMUが搭載されないため、姿勢異常チェックと転倒検出は自動的に無効化される。`/imu/data`トピックの購読開始後に最初のメッセージを受信した時点で有効化する。
