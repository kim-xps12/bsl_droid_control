# biped_safety パッケージ モジュール設計

## 1. 概要

`biped_safety`は、BSL-Droid二脚ロボットのリアルタイム安全監視を担うament_pythonパッケージである。200Hzの高頻度で関節状態・姿勢・コマンド入力を監視し、安全違反を検出した際に緊急停止信号を発行してロボットの破損や転倒被害を防止する。

本パッケージは単一ノード`biped_safety_node`のみで構成される。

---

## 2. ノードインターフェース

### 2.1 Subscribe トピック

| トピック | メッセージ型 | 周波数 | 用途 |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 200Hz | 関節位置・速度の取得 |
| `/imu/data` | `sensor_msgs/Imu` | 200Hz | IMU姿勢・角速度の取得（Phase 3のみ） |
| `/cmd_vel` | `geometry_msgs/Twist` | ~50Hz | 速度指令の受信監視（ウォッチドッグ用） |

### 2.2 Publish トピック

| トピック | メッセージ型 | 周波数 | 用途 |
|---|---|---|---|
| `/safety_status` | `biped_msgs/SafetyStatus` | 200Hz | 安全状態の連続配信（is_safe, warning_flags, error_flags） |
| `/emergency_stop` | `std_msgs/Bool` | イベント駆動 | 安全違反検出時にtrueを発行 |

`/safety_status`は毎サイクル無条件にpublishする。`/emergency_stop`は安全違反を検出した瞬間のみpublishする（通常時は発行しない）。

---

## 3. 安全チェック項目（200Hzサイクル）

biped_safety_nodeは200Hzのタイマーコールバック内で以下のチェックを順次実行する。

### 3.1 関節位置リミットチェック

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

### 3.2 関節速度リミットチェック

各関節の角速度がRS02モータ仕様の最大速度を超えていないかを検査する。

- **最大関節速度**: 25.0 rad/s（RS02モータの無負荷最大速度に基づく）
- Warning閾値: 最大速度の80%（20.0 rad/s）
- Error閾値: 最大速度の100%（25.0 rad/s）

関節速度は`/joint_states`メッセージのvelocityフィールドから取得する。velocityフィールドが空の場合は、前回のpositionとの差分とタイムスタンプ差から推定する。

### 3.3 姿勢異常チェック（Phase 3のみ）

IMUデータからRoll角およびPitch角を算出し、過度な傾斜を検出する。要件SR-01に準拠する。

- Warning閾値: Roll/Pitchが45度を超過
- Error閾値: Roll/Pitchが60度（1.047 rad）を超過 → 緊急停止

Phase 1（vizモード）ではIMUが搭載されないため、このチェックは無効化する。`/imu/data`トピックの受信有無により自動的に有効・無効を切り替える。

### 3.4 転倒検出（Phase 3のみ）

IMUデータと運動学計算から推定したベース高さが閾値を下回った場合に転倒と判定する。要件SR-02に準拠する。

- Error閾値: ベース高さ < 0.1m → 緊急停止
- BSL-Droidの立位時ベース高さは約0.35mであり、0.1mは明確な転倒状態を示す

Phase 1では姿勢異常チェックと同様に無効化する。

### 3.5 コマンドウォッチドッグ

`/cmd_vel`トピックの最終受信時刻を監視し、一定時間以上受信がない場合に警告フラグを立てる。

- Warning閾値: 最終受信から1.0秒（`cmd_vel_timeout`）経過
- この警告は緊急停止には至らないが、`/safety_status`のwarning_flagsに反映する
- ゲームパッド切断等のフェイルセーフ検出に寄与する（要件FR-07と連携）

---

## 4. Warning と Error の区別

### 4.1 Warning（警告）

- 安全限界に接近している状態を示す
- `SafetyStatus.warning_flags`にビットを立てる
- ロボットは動作を継続する
- オペレータへの注意喚起が目的

### 4.2 Error（異常）

- 安全限界を超過した状態を示す
- `SafetyStatus.error_flags`にビットを立てる
- `SafetyStatus.is_safe`をfalseに設定する
- `/emergency_stop`トピックにtrue値をpublishする
- ロボットは即座に停止しなければならない

---

## 5. 緊急停止の挙動

### 5.1 停止動作

`/emergency_stop`にtrue値がpublishされた場合、全制御ノードは以下の動作を行う:

- forward_position_controllerは現在位置を保持する（急なトルクオフは転倒リスクがあるため行わない）
- biped_rl_policy_nodeは新たなアクション出力を停止する
- biped_teleop_nodeは速度指令の送信を停止する

### 5.2 復帰手順

緊急停止からの復帰には明示的な操作を必要とする。自動復帰は行わない。

- 復帰方法: リセットサービスの呼び出し、またはゲームパッドの専用ボタン操作
- 復帰時はbiped_safety_nodeが全チェック項目を再評価し、全項目がWarning以下であることを確認してから`/emergency_stop`にfalse値をpublishする

---

## 6. パラメータ一覧

| パラメータ名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `joint_limits` | dict | URDFから読込 | 各関節の可動範囲（上限・下限）。URDFまたはconfigで指定 |
| `max_joint_velocity` | double | 25.0 | 関節速度の上限 [rad/s]。RS02モータ仕様に基づく |
| `max_tilt_angle` | double | 1.047 | 姿勢異常のError閾値 [rad]。60度に相当 |
| `min_base_height` | double | 0.1 | 転倒検出の閾値 [m] |
| `warning_threshold_ratio` | double | 0.8 | Warning閾値をError閾値に対する比率で指定 |
| `cmd_vel_timeout` | double | 1.0 | コマンドウォッチドッグのタイムアウト [s] |

パラメータはlaunchファイルからYAMLファイル（`biped_bringup/config/safety_limits.yaml`）経由で読み込む。URDFから関節リミットを自動取得する機能も備えるが、設定ファイルでの明示的なオーバーライドを優先する。

---

## 7. スケジューリングとリアルタイム性

### 7.1 スレッド優先度

Jetson Orin Nano Super上での実行時、biped_safety_nodeはSCHED_FIFO優先度45で動作する。これは次期ノード設計の性能要件表に準拠した値であり、以下の優先度階層に位置する。

| ノード | SCHED_FIFO優先度 | 周波数 |
|---|---|---|
| robstride_hardware | 50 | 200Hz |
| biped_safety_node | 45 | 200Hz |
| state_estimator | 40 | 200Hz |
| rl_policy等 | SCHED_OTHER | 50-100Hz |

### 7.2 処理時間制約

200Hzタイマーコールバックの1サイクルの処理（全チェック項目の実行とpublish）は1ms以内に完了しなければならない（NFR-03に準拠）。安全違反の検出から`/emergency_stop`信号の発行までの遅延は5ms以内とする。

### 7.3 CPUアフィニティ

Jetson Orin Nano Super（6コア）において、biped_safety_nodeはCore 2に割り当てる。state_estimatorと同一コアに配置し、安全監視に必要な状態データへのキャッシュ局所性を確保する。

---

## 8. Phase別の有効チェック項目

| チェック項目 | Phase 1 (viz) | Phase 2 (安全整備) | Phase 3 (実機) |
|---|---|---|---|
| 関節位置リミット | 有効 | 有効 | 有効 |
| 関節速度リミット | 有効 | 有効 | 有効 |
| 姿勢異常（IMU） | 無効 | 無効 | 有効 |
| 転倒検出（IMU） | 無効 | 無効 | 有効 |
| コマンドウォッチドッグ | 有効 | 有効 | 有効 |

Phase 1およびPhase 2ではIMUが搭載されないため、姿勢異常チェックと転倒検出は自動的に無効化される。`/imu/data`トピックの購読開始後に最初のメッセージを受信した時点で有効化する。

---

## 9. SafetyStatusメッセージのフラグ定義

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

---

## 10. 実装上の留意事項

- 関節名は`biped_description` URDFに定義された名称（`left_hip_yaw_joint`等）を使用し、関節順序は統一関節インターフェースのALL_JOINTS定義に準拠する
- knee_pitchは逆関節であり、可動範囲が負の値のみ（-2pi/3〜0）である点に注意する。リミットチェックの符号処理を誤らないこと
- `/joint_states`のnameフィールドの順序はpublisherに依存するため、名前ベースで各関節の値を取得すること（インデックスベースのアクセスは禁止）
- 複数のError条件が同時に成立する場合は、全てのerror_flagsビットを立てた上で`/emergency_stop`を1回だけpublishする
