# aoba_hardware パッケージ モジュール設計

## 1. 概要

`aoba_hardware`は、BSL-Droid 二脚ロボットの全 10 関節（左右各 5 軸）を RobStride RS02 モータで駆動するための `ros2_control` Hardware Interface パッケージである。

| コンポーネント | 実装状態 |
|---|---|
| `AobaDriver` — SocketCAN + MIT プロトコル | [実装済み] |
| `AobaHardware` — ros2_control SystemInterface | [実装済み] |
| `can_latency_test` — 単一モータ CAN レイテンシ計測 | [実装済み] |
| `multi_bus_latency_test` — 多モータ・多バス CAN レイテンシ計測 | [実装済み] |

---

## 2. パッケージ構成

```
ros2_ws/src/aoba_hardware/
  package.xml
  CMakeLists.txt
  aoba_hardware_plugin.xml       # pluginlib 登録
  include/aoba_hardware/
    aoba_driver.hpp              # CAN ドライバ API
    aoba_hardware.hpp            # SystemInterface ヘッダ
  src/
    aoba_driver.cpp              # SocketCAN + MIT プロトコル実装
    aoba_hardware.cpp            # ros2_control Hardware Interface 実装
  test/
    can_latency_test.cpp              # 単一モータ CAN レイテンシ計測
    multi_bus_latency_test.cpp        # 多モータ・多バス CAN レイテンシ計測
  scripts/
    single_motor_test_commander.py    # 単一モータ手動コマンドツール
    multi_motor_zero_commander.py     # 複数モータゼロ点移動テストコマンダ
    analyze_timing_log.py             # CAN 通信統計のログ分析ユーティリティ
  launch/
    bringup.launch.py                 # 10関節 ros2_control 起動（可視化モデルなし）
    single_motor_test.launch.py       # 単一モータデバッグ起動
    multi_motor_zero_test.launch.py   # 複数モータゼロ点テスト起動
  config/
    controllers.yaml                  # forward_position_controller（10 関節）
    single_motor_controllers.yaml     # 単一モータ用コントローラ設定
  urdf/
    aoba_system.urdf.xacro       # 10 関節定義（xacro マクロ）
    single_motor_test.urdf.xacro      # 単一モータテスト用 URDF
  doc/
    README.md                         # クイックリファレンス
```

- ビルドシステム: ament_cmake
- pluginlib 登録: `aoba_hardware_plugin.xml` で `hardware_interface::SystemInterface` として登録

---

## 3. 制御アーキテクチャ

### 同期送受信パターン

すべての CAN I/O は Controller Manager の RT スレッド内の `write()` で実行される。`read()` は CAN I/O を行わない（前回 `write()` で更新済みの状態を使用する）。

図: [ros2_walking_module_aoba_hardware.drawio.svg](../fig/ros2_walking_module_aoba_hardware.drawio.svg)

```
Controller Manager RT ループ (200Hz, CPU 2, SCHED_FIFO 90)
┌──────────────────────────────────────────────────────────────────┐
│ read()  → no-op（状態は前回 write() で更新済み）                   │
│ Controller → 制御計算                                             │
│ write()                                                           │
│   Phase 1: 全バスへコマンドをバースト送信                           │
│             can1 の全モータ（5 軸）→ can2 の全モータ（5 軸）        │
│   Phase 2: 全バスから応答をバースト受信                             │
│             can1 の全モータ（poll, 初回 3ms / 以降 1ms）           │
│             can2 の全モータ（poll, 初回 3ms / 以降 1ms）           │
│   Phase 3: missed response 処理（last-known-value 保持）           │
│   Phase 4: DiagnosticSnapshot へ集計（非 RT ログスレッドが出力）    │
└──────────────────────────────────────────────────────────────────┘
```

### タイミング予算

- CAN bus: 1 Mbps、Extended frame ≈ 130 bits → ~130 μs/frame
- 1 バスあたり: 送信 5 フレーム + 受信 5 フレーム = 10 フレーム ≈ 1.3 ms
- 2 バス逐次合計: 典型 ~2.6 ms（5 ms 周期内に十分収まる）

### タイミング診断ログ

200 サイクルごとに、RT スレッドが `DiagnosticSnapshot`（固定サイズ POD）にタイミング統計とモータ状態を書き込み、非 RT ログスレッドが RCLCPP_INFO で出力する（SPSC atomic flag パターン）。RT スレッド内では一切のログ出力・動的メモリ割当を行わない。設計の詳細は [rt_safe_diagnostic_logging.md](../../../article/art006_description_aoba_hardware_logging/art006_description_aoba_hardware_logging.md) を参照。

出力形式:

```
[Timing] write() min=1800us avg=2600us max=3100us stddev=180.5us | send min=200 avg=320 max=450us | recv min=800 avg=980 max=1200us | missed=0/2000
[Timing]   can1: send=320us, recv=980us, ok=5/5
[Timing]   can2: send=330us, recv=970us, ok=5/5
[State] left_hip_yaw_joint (id=11): cmd=0.0000 rad | pos=0.0012 rad, vel=0.0000 rad/s, effort=0.0000 Nm
```

### エラーハンドリング

- 応答がないモータは last-known-value を保持（状態を更新しない）
- 連続 10 サイクル（50 ms）未応答で WARN ログを出力
- 安全判断は `biped_safety` ノードが担当

> **注意**: `aoba_system.urdf.xacro` は ros2_control 定義（CAN/モータID/ゲイン）のみを含み、ロボットのリンク形状・TFツリーは `biped_description/urdf` が提供する。`bringup.launch.py` 単体ではRVizでのロボットモデル可視化はできない。

---

## 4. CAN バス・モータ ID マッピング

| CAN バス | 関節名 | モータ ID |
|---|---|---|
| can1 | left_hip_yaw_joint | 11 |
| can1 | left_hip_roll_joint | 12 |
| can1 | left_hip_pitch_joint | 13 |
| can1 | left_knee_pitch_joint | 14 |
| can1 | left_ankle_pitch_joint | 15 |
| can2 | right_hip_yaw_joint | 21 |
| can2 | right_hip_roll_joint | 22 |
| can2 | right_hip_pitch_joint | 23 |
| can2 | right_knee_pitch_joint | 24 |
| can2 | right_ankle_pitch_joint | 25 |

---

## 5. ライフサイクル管理

`AobaHardware` は `hardware_interface::SystemInterface` の lifecycle に準拠する。

| ライフサイクル | 処理内容 |
|---|---|
| `on_init()` | URDF `<ros2_control>` セクションから関節パラメータ（`can_interface`, `motor_id`, `kp`, `kd`）を読み込む。バスごとに `AobaDriver` インスタンスを生成 |
| `on_configure()` | 各 `AobaDriver` を CAN バスに接続（`connect()`） |
| `on_activate()` | (1) `disable_auto_report()` — 自発的フィードバックフレームを無効化 (2) `enable()` — モータ有効化 (3) `set_mode(MIT)` — MIT 制御モード設定 (4) `drain_rx_buffer()` — 起動直後の残留フレームを破棄 (5) zero-torque probe コマンドで通信確認 (6) 非 RT 診断ログスレッドを起動 |
| `on_deactivate()` | (1) 非 RT ログスレッドを停止（`join()`） (2) 各モータにゼロトルク安全コマンドを送信（kp=0, kd=0, torque_ff=0） (3) 各モータを `disable()` |
| `on_cleanup()` | 全 `AobaDriver` を `disconnect()` |

---

## 6. RT 安全性

| 項目 | 実装 |
|---|---|
| メモリ事前割当 | `on_init()` で `std::vector` のバッファを `resize()` により確保・初期化 |
| poll() タイムアウト | 各バスの受信フェーズで初回 3ms / 以降 1ms のタイムアウトを指定。最悪ケースで `n_buses × 初回timeout` に収束 |
| 動的割当の回避 | RT パス（`read()` / `write()`）内で new/delete を使用しない |
| タイミング統計 | 200 サイクルごとに `DiagnosticSnapshot`（固定サイズ POD）へ集計し、非 RT ログスレッドが RCLCPP_INFO で出力（SPSC atomic flag パターン）。RT スレッド内でのログ出力は一切行わない |

---

## 7. インターフェース

### StateInterface

| インターフェース | 単位 |
|---|---|
| `<joint>/position` | rad |
| `<joint>/velocity` | rad/s |
| `<joint>/effort` | Nm |

### CommandInterface

| インターフェース | 単位 |
|---|---|
| `<joint>/position` | rad |

### URDF パラメータ（関節ごと）

| パラメータ | 型 | 説明 |
|---|---|---|
| `can_interface` | string | CAN バス名（例: `can1`） |
| `motor_id` | int | モータ CAN ID（例: `11`） |
| `kp` | double | MIT 位置ゲイン [Nm/rad]（例: `30.0`） |
| `kd` | double | MIT ダンピングゲイン [Nm/(rad/s)]（例: `1.0`） |

これらのパラメータは `on_init()` の `info_.joints[i].parameters` から読み取る。これは ros2_control の正規のベストプラクティスに準じており、ロボットの物理的構成（どの関節がどのバスのどのモータに対応するか）の Single Source of Truth として URDF を使用する。

---

## 8. テスト・デバッグツール

### single_motor_test

単一モータを接続した状態で動作確認を行う自己完結型テスト。launch 起動のみで、コマンド送信・統計分析・シャットダウンまで自動で完結する。

```bash
# デフォルト（0 rad, 0.5 rad/s, 10秒ホールド）
ros2 launch aoba_hardware single_motor_test.launch.py

# パラメータ指定
ros2 launch aoba_hardware single_motor_test.launch.py \
  target_position:=1.57 max_velocity:=0.3 hold_duration:=15.0
```

起動シーケンス:
1. Controller Manager + Robot State Publisher
2. joint_state_broadcaster → forward_position_controller スポーン
3. single_motor_test_commander ノード自動起動（目標位置へ移動 → ホールド）
4. テスト完了 → analyze_timing_log.py による統計分析
5. 分析完了 → 全体シャットダウン

### can_latency_test

単一バス・単一モータで CAN 往復レイテンシを計測する。

```bash
# Usage: can_latency_test [interface] [motor_id] [iterations]
ros2 run aoba_hardware can_latency_test can1 11 1000
```

### multi_motor_zero_test

複数モータを同時にゼロ点（0 rad）へ移動し、CAN 通信品質を定量評価する。モータ構成は launch 引数で動的に指定可能（URDF とコントローラ設定を動的生成）。テスト完了後に `analyze_timing_log.py` による統計分析を自動実行し、シャットダウンする。

```bash
# 2モータ on can1
ros2 launch aoba_hardware multi_motor_zero_test.launch.py motors:='can1:11,12'

# 本番と同等の10モータ構成
ros2 launch aoba_hardware multi_motor_zero_test.launch.py \
  motors:='can1:11,12,13,14,15 can2:21,22,23,24,25'

# ゲイン・保持時間のカスタマイズ
ros2 launch aoba_hardware multi_motor_zero_test.launch.py \
  motors:='can1:11,12 can2:21,22' kp:=20.0 kd:=0.5 hold_duration:=15.0
```

### multi_bus_latency_test

本番と同等の多モータ・多バス構成で RT 品質を計測する。全バスへのバースト送信→全バスからのバースト受信という `write()` と同一のパターンで計測するため、実際の制御ループ遅延を反映した数値が得られる。

```bash
# Usage: multi_bus_latency_test <bus:id,...> [<bus:id,...> ...] [--iterations=N]
# 例: 本番と同等の10モータ構成
ros2 run aoba_hardware multi_bus_latency_test can1:11,12,13,14,15 can2:21,22,23,24,25

# 例: 部分構成（2モータ）
ros2 run aoba_hardware multi_bus_latency_test can1:11,12
```

出力統計:
- サイクル全体: min / avg / max / stddev / P95 / P99 [us]
- バスごとの内訳: send_us, recv_us, ok/expected
- 200Hz フィージビリティ判定: P99 < 5000us → OK
- missed response 率

---

## 9. 制約

- Linux / SocketCAN 前提（macOS 非対応）
- RS02 スケールファクタ: position ±4π rad、velocity ±44 rad/s、torque ±17 Nm、kp 0-500 Nm/rad、kd 0-5 Nm/(rad/s)
- `joint_state_broadcaster` による状態 publish に統一（独自 publisher なし）
- `kp` / `kd` はランタイム変更不可（URDF で静的に定義）。変更要件が生じた際は YAML 化を検討する
