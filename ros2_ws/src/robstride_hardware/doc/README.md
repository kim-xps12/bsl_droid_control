# robstride_hardware 詳細

## 1. スコープ

BSL-Droid の全 10 関節（左右各 5 軸）を RobStride RS02 モータで駆動するための `ros2_control` Hardware Interface パッケージ。デュアル CAN バス（can1: 左脚、can2: 右脚）に対応。

## 2. パッケージ構成

| 要素 | 実体 | 補足 |
|---|---|---|
| ドライバ | `robstride_driver.*` | SocketCAN + MIT コマンド送受信、`poll()` ベースのタイムアウト付き読み取り |
| Hardware Interface | `RobStrideHardware` | `ros2_control` の `SystemInterface`（多モータ・多バス対応） |
| URDF | `urdf/robstride_system.urdf.xacro` | 10 関節定義（xacro マクロ）、関節ごとに CAN バス・モータ ID・ゲインを設定 |
| コントローラ設定 | `config/controllers.yaml` | `forward_position_controller`（10 関節） |
| launch | `bringup.launch.py` | Controller Manager + Robot State Publisher + コントローラ起動 |
| launch | `demo_sinusoidal_motion.launch.py` | 正弦波の目標位置を送るデモ |

## 3. 制御アーキテクチャ

### 同期送受信パターン

すべての CAN I/O は Controller Manager の RT スレッド内の `write()` で実行される。`read()` は CAN I/O を行わない。

```
Controller Manager RT ループ (200Hz, CPU 2, SCHED_FIFO 90)
┌─────────────────────────────────────────────────────────────┐
│ read()   → 何もしない（状態は前回 write() で更新済み）        │
│ Controller → 制御計算                                        │
│ write()  → Phase 1: can1 に 5 コマンド送信                   │
│              Phase 2: can2 に 5 コマンド送信                   │
│              Phase 3: can1 から 5 応答受信（poll, timeout 2ms）│
│              Phase 4: can2 から 5 応答受信（poll, timeout 2ms）│
│              Phase 5: タイミング統計のログ出力（1秒ごと）       │
└─────────────────────────────────────────────────────────────┘
```

### タイミング

- CAN bus: 1 Mbps、Extended frame ≈ 130 bits → ~130 μs/frame
- 1 バスあたり: 送信 5 + 受信 5 = 10 フレーム ≈ 1.3 ms
- 2 バス逐次合計: 典型 ~2.6 ms（5 ms 周期内に十分収まる）

### タイミング診断ログ

1 秒ごとに以下の形式でログ出力される:

```
[Timing] write() avg=2600us, max=3100us, missed=0/2000
[Timing]   can1: send=320us, recv=980us, ok=5/5
[Timing]   can2: send=330us, recv=970us, ok=5/5
```

### エラーハンドリング

- 応答がないモータは last-known-value を保持（状態を更新しない）
- 連続 10 サイクル（50 ms）未応答で WARN ログを出力
- 安全判断は `biped_safety` ノードが担当

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

## 5. 制約

- Linux / SocketCAN 前提（macOS 非対応）
- `joint_state_broadcaster` による状態 publish に統一（独自 publisher なし）

起動コマンドはプロジェクトルートの `README.md` を参照してください。
