# sysid_ws — RobStride RS-02 システム同定ワークスペース

RS-02 QDD モータ（ピーク 17 Nm、ギア比 7.75:1）の物理パラメータ
（`armature` / `frictionloss` / `damping`）を実機データから同定し、
MuJoCo シミュレーションの sim2real ギャップを縮小するためのツール群。

---

## ディレクトリ構成

```
sysid_ws/
├── doc/sysid.md                   # 同定手法の理論的背景・参考資料
├── models/
│   └── rs02_joint.xml             # MuJoCo 単一DOFモデル（同定対象）
├── recorder/                      # C++ 1 kHz 励振・記録ツール
│   ├── CMakeLists.txt
│   ├── include/sysid_recorder/excitation.hpp
│   └── src/main.cpp
└── optimizer/                     # Python 最適化・検証スクリプト
    ├── pyproject.toml             # uv 管理
    └── sysid/
        ├── optimize.py            # シミュレーションリプレイ最適化
        ├── validate.py            # 初期値 vs 同定値 vs 実機 比較
        └── plot_recording.py      # 録画 CSV の時系列可視化（励振品質確認用）
```

`recorder/` は [ros2_ws/src/aoba_hardware](../ros2_ws/src/aoba_hardware) の
`AobaDriver`（ROS非依存CAN通信ドライバ）を CMake 経由で直接コンパイルして再利用する。
`AOBA_DRIVER_ROOT` 変数で参照先を変更可能。

---

## 全体の流れ

```
┌──────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│ recorder (sysid) │ →  │   optimize.py    │ →  │ identified_      │
│ multi-sine 励振  │    │ least_squares TRF│    │ params.json      │
└──────────────────┘    └──────────────────┘    └──────────────────┘
                                                          │
                                                          ↓
┌──────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│ recorder         │ →  │   validate.py    │ →  │ validation.png   │
│ (--validate, PD) │    │ クローズドループ │    │ + RMSE レポート  │
└──────────────────┘    └──────────────────┘    └──────────────────┘
```

---

## 動作環境

### recorder（C++ 記録ツール）

**前提**: Linux + SocketCAN対応のCANインターフェース。
Jetson Orin Nano Super（オンボードCAN）でも、
通常のUbuntuマシン + USB-CANアダプタでも動作する。

| 構成 | 動作 | RT性能（1kHzジッタ） |
|---|---|---|
| Jetson Orin（オンボードCAN + SCHED_FIFO） | ✓ | 良好（数十μs） |
| Ubuntu + PCIe CANカード | ✓ | Jetsonと同等 |
| Ubuntu + USB-CANアダプタ | ✓ | やや悪化（USB割り込み次第で数百μs） |
| PREEMPT_RTカーネル | ✓ | 最良 |

SocketCAN対応のUSB-CANアダプタ例:
- PEAK PCAN-USB（`peak_usb` ドライバ）
- Innomaker USB2CAN（`gs_usb` ドライバ）
- CANable（オープンソースファームウェア）
- Kvaser Leaf（`kvaser_usb` ドライバ）

### optimizer / validator（Python）

macOS / Linux いずれも可（実機不要）。

---

## セットアップ

### recorder ビルド（Linux）

```bash
# 1. CAN インターフェース設定
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# 2. recorder ビルド
cd sysid_ws/recorder
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

USB-CANアダプタの場合、デバイス名は `can0` 以外になることがあるので
`ip link` で確認し、`--interface` 引数で指定する。

### optimizer セットアップ（macOS / Linux）

```bash
cd sysid_ws/optimizer
uv sync
```

---

## 使い方

### Step 1: 励振データ記録（recorder実行機）

マルチサイン純トルク励振（`kp=0, kd=0, torque_ff=multi_sine(t)`）を 1 kHz で 10 秒送信し、
位置・速度・推定トルクを CSV に記録する。

```bash
cd sysid_ws/recorder
sudo build/sysid_recorder \
    --interface can0 \
    --motor-id 1 \
    --freq 4.5 \
    --amp 2.5 \
    --duration 10 \
    --output ../optimizer/data/recording_sysid.csv
```

#### 励振信号

```
torque(t) = amp × (sin(2π·f·t) + 0.6·sin(2π·3.4f·t) + 0.3·sin(2π·7.4f·t))
```

非有理数比 `1.0 : 3.4 : 7.4` で周波数同士が相殺せず、複数領域を同時に励起する。

#### RS-02 推奨パラメータ

| パラメータ | 値 | 根拠 |
|---|---|---|
| `--freq` | 4–5 Hz | 低すぎ → 速度飽和、高すぎ → 摩擦不可同定 |
| `--amp`  | 2–3 Nm | 高すぎ → トルク-速度曲線に当たる |
| `--duration` | 10 s | 標準。短いと収束しにくい |

#### 出力 CSV

| カラム | 単位 | 説明 |
|---|---|---|
| `timestamp` | s | ループ開始からの経過時間 |
| `cmd_torque` | Nm | コントローラ理論指令値（クランプ前。validateモードでは PD 計算結果そのまま、sysidモードでは multi-sine 値） |
| `cmd_torque_clamped` | Nm | motor 側 `set_torque_limit`（±12 Nm）でクランプ後の値。実際にロータに加わるトルクの上限 |
| `target_position` | rad | sysidモードでは 0 |
| `position` | rad | モータ実測位置 |
| `velocity` | rad/s | モータ実測速度 |
| `estimated_torque` | Nm | モータ推定トルク（LPF 済みと思われ、cmd_torque より振幅が小さく見えることがある） |
| `valid` | 0/1 | 1=応答あり、0=タイムアウト |

#### 確認ポイント

- 行数が 10,000 行（10 s × 1 kHz）程度
- `valid==0` 行が 1% 未満
- `position` が複数 Hz の重畳波形になっている
- 速度がほぼ ±44 rad/s（モータ仕様上限）に張り付いていない

#### 波形を可視化する（`plot_recording.py`）

録画した CSV を時系列プロットで確認するツール。励振品質や安全マージンを目視できる。
sysid・validate 両モードに対応（`target_position` 列が非ゼロなら自動的に階段線で重ね描き）。

```bash
cd sysid_ws/optimizer
uv run python sysid/plot_recording.py \
    --csv data/recording_sysid.csv \
    --output-png data/recording.png
```

**出力**: 4 段時系列プロット
1. `cmd_torque` … `±12 Nm` の torque_limit を破線で重ね描き
2. `estimated_torque` … 同じく `±12 Nm` 線
3. `position` … `initial_position ± 2π` の drift_guard 線、validate モードなら `target_position` の階段線も重ね描き
4. `velocity` … `±30 rad/s` の vel_guard と `±44 rad/s` の RS-02 仕様上限

stdout に samples 数、duration、`valid_rate`、各信号の min/max が出るのでクイックチェックに便利。

**確認ポイント**:
- `cmd_torque` が想定振幅か（sysid: `amp × 1.9`、validate: step 遷移時に PD 飽和）
- `estimated_torque` と `cmd_torque` の乖離（電流ループ帯域の影響を観察）
- `position` が drift_guard 線に近づいていないか
- `velocity` が RS-02 仕様 ±44 rad/s に近づいていないか

`--output-png` を省略すると GUI で表示（X11 / wayland 環境が必要）。

#### `sudo` を回避する場合

```bash
sudo setcap cap_sys_nice,cap_ipc_lock+ep build/sysid_recorder
./build/sysid_recorder ...
```

`SCHED_FIFO` / `mlockall` が失敗しても警告のみで続行する（ジッターは増えるが記録は可能）。

---

### Step 2: パラメータ最適化（任意のマシン）

シミュレーションリプレイ法で
`scipy.optimize.least_squares`（Trust Region Reflective）により位置・速度の重み付き残差ノルムを最小化する θ を探索する。

```bash
cd sysid_ws/optimizer
uv run python sysid/optimize.py \
    --csv data/recording_sysid.csv \
    --model ../models/rs02_joint.xml \
    --output results/identified_params.json
```

#### コスト関数

```
cost(θ) = pos_weight · MSE(q_sim, q_real) + vel_weight · MSE(v_sim, v_real)
   θ = [armature, frictionloss, damping]
```

`vel_weight=0.1`（デフォルト）は CAN の速度フィードバック分解能（12 bit / ±44 rad/s ≈ 0.021 rad/s）が
位置より低いことを反映している。`--pos-weight` / `--vel-weight` で上書き可能。

#### 期待される値（RS-02）

| パラメータ | 期待レンジ |
|---|---|
| `armature` | 0.01 – 0.03 kg·m² |
| `frictionloss` | 0.05 – 0.5 Nm |
| `damping` | 0.01 – 0.1 Nm/(rad/s) |

#### 出力 JSON

```json
{
  "armature": 0.018234,
  "frictionloss": 0.123456,
  "damping": 0.045678
}
```

#### コストが収束しない場合

- `--freq` / `--amp` を変えて記録し直す（励振が不十分）
- `recording.csv` の波形を [`plot_recording.py`](#波形を可視化する-plot_recordingpy) で可視化して、速度飽和や位置範囲超過がないか確認
- 初期値 `x0` を変更（`optimize.py` 内）

---

### Step 3: 検証データ記録（recorder実行機）

PD コントローラ（`kp=8, kd=0.5`）にランダム位置目標（±π rad、1 秒毎更新）を与え、
1 kHz で 10 秒記録する。

```bash
cd sysid_ws/recorder
sudo build/sysid_recorder \
    --interface can0 \
    --motor-id 1 \
    --validate \
    --duration 10 \
    --output ../optimizer/data/recording_validate.csv
```

`--kp` / `--kd` で PD ゲインを変更可能（デフォルト: `kp=8, kd=0.5`）。

---

### Step 4: 検証分析（任意のマシン）

> **重要**: 検証はオープンループのトルクリプレイではなく
> **クローズドループ PD シミュレーション**。
> CSV 中の `target_position` をシミュレーション内 PD コントローラに与え、
> シミュレーション自身の `qpos`/`qvel` でフィードバックを再計算する。

```bash
cd sysid_ws/optimizer
uv run python sysid/validate.py \
    --csv data/recording_validate.csv \
    --model ../models/rs02_joint.xml \
    --params results/identified_params.json \
    --output-png results/validation.png
```

#### 出力

- 4 パネルの比較図（位置・速度 × トレース・誤差）
- 初期値・同定値の RMSE 改善率（コンソール出力）

#### 成功基準

- 位置 RMSE < 0.05 rad
- `validation.png` 上で同定値（緑）と実機（青）が視覚的に区別困難
- 改善率（位置・速度共に）50%以上

---

### Step 5: RL モデルへの適用

同定パラメータを `rl_ws/` の MuJoCo XML に反映する。
RS-02 を使う全関節（脚部 6DOF × 2 など）の `<joint>` 要素または `<default>` ブロックに：

```xml
<joint armature="0.018234" frictionloss="0.123456" damping="0.045678"/>
```

を設定して再学習または fine-tuning。

---

## 動作モードまとめ

| 用途 | recorder モード | optimize/validate |
|---|---|---|
| パラメータ同定 | `--sysid`（デフォルト） | `optimize.py` |
| 同定結果の検証 | `--validate` | `validate.py` |

`recorder/src/main.cpp` 内で `kp=0, kd=0` の純トルク制御（sysidモード）と
`kp=8, kd=0.5` の PD制御（validateモード）を切り替える。

---

## 設計の重要ポイント

### 1 kHz リアルタイムループ

- `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)` による絶対時刻スリープ
  → 累積ドリフトなし
- `SCHED_FIFO` priority 80 + `mlockall(MCL_CURRENT | MCL_FUTURE)`
  → ページフォルト・優先度逆転を抑制
- ファイル I/O はループ終了後に一括フラッシュ
  → RT ループ内ではメモリバッファのみ

### 純トルク励振

`MitCommand{kp=0, kd=0, torque_ff=tau}` を送ることで
モータ内部の PD 制御を無効化し、励振信号で識別したい物理現象がマスクされない。

### CPU MuJoCo + 数値ヤコビアン

`mujoco-sysid` ライブラリは使わない。
理由: MJX の `dof_frictionloss` 勾配が信頼できない（[mujoco issue #1344](https://github.com/google-deepmind/mujoco/issues/1344)）。
frictionloss は MuJoCo 内で「制約 (constraint)」として実装されており、
MJX の制約ソルバが `jax.lax.while_loop` ベースなので逆モード自動微分の経路が切れる。
CPU の MuJoCo + `scipy.optimize.least_squares`（Trust Region Reflective、数値ヤコビアン）
で十分高速（10,000 ステップ × ~1 μs/step ≈ 10 ms/評価、3 パラメータなので
1 ヤコビアン評価 ~40 ms × 数イテレーションで収束）。

`scipy.optimize.minimize(L-BFGS-B)` より `least_squares` を使う理由:
コスト関数が MSE（最小二乗）構造を持つため、ガウス・ニュートン的な曲率近似が
直接効き、`x_scale="jac"` でパラメータのスケール差（`armature` ~10⁻², `frictionloss` ~10⁻¹）
が自動正規化される。

### CSV スキーマの統一

sysid モードと validate モードで同一スキーマを使い、
`target_position` 列を sysid モードでは 0 に設定。
これにより両 CSV を同じローダで処理可能。

---

## トラブルシューティング

| 症状 | 原因と対処 |
|---|---|
| `Failed to connect to can0` | `ip link set can0 up` を実行、`bitrate=1000000` を確認 |
| `Motor did not respond to probe` | モータ電源、CAN 配線、motor-id を確認 |
| `valid==0` が多発（>5%） | `sudo` で実行、または `setcap` で権限付与。CAN バスの他デバイスとの干渉も確認 |
| 速度が ±44 rad/s に張り付く | `--amp` を下げるか `--freq` を上げる |
| 位置が ±4π を超える | 同上、または励振時間を短くする |
| optimize.py が収束しない | 励振データを記録し直す、`--freq` / `--amp` を調整 |
| validate.py の RMSE が改善しない | `plot_recording.py` で sysid 記録の波形を見て励振品質を確認 |

---

## 参考資料

- 同定手法の詳細: [doc/sysid.md](doc/sysid.md)
- AobaDriver API: [../ros2_ws/src/aoba_hardware/include/aoba_hardware/aoba_driver.hpp](../ros2_ws/src/aoba_hardware/include/aoba_hardware/aoba_driver.hpp)
- MuJoCo XML リファレンス: https://mujoco.readthedocs.io/en/stable/XMLreference.html
