# MuJoCoモデルにおけるモータのモデル同定手順

本ドキュメントはOpen Duck Mini / Open Duck Playgroundプロジェクトのソースコード解析に基づき、MuJoCo物理シミュレータ上でのモータ（アクチュエータ）モデル同定の具体的手順を記述する。BSL-Droidへの応用を念頭に、手法の全貌を体系的に整理する。

## 目次

1. [概観：Sim-to-Real転移の全体像](#1-概観sim-to-real転移の全体像)
2. [同定対象パラメータ](#2-同定対象パラメータ)
3. [物理モデル構造](#3-物理モデル構造)
4. [データ収集手順](#4-データ収集手順)
5. [パラメータフィッティング](#5-パラメータフィッティング)
6. [バックラッシュモデリング](#6-バックラッシュモデリング)
7. [Domain Randomization](#7-domain-randomization)
8. [推論パイプライン](#8-推論パイプライン)
9. [BSL-Droidへの適用指針](#9-bsl-droidへの適用指針)
10. [参照ファイル一覧](#10-参照ファイル一覧)

---

## 1. 概観：Sim-to-Real転移の全体像

Open Duckプロジェクトのアプローチは以下の3層で構成される。

1. **大まかな同定**: 実機の正弦波応答を計測し、MuJoCoのアクチュエータパラメータを手動チューニング
2. **Domain Randomization**: 同定されたパラメータに一定の幅でランダム化を施し、不確かさに頑健なポリシーを学習
3. **アクションパイプライン設計**: 遅延モデル・速度制限・ローパスフィルタ等を訓練時と推論時の双方で一致させる

**核心的な思想**: 完璧な同定を目指すのではなく、「大まかな同定 + ランダム化」というプラグマティックなアプローチで現実のばらつきを吸収する。

---

## 2. 同定対象パラメータ

MuJoCoのアクチュエータモデルで同定が必要なパラメータは以下の6つである。

| パラメータ | MuJoCo XMLタグ | 物理的意味 | 単位 | Open Duckでの値 |
|---|---|---|---|---|
| PDゲイン（P） | `<position kp="...">` | 位置制御の比例ゲイン | N/rad | 17.11 → 17.8 |
| PDゲイン（D） | `<position kv="...">` | 位置制御の微分ゲイン | N·s/rad | 0.0 |
| 最大トルク | `forcerange` | アクチュエータ出力上限 | N·m | ±3.23 → ±3.35 |
| 関節粘性減衰 | `<joint damping="...">` | 速度に比例する抵抗力 | N·s/rad | 0.56 → 0.60 |
| クーロン摩擦 | `<joint frictionloss="...">` | 速度非依存の静的摩擦 | N·m | 0.068 → 0.052 |
| ロータ慣性 | `<joint armature="...">` | モータ回転子の等価慣性 | kg·m² | 0.027 → 0.028 |

> **注記**: `→` は反復チューニングによる値の変遷を示す。Open Duckのコードベースには旧値がコメントアウトで残存しており、複数回の同定サイクルが行われたことがわかる。

### MuJoCo XML定義の実例

```xml
<!-- ref/Open_Duck_Playground/playground/open_duck_mini_v2/xmls/joints_properties.xml -->
<default>
    <default class="sts3215">
        <geom contype="0" conaffinity="0"/>
        <joint damping="0.60" frictionloss="0.052" armature="0.028"/>
        <position kp="17.8" kv="0.0" forcerange="-3.35 3.35"/>
    </default>
</default>
```

Open Duckでは全関節が同一サーボ（STS3215）であるため、`default class`として一括定義している。関節ごとに異なるサーボを使う場合は個別に定義が必要となる。

---

## 3. 物理モデル構造

### 3.1 アクチュエータの制御方式

Open Duckでは**位置制御（position servo）** を採用している。

```xml
<!-- Playground版（学習用） -->
<actuator>
    <position class="sts3215" name="left_hip_yaw" joint="left_hip_yaw" inheritrange="1"/>
    <!-- ... 他の関節も同様 ... -->
</actuator>
```

```xml
<!-- Mini版（旧モデル） -->
<actuator>
    <motor name="left_hip_yaw" joint="left_hip_yaw"/>
    <!-- ... トルク直接制御 ... -->
</actuator>
```

両者の違いは重要で、`<position>` はMuJoCoの内蔵PDコントローラを利用するのに対し、`<motor>` はトルクを直接指定する。位置制御方式は実機のサーボ動作に近く、同定が容易。

### 3.2 制御ループのタイミング

```
物理シミュレーション: sim_dt = 0.002 [s] (500 Hz)
制御ループ:          ctrl_dt = 0.02  [s] (50 Hz)
decimation:          10 (= ctrl_dt / sim_dt)
```

1回の制御指令に対して10回の物理ステップが実行される。これは実機のサーボ応答遅延を模擬するとともに、シミュレーション精度を確保する。

---

## 4. データ収集手順

### 4.1 概要

同定データの収集は `ref/Open_Duck_Mini/experiments/identification/get_data.py` で行う。**実機とMuJoCoに同一の正弦波を同時印加**し、応答を比較する。

### 4.2 装置構成

```
[PC] --- USB --- [U2D2] --- Serial Bus --- [STS3215サーボ x N]
  |
  +--- MuJoCoシミュレータ（同一PCで並列実行）
```

### 4.3 実行コマンド

```bash
python get_data.py \
    --dof left_ankle \       # 対象関節名
    --move_freq 10 \         # 正弦波周波数 [Hz]
    --move_amp 0.5 \         # 正弦波振幅 [rad]
    --ctrl_freq 30 \         # 制御指令送信周波数 [Hz]
    --sampling_freq 100 \    # データサンプリング周波数 [Hz]
    --duration 5             # 収集時間 [s]
```

### 4.4 データ収集の詳細フロー

```python
# 1. 初期化
model = mujoco.MjModel.from_xml_path("scene.xml")  # MuJoCoモデル読み込み
hwi = HWI(usb_port="/dev/ttyUSB0")                   # 実機接続
hwi.turn_on()
hwi.set_pid_all([500, 0, 500])                        # サーボPID設定
time.sleep(3)                                          # 安定待ち

# 2. メインループ（duration秒間）
while t < duration:
    # 2a. 制御指令生成（ctrl_freq Hzで実行）
    target = init_pos[dof] + sin(2π × move_freq × t) × move_amp
    
    # 2b. MuJoCoに指令
    data.ctrl[dof_id] = target
    mujoco.mj_step(model, data, 5)  # 5物理ステップ
    
    # 2c. 実機に指令（ActionFilter経由）
    action_filter.push(target)
    filtered = action_filter.get_filtered_action()
    hwi.set_position(dof_name, filtered)
    
    # 2d. データ記録（sampling_freq Hzで実行）
    mujoco_command_value.append([data.ctrl[:], data.qpos[:]])
    robot_command_value.append([last_command, hwi.get_present_positions()])

# 3. 保存
pickle.dump({
    "config": {...},
    "mujoco": mujoco_command_value,
    "robot": robot_command_value,
}, file)
```

### 4.5 記録されるデータ

各タイムステップで以下の4系列を記録する。

| データ | ソース | 内容 |
|---|---|---|
| MuJoCo指令値 | `data.ctrl[:]` | シミュレータに送った位置指令 |
| MuJoCo応答値 | `data.qpos[:]` | シミュレータ内の実際の関節角度 |
| 実機指令値 | `last_robot_command` | サーボに送った位置指令 |
| 実機応答値 | `hwi.get_present_positions()` | サーボのエンコーダ読み取り値 |

### 4.6 速度応答の追加計測

`check_speed.py` ではさらに**速度応答特性**を計測する。

```python
# 特定の関節に正弦波を印加し、速度応答を比較
dof = 7          # 左膝
a = 0.3          # 振幅 [rad]
f = 3            # 周波数 [Hz]
pid = [1100, 0, 0]  # 高ゲインで速い追従を要求

while True:
    target = a * sin(2π × f × t)
    # MuJoCoとrobotに同時印加
    recording["mujoco_vel"].append(data.qvel[dof])
    recording["robot_vel"].append(hwi.get_present_velocities()[dof])
```

この速度応答データから、モータの最大角速度 `max_motor_velocity = 5.24 rad/s` が決定された。

### 4.7 データの全関節網羅

同定は**各関節ごとに個別に実施**する。これは各関節の負荷条件（重力方向、リンク質量）が異なるためである。

```bash
# 全関節の同定データを収集
for dof in left_ankle left_knee left_hip_pitch left_hip_roll left_hip_yaw \
           right_ankle right_knee right_hip_pitch right_hip_roll right_hip_yaw; do
    python get_data.py --dof $dof --move_freq 10 --move_amp 0.5 --duration 5
done
```

---

## 5. パラメータフィッティング

### 5.1 可視化による評価

`plot.py` でMuJoCoと実機の応答を重ね描きする。

```bash
python plot.py -d data/left_ankle.pkl
# → robot(1) or mujoco(2) ? を選択
# → 全15関節の指令値vs応答値をサブプロットで表示
```

`plot_obs.py` ではさらに、MuJoCoの観測値（Observation）と実機の観測値を全チャネル（関節角・角速度・ジャイロ・加速度計）にわたって比較する。

### 5.2 フィッティングプロセス

Open Duckプロジェクトでは**手動チューニングの反復ループ**を採用している。

```
    ┌─────────────────────────────────────┐
    │  1. 初期値でデータ収集             │
    │     (get_data.py)                    │
    │              │                       │
    │              ▼                       │
    │  2. 応答波形を重ね描き             │
    │     (plot.py / plot_speeds.py)       │
    │              │                       │
    │              ▼                       │
    │  3. 乖離パターンを読み取り         │
    │     ・位相遅れ → damping調整        │
    │     ・振幅不足 → kp増加            │
    │     ・定常偏差 → frictionloss調整   │
    │     ・応答が鈍い → armature減少     │
    │              │                       │
    │              ▼                       │
    │  4. XMLパラメータを修正            │
    │     (joints_properties.xml)          │
    │              │                       │
    │              ▼                       │
    │  5. 再度データ収集（→ 1に戻る）   │
    └─────────────────────────────────────┘
```

### 5.3 パラメータ調整の指針

| 観測される乖離 | 原因 | 調整するパラメータ | 調整方向 |
|---|---|---|---|
| MuJoCo応答が実機より速い | PDゲインが高すぎる | `kp` | 減少 |
| MuJoCo応答が実機より遅い | PDゲインが低すぎる / 慣性が大きすぎる | `kp` / `armature` | 増加 / 減少 |
| 実機にある位相遅れがMuJoCoにない | 粘性減衰が足りない | `damping` | 増加 |
| 実機で振幅が減衰している | 摩擦が足りない | `frictionloss` | 増加 |
| 実機で方向反転時に停滞する | バックラッシュ（ギヤの遊び） | backlash joint追加 | 下記参照 |
| 実機で特定方向だけ応答が遅い | 重力負荷の非対称性 | アクションオフセット | 調整 |
| MuJoCo速度が実機を超える | 速度上限がない | `max_motor_velocity` | 設定 |

### 5.4 パラメータ変遷の実例

Open Duckコードベースに残るコメントアウトから復元した変遷：

```
[初期] kp=17.11, damping=0.56, frictionloss=0.068, armature=0.027, force=±3.23
    ↓ 同定サイクル
[最終] kp=17.8,  damping=0.60, frictionloss=0.052, armature=0.028, force=±3.35
```

- `kp` を微増: 実機の追従が若干速かった
- `damping` を増加: 位相遅れを再現
- `frictionloss` を減少: 実機の摩擦は想定より小さかった
- `armature` を微増: 高速動作時の応答を合わせた
- `force` を増加: 実機のトルク上限がMuJoCoのクランプより大きかった

---

## 6. バックラッシュモデリング

ホビーサーボのギヤには機械的な遊び（バックラッシュ）がある。Open Duckではこれを**ダミー関節**としてモデル化している。

### 6.1 XML定義

```xml
<!-- 各アクチュエータ関節に隣接してbacklash jointを追加 -->
<joint name="left_hip_yaw" type="hinge" range="..." class="sts3215"/>
<joint name="left_hip_yaw_backlash" pos="0 0 0" axis="0 0 1" class="backlash"/>
```

```xml
<!-- backlashクラスの定義 -->
<default class="backlash">
    <!-- ±0.5° (0.00873 rad) の遊び -->
    <joint damping="0.01" frictionloss="0" armature="0.01" limited="true"
        range="-0.008726646259971648 0.008726646259971648"/>
</default>
```

### 6.2 動作原理

- backlash jointは同じ軸のhinge jointで、微小範囲のみ自由に回転可能
- アクチュエータが方向を反転する際、この遊び分だけ「空振り」が発生
- `damping=0.01`, `frictionloss=0` で抵抗が極めて小さく、遊びの範囲内で自由に動く
- 結果として、指令と出力の間にヒステリシスが生じる

### 6.3 バックラッシュの同定方法

バックラッシュ幅は以下の方法で計測する：

1. 関節を一方向にゆっくり回転させ、応答の追従を記録
2. 方向を反転し、再び追従を記録
3. 反転時に応答が停滞する時間・角度から遊び幅を推定
4. ±0.5°はSTS3215サーボのギヤ特性として経験的に設定された値

### 6.4 学習時の使い分け

Open Duckでは学習シーン（XML）を2種類用意している：

| XMLファイル | バックラッシュ | 用途 |
|---|---|---|
| `scene_flat_terrain.xml` | なし | 高速学習・基礎実験 |
| `scene_flat_terrain_backlash.xml` | あり | Sim-to-Real精度重視の学習 |

---

## 7. Domain Randomization

同定パラメータが完璧でなくても実機で動作するよう、学習時にパラメータをエピソードごとにランダム化する。

### 7.1 ランダム化パラメータ一覧

`ref/Open_Duck_Playground/playground/common/randomize.py` で実装。

| パラメータ | 分布 | 範囲 | 意図 |
|---|---|---|---|
| 床摩擦係数 | U(0.5, 1.0) | 絶対値 | 路面条件の変動 |
| 関節摩擦 (frictionloss) | ×U(0.9, 1.1) | 基準値比 | サーボ個体差 |
| ロータ慣性 (armature) | ×U(1.0, 1.05) | 基準値比 | 慣性の不確かさ（狭い範囲で安定性確保） |
| 重心位置 (body_ipos) | +U(-0.05, 0.05) m | 加算 | 組立誤差 |
| 全リンク質量 (body_mass) | ×U(0.9, 1.1) | 基準値比 | 部品の重量ばらつき |
| 胴体追加質量 | +U(-0.1, 0.1) kg | 加算 | ペイロード変動 |
| 初期関節角 (qpos0) | +U(-0.03, 0.03) rad | 加算 | 起動時の姿勢誤差 |
| **PDゲイン (kp)** | **×U(0.9, 1.1)** | **基準値比** | **サーボ応答の個体差・経年変化** |

### 7.2 実装の要点

```python
@jax.vmap  # バッチ内の全環境に対してベクトル化
def rand_dynamics(rng):
    # PDゲインのランダム化（最重要）
    factor = jax.random.uniform(key, shape=(model.nu,), minval=0.9, maxval=1.1)
    current_kp = model.actuator_gainprm[:, 0]
    actuator_gainprm = model.actuator_gainprm.at[:, 0].set(current_kp * factor)
    # MuJoCoのposition actuatorではbiasprm[1] = -kp
    actuator_biasprm = model.actuator_biasprm.at[:, 1].set(-current_kp * factor)
    ...
```

### 7.3 ランダム化範囲の設計原則

- **安定性に影響するパラメータ**（armature）は狭い範囲（×1.0〜1.05）
- **ロバスト性に影響するパラメータ**（摩擦・質量）は広い範囲（×0.9〜1.1）
- PDゲインのランダム化は**同定誤差の吸収**に直結する最重要項目

---

## 8. 推論パイプライン

学習時と推論時（MuJoCo検証 / 実機デプロイ）で一貫したアクションパイプラインを維持する必要がある。

### 8.1 アクション変換の全体フロー

```
NNポリシー出力 (tanh → [-1, 1])
    │
    ▼
clip(-1, 1)
    │
    ▼
アクション遅延注入（学習時: 0〜3ステップのランダム遅延）
    │
    ▼
スケーリング: motor_target = default_actuator + action × action_scale(=0.25)
    │
    ▼
モータ速度制限: |target - prev_target| ≤ max_motor_velocity × dt
    │
    ▼
MuJoCo ctrl / 実機サーボに送信
```

### 8.2 観測ベクトルの構成

```python
obs = [
    gyro,                                    # 3: ジャイロスコープ [rad/s]
    accelerometer,                           # 3: 加速度計 [m/s²]（accelerometer[0] += 1.3 補正あり）
    command,                                 # 3: 速度指令 (vx, vy, ωz)
    joint_angles - default_actuator,         # 10: 関節角度偏差 [rad]
    joint_vel × dof_vel_scale(=0.05),        # 10: 関節速度（スケーリング済み）
    last_action,                             # 10: 前回アクション
    last_last_action,                        # 10: 前々回アクション
    last_last_last_action,                   # 10: 前々々回アクション
    motor_targets,                           # 10: 現在のモータ目標位置
    contact,                                 # 2: 足の接地状態 [bool]
    imitation_phase,                         # 2: 参照歩容の位相 [cos, sin]
]
# 合計: 73次元（脚10自由度ロボットの場合）
```

### 8.3 観測の正規化

学習中に蓄積されたrunning statistics（平均・標準偏差）がONNXモデル内に埋め込まれる。

```python
# export_onnx.py での正規化層
def call(self, inputs):
    inputs = (inputs - self.mean) / self.std  # ← ネットワーク内で正規化
    logits = self.mlp_block(inputs)
    loc, _ = tf.split(logits, 2, axis=-1)
    return tf.tanh(loc)
```

### 8.4 実機デプロイ時の注意

```python
# ref/Open_Duck_Mini/experiments/real_robot/rl_walk.py
pd_action_offset = [0.0, -0.57, 0.52, ...]  # 関節ごとのオフセット
pd_action_scale = [0.98, 1.4, 1.47, ...]    # 関節ごとのスケール

# 実機への指令変換
action = policy.infer(obs)
action = np.clip(action, -1, 1)
motor_target = pd_action_offset + pd_action_scale × action
```

**旧アーキテクチャ**（Isaac Gym）では`pd_action_offset`/`pd_action_scale`を使用していたが、**新アーキテクチャ**（MuJoCo Playground）では`default_actuator + action × action_scale`に統一されている。

---

## 9. BSL-Droidへの適用指針

### 9.1 ハードウェア差分の整理

| 項目 | Open Duck (STS3215) | BSL-Droid (RobStride RS02) |
|---|---|---|
| モータ種別 | ホビーサーボ（ギヤ付きDCモータ） | BLDC（準ダイレクトドライブ） |
| 通信 | TTLシリアル (USB-U2D2) | CAN Bus |
| 制御モード | 位置制御（サーボ内蔵PD） | 位置/速度/電流制御（切替可能） |
| バックラッシュ | 大（ギヤ減速比大） | 小（低減速比） |
| トルク帯域 | 低（3.35 N·m） | 高（12 N·m以上） |
| 制御周波数 | 30〜50 Hz | 200 Hz（CAN経由） |

### 9.2 同定手順のカスタマイズ

1. **データ収集スクリプトの修正**:
   - HWIクラスをRobStride CAN通信インターフェースに置換
   - `ctrl_freq` を200 Hzに変更
   - MuJoCoシーンをBSL-DroidのURDF/MJCFに変更

2. **正弦波パラメータの調整**:
   - RS02の帯域に合わせて `move_freq` を広範囲（1〜50 Hz）で掃引
   - `move_amp` はRS02の関節可動範囲の50%程度に設定

3. **XMLパラメータの初期値設定**:
   - RS02のデータシートから `forcerange`（最大トルク）を設定
   - `armature` はロータ慣性 × ギヤ比² で計算
   - `damping` と `frictionloss` は保守的な値（0.1程度）から開始

4. **バックラッシュ**:
   - RS02は低減速比のため、バックラッシュは小さい可能性が高い
   - 実測で±0.1° 以下であればモデル化不要

5. **Domain Randomization範囲**:
   - 初回同定後の残差に基づいてランダム化範囲を決定
   - PDゲイン: ×U(0.8, 1.2)（RS02は個体差が大きい可能性）
   - 通信遅延: CAN Busのレイテンシを実測（通常1〜5 ms）して遅延ステップ数に反映

---

## 10. 参照ファイル一覧

### 同定関連

| ファイル | 役割 |
|---|---|
| `ref/Open_Duck_Mini/experiments/identification/get_data.py` | 正弦波応答データ収集 |
| `ref/Open_Duck_Mini/experiments/identification/check_speed.py` | 速度応答計測 |
| `ref/Open_Duck_Mini/experiments/identification/plot.py` | 指令値vs応答値の可視化 |
| `ref/Open_Duck_Mini/experiments/identification/plot_speeds.py` | 速度応答の可視化 |
| `ref/Open_Duck_Mini/experiments/identification/plot_obs.py` | 観測値の比較（MuJoCo vs 実機） |
| `ref/Open_Duck_Mini/experiments/identification/utils.py` | 関節ID/名前マッピング・初期姿勢 |

### MuJoCoモデル定義

| ファイル | 役割 |
|---|---|
| `ref/Open_Duck_Playground/.../xmls/joints_properties.xml` | アクチュエータ物理パラメータ定義 |
| `ref/Open_Duck_Playground/.../xmls/open_duck_mini_v2_backlash.xml` | バックラッシュ付きロボットモデル |
| `ref/Open_Duck_Playground/.../xmls/scene_flat_terrain.xml` | 学習シーン（バックラッシュなし） |
| `ref/Open_Duck_Playground/.../xmls/scene_flat_terrain_backlash.xml` | 学習シーン（バックラッシュあり） |

### 学習・推論

| ファイル | 役割 |
|---|---|
| `ref/Open_Duck_Playground/playground/common/randomize.py` | Domain Randomization実装 |
| `ref/Open_Duck_Playground/playground/open_duck_mini_v2/joystick.py` | 学習環境（観測・報酬・アクション処理） |
| `ref/Open_Duck_Playground/playground/open_duck_mini_v2/base.py` | 環境基底クラス（関節管理・backlash処理） |
| `ref/Open_Duck_Playground/playground/open_duck_mini_v2/mujoco_infer.py` | MuJoCo上での推論 |
| `ref/Open_Duck_Playground/playground/open_duck_mini_v2/mujoco_infer_base.py` | 推論基底クラス |
| `ref/Open_Duck_Playground/playground/common/export_onnx.py` | ONNX書き出し（正規化層含む） |
| `ref/Open_Duck_Playground/playground/common/utils.py` | ローパスフィルタ |

### 実機デプロイ

| ファイル | 役割 |
|---|---|
| `ref/Open_Duck_Mini/experiments/real_robot/rl_walk.py` | 実機歩行実行 |
| `ref/Open_Duck_Mini/mini_bdx/mini_bdx/utils/rl_utils.py` | 関節順序変換・アクション変換 |
