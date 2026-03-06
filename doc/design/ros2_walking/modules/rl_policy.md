# biped_rl_policy パッケージ モジュール設計書

## 1. 概要

`biped_rl_policy` は、強化学習（RL）で学習済みのポリシーを用いて、速度指令から関節目標位置を50Hzで推論するament_pythonパッケージである。本パッケージはROS 2ノード `biped_rl_policy_node` を1つだけ提供する。

速度指令（geometry_msgs/Twist）を受け取り、50次元の観測ベクトルを構築してPyTorchモデル（MLP）に入力し、10次元のアクションを関節目標位置に変換して出力する。動作モードとして、RViz可視化用の「vizモード」と実機制御用の「controlモード」を備える。

### ノード内部構造図

![biped_rl_policy_node 内部アーキテクチャ](../fig/ros2_walking_module_biped_rl_policy.drawio.svg)

---

## 2. ノードインタフェース

### 2.1 ノード名

`biped_rl_policy_node`

### 2.2 Subscribe トピック

| トピック名 | メッセージ型 | 用途 | モード |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | テレオペからの速度指令（lin_vel_x, lin_vel_y, ang_vel_yaw） | 共通 |
| `/joint_states` | `sensor_msgs/JointState` | エンコーダフィードバック（実関節角度・速度） | controlのみ |
| `/imu/data` | `sensor_msgs/Imu` | IMUデータ（角速度・姿勢クォータニオン） | controlのみ |

### 2.3 Publish トピック

| トピック名 | メッセージ型 | 用途 | モード |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | robot_state_publisher向けの関節状態（可視化用） | vizのみ |
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | ros2_controlへの関節目標位置指令 | controlのみ |
| `/rl_policy_state` | `biped_msgs/RLPolicyState` | デバッグ用テレメトリ（観測・行動・推論時間） | 共通 |

### 2.4 ROSパラメータ

| パラメータ名 | 型 | デフォルト | 説明 |
|---|---|---|---|
| `mode` | string | `viz` | 動作モード。`viz`（可視化）または `control`（実機制御） |
| `model_path` | string | （必須） | 学習済みモデルファイルのパス（`.pt`） |
| `config_path` | string | （必須） | 学習設定ファイルのパス（`cfgs.pkl`） |

---

## 3. 動作モード

### 3.1 vizモード（Phase 1）

物理ハードウェアを使用しない可視化専用モードである。

- `/joint_states` を publish し、`robot_state_publisher` 経由でRViz上にロボットの姿勢を表示する。
- IMUデータとエンコーダデータはモック値で代替する。
  - `base_lin_vel` = [0, 0, 0]
  - `base_ang_vel` = [0, 0, 0]
  - `projected_gravity` = [0, 0, -1]
  - `contact_state` = [1, 1]（常時両足接地）
- 関節位置は前回のポリシー出力をフィードバックして使用する。
- 関節速度は前回出力との有限差分で算出する。

### 3.2 controlモード（Phase 3）

実機制御モードである。

- `/forward_position_controller/commands` に `std_msgs/Float64MultiArray` を publish し、ros2_controlの `ForwardCommandController` を介してアクチュエータを駆動する。
- `/joint_states` から実エンコーダデータを取得して観測ベクトルを構築する。
- `/imu/data` から実IMUデータを取得して以下の観測値を算出する。
  - `base_ang_vel`: IMUジャイロスコープの角速度
  - `projected_gravity`: IMUクォータニオンから算出
  - `base_lin_vel`: IMU加速度の積分または運動学推定
- 足接地状態は関節トルクから推定する（力センサなし）。

### 3.3 モード選択

ROSパラメータ `mode` により起動時に決定する。実行中の動的切替は行わない。

---

## 4. 観測ベクトル（50次元）

観測ベクトルは学習環境 `droid_env_unitree.py` の `_update_observation()` メソッド（L714-730）と完全に一致させる必要がある。以下に各要素の詳細を示す。

### 4.1 観測要素一覧

| インデックス | 要素名 | 次元 | vizモードでのソース | controlモードでのソース | スケーリング |
|---|---|---|---|---|---|
| [0:3] | `base_lin_vel` | 3 | [0, 0, 0]（モック） | IMU積分/運動学推定 | `obs_scales.lin_vel` = 2.0 |
| [3:6] | `base_ang_vel` | 3 | [0, 0, 0]（モック） | IMUジャイロスコープ | `obs_scales.ang_vel` = 0.25 |
| [6:9] | `projected_gravity` | 3 | [0, 0, -1]（モック） | IMUクォータニオンから算出 | なし（スケーリング不要） |
| [9:12] | `commands` | 3 | `/cmd_vel` から取得 | `/cmd_vel` から取得 | `commands_scale`（後述） |
| [12:22] | `dof_pos - default_dof_pos` | 10 | 前回出力 - default | エンコーダ - default | `obs_scales.dof_pos` = 1.0 |
| [22:32] | `dof_vel` | 10 | 有限差分で算出 | エンコーダ速度 | `obs_scales.dof_vel` = 0.05 |
| [32:42] | `actions` | 10 | 前回のアクション出力 | 前回のアクション出力 | なし |
| [42] | `gait_phase_sin` | 1 | sin(2 * pi * phase) | sin(2 * pi * phase) | なし |
| [43] | `gait_phase_cos` | 1 | cos(2 * pi * phase) | cos(2 * pi * phase) | なし |
| [44:46] | `leg_phase` | 2 | gait_phaseから導出 | gait_phaseから導出 | なし |
| [46:48] | `feet_pos_z` | 2 | FK計算または定数 | FK計算または定数 | なし |
| [48:50] | `contact_state` | 2 | [1, 1]（モック） | 関節トルクから推定 | なし |

合計: 3 + 3 + 3 + 3 + 10 + 10 + 10 + 1 + 1 + 2 + 2 + 2 = **50次元**

### 4.2 commands_scale

`commands_scale` はコマンド入力をスケーリングするベクトルであり、`obs_scales` から構成される。

```python
commands_scale = [obs_scales["lin_vel"], obs_scales["lin_vel"], obs_scales["ang_vel"]]
              # = [2.0, 2.0, 0.25]
```

対応する学習設定値（`droid_train_omni_v21.py`）:

```python
obs_scales = {
    "lin_vel": 2.0,
    "ang_vel": 0.25,
    "dof_pos": 1.0,
    "dof_vel": 0.05,
}
```

`/cmd_vel` から取得した `[lin_vel_x, lin_vel_y, ang_vel_yaw]` に `commands_scale` を要素ごとに乗じた値が観測ベクトルの `[9:12]` に格納される。

### 4.3 projected_gravity の算出（controlモード）

IMUから取得したクォータニオン `q` を用いて、ワールド座標系の重力ベクトル `[0, 0, -1]` をボディローカル座標系に変換する。

```python
gravity_world = [0, 0, -1]
projected_gravity = transform_by_quat(gravity_world, inv_quat(q))
```

### 4.4 leg_phase

`leg_phase` は左右の脚それぞれの位相を示す2次元ベクトルである。左脚はgait_phaseそのものに基づき、右脚はpi（半周期）のオフセットを持つ。

### 4.5 feet_pos_z

左右の足先のZ座標（高さ）である。vizモードではFK（順運動学）による算出または定数値を使用する。controlモードでは同様にFKから推定する。

### 4.6 contact_state

左右の足の接地状態を示す2次元ベクトルである。vizモードでは常に `[1, 1]`（両足接地）とする。controlモードでは関節トルク等から推定する。

---

## 5. アクションから関節目標位置への変換

### 5.1 変換式

ポリシーの出力（10次元アクション）から関節目標位置への変換は以下の式で行う。

```
target_position = action * action_scale + default_dof_pos
```

この式は `droid_env_unitree.py` L487 に対応する。

### 5.2 action_scale

```
action_scale = 0.25  (rad, 約14度)
```

出典: `droid_train_omni_v21.py` の `env_cfg["action_scale"]`（L172）

### 5.3 default_dof_pos（デフォルト関節角度）

出典: `droid_train_omni_v21.py` L119-150

| 関節名 | 角度（度） | 角度（rad） |
|---|---|---|
| hip_yaw（左右共通） | 0 | 0.0 |
| hip_roll（左右共通） | 0 | 0.0 |
| hip_pitch（左右共通） | 60 | 1.047 |
| knee_pitch（左右共通） | -100 | -1.745 |
| ankle_pitch（左右共通） | 45 | 0.785 |

### 5.4 関節順序

ポリシーの出力は以下の順序で10個の関節に対応する。この順序は学習環境の `env_cfg["joint_names"]` と一致する。

| インデックス | 関節名 | ROS 2 joint名 |
|---|---|---|
| 0 | L_hip_yaw | `left_hip_yaw_joint` |
| 1 | L_hip_roll | `left_hip_roll_joint` |
| 2 | L_hip_pitch | `left_hip_pitch_joint` |
| 3 | L_knee | `left_knee_pitch_joint` |
| 4 | L_ankle | `left_ankle_pitch_joint` |
| 5 | R_hip_yaw | `right_hip_yaw_joint` |
| 6 | R_hip_roll | `right_hip_roll_joint` |
| 7 | R_hip_pitch | `right_hip_pitch_joint` |
| 8 | R_knee | `right_knee_pitch_joint` |
| 9 | R_ankle | `right_ankle_pitch_joint` |

---

## 6. モデルのロード

### 6.1 モデルファイル

- パス形式: `rl_ws/logs/droid-walking-omni-v{N}/model_{iter}.pt`
- 設定ファイル: `rl_ws/logs/droid-walking-omni-v{N}/cfgs.pkl`

`cfgs.pkl` には学習時の全設定（`env_cfg`, `obs_cfg`, `reward_cfg`, `command_cfg`, `train_cfg`）が辞書形式で保存されている。ノード起動時にこのファイルから `obs_scales`, `action_scale`, `default_joint_angles`, `gait_frequency` 等を復元する。

### 6.2 ネットワーク構造

`rsl_rl` ライブラリの `ActorCritic` クラスを使用する。

- 入力: 50次元（観測ベクトル）
- 隠れ層: 512 - 256 - 128（3層MLP）
- 出力: 10次元（アクション）
- 活性化関数: 学習設定 `train_cfg["policy"]["activation"]` に従う

### 6.3 ロード手順

```python
import pickle
import torch
from rsl_rl.modules import ActorCritic

# 設定ファイルの読み込み
with open(config_path, "rb") as f:
    cfgs = pickle.load(f)
    env_cfg = cfgs["env_cfg"]
    obs_cfg = cfgs["obs_cfg"]
    train_cfg = cfgs["train_cfg"]

# モデルの構築とロード
policy = ActorCritic(
    num_actor_obs=obs_cfg["num_obs"],
    num_critic_obs=obs_cfg["num_obs"],
    num_actions=env_cfg["num_actions"],
    actor_hidden_dims=train_cfg["policy"]["actor_hidden_dims"],
    critic_hidden_dims=train_cfg["policy"]["critic_hidden_dims"],
    activation=train_cfg["policy"]["activation"],
)
state_dict = torch.load(model_path, map_location="cpu", weights_only=True)
policy.load_state_dict(state_dict["model_state_dict"])
policy.eval()
```

### 6.4 ROSパラメータによるパス指定

- `model_path`: `.pt` ファイルへの絶対パスまたは相対パス
- `config_path`: `cfgs.pkl` ファイルへの絶対パスまたは相対パス

これらはlaunchファイルから設定する。

---

## 7. 歩行位相クロック

### 7.1 概要

内部に歩行位相クロックを保持し、50Hzのタイマーコールバック毎に位相を進行させる。歩行位相は観測ベクトルの `gait_phase_sin`, `gait_phase_cos`, `leg_phase` の算出に使用される。

### 7.2 位相更新式

```
phase += gait_frequency * dt
```

- `dt` = 1/50 = 0.02 [s]
- `gait_frequency`: 学習設定から取得（`reward_cfg["gait_frequency"]`）。`droid_train_omni_v21.py` では 1.5 [Hz]。
- `phase` は [0, 1) の範囲で循環する（1.0を超えたら1.0を減算）。

### 7.3 位相から観測値への変換

```python
gait_phase_sin = sin(2 * pi * phase)    # [42]
gait_phase_cos = cos(2 * pi * phase)    # [43]
leg_phase = [f(phase), f(phase + 0.5)]  # [44:46] 左脚, 右脚（半周期オフセット）
```

右脚は左脚に対して pi（0.5周期）のオフセットを持ち、交互歩行パターンを生成する。

---

## 8. 内部処理ループ（50Hz タイマー）

50Hz（20ms周期）のROSタイマーコールバックとして以下の処理を順次実行する。

### 8.1 処理フロー

1. **速度指令の取得**: 最新の `/cmd_vel` メッセージから `[lin_vel_x, lin_vel_y, ang_vel_yaw]` を読み取る。
2. **関節状態の取得**:
   - vizモード: 前回のポリシー出力値を使用する。
   - controlモード: 最新の `/joint_states` メッセージからエンコーダ値を読み取る。
3. **IMUデータの取得**:
   - vizモード: モック値を使用する（`base_lin_vel=[0,0,0]`, `base_ang_vel=[0,0,0]`, `projected_gravity=[0,0,-1]`）。
   - controlモード: 最新の `/imu/data` メッセージから角速度・姿勢を読み取る。
4. **50次元観測ベクトルの構築**: 第4章に記載した仕様に従い、全50要素を結合する。
5. **ポリシー推論**: `model.eval()` 状態で `torch.no_grad()` コンテキスト内にてforward passを実行する。
6. **アクション→関節位置変換**: `target_position = action * action_scale + default_dof_pos`
7. **出力のPublish**:
   - vizモード: `/joint_states`（`sensor_msgs/JointState`）をpublishする。
   - controlモード: `/forward_position_controller/commands`（`std_msgs/Float64MultiArray`）をpublishする。
8. **デバッグテレメトリの送信**: `/rl_policy_state`（`biped_msgs/RLPolicyState`）をpublishする。観測ベクトル、アクション、関節目標位置、推論時間を含む。

### 8.2 タイミング制約

- ポリシー推論のレイテンシは10ms以下を目標とする（NFR-01）。
- PyTorch MLPの50Hz推論はCPUのみで十分達成可能である。GPU（CUDA）は使用しない。
- Jetson Orin Nano Super（ARM64, 8GB RAM）での動作を前提とする。

### 8.3 内部状態バッファ

ノードは以下の内部状態を保持する。

| 状態変数 | 型 | 初期値 | 用途 |
|---|---|---|---|
| `prev_actions` | float[10] | 全て0.0 | 前回アクション（観測ベクトル[32:42]） |
| `prev_dof_pos` | float[10] | `default_dof_pos` | 前回関節位置（速度算出用） |
| `current_dof_pos` | float[10] | `default_dof_pos` | 現在関節位置 |
| `phase` | float | 0.0 | 歩行位相クロック |
| `latest_cmd_vel` | float[3] | [0, 0, 0] | 最新の速度指令 |
| `latest_imu` | Imu | （モック値） | 最新のIMUデータ |

---

## 9. 参照ファイル一覧

| ファイルパス | 参照箇所 | 内容 |
|---|---|---|
| `rl_ws/biped_walking/envs/droid_env_unitree.py` L714-730 | 第4章 | 観測ベクトルの構築処理 |
| `rl_ws/biped_walking/envs/droid_env_unitree.py` L487 | 第5章 | アクション→関節位置変換式 |
| `rl_ws/biped_walking/envs/droid_env_unitree.py` L295-298 | 第4.2節 | `commands_scale` の定義 |
| `rl_ws/biped_walking/biped_eval_gamepad.py` | 全体 | ゲームパッド評価ループの参考実装 |
| `rl_ws/biped_walking/train/droid_train_omni_v21.py` L119-122 | 第5.3節 | デフォルト関節角度の定義 |
| `rl_ws/biped_walking/train/droid_train_omni_v21.py` L124-189 | 第4.2節, 第5.2節 | `env_cfg`, `obs_cfg` の定義 |
