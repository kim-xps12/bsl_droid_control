# EXP002: BSL-Droid二脚ロボット sim2sim実験計画

## 概要

BSL-Droid二脚ロボット（Digitigrade/逆関節型）に対して、Genesis物理シミュレータで強化学習により歩容を獲得し、MuJoCoで汎化性能を検証するsim2sim実験を実施する。

## 実験方針

### バージョン管理とコード信頼性

強化学習環境とトレーニングスクリプトのバージョン管理において、以下の原則を厳守する：

1. **完全コピー原則**: 新バージョン（V(n+1)）を作成する際は、前バージョン（Vn）のスクリプトを完全にコピーしてから編集を開始する
   ```bash
   # 例: V4からV5を作成する場合
   cp biped_env_v4.py biped_env_v5.py
   cp biped_train_v4.py biped_train_v5.py
   ```

2. **最小変更原則**: コピー後は、改善目的に必要な箇所**のみ**を編集する
   - クラス名・docstring・報酬関数・報酬スケールなど、変更が必要な部分だけを特定
   - 既存の動作検証済みコードは可能な限り保持
   - 不必要な再実装や構造変更を避ける

3. **段階的検証**: 各バージョンで以下を確認
   - 構文エラーがないこと（実行前に検証）
   - 依存パラメータ（command_cfg等）の整合性
   - 初期化処理の完全性（qpos次元数等）

4. **理由**: 
   - ゼロから再実装すると、細かい設定ミス（qpos次元、command_cfg、feet_indices初期化等）が混入しやすい
   - 既存の動作確認済みコードをベースにすることで、新規追加部分のデバッグに集中できる
   - 信頼性の高い反復的改善が可能

### 実験記録

- 各バージョンの目的、変更内容、結果を本ドキュメントに記録
- 失敗した試みも含めて記述し、次の改善に活かす

## 背景

- 既存環境: `rl_ws/`にてUnitree Go2（四脚12DOF）のsim2simパイプラインが確立済み
- 対象ロボット: `ros2_ws/src/biped_description/urdf/biped_digitigrade.urdf.xacro`
- ゴール: 二脚歩行ポリシーの獲得とMuJoCoでの検証

## ロボット仕様比較

| 項目 | Unitree Go2 | BSL-Droid Biped |
|------|-------------|-----------------|
| 脚数 | 4脚 | 2脚 |
| DOF | 12 (片脚3関節×4) | 10 (片脚5関節×2) |
| 関節構成 | hip, thigh, calf | hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch |
| 膝構造 | 標準（後方屈曲） | 逆関節（前方屈曲） |
| 総質量 | 約12kg | 約6.2kg |
| 脚長 | 約0.4m | 約0.53m (thigh+shank+foot) |

### BSL-Droid関節詳細

```
左脚 (left):
  - left_hip_yaw_joint    (Z軸, ±30°)
  - left_hip_roll_joint   (X軸, ±20°)
  - left_hip_pitch_joint  (Y軸, ±90°)
  - left_knee_pitch_joint (Y軸, -120°〜0°, 逆関節)
  - left_ankle_pitch_joint(Y軸, -45°〜+135°)

右脚 (right):
  - right_hip_yaw_joint   (同上)
  - right_hip_roll_joint
  - right_hip_pitch_joint
  - right_knee_pitch_joint
  - right_ankle_pitch_joint

センサ:
  - imu_link (base_link上面中央, fixed joint)
```

## 実験フェーズ

### Phase 1: 環境準備

#### 1.1 URDFエクスポート・検証

- [ ] 既存の二脚ロボットのURDFに与えられた全ての物理パラメータが現実に即したものであるかをレビューして必要に応じて適切に修正する
- [ ] `rl_ws/assets/export_urdf.py`を使用してURDFをエクスポート
- [ ] エクスポートしたURDFの関節・リンク構造を確認
- [ ] MuJoCoでのロード可否を検証

```bash
cd rl_ws
uv run python assets/export_urdf.py
```

#### 1.2 URDF→MJCF変換スクリプト作成

- [ ] `scripts/convert_biped_urdf_to_mjcf.py`を新規作成
  - Go2用スクリプトをベースに改変
  - 地面との接触設定
  - freejoint追加
  - アクチュエータ定義（10関節分）
  - 初期姿勢（home keyframe）設定

#### 1.3 成果物

- `rl_ws/assets/biped_digitigrade.urdf` (エクスポート済みURDF)
- `rl_ws/assets/biped_digitigrade.xml` (変換済みMJCF)

---

### Phase 2: Genesis訓練環境構築

#### 2.1 環境クラス作成

`rl_ws/genesis_official/examples/locomotion/biped_env.py`を新規作成:

```python
# 主要な変更点:
- num_actions: 12 → 10
- num_obs: 45 → 39 (3+3+3+10+10+10)
- joint_names: 10関節リスト
- default_joint_angles: 二脚立位姿勢
- URDFパス: biped_digitigrade.urdf
- base_init_pos: [0, 0, 0.5] (脚長に応じて調整)
```

#### 2.2 報酬関数設計（二脚特化）

| 報酬項目 | 説明 | スケール |
|---------|------|---------|
| tracking_lin_vel | 前進速度追従 | 1.0 |
| tracking_ang_vel | 回転速度追従 | 0.2 |
| lin_vel_z | 上下動ペナルティ | -1.0 |
| base_height | 高さ維持 | -50.0 |
| action_rate | 動作滑らかさ | -0.005 |
| similar_to_default | 初期姿勢維持 | -0.1 |
| **feet_alternating** | 左右交互接地 | **1.0** (新規) |
| **balance** | CoMバランス | **0.5** (新規) |

#### 2.3 初期姿勢設定

逆関節の特性を考慮した立位姿勢:

```python
default_joint_angles = {
    "left_hip_yaw_joint": 0.0,
    "left_hip_roll_joint": 0.0,
    "left_hip_pitch_joint": 0.0,     # 股関節前傾なし
    "left_knee_pitch_joint": -0.52,  # 約-30°（軽く前方屈曲）
    "left_ankle_pitch_joint": 0.52,  # 膝と相殺
    "right_hip_yaw_joint": 0.0,
    "right_hip_roll_joint": 0.0,
    "right_hip_pitch_joint": 0.0,
    "right_knee_pitch_joint": -0.52,
    "right_ankle_pitch_joint": 0.52,
}
```

#### 2.4 成果物

- `rl_ws/genesis_official/examples/locomotion/biped_env.py`
- `rl_ws/genesis_official/examples/locomotion/biped_train.py`
- `rl_ws/genesis_official/examples/locomotion/biped_eval.py`

---

### Phase 3: 訓練実行

#### 3.1 訓練パラメータ

```python
train_cfg = {
    "max_iterations": 500,      # 初期検証
    "num_envs": 4096,           # 並列環境数
    "num_steps_per_env": 24,
    "save_interval": 100,
}
```

#### 3.2 訓練実行

```bash
cd rl_ws

# 初期訓練（500イテレーション）
uv run python genesis_official/examples/locomotion/biped_train.py \
    --exp_name biped-walking \
    --max_iterations 500

# Genesisビューアで評価
uv run python genesis_official/examples/locomotion/biped_eval.py
```

#### 3.3 TensorBoard監視

```bash
cd rl_ws
uv run tensorboard --logdir logs/biped-walking
```

#### 3.4 チェックポイント

訓練途中で以下を確認:
- 100イテレーション: 転倒しなくなる
- 300イテレーション: 前進開始
- 500イテレーション: 安定歩行

---

### Phase 4: MuJoCo sim2sim検証

#### 4.1 評価スクリプト作成

`rl_ws/scripts/biped_eval_mujoco.py`を新規作成:

```python
# Go2版との主要な差分:
- num_obs: 39
- num_actions: 10
- joint_mapping: 二脚用
- default_dof_pos: 二脚立位
```

#### 4.2 評価実行

```bash
cd rl_ws

# ビューア付きで評価
uv run mjpython scripts/biped_eval_mujoco.py --duration 10

# 数値比較
uv run python scripts/biped_eval_mujoco.py --no-viewer --duration 5
```

#### 4.3 成果物

- `rl_ws/scripts/biped_eval_mujoco.py`
- `rl_ws/logs/biped-walking/` (チェックポイント)

---

### Phase 5: 結果分析・文書化

#### 5.1 評価指標

| 指標 | 目標値 | Genesis | MuJoCo | 差分許容 |
|------|---------|--------|---------|---------|
| 前進速度 [m/s] | - | - | - | ±20% |
| 転倒率 [%] | - | - | - | +10%以内 |
| 歩容周期 [s] | - | - | - | ±10% |

#### 5.2 文書化

- 実験結果をこのドキュメントに追記
- 図・グラフを`doc/experiments/`に保存

---

## ファイル構成（予定）

```
rl_ws/
├── assets/
│   ├── biped_digitigrade.urdf      # Phase 1で生成
│   └── biped_digitigrade.xml       # Phase 1で生成
├── genesis_official/examples/locomotion/
│   ├── biped_env.py                # Phase 2で作成
│   ├── biped_train.py              # Phase 2で作成
│   └── biped_eval.py               # Phase 2で作成
├── scripts/
│   ├── convert_biped_urdf_to_mjcf.py  # Phase 1で作成
│   └── biped_eval_mujoco.py           # Phase 4で作成
└── logs/
    └── biped-walking/              # Phase 3で生成
        ├── model_*.pt
        └── cfgs.pkl
```

---

## リスクと対策

### R1: 二脚歩行の学習困難性

- **リスク**: 四脚と比較して動的バランスが困難
- **対策**: 
  - カリキュラム学習（静止→歩行）
  - バランス報酬の追加
  - 訓練イテレーション増加（500→2000）

### R2: 逆関節の特殊性

- **リスク**: 報酬設計がGo2と異なる可能性
- **対策**: 
  - 膝関節角度に対する報酬調整
  - 足接地パターンの監視

### R3: URDF→MJCF変換エラー

- **リスク**: メッシュ/慣性パラメータの不整合
- **対策**: 
  - `compare_models.py`相当のツールで検証
  - 慣性テンソル手動修正

---

## 承認事項

以下の内容で実験を進めてよいか確認をお願いします:

1. **実験スコープ**: Phase 1〜5の全工程を実施
2. **リソース**: `rl_ws/`への新規ファイル追加
3. **訓練時間**: 初期検証として500イテレーション（約30分〜1時間）
4. **優先度**: Phase 1-2を先行し、動作確認後にPhase 3以降へ進行

---

## 参考資料

- [rl_ws/README.md](../../rl_ws/README.md) - 既存sim2sim環境
- [biped_description](../../ros2_ws/src/biped_description/) - ロボットURDF
- [Genesis Locomotionチュートリアル](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/locomotion.html)

---

## 実験実行記録

本セクションでは、実験の実施過程、試行錯誤、および各バージョンでの改善内容を時系列で記録する。


### V1: 初期実装と基本歩行の獲得

#### 実施内容

1. **環境構築**
   - `biped_env.py`: Go2環境をベースに10DOF二脚用に改変
   - `biped_train.py`: PPOトレーニングスクリプト
   - `biped_eval.py`: 評価スクリプト（後に統一版に整理）

2. **報酬設定（V1）**
   ```python
   reward_scales = {
       "tracking_lin_vel": 1.0,
       "tracking_ang_vel": 0.2,
       "lin_vel_z": -1.0,
       "base_height": -50.0,
       "action_rate": -0.005,
       "similar_to_default": -0.1,
       "orientation": -5.0,
   }
   ```

3. **トレーニング実行**
   - 500イテレーション、4096並列環境
   - macOS Metal (MPS) バックエンドで実行

#### 結果

| 指標 | 値 |
|------|-----|
| 最終位置 (x) | 2.074 m |
| action_rms | 0.52 |
| 歩行 | ✅ 成功 |

#### 問題点

- **ジャーキーな動作**: アクションが細かく振動し、実機デプロイに不適
- 関節が高速で小刻みに動き、滑らかさに欠ける
- 「歩く」というよりは「震えながら前進」する状態

#### ユーザーからのフィードバック

> 「もっとゆったり生き物のように尚且つ実機にデプロイしやすいモーションになるように」

---

### V2: 滑らかさ強化（失敗）

#### 改善方針

legged_gymの報酬設計を参考に、滑らかな動作を促進するペナルティを強化：

1. `action_rate`を20倍に強化 (-0.005 → -0.1)
2. `dof_vel`ペナルティを新規追加 (-0.001)
3. `dof_acc`ペナルティを新規追加
4. `smoothness`ペナルティを新規追加

#### 報酬設定（V2）

```python
reward_scales = {
    "tracking_lin_vel": 1.5,
    "tracking_ang_vel": 0.5,
    "action_rate": -0.1,        # V1の20倍
    "dof_vel": -0.001,          # 新規
    "dof_acc": -1e-6,           # 新規
    "smoothness": -0.01,        # 新規
    "feet_air_time": 1.0,       # 新規
    "no_fly": -0.5,             # 新規
    ...
}
```

#### 結果

| 指標 | V1 | V2 |
|------|-----|-----|
| 最終位置 (x) | 2.074 m | **0.014 m** |
| action_rms | 0.52 | 0.36 |
| 歩行 | ✅ | ❌ **失敗** |

#### 問題の分析

**ヘッドレス評価で詳細確認:**

```
Step   0: pos=(-0.000, 0.000, 0.447), vel=(-0.00, 0.00), action_rms=0.520
Step 450: pos=(0.013, 0.008, 0.443), vel=(0.00, 0.00), action_rms=0.358
Final position: x=0.014 m
```

- ロボットは**全く歩かない** - ただ立っているだけ
- dof_vel_rmsが0.01と極めて低い（関節がほぼ動いていない）

**根本原因:**
- ペナルティが過剰で、「動かないことが最も安全」と学習
- `action_rate: -0.1`は前進報酬（tracking_lin_vel: 1.5）を打ち消す
- 「滑らかに歩く」ではなく「全く動かない」が最適解に

#### 教訓

> ペナルティの強化は慎重に。報酬とペナルティのバランスが崩れると、意図しない局所解に収束する。

---

### V3: バランス調整版（部分成功）

#### 改善方針

V2の過剰ペナルティを緩和しつつ、V1より滑らかさを改善：

| 報酬項目 | V1 | V2 | V3 |
|---------|-----|-----|-----|
| tracking_lin_vel | 1.0 | 1.5 | **2.0** |
| action_rate | -0.005 | -0.1 | **-0.02** |
| dof_vel | なし | -0.001 | **-0.0001** |
| smoothness | なし | -0.01 | **削除** |
| action_scale | 0.25 | 0.5 | **0.4** |

**設計意図:**
- `tracking_lin_vel`を強化して前進を最重要視
- `action_rate`はV1の4倍程度に抑える（V2の-0.1は過剰）
- `dof_vel`は大幅緩和（動きを過度に抑制しない）
- `smoothness`は`dof_acc`と重複するため削除

#### 結果

| 指標 | V1 | V2 | V3 |
|------|-----|-----|-----|
| 最終位置 (x) | 2.074 m | 0.014 m | **2.647 m** ✅ |
| action_rms | 0.52 | 0.36 | **0.95** |
| 平均前進速度 | - | - | 0.274 m/s |
| 歩行 | ✅ | ❌ | ✅ |

#### 問題点

GUI評価で動作を観察した結果、以下の問題が判明：

1. **すり足歩行**: 足を持ち上げず、地面を滑るように移動
2. **脚の固定化**: 左右の脚が「前足」「後ろ足」として固定され交互に動かない
3. **剣道のすり足状態**: 両足が常に接地したまま前進

#### ユーザーからのフィードバック

> 「ロボットの2本の足が前足と後ろ足に完全に固定されたまま剣道のすり足の動きをしていました。これは私が所望するモーションではありません。人間やduckのようにきちんと脚を交互に前に出して歩かせなさい。」

#### 分析

V3は「前進する」という目標は達成したが、「歩行」の定義を満たしていない：
- 真の歩行 = 左右の脚を交互に持ち上げて前に出す
- V3の動作 = 両足を接地したまま体を前に滑らせる

**原因:**
- `feet_air_time`報酬があっても、交互に動く動機付けがない
- 両足接地状態（すり足）が前進の最も効率的な方法として学習された

---

### V4: 交互歩行報酬の導入（実験中）

#### 改善方針

すり足を防ぎ、人間やアヒルのような交互歩行を実現するため、以下の報酬を新規追加：

1. **`alternating_gait`報酬**: 左右の足が交互に接地することを報酬
   - XOR演算: 片方だけが接地している場合に報酬
   - 両足接地（すり足）または両足離地は0報酬

2. **`foot_swing`報酬**: 足を持ち上げてスイングすることを報酬
   - 足が空中にいる時間を報酬（0.1〜0.3秒が理想）

3. **`single_stance`報酬**: 片足立ち状態を報酬
   - 交互歩行の前提条件として片足支持期を促進

#### 報酬設定（V4）

```python
reward_scales = {
    # 追従報酬
    "tracking_lin_vel": 2.0,
    "tracking_ang_vel": 0.3,
    
    # 交互歩行報酬（新規・重要）
    "alternating_gait": 1.5,     # 左右交互接地
    "foot_swing": 0.8,           # 足のスイング
    "feet_air_time": 1.0,        # 長いストライド
    "no_fly": -1.0,              # 両足同時離地ペナルティ強化
    "single_stance": 0.5,        # 片足立ち
    
    # 滑らかさペナルティ
    "action_rate": -0.02,
    "dof_vel": -0.0001,
    "dof_acc": -1e-7,
    
    # 姿勢・高さ維持
    "orientation": -3.0,
    "base_height": -30.0,
    ...
}
```

#### 新規報酬関数の実装詳細

**`_reward_alternating_gait`:**
```python
def _reward_alternating_gait(self):
    """左右の足が交互に接地していることを報酬"""
    left_contact = contacts[:, 0]
    right_contact = contacts[:, 1]
    
    # XOR: 片方だけが接地している場合に報酬
    alternating = left_contact ^ right_contact
    return alternating.float() * has_command
```

**`_reward_foot_swing`:**
```python
def _reward_foot_swing(self):
    """足が空中にいる時間を報酬（スイング促進）"""
    air_time_reward = torch.clamp(self.feet_air_time, 0.0, 0.3)
    return torch.sum(air_time_reward, dim=1) * has_command
```

**`_reward_single_stance`:**
```python
def _reward_single_stance(self):
    """片足だけが接地している状態を報酬"""
    single_stance = contacts.sum(dim=1) == 1
    return single_stance.float() * has_command
```

#### 期待される効果

| 状態 | alternating_gait | foot_swing | single_stance | 合計 |
|------|------------------|------------|---------------|------|
| 両足接地（すり足）| 0 | 0 | 0 | 0 |
| 左足のみ接地 | 1.5 | + | 0.5 | 高 |
| 右足のみ接地 | 1.5 | + | 0.5 | 高 |
| 両足離地 | 0 | + | 0 (+ no_fly -1.0) | 低 |

すり足状態（両足常時接地）では報酬が得られないため、交互歩行を学習することが期待される。

#### 現在のステータス

- **V4トレーニング**: 実行中（1000イテレーション）
- **使用ファイル**: 
  - `biped_env_v4.py` (交互歩行報酬を含む環境)
  - `biped_train_v4.py` (V4トレーニングスクリプト)

---

### コード整理

実験の過程で、評価スクリプトが複数作成されたため統一を実施：

#### 統合前

```
biped_eval.py      (V1用)
biped_eval_v2.py   (V2用)
biped_eval_v3.py   (V3用)
```

#### 統合後

```
biped_eval.py      (統一版: -e オプションで全バージョン評価可能)
```

**使用方法:**
```bash
cd rl_ws
uv run python genesis_official/examples/locomotion/biped_eval.py -e biped-walking-v4
uv run python genesis_official/examples/locomotion/biped_eval.py -e biped-walking-v3 --ckpt 400
```

---

### バージョン比較サマリー

| バージョン | 実験名 | 移動距離(10s) | 平均速度 | action_rms | 歩行 | 問題点 |
|-----------|--------|--------------|----------|------------|------|--------|
| V1 | biped-walking | 2.074 m | 0.197 m/s | 0.52 | ✅ | ジャーキー、振動 |
| V2 | biped-walking-v2 | 0.014 m | 0.001 m/s | 0.36 | ❌ | 動かない |
| V3 | biped-walking-v3 | 2.647 m | 0.274 m/s | 0.95 | ✅ | すり足、脚固定 |
| V4 | biped-walking-v4 | - | - | ~1.15 | ✅ | 交互歩行だが歩幅小 |
| V5 | biped-walking-v5 | 2.853 m | 0.286 m/s | 1.09 | ⚠️ | feet_namesバグで報酬関数無効 |
| V6 | biped-walking-v6 | - | - | - | 🔄 | **実験中** feet_names修正版 |

---

### V5: 大きな歩幅と高い足上げの実現（失敗）

#### 目的

V4で獲得した交互歩行を維持しつつ、以下を改善：
1. **歩幅の拡大**: 小刻みな動きから、ゆったりとした大きな歩幅へ
2. **足の持ち上げ強化**: 足のクリアランス（地面からの高さ）を増加

#### トレーニングパラメータ

- イテレーション: 500（V4の半分で評価）
- 並列環境数: 4096
- エピソード長: 20秒
- V4からの完全コピー＋最小変更原則適用 ✅

#### 新規報酬関数

**`foot_clearance`報酬（係数1.0）:**
- 遊脚時に足を10cm以上持ち上げることを促進
- 空中の足のみ評価、接地中は無視

**`stride_length`報酬（係数0.8）:**
- 前回接地位置から20cm以上の歩幅を奨励
- 大きな前進動作を促進

#### 評価結果（model_499.pt, 500イテレーション）

**数値データ（10秒間、GUIなし）:**
```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 2.793 m
  Y: 0.580 m
  Total: 2.853 m

Average velocity:
  X: 0.286 m/s (target: 0.300)
  Y: 0.001 m/s

Base height:
  Mean: 0.403 m
  Std: 0.0063 m

DOF velocity RMS: 1.024 rad/s
Action RMS: 1.093
```

**V3/V4との比較:**

| 指標 | V3 | V4 | V5 | 改善 |
|------|-----|-----|-----|------|
| 移動距離(10s) | 2.647 m | - | 2.853 m | +7.8% |
| 平均速度 | 0.274 m/s | - | 0.286 m/s | +4.4% |
| Y方向ドリフト | - | - | 0.580 m | 要改善 |
| Base高さ安定性 | - | - | 0.0063 m (std) | ✅ |
| action_rms | 0.95 | ~1.15 | 1.09 | 中間 |

#### トレーニングログ分析（最終イテレーション499）

```
Mean total reward: 40.38
Mean episode rew_tracking_lin_vel: 0.7724
Mean episode rew_foot_clearance: 0.0000    ⚠️ 学習失敗
Mean episode rew_stride_length: 0.0000     ⚠️ 学習失敗
Mean episode rew_alternating_gait: 0.0000  ⚠️ 学習失敗
Mean episode rew_foot_swing: 0.0000
Mean episode rew_feet_air_time: 0.0000
```

#### 問題点の診断

**致命的な問題: 新規報酬が全く学習されていない**

1. **`foot_clearance = 0.0000`**: 足の持ち上げ報酬が機能していない
2. **`stride_length = 0.0000`**: 歩幅報酬が機能していない
3. **V4の交互歩行報酬も退化**: `alternating_gait = 0.0000`（V4では機能していたはず）

**原因仮説:**

1. **報酬関数の実装バグ**: 
   - `_get_foot_heights()`や`_get_foot_positions()`が正しく動作していない可能性
   - `last_feet_pos`の更新タイミングが不適切

2. **報酬スケールの不均衡**:
   - 既存の`tracking_lin_vel`報酬（0.77）が支配的
   - 新規報酬の係数（1.0, 0.8）が相対的に弱い可能性

3. **観測空間の不足**:
   - 足の位置情報が観測に含まれていない
   - ポリシーが足の動きを制御できない

4. **報酬の相反**:
   - `base_height`ペナルティ（-30.0）が足を上げる動作を阻害？
   - `action_rate`ペナルティが大きな動きを抑制？

#### ユーザーフィードバック（実機GUI評価）

> 「すり足もマシになってきましたね。しかしまだ歩幅が小刻みです。もっとゆったり大きな歩幅にしたい。更には足の持ち上げももうちょっと欲しいね。」

**視覚的観察:**
- ✅ すり足は改善傾向（V3よりマシ）
- ❌ 歩幅は依然として小刻み
- ❌ 足の持ち上げは不十分
- ❌ 左右の脚の交差が甘い

**結論**: V5の改善は数値的にわずか（+7.8%）だが、視覚的には顕著な改善なし。新規報酬関数が学習されていないため、V4からの本質的な進歩がない。

---

### V6: feet_namesバグ修正

#### 根本原因の特定

デバッグスクリプトにより、V5の報酬関数が全てゼロだった原因が判明：

```
=== feet_indices ===
feet_indices: None          ← ⚠️ 足のリンクが見つかっていない！
feet_names: ['left_toe', 'right_toe']

=== Reward function debug ===
contacts: None
foot_heights: None
foot_positions shape: None
```

**原因**: 設定で`feet_names: ["left_toe", "right_toe"]`としていたが、URDFの実際のリンク名は：
- `left_foot_link`
- `right_foot_link`

リンク名が一致しないため、`feet_indices`が`None`となり、すべての足関連報酬がゼロを返していた。

#### V6の修正内容

V5をコピーして以下のみ変更（完全コピー原則に従う）：

1. **`feet_names`の修正**:
   ```python
   # V5（間違い）
   "feet_names": ["left_toe", "right_toe"]
   
   # V6（正しい）
   "feet_names": ["left_foot_link", "right_foot_link"]
   ```

2. **その他の変更なし**: 報酬スケール、観測空間は維持

#### 期待される効果

V6では以下の報酬関数が正常に動作するようになる：

| 報酬関数 | V5での値 | V6での期待 |
|---------|---------|-----------|
| `foot_clearance` | 0.0000 | > 0（足の高さに応じて） |
| `stride_length` | 0.0000 | > 0（歩幅に応じて） |
| `alternating_gait` | 0.0000 | > 0（交互接地時） |
| `foot_swing` | 0.0000 | > 0（滞空時間に応じて） |
| `single_stance` | 0.0000 | > 0（片足支持時） |
| `feet_air_time` | 0.0000 | > 0（着地時） |

#### トレーニング実行

```bash
cd rl_ws
uv run python genesis_official/examples/locomotion/biped_train_v6.py --max_iterations 500
```

#### 評価

```bash
cd rl_ws
uv run python genesis_official/examples/locomotion/biped_eval.py -e biped-walking-v6 --no-viewer --duration 10
```

---

#### 改善方針（V7以降の候補）

V6でバグが修正され報酬が機能するようになった後、以下の調整を検討：

**優先度1: 報酬スケール再調整**
- `foot_clearance`: 1.0 → 2.0〜3.0（強化）
- `stride_length`: 0.8 → 1.5〜2.0（強化）
- `tracking_lin_vel`: 2.0 → 1.5（弱化、他報酬とのバランス）

**優先度2: 観測空間の拡張**
- 足のXY位置を観測に追加（2×2 = 4次元）
- 観測次元: 39 → 43

**優先度3: 相反する報酬の緩和**
- `base_height`: -30.0 → -10.0（足上げを阻害しないように）
- `action_rate`: -0.02 → -0.01（大きな動きを許容）

---

### V6: feet_namesバグ修正版

#### 評価結果

```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 0.475 m
  Y: -1.008 m
  Total: 1.115 m

Average velocity:
  X: 0.121 m/s (target: 0.300)
  Y: 0.015 m/s

Base height:
  Mean: 0.440 m
  Std: 0.0028 m
```

#### V4との比較

| 指標 | V4 | V6 | 評価 |
|------|-----|-----|------|
| X方向速度 | 0.286 m/s | 0.121 m/s | V4が2.4倍速い |
| ベース高さ | 0.403 m | 0.440 m | V6が3.7cm高い |
| 高さ安定性（Std） | 0.0063 m | 0.0028 m | **V6が2.3倍安定** |

#### 結論

バグ修正により足関連報酬が機能するようになったが、前進速度が大幅に低下。
報酬バランスの調整が必要。

---

### V7: Phase-based Reference軌道追従

#### 目的

ROS2のbiped_gait_controlから移植した楕円弧軌道を参照として、滑らかな周期歩行を実現する。

#### 新規追加要素

1. **Phase信号の観測追加**: 39次元 → 43次元
   - `sin(phase)`, `cos(phase)`, `sin(2*phase)`, `cos(2*phase)`

2. **CamberTrajectory クラス**: 楕円弧軌道生成
   - Stance相（0.0-0.5）: 線形接地移動
   - Swing相（0.5-1.0）: 半楕円リフト

3. **新報酬関数**:
   - `trajectory_tracking`（係数5.0）: 足先位置の参照軌道追従
   - `phase_consistency`（係数2.0）: 位相と接地状態の一致

#### 歩容パラメータ

```python
"gait_cfg": {
    "step_height": 0.04,      # 足の持ち上げ高さ [m]
    "step_length": 0.08,      # 歩幅 [m]
    "step_frequency": 0.8,    # 歩行周波数 [Hz]
}
```

#### 評価結果（時系列データ）

```
t= 0.00s | pos=(-0.000,  0.000, 0.447) | vel=(-0.00,  0.00) | rpy=(  0.0,   0.0,   0.0)° | dof_vel_rms= 0.00
t= 1.00s | pos=( 0.161,  0.000, 0.432) | vel=( 0.25, -0.05) | rpy=(  6.8,   0.6,  13.3)° | dof_vel_rms= 0.87
t= 2.00s | pos=( 0.412, -0.002, 0.443) | vel=( 0.23, -0.00) | rpy=(  6.7,  -0.3,  13.1)° | dof_vel_rms= 0.86
t= 3.00s | pos=( 0.662,  0.033, 0.439) | vel=( 0.25, -0.14) | rpy=( -2.9,  -2.8,  16.0)° | dof_vel_rms= 1.62
t= 4.00s | pos=( 0.915,  0.065, 0.438) | vel=( 0.25, -0.02) | rpy=(  0.3,  -3.9,  19.2)° | dof_vel_rms= 0.82
t= 5.00s | pos=( 1.173,  0.100, 0.429) | vel=( 0.38,  0.03) | rpy=(  6.1,  -1.4,  19.0)° | dof_vel_rms= 1.34
t= 6.00s | pos=( 1.423,  0.128, 0.433) | vel=( 0.30, -0.01) | rpy=(  8.2,   2.2,  21.2)° | dof_vel_rms= 0.92
t= 7.00s | pos=( 1.653,  0.170, 0.443) | vel=( 0.22, -0.01) | rpy=(  7.4,   1.1,  22.4)° | dof_vel_rms= 0.84
t= 8.00s | pos=( 1.895,  0.241, 0.439) | vel=( 0.25, -0.14) | rpy=( -2.5,  -3.2,  24.9)° | dof_vel_rms= 1.62
t= 9.00s | pos=( 2.139,  0.309, 0.437) | vel=( 0.26,  0.02) | rpy=(  1.0,  -3.9,  27.7)° | dof_vel_rms= 1.09

=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 2.374 m
  Y: 0.367 m
  Total: 2.402 m

Average velocity:
  X: 0.237 m/s (target: 0.300)
  Y: -0.046 m/s

Base height:
  Mean: 0.437 m
  Std: 0.0036 m

DOF velocity RMS: 1.263 rad/s
Action RMS: 0.809
```

#### V4との比較

| 指標 | V4 | V7 | 評価 |
|------|-----|-----|------|
| X移動距離（10s） | 2.793 m | 2.374 m | V4が18%速い |
| 平均X速度 | 0.286 m/s | 0.237 m/s | 目標の79% |
| Yaw回転（10s） | ~16° | ~28° | **V7がより回転する問題** |
| ベース高さ | 0.403 m | 0.437 m | V7が3.4cm高い |
| 高さ安定性（Std） | 0.0063 m | 0.0036 m | **V7が1.8倍安定** |
| Roll振れ | 1-3° | -3°〜+8° | V7がロール変動大 |
| Action RMS | 1.093 | 0.809 | V7がより滑らか |

#### ユーザーフィードバック（GUI評価）

> 「軽く左足を出してから大きく左足を出し、軽く右足を出してから大きく右足を出す」挙動。4ステップでカクカク歩く感じ。

#### 問題点分析

1. **Yaw回転の蓄積**: 10秒で28°回転（V4は16°）→ 直進性が低い
2. **ロール変動が大きい**: V4は1-3°安定、V7は-3°〜+8°変動 → 「カクカク」の原因
3. **4ステップカクカク現象**: 
   - step_frequency=0.8Hzに固定されているため、ポリシーがこの周期に強制的に従う
   - 1サイクル = 1.25秒 = 約62.5ステップ
   - 左小→左大→右小→右大の4段階動作になっている

#### 根本原因の考察

Phase-based Reference方式は「固定周期」を前提としているが：
- ロボットの実際の動力学と軌道生成周期が不一致
- 強制的な周期に従おうとしてバランスを崩す
- 各半周期で2段階の動作（準備動作→本動作）が発生

---

### V8: Yaw補正・ロール安定化・新規報酬関数追加（失敗）

#### 目的

V7の問題（Yaw回転蓄積、4ステップカクカク）を解決するため：
1. パラメータ調整（Yaw補正強化、ロール安定化）
2. 新規報酬関数の追加（symmetric_gait, smooth_joint_velocity, heading_alignment）

#### V7からの変更点

**パラメータ調整:**

| パラメータ | V7 | V8 | 目的 |
|-----------|-----|-----|------|
| tracking_ang_vel | 0.3 | 1.5 | Yaw回転抑制 |
| orientation | -3.0 | -8.0 | ロール変動抑制 |
| ang_vel_xy | -0.05 | -0.2 | XY角速度変動抑制 |
| trajectory_tracking | 5.0 | 3.0 | 速度追従とのバランス |
| phase_consistency | 2.0 | 1.5 | 同上 |
| action_rate | -0.02 | -0.015 | 動きの自由度確保 |
| step_frequency | 0.8 Hz | 0.6 Hz | ゆったりした歩行ペース |

**新規報酬関数:**

| 報酬関数 | 係数 | 説明 |
|---------|------|------|
| symmetric_gait | 1.0 | 左右対称な歩行を促進（Yaw回転抑制） |
| smooth_joint_velocity | -1.0 | ジャーク（急激な速度変化）を抑制 |
| heading_alignment | 0.8 | 進行方向と向きを一致させる |

#### 評価結果（model_499.pt）

```
t= 0.00s | pos=(-0.000,  0.000, 0.447) | vel=(-0.00,  0.00) | rpy=(  0.0,   0.0,   0.0)° | dof_vel_rms= 0.00
t= 1.00s | pos=(-0.181, -0.002, 0.368) | vel=(-1.12, -0.00) | rpy=(  0.1, -23.7,   0.2)° | dof_vel_rms= 1.06
t= 2.00s | pos=(-0.075, -0.001, 0.408) | vel=(-0.61, -0.00) | rpy=( -0.3, -15.7,   0.0)° | dof_vel_rms= 0.23
t= 3.00s | pos=(-0.017, -0.000, 0.428) | vel=(-0.25,  0.01) | rpy=( -0.2,  -5.4,  -0.0)° | dof_vel_rms= 0.39
t= 4.00s | pos=(-0.000, -0.000, 0.440) | vel=( 0.01, -0.00) | rpy=(  0.0,   2.2,   0.0)° | dof_vel_rms= 0.51
t= 5.00s | pos=(-0.206, -0.001, 0.349) | vel=(-1.25, -0.00) | rpy=( -0.1, -25.2,  -0.0)° | dof_vel_rms= 1.25
...
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: -0.099 m  ← 後退している！
  Y: -0.001 m
  Total: 0.099 m

Average velocity:
  X: -0.629 m/s (target: 0.300)  ← 大きく負の速度
  Y: -0.001 m/s

Base height:
  Mean: 0.396 m
  Std: 0.0493 m  ← 高さ変動が非常に大きい（V7の14倍）
```

#### 学習ログ分析

**Mean Reward推移:**
```
  iter    0:    -2.92  ← 非常に低いスタート
  iter  100:     1.58
  iter  200:     2.09
  iter  499:     2.09  ← 収束したが非常に低い
```

**V7 vs V8 最終報酬比較:**

| 指標 | V7 | V8 | 備考 |
|------|-----|-----|------|
| **Final Reward** | **91.05** | **2.09** | V8は1/43に低下 |
| tracking_lin_vel | 1.720310 | 0.010177 | 169倍悪化 |
| tracking_ang_vel | 0.225448 | 0.042137 | 5倍悪化 |
| trajectory_tracking | 0.000000 | 0.000000 | 両方ゼロ |
| orientation | -0.000405 | -0.000965 | やや悪化 |
| base_height | -0.037546 | -0.003019 | 改善したが意味なし |

#### 失敗の原因分析

1. **過度なペナルティ強化**:
   - `orientation`: -3.0 → -8.0（2.7倍）
   - `ang_vel_xy`: -0.05 → -0.2（4倍）
   - これらの強化により、ロボットが「動かないこと」を学習してしまった

2. **新規報酬関数の副作用**:
   - `smooth_joint_velocity`（ジャークペナルティ）が動きを過度に抑制
   - ロボットは「立っているだけ」が最も報酬が高い状態に陥った

3. **報酬バランスの崩壊**:
   - V7: tracking_lin_vel = 1.72（歩行を促進）
   - V8: tracking_lin_vel = 0.01（ほぼゼロ）
   - 歩行の報酬よりペナルティの方が大きくなり、歩かない方が得になった

4. **実際の挙動**:
   - 立っている状態から周期的に前傾して倒れかけ、戻る
   - Pitch角が-27°まで傾く異常な動作
   - 高さ標準偏差が0.0493m（V7の0.0036mの14倍）

#### 教訓

1. **ペナルティの強化は慎重に**: 2倍以上の強化は危険
2. **新規報酬関数は1つずつ追加**: 複数同時追加は原因特定が困難
3. **V2の失敗と同じパターン**: 過度なペナルティ → 動かない

---

### バージョン比較サマリー

| バージョン | 実験名 | X移動(10s) | 平均速度 | Yaw回転 | 高さStd | 特徴・問題点 |
|-----------|--------|-----------|----------|--------|---------|-------------|
| V1 | biped-walking | 2.074 m | 0.197 m/s | - | - | ジャーキー |
| V2 | biped-walking-v2 | 0.014 m | 0.001 m/s | - | - | 動かない |
| V3 | biped-walking-v3 | 2.647 m | 0.274 m/s | - | - | すり足 |
| V4 | biped-walking-v4 | 2.793 m | 0.286 m/s | ~16° | 0.0063 | **交互歩行◎** |
| V5 | biped-walking-v5 | 2.853 m | 0.286 m/s | - | 0.0063 | バグで失敗 |
| V6 | biped-walking-v6 | 0.475 m | 0.121 m/s | - | 0.0028 | 速度低下 |
| V7 | biped-walking-v7 | 2.374 m | 0.237 m/s | ~28° | 0.0036 | 4ステップカクカク |
| V8 | biped-walking-v8 | -0.099 m | -0.629 m/s | ~0° | 0.0493 | **完全失敗（倒れる）** |

---

### 次のステップ（V9の方針）

V8の失敗から学び、以下の方針でV9を設計：

1. **V7をベースに最小変更**:
   - V7は歩けていた（問題はあるが）
   - V8のような大幅変更は避ける

2. **パラメータ調整は控えめに**:
   - `tracking_ang_vel`: 0.3 → 0.5（V8の1.5は過剰）
   - `orientation`: -3.0 → -4.0（V8の-8.0は過剰）
   - 他は変更しない

3. **新規報酬関数は1つだけ**:
   - `symmetric_gait`のみ追加（係数0.5で控えめに）
   - `smooth_joint_velocity`と`heading_alignment`は削除

---

### V10: 研究調査に基づく報酬設計の根本的見直し

#### 背景

V7/V8での試行錯誤的アプローチの限界を受け、以下の研究調査を実施し、根本的な報酬設計の見直しを行った。

#### 調査した文献・リソース

1. **legged_gym / Isaac Gym**
   - [leggedrobotics/legged_gym](https://github.com/leggedrobotics/legged_gym) - ANYmal標準設定
   - [unitreerobotics/unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/legged_gym/envs/g1/g1_config.py) - G1ヒューマノイド設定

2. **研究論文**
   - ["Not Only Rewards But Also Constraints"](https://arxiv.org/abs/2308.12517) (TRO 2024)
     - 報酬項目を3つに削減し、制約として物理制限を扱う手法
   - ["Learning to Walk in Minutes"](https://arxiv.org/abs/2109.11978) (CoRL 2021)
     - 大規模並列学習での報酬設計
   - ["Gait-Conditioned RL with Multi-Phase Curriculum"](https://arxiv.org/abs/2505.20619) (2025)
     - 歩行・走行・遷移を単一ポリシーで学習

3. **報酬設計ベストプラクティス**
   - [Reward Hacking in RL](https://lilianweng.github.io/posts/2024-11-28-reward-hacking/) - 報酬ハッキングの問題と対策
   - [ROGER: Reward Gain Adaptation](https://arxiv.org/abs/2510.10759) - 適応的報酬重み調整

#### 失敗パターンの分析

**V2とV8は同一パターンの失敗:**

| 項目 | V2 | V8 |
|------|-----|-----|
| 失敗モード | 動かない | 倒れる（後退） |
| 原因 | action_rate=-0.1（過剰） | orientation=-8.0, ang_vel_xy=-0.2（過剰） |
| 根本原因 | ペナルティ > 正の報酬 | ペナルティ > 正の報酬 |
| 学習された行動 | 「動かないこと」が最適 | 「動かないこと」が最適 |

**研究からの知見:**
> "Poor reward function design can lead to the algorithm getting stuck in local minima, very slow initial learning if gradients of the reward are shallow, or solutions that yield high rewards but do not resemble the intended target behavior." - Reward Engineering Survey

#### 研究に基づく報酬スケール比較

| 報酬項目 | G1標準 | V4 | V8(失敗) | V10(提案) | 根拠 |
|----------|--------|-----|----------|-----------|------|
| tracking_lin_vel | 1.0 | 2.0 | 2.0 | **1.5** | 主タスク、適度に強化 |
| tracking_ang_vel | 0.5 | 0.3 | 1.5 | **0.5** | 研究標準値 |
| orientation | -1.0 | -3.0 | -8.0 | **-1.5** | V8過剰を回避 |
| ang_vel_xy | -0.05 | -0.05 | -0.2 | **-0.05** | 研究標準値 |
| base_height | -10.0 | -30.0 | -30.0 | **-15.0** | 研究値とV4の中間 |
| dof_vel | -1e-3 | -1e-4 | -1e-4 | **-5e-4** | 研究値とV4の中間 |
| action_rate | -0.01 | -0.02 | -0.015 | **-0.01** | 研究標準値 |

#### V10の設計原則

1. **V4をベースに**
   - V4は交互歩行を達成（X移動2.793m）
   - feet_namesバグ修正（V6から継承）

2. **Phase-based trajectory trackingを削除**
   - V7で「4ステップカクカク」問題を引き起こした
   - 強制的な周期がロボットの自然な動力学と不一致

3. **報酬バランスの確保**
   - 正の報酬合計 > ペナルティ合計（初期状態で）
   - `forward_progress`報酬を追加（standing still防止）
   - `alive`報酬を追加（小さい値）

4. **ペナルティの抑制**
   - 全ペナルティを研究標準値程度に
   - 2倍以上の強化は禁止

#### 報酬構成

**正の報酬（タスク達成）:**
```python
"tracking_lin_vel": 1.5,      # 主タスク
"tracking_ang_vel": 0.5,      # 回転追従
"alive": 0.1,                 # 生存
"forward_progress": 0.3,      # 前進インセンティブ
"alternating_gait": 1.0,      # 交互歩行（二脚コア）
"foot_swing": 0.5,            # スイング
"feet_air_time": 0.5,         # 滞空時間
"single_stance": 0.3,         # 片足立ち
```

**ペナルティ（制約）:**
```python
"no_fly": -0.5,               # 両足離地
"orientation": -1.5,          # 姿勢（研究標準の1.5倍）
"base_height": -15.0,         # 高さ
"lin_vel_z": -1.0,            # 上下動
"ang_vel_xy": -0.05,          # 揺れ
"action_rate": -0.01,         # 滑らかさ
"dof_vel": -5e-4,             # 関節速度
"dof_acc": -2.5e-7,           # 関節加速度
"torques": -5e-5,             # トルク
"similar_to_default": -0.02,  # デフォルト維持
```

#### 実行方法

```bash
cd rl_ws
uv run python genesis_official/examples/locomotion/biped_train_v10.py --max_iterations 500
```

#### 期待される効果

1. **standing still問題の回避**: forward_progress報酬 + ペナルティ抑制
2. **安定した交互歩行**: V4の報酬構造を継承
3. **自然な歩行周期**: Phase強制なしでロボットが自律的に発見
4. **バランスの取れた学習**: 研究標準値に基づくスケール

#### 評価予定

- [ ] 500イテレーション学習完了
- [ ] 10秒間のheadless評価
- [ ] GUI評価で歩行品質確認
- [ ] V4との比較（X移動距離、Yaw回転、高さ安定性）
