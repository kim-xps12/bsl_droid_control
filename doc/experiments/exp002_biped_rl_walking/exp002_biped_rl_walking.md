# EXP002: BSL-Droid二脚ロボットの強化学習による歩容獲得実験（暫定的なurdfを用いる版）

## 概要

BSL-Droid二脚ロボット（Digitigrade/逆関節型）に対して、Genesis物理シミュレータで強化学習により歩容を獲得する。MuJoCoで汎化性能を検証するsim2simのパイプライン構築も併せて確認する．

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

## ファイル構成

```
rl_ws/
├── assets/
│   ├── biped_digitigrade.urdf      # エクスポート済みURDF
│   ├── biped_digitigrade.xml       # 変換済みMJCF
│   ├── go2_genesis.xml             # Go2用MJCF（参考）
│   └── export_urdf.py              # URDFエクスポートスクリプト
├── envs/                           # 強化学習環境（バージョン管理）
│   ├── __init__.py
│   ├── biped_env.py                # V1環境
│   ├── biped_env_v2.py             # V2環境（滑らかさペナルティ）
│   ├── biped_env_v4.py             # V4環境（交互歩行報酬）
│   ├── biped_env_v5.py             # V5環境（大歩幅・足上げ）
│   ├── biped_env_v6.py             # V6環境（feet_names修正）
│   ├── biped_env_v7.py             # V7環境（Phase-based参照軌道）
│   ├── biped_env_v8.py             # V8環境（Yaw補正強化）
│   ├── biped_env_v9.py             # V9環境（対称歩行報酬）
│   ├── biped_env_v10.py            # V10環境（研究ベース報酬設計）
│   └── biped_env_v11.py            # V11環境（V10+V4交互報酬+hip逆相）
├── scripts/                        # トレーニング・評価スクリプト
│   ├── biped_train.py              # V1トレーニング
│   ├── biped_train_v2.py〜v11.py   # 各バージョンのトレーニング
│   ├── biped_eval.py               # 統一評価スクリプト（-eオプションで全バージョン対応）
│   ├── biped_eval_headless.py      # ヘッドレス評価
│   ├── biped_eval_mujoco.py        # MuJoCo sim2sim評価
│   ├── convert_biped_urdf_to_mjcf.py  # 二脚URDF→MJCF変換
│   ├── convert_urdf_to_mjcf.py     # 汎用URDF→MJCF変換
│   └── compare_models.py           # モデルパラメータ比較
└── logs/                           # トレーニングログ（※.ptファイルはgit管理外）
    ├── .gitignore                  # *.pt を除外
    ├── biped-walking/              # V1実験
    ├── biped-walking-v2/           # V2実験
    ├── biped-walking-v3/           # V3実験
    ├── biped-walking-v4/           # V4実験（交互歩行◎）
    ├── biped-walking-v5/           # V5実験
    ├── biped-walking-v6/           # V6実験
    ├── biped-walking-v7/           # V7実験（Phase-based）
    ├── biped-walking-v8/           # V8実験（失敗）
    ├── biped-walking-v9/           # V9実験（振動歩行）
    ├── biped-walking-v10/          # V10実験（両脚同期）
    ├── biped-walking-v11/          # V11実験（V10改良版）
    │   ├── model_*.pt              # チェックポイント（git管理外）
    │   └── cfgs.pkl                # 訓練設定
    └── go2-walking/                # Go2歩行ポリシー（参考）
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
| V9 | biped-walking-v9 | 2.309 m | 0.231 m/s | ~20° | 0.0047 | 小刻み振動歩行 |

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

### V9: symmetric_gait報酬追加版

#### 評価結果（model_499.pt, 500イテレーション）

**数値データ（10秒間、GUIなし）:**

```
t= 0.00s | pos=(-0.000,  0.000, 0.447) | vel=(-0.00,  0.00) | rpy=(  0.0,   0.0,   0.0)° | dof_vel_rms= 0.00
t= 1.00s | pos=( 0.097, -0.008, 0.441) | vel=( 0.20,  0.18) | rpy=(  2.1,  -4.9,  10.6)° | dof_vel_rms= 2.31
t= 2.00s | pos=( 0.361,  0.009, 0.449) | vel=( 0.21,  0.07) | rpy=(  7.8,  -6.1,  11.9)° | dof_vel_rms= 2.19
t= 3.00s | pos=( 0.623,  0.055, 0.452) | vel=( 0.22, -0.18) | rpy=( -1.4,  -2.8,  14.2)° | dof_vel_rms= 0.93
t= 4.00s | pos=( 0.853,  0.081, 0.443) | vel=( 0.21, -0.17) | rpy=(  1.5,  -7.7,  14.4)° | dof_vel_rms= 2.57
t= 5.00s | pos=( 1.097,  0.119, 0.446) | vel=( 0.30,  0.14) | rpy=(  1.5,  -4.4,  12.7)° | dof_vel_rms= 0.91
t= 6.00s | pos=( 1.352,  0.137, 0.448) | vel=( 0.24,  0.10) | rpy=(  6.4,  -1.6,  14.5)° | dof_vel_rms= 0.85
t= 7.00s | pos=( 1.595,  0.170, 0.456) | vel=( 0.17,  0.08) | rpy=(  8.0,  -5.1,  15.0)° | dof_vel_rms= 1.96
t= 8.00s | pos=( 1.836,  0.255, 0.453) | vel=( 0.25, -0.14) | rpy=( -1.9,  -4.6,  17.9)° | dof_vel_rms= 1.41
t= 9.00s | pos=( 2.076,  0.301, 0.442) | vel=( 0.19, -0.22) | rpy=(  1.9,  -7.4,  19.7)° | dof_vel_rms= 2.70

=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 2.309 m
  Y: 0.362 m
  Total: 2.337 m

Average velocity:
  X: 0.231 m/s (target: 0.300)
  Y: -0.023 m/s

Base height:
  Mean: 0.447 m
  Std: 0.0047 m

DOF velocity RMS: 1.900 rad/s
Action RMS: 0.928
```

#### V7との比較

| 指標 | V7 | V9 | 評価 |
|------|-----|-----|------|
| X移動距離（10s） | 2.374 m | 2.309 m | V7が2.7%速い |
| 平均X速度 | 0.237 m/s | 0.231 m/s | ほぼ同等 |
| Yaw回転（10s） | ~28° | ~20° | **V9が29%改善** |
| ベース高さStd | 0.0036 m | 0.0047 m | V7が30%安定 |
| DOF velocity RMS | 1.263 rad/s | 1.900 rad/s | **V9が50%高い（振動）** |
| Action RMS | 0.809 | 0.928 | V9がアクション大 |

#### 問題点の分析

**GUI評価での観察:**
> 「まだ生き物っぽくない小刻みな振動による移動の印象」

**数値的根拠:**
1. **DOF velocity RMS = 1.900 rad/s**: V7（1.263）の50%増加
   - 関節が高速で振動しながら動いている
   - 「震えながら前進」する状態

2. **Roll/Pitch変動が大きい**:
   - Roll: -1.9° 〜 +8.0°（約10°の変動幅）
   - Pitch: -7.7° 〜 -1.6°（約6°の変動幅）
   - 体が左右・前後に揺れながら移動

3. **symmetric_gait報酬の効果**:
   - Yaw回転は28° → 20°に改善（目的達成）
   - しかし、対称性を強制することで小刻み動作が増加した可能性

#### 結論

V9は**Yaw回転抑制には成功**したが、**振動的な動作が悪化**した。
「生き物らしい滑らかな歩行」という目標には到達していない。

**課題:**
- DOF velocity RMSを1.0以下に抑える必要がある
- action_rateペナルティの強化を検討
- Phase-basedではない別の周期性報酬を検討

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

#### V10評価結果（model_499.pt）

**評価コマンド**:
```bash
cd rl_ws
uv run python scripts/biped_eval.py -e biped-walking-v10 --no-viewer --duration 10
```

**基本統計**:

| 指標 | V10 | V9 | V4 | V7（参考） |
|------|-----|-----|-----|-----------|
| X移動距離 [m] | 2.281 | 2.309 | **2.793** | 2.1 |
| Y移動距離 [m] | -0.416 | 0.362 | 0.580 | - |
| 平均X速度 [m/s] | 0.220 | 0.231 | **0.286** | 0.21 |
| 高さ平均 [m] | 0.427 | 0.447 | 0.403 | - |
| 高さ標準偏差 [m] | 0.0043 | 0.0047 | 0.0063 | - |
| DOF velocity RMS [rad/s] | **1.005** | 1.900 | 1.024 | 1.263 |
| Action RMS | 1.115 | 0.928 | 1.093 | - |
| Yaw回転（10秒後）[°] | -10.1 | 19.7 | 15.6 | 28 |

**関節動作分析**:

| 関節 | V10 L [rad] | V10 R [rad] | V4 L [rad] | V4 R [rad] |
|------|------------|------------|------------|------------|
| hip_pitch | 0.525 | 0.249 | 0.457 | **0.628** |
| hip_roll | 0.413 | 0.293 | 0.094 | 0.147 |
| knee | 0.655 | 1.034 | 0.634 | **0.940** |
| ankle_pitch | 0.360 | 0.302 | **0.725** | **0.623** |
| ankle_roll | 0.326 | 0.638 | 0.423 | 0.454 |

**左右hip_pitch相関**:

| バージョン | 相関係数 | 解釈 |
|-----------|---------|------|
| V10 | **0.957** | ほぼ完全同期（両脚固定） |
| V9 | 0.240 | 弱い相関（震え歩行） |
| V4 | 0.772 | 中程度（部分的交互） |
| 理想 | **-1.0** | 完全交互歩行 |

**問題点の特定**:

1. **両脚同期問題（最重要）**: 左右hip_pitch相関が0.957と極めて高く、両脚がほぼ同時に動いている。これが「前脚と後脚が固定されている」印象の原因。

2. **交互歩行報酬の弱体化**: V10では研究標準値に合わせて報酬を控えめにした結果、V4で効いていた`alternating_gait`が弱くなった。
   - V4: `alternating_gait=1.5`, `foot_swing=0.8`, `feet_air_time=1.0`
   - V10: `alternating_gait=1.0`, `foot_swing=0.5`, `feet_air_time=0.5`

3. **Pitch傾斜**: Pitch角が-17°〜-18°で安定しており、前傾姿勢で「滑り降りる」動作になっている。

**良い点**:

- DOF velocity RMS: 1.005 rad/s（V9の1.900より47%改善、V7の1.263より20%改善）
- Yaw回転: -10.1°（V9の19.7°より改善、方向安定性向上）
- 高さ安定性: 標準偏差0.0043m（V9の0.0047mより改善）

**V11への教訓**:

1. **交互歩行報酬はV4の値を維持すべき**: 研究標準値への最適化よりも、実際に動作したV4の設定を尊重する。
2. **hip_pitch相関を監視指標に**: 評価時に左右相関を計算し、-0.5以下を目標とする。
3. **Pitch補正**: 前傾を抑えるために`orientation`または`lin_vel_z`ペナルティを調整。

---

## V11設計に向けた先行研究調査

### 調査目的

V9の評価結果で「小刻みな振動による移動」という問題が確認された。この問題を解決し、生き物らしい滑らかな歩行を実現するため、二脚ロボットの強化学習による歩容獲得に関する先行研究を調査した。

### 主要な先行研究

#### 1. Real-World Humanoid Locomotion with Reinforcement Learning (Berkeley, 2023)

**論文**: [arXiv:2303.03381](https://arxiv.org/abs/2303.03381)

**概要**:
- Causal Transformerを用いたヒューマノイド歩行制御
- 観測-行動履歴を入力として次の行動を予測
- シミュレーションで学習し、実世界にゼロショット転送

**重要な知見**:
- **履歴情報の活用**: 過去の観測・行動履歴がin-context adaptationを可能にする
- **大規模並列学習**: 多数のランダム化環境でのロバスト性獲得
- **外乱に対する適応**: 学習中のランダムプッシュが汎化性能を向上

#### 2. Learning Agile Soccer Skills for a Bipedal Robot (DeepMind, 2023)

**論文**: [arXiv:2304.13653](https://arxiv.org/abs/2304.13653) (Science Robotics掲載)

**概要**:
- 20関節の小型ヒューマノイドロボットでサッカースキルを学習
- 歩行、転回、キック、起き上がりなど複合スキルを単一ポリシーで習得
- シミュレーションから実機へゼロショット転送

**重要な知見**:
- **高周波制御**: 十分に高い制御周波数（50Hz以上）が安定性に重要
- **Targeted Dynamics Randomization**: 摩擦、質量などの物理パラメータランダム化
- **基本的な正則化で安全な動作**: 過度な報酬エンジニアリングなしでも安全で効果的な動作を学習
- **スキル間の滑らかな遷移**: 複数スキルの自然な切り替え

**実績**:
- 歩行速度: スクリプトベースラインの181%
- 転回速度: 302%
- 起き上がり時間: 63%短縮

#### 3. Reinforcement Learning for Versatile, Dynamic, and Robust Bipedal Locomotion Control (UC Berkeley, 2024)

**論文**: [arXiv:2401.16889](https://arxiv.org/abs/2401.16889) (IJRR掲載)

**概要**:
- Cassieロボットで歩行、走行、ジャンプ、立位など多様なスキルを学習
- **Dual-History Architecture**: 長期・短期のI/O履歴を活用

**重要な知見**:
- **Task Randomization**: タスク条件のランダム化がロバスト性と汎化に重要
- **Contact Event Adaptation**: 接触イベントへの適応にI/O履歴が有効
- **時変・時不変のダイナミクス変化への対応**: 履歴情報で両方に適応可能

**実績**:
- 400mダッシュの実機デモ
- 立ち幅跳び、高跳びなど多様なジャンプスキル

#### 4. Not Only Rewards But Also Constraints (KAIST, 2024)

**論文**: [arXiv:2308.12517](https://arxiv.org/abs/2308.12517) (IEEE T-RO掲載)

**概要**:
- 報酬だけでなく**制約**を用いた強化学習フレームワーク
- 報酬エンジニアリングの労力を大幅に削減

**重要な知見**:
- **報酬項目の削減**: 単一の報酬係数チューニングで高性能なコントローラを実現
- **制約の解釈可能性**: 制約は報酬より直感的に設計可能
- **制約の汎化性**: 異なるロボット形態に同じ制約を適用可能

**提案する制約タイプ**:
1. **Hard Constraint**: 物理的に絶対に守るべき制約（関節限界など）
2. **Soft Constraint**: 望ましいが厳密でない制約（姿勢維持など）

#### 5. Humanoid Locomotion as Next Token Prediction (Berkeley, 2024)

**論文**: [arXiv:2402.19469](https://arxiv.org/abs/2402.19469)

**概要**:
- ヒューマノイド制御を「次トークン予測」問題として定式化
- シミュレーション軌道、モーションキャプチャ、YouTube動画からも学習

**重要な知見**:
- **データソースの多様化**: 異なるモダリティのデータを統合学習
- **27時間の歩行データで実機転送成功**: 必要なデータ量は意外と少ない
- **未見コマンドへの汎化**: 後ろ歩きなど訓練時にないコマンドにも対応

### 標準的な報酬関数設計（legged_gym / Isaac Gym）

[leggedrobotics/legged_gym](https://github.com/leggedrobotics/legged_gym)はETH Zurichによる四脚・二脚ロボットの強化学習環境の標準実装である。

#### 基本報酬スケール（LeggedRobotCfg）

```python
class scales:
    # タスク報酬
    tracking_lin_vel = 1.0      # 線速度追従
    tracking_ang_vel = 0.5      # 角速度追従
    feet_air_time = 1.0         # 滞空時間
    
    # ペナルティ
    termination = -0.0          # 終端
    lin_vel_z = -2.0            # 上下動
    ang_vel_xy = -0.05          # Roll/Pitch角速度
    orientation = -0.0          # 姿勢
    torques = -0.00001          # トルク
    dof_vel = -0.0              # 関節速度
    dof_acc = -2.5e-7           # 関節加速度（重要！）
    base_height = -0.0          # 高さ
    collision = -1.0            # 衝突
    action_rate = -0.01         # アクション変化率（重要！）
    stand_still = -0.0          # 静止時のペナルティ
```

#### Cassie（二脚ロボット）の設定

```python
class scales:
    termination = -200.         # 転倒に大きなペナルティ
    tracking_ang_vel = 1.0
    torques = -5.e-6
    dof_acc = -2.e-7
    lin_vel_z = -0.5
    feet_air_time = 5.          # 滞空時間を重視
    dof_pos_limits = -1.
    no_fly = 0.25               # 両足離地を軽くペナルティ
```

#### Unitree G1（ヒューマノイド）の設定

[unitreerobotics/unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym)より：

```python
class scales:
    tracking_lin_vel = 1.0
    tracking_ang_vel = 0.5
    lin_vel_z = -2.0
    ang_vel_xy = -0.05
    orientation = -1.0
    base_height = -10.0         # 高さ維持を重視
    dof_acc = -2.5e-7
```

### 振動抑制のための具体的手法

#### 1. Action Rate Penalty（アクション変化率ペナルティ）

**定義**:
```python
def _reward_action_rate(self):
    return torch.sum(torch.square(self.last_actions - self.actions), dim=1)
```

**効果**: 連続するアクション間の変化を抑制し、滑らかな動作を促進

**推奨スケール**: `-0.01` 〜 `-0.05`

**注意**: 強すぎると動かなくなる（V2の失敗パターン）

#### 2. DOF Acceleration Penalty（関節加速度ペナルティ）

**定義**:
```python
def _reward_dof_acc(self):
    return torch.sum(torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=1)
```

**効果**: 急激な関節速度変化（ジャーク）を抑制

**推奨スケール**: `-2.5e-7` 〜 `-1e-6`

#### 3. DOF Velocity Penalty（関節速度ペナルティ）

**定義**:
```python
def _reward_dof_vel(self):
    return torch.sum(torch.square(self.dof_vel), dim=1)
```

**効果**: 高速な関節運動を抑制

**推奨スケール**: `-0.0` 〜 `-1e-3`（軽めに設定）

**注意**: 強すぎると歩行速度が低下

#### 4. Torque Penalty（トルクペナルティ）

**定義**:
```python
def _reward_torques(self):
    return torch.sum(torch.square(self.torques), dim=1)
```

**効果**: エネルギー効率の良い動作を促進、振動も間接的に抑制

**推奨スケール**: `-1e-5` 〜 `-1e-4`

### 歩行周期の制御手法

#### 1. Gait Clock / Phase Signal

**概要**: 歩行周期を表す位相信号を観測に追加

```python
phase = (time * gait_frequency) % 1.0  # [0, 1]の周期信号
obs_phase = [sin(2π * phase), cos(2π * phase)]
```

**利点**: ポリシーが周期的な動作を学習しやすい

**欠点**: 周波数を事前に固定する必要がある（V7の「4ステップカクカク」問題の原因）

#### 2. Feet Air Time Reward

**概要**: 足の滞空時間に応じて報酬を付与

```python
def _reward_feet_air_time(self):
    contact = self.contact_forces[:, self.feet_indices, 2] > 1.
    first_contact = (self.feet_air_time > 0.) * contact
    self.feet_air_time += self.dt
    rew = torch.sum((self.feet_air_time - 0.5) * first_contact, dim=1)
    self.feet_air_time *= ~contact
    return rew
```

**利点**: 明示的な周期指定なしで自然な歩行周期を発見

**推奨スケール**: `1.0` 〜 `5.0`

#### 3. No-Fly Reward（Cassie）

**概要**: 片足だけが接地している状態を報酬

```python
def _reward_no_fly(self):
    contacts = self.contact_forces[:, self.feet_indices, 2] > 0.1
    single_contact = torch.sum(1.*contacts, dim=1) == 1
    return 1.*single_contact
```

**利点**: 両足離地（ジャンプ）や両足接地（すり足）を抑制

### V11への具体的提案

#### 課題の整理（V10評価結果を反映）

| 問題 | V9での状況 | V10での状況 | V11対策方針 |
|------|-----------|-------------|-------------|
| 両脚同期 | 相関0.240 | **相関0.957** | hip_pitch逆相報酬追加 |
| 小刻み振動 | DOF vel RMS = 1.9 | 1.005 | action_rate 3倍強化 |
| Pitch前傾 | - | -17° | pitch_penalty追加 |
| 交互歩行不足 | - | 報酬弱化 | V4レベル復元 |

---

## V11: V10改良版（実装済み）

### 設計方針

V10の評価結果（両脚同期、Pitch前傾）と先行研究調査を反映し、以下の方針でV11を設計した。

1. **交互歩行報酬をV4レベルに復元**: V10で控えめにした報酬を戻す
2. **振動抑制ペナルティを強化**: 先行研究（legged_gym）の知見を適用
3. **新規報酬追加**: hip_pitch逆相報酬、Pitch角ペナルティ

### 報酬設計の比較

| 報酬項目 | V4 | V10 | V11 | 変更理由 |
|----------|-----|-----|-----|---------|
| **交互歩行報酬** |
| alternating_gait | 1.5 | 1.0 | **1.5** | V4レベル復元 |
| foot_swing | 0.8 | 0.5 | **0.8** | V4レベル復元 |
| feet_air_time | 1.0 | 0.5 | **1.0** | V4レベル復元 |
| single_stance | 0.5 | 0.3 | **0.5** | V4レベル復元 |
| no_fly | -1.0 | -0.5 | **-1.0** | V4レベル復元 |
| hip_pitch_alternation | - | - | **1.0** | **新規**: 逆相運動報酬 |
| **振動抑制** |
| action_rate | -0.02 | -0.01 | **-0.03** | 3倍強化 |
| dof_vel | -1e-4 | -5e-4 | **-1e-3** | 2倍強化 |
| dof_acc | -1e-7 | -2.5e-7 | **-5e-7** | 2倍強化 |
| **姿勢補正** |
| orientation | -3.0 | -1.5 | **-2.5** | 強化 |
| lin_vel_z | -1.5 | -1.0 | **-2.0** | 強化 |
| pitch_penalty | - | - | **-3.0** | **新規**: Pitch直接補正 |

### V11新規報酬関数

#### hip_pitch_alternation

```python
def _reward_hip_pitch_alternation(self):
    """左右hip_pitchの逆相運動報酬
    
    左右のhip_pitchが逆方向に動くことを報酬。
    両脚同期（相関0.957）から交互歩行（相関-1.0）へ誘導。
    """
    left_hip_vel = self.dof_vel[:, self.left_hip_pitch_idx]
    right_hip_vel = self.dof_vel[:, self.right_hip_pitch_idx]
    
    # 速度の積が負 = 逆方向に動いている
    opposite_motion = -left_hip_vel * right_hip_vel
    reward = torch.clamp(opposite_motion, min=0.0, max=1.0)
    
    has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
    return reward * has_command
```

#### pitch_penalty

```python
def _reward_pitch_penalty(self):
    """Pitch角ペナルティ
    
    V10で-17°の前傾が観測されたため、Pitch角を直接ペナルティ。
    """
    pitch_rad = self.base_euler[:, 1] * 3.14159 / 180.0
    return torch.square(pitch_rad)
```

### 実行方法

```bash
cd rl_ws
uv run python scripts/biped_train_v11.py --max_iterations 500
```

### 数値目標

| 指標 | V10 | V11目標 |
|------|-----|---------|
| hip_pitch相関 | 0.957 | **< -0.3** |
| DOF velocity RMS | 1.005 | **< 0.8** |
| Pitch角 | -17° | **< 10°** |
| X移動距離(10s) | 2.28 m | **> 2.0 m** |
| Yaw回転(10s) | -10° | **< 15°** |

### 評価結果

500イテレーション学習完了後の評価結果：

#### 定量評価（10秒間headless評価）

| 指標 | V10 | V11目標 | V11結果 | 達成 |
|------|-----|---------|---------|------|
| hip_pitch相関 | 0.957 | < -0.3 | **0.495** | △ |
| DOF velocity RMS | 1.005 | < 0.8 | 1.618 | ✗ |
| Pitch角 | -17° | < 10° | **-11.3°** | ○ |
| X移動距離(10s) | 2.28 m | > 2.0 m | **2.528 m** | ○ |
| Yaw回転(10s) | -10° | < 15° | -4.3° | ○ |

#### 詳細統計

```
Travel distance: X=2.528m, Y=-0.556m, Total=2.589m
Average velocity: X=0.243 m/s (target: 0.300)
Base height: Mean=0.434m, Std=0.0037m
DOF velocity RMS: 1.618 rad/s
Action RMS: 1.111

DOF position range:
  hip_pitch: L=[−0.529, 0.000](0.529) R=[−0.196, 0.000](0.196)
  knee: L=[0.000, 1.002](1.002) R=[−0.084, 0.757](0.841)

Left-Right hip_pitch correlation: 0.495
Total DOF range sum: 4.982 rad
```

#### 定性評価

**改善点：**
- 「生き物っぽく歩いている」脚の振り方が実現
- V10と比較してhip_pitch相関が大幅に改善（0.957 → 0.495）
- Pitch角の前傾が改善（-17° → -11.3°）
- 移動距離が向上

**残存課題：**
- **脚が交差しない**: 左右の脚が固定的な前後関係のまま歩行
  - 左hip_pitchの範囲: [-0.529, 0.000]（常に後方〜中立）
  - 右hip_pitchの範囲: [-0.196, 0.000]（常に後方〜中立）
  - 正の値（前方への振り出し）が一度も出現していない
- hip_pitch相関が目標の-0.3に未達（0.495）
- DOF velocity RMSが目標を超過（振動成分の残存）

#### V12への改善方針

脚の交差を実現するには、hip_pitchが**正負両方の領域**を使用する必要がある：
1. 歩行サイクル報酬の導入（swing phase/stance phaseの明確化）
2. hip_pitchの可動範囲全体を活用する報酬
3. 左右のhip_pitchが反対符号になる報酬（真の逆位相）

### 参考文献

1. Radosavovic et al., "Real-World Humanoid Locomotion with Reinforcement Learning", arXiv:2303.03381, 2023
2. Haarnoja et al., "Learning Agile Soccer Skills for a Bipedal Robot with Deep Reinforcement Learning", Science Robotics, 2024
3. Li et al., "Reinforcement Learning for Versatile, Dynamic, and Robust Bipedal Locomotion Control", IJRR, 2024
4. Kim et al., "Not Only Rewards But Also Constraints: Applications on Legged Robot Locomotion", IEEE T-RO, 2024
5. Rudin et al., "Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning", CoRL, 2021
6. leggedrobotics/legged_gym, https://github.com/leggedrobotics/legged_gym
7. unitreerobotics/unitree_rl_gym, https://github.com/unitreerobotics/unitree_rl_gym

---

## V12: 脚交差問題への対処

V11で「生き物っぽく歩いている」脚の振り方が実現したが、**脚同士が交差しない**問題が残存。
V12では、hip_pitchが正負両方の領域を使用するように報酬設計を改良する。

### V11の問題点（詳細分析）

| 指標 | 観測値 | 問題 |
|------|--------|------|
| 左hip_pitch範囲 | [-0.529, 0.000] | **正の領域（前方）を使用していない** |
| 右hip_pitch範囲 | [-0.196, 0.000] | **正の領域（前方）を使用していない** |
| hip_pitch相関 | 0.495 | 目標（< -0.3）に未達 |

**根本原因の仮説：**
1. `similar_to_default`報酬がdefault位置（hip_pitch=0）への回帰を促進しすぎ
2. 前方スイング（hip_pitch > 0）を明示的に報酬していない
3. 左右が「反対符号」になることを報酬していない

### V12の設計

#### 報酬構造の変更

| 報酬名 | V11 | V12 | 理由 |
|--------|-----|-----|------|
| similar_to_default | -0.02 | **削除** | 脚の振り幅を制限する原因 |
| hip_pitch_alternation | 1.0 | **1.5** | 逆相運動を強化 |
| hip_pitch_opposite_sign | - | **1.5** | 新規：反対符号を報酬 |
| hip_pitch_range | - | **1.0** | 新規：前方スイングを促進 |
| stride_length | - | **0.8** | 新規：ストライド長を促進 |

#### 新規報酬関数

##### hip_pitch_opposite_sign

```python
def _reward_hip_pitch_opposite_sign(self):
    """左右hip_pitchが反対符号になる報酬
    
    V11の問題: 両方のhip_pitchが負の領域のみを使用
    解決策: 左右のhip_pitchが反対符号（片方が前、片方が後ろ）になることを報酬
    
    計算方法:
    - left_hip_pitch * right_hip_pitch < 0 なら報酬（反対符号）
    - 積の絶対値が大きいほど報酬が大きい（両方の振幅が大きい）
    """
    left_hp = self.dof_pos[:, self.left_hip_pitch_idx]
    right_hp = self.dof_pos[:, self.right_hip_pitch_idx]
    
    # 積が負 = 反対符号
    product = left_hp * right_hp
    opposite_sign_reward = torch.clamp(-product, min=0.0)
    
    # 正規化（振幅0.5rad x 0.5rad = 0.25を最大として）
    reward = torch.clamp(opposite_sign_reward / 0.25, max=1.0)
    
    has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
    return reward * has_command
```

##### hip_pitch_range

```python
def _reward_hip_pitch_range(self):
    """hip_pitchの可動範囲利用報酬
    
    V11の問題: hip_pitchが[-0.53, 0.0]の狭い範囲しか使わない
    解決策: 正の領域（前方スイング）も使うことを報酬
    
    計算方法:
    - hip_pitchが正（前方）に行くことを報酬
    - 前方スイングを明示的に誘導
    """
    left_hp = self.dof_pos[:, self.left_hip_pitch_idx]
    right_hp = self.dof_pos[:, self.right_hip_pitch_idx]
    
    # 正の値（前方スイング）を報酬
    left_forward = torch.clamp(left_hp, min=0.0)
    right_forward = torch.clamp(right_hp, min=0.0)
    
    # 両脚の前方スイング量の合計を報酬
    reward = (left_forward + right_forward) / 0.5  # 0.5radで正規化
    
    has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
    return reward * has_command
```

##### stride_length

```python
def _reward_stride_length(self):
    """ストライド長報酬
    
    左右hip_pitchの差分（ストライド長に相当）を報酬。
    差が大きいほど大きな歩幅で歩いている。
    """
    left_hp = self.dof_pos[:, self.left_hip_pitch_idx]
    right_hp = self.dof_pos[:, self.right_hip_pitch_idx]
    
    # hip_pitch差分の絶対値（ストライド長に相当）
    stride = torch.abs(left_hp - right_hp)
    
    # 0.5rad差で報酬1.0
    reward = stride / 0.5
    
    has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
    return torch.clamp(reward, max=1.5) * has_command
```

### 実行方法

```bash
cd rl_ws
uv run python scripts/biped_train_v12.py --max_iterations 500
```

### 数値目標

| 指標 | V11 | V12目標 |
|------|-----|---------|
| hip_pitch相関 | 0.495 | **< -0.3** |
| 左hip_pitch最大値 | 0.000 | **> 0.2** |
| 右hip_pitch最大値 | 0.000 | **> 0.2** |
| ストライド長（L-R差分） | ~0.33 | **> 0.5** |
| X移動距離(10s) | 2.53 m | **> 2.0 m** |

### 評価結果（失敗）

**V12は過去最悪の結果となった。**

#### 定量評価（10秒間headless評価）

| 指標 | V11 | V12目標 | V12結果 | 達成 |
|------|-----|---------|---------|------|
| hip_pitch相関 | 0.495 | < -0.3 | **0.936** | ✗✗ |
| X移動距離(10s) | 2.53 m | > 2.0 m | **0.66 m** | ✗✗ |
| Roll角 | - | - | **-20°** | ✗✗ |
| Pitch角 | -11.3° | - | 6° | - |
| Base高さ | 0.434m | - | **0.326m** | ✗ |
| DOF velocity RMS | 1.618 | - | 1.747 | ✗ |

#### 詳細統計

```
Travel distance: X=0.660m, Y=-0.365m, Total=0.754m
Average velocity: X=0.075 m/s (target: 0.300)  ← 目標の25%しか達成せず
Base height: Mean=0.326m, Std=0.0146m  ← 大幅に低下

DOF position range:
  hip_pitch: L=[-0.497, 0.037](0.534) R=[-0.369, 0.000](0.369)
  hip_roll:  L=[0.000, 0.361](0.361)  R=[0.000, 0.356](0.356)  ← 両脚外側に開いている

Left-Right hip_pitch correlation: 0.936  ← V10より悪化！
```

#### 定性評価

**致命的な問題：**

![V12の失敗姿勢](両脚を大きく開いて固定した状態)

- **両脚を大きく前後に開いて固定**: stride_length報酬が「脚を開いたまま固定」という局所最適解に収束
- **Roll角-20°**: hip_roll報酬がないため横方向に大きく傾斜
- **歩行せずスライド**: 脚を動かさずに重心移動で前進する異常な動作
- **hip_pitch相関0.936**: V10（0.957）と同様に両脚同期、むしろ悪化

#### 失敗の原因分析

1. **stride_length報酬の設計ミス**
   - `|left_hp - right_hp|`を報酬 → 脚を開いたまま固定することで常に報酬を得られる
   - 歩行サイクル内での動的な変化を報酬していない

2. **hip_pitch_opposite_sign報酬の副作用**
   - `left_hp * right_hp < 0`を報酬 → 片方を前、片方を後ろに固定して報酬を得る
   - 動的な交差ではなく静的な開脚に収束

3. **similar_to_default削除の影響**
   - default姿勢への回帰がないため、極端な姿勢に収束
   - 特にhip_rollが制御されず横傾斜が発生

4. **基本的な姿勢維持報酬の不足**
   - Roll角へのペナルティが不足
   - base_height低下へのペナルティが不十分

#### 教訓

**静的な状態量ではなく、動的な変化を報酬すべき**:
- ✗ `|left_hp - right_hp|` → 開脚固定で報酬
- ✗ `left_hp * right_hp < 0` → 前後開脚固定で報酬
- ○ 速度ベースの報酬（hip_pitch_alternation）は有効だった

**V13への改善方針**:
1. stride_length, hip_pitch_opposite_sign, hip_pitch_range を削除
2. similar_to_defaultを復元
3. V11ベースに戻し、別のアプローチを検討
4. 周期的な歩行パターンを明示的に誘導（phase-based報酬の再検討）

---

## V13: 動的歩行報酬（V12失敗の反省）

V12の失敗を踏まえ、**静的な位置ではなく動的な速度・変化を報酬**する設計に変更。

### V12失敗の教訓

| アプローチ | 結果 | 理由 |
|-----------|------|------|
| stride_length（位置差分） | ✗ | 脚を開いたまま固定で報酬獲得 |
| hip_pitch_opposite_sign（位置積） | ✗ | 前後開脚固定で報酬獲得 |
| hip_pitch_range（前方位置） | ✗ | 前方に固定で報酬獲得 |
| **hip_pitch_alternation（速度積）** | **○** | 動いていないと報酬なし |

### V13の設計原則

**「静的な位置を報酬しない、動的な変化のみを報酬する」**

### 報酬構造の変更

| 報酬名 | V11 | V12 | V13 | 備考 |
|--------|-----|-----|-----|------|
| similar_to_default | -0.02 | 削除 | **-0.02** | 復元 |
| hip_pitch_alternation | 1.0 | 1.5 | **2.0** | さらに強化 |
| stride_length | - | 0.8 | **削除** | 静的→失敗 |
| hip_pitch_opposite_sign | - | 1.5 | **削除** | 静的→失敗 |
| hip_pitch_range | - | 1.0 | **削除** | 静的→失敗 |
| hip_pitch_velocity | - | - | **0.5** | 新規：動的 |
| contact_alternation | - | - | **0.8** | 新規：動的 |
| roll_penalty | - | - | **-3.0** | V12対策 |

### 新規報酬関数

#### hip_pitch_velocity（動的）

```python
def _reward_hip_pitch_velocity(self):
    """hip_pitchが動いていることを報酬（静的固定を防ぐ）"""
    left_hip_vel = torch.abs(self.dof_vel[:, self.left_hip_pitch_idx])
    right_hip_vel = torch.abs(self.dof_vel[:, self.right_hip_pitch_idx])
    
    # 速度の合計を報酬（動いていることを報酬）
    velocity_sum = left_hip_vel + right_hip_vel
    reward = torch.clamp(velocity_sum / 2.0, max=1.0)
    
    has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
    return reward * has_command
```

#### contact_alternation（動的）

```python
def _reward_contact_alternation(self):
    """接地タイミングの交互性（両脚同時変化にペナルティ）"""
    contacts = self._get_foot_contacts()
    
    left_changed = contacts[:, 0] != self.last_contacts[:, 0]
    right_changed = contacts[:, 1] != self.last_contacts[:, 1]
    
    # 片方だけが変化 = 交互歩行
    one_changed = left_changed ^ right_changed
    # 両方同時に変化 = 両脚同期
    both_changed = left_changed & right_changed
    
    return (one_changed.float() - both_changed.float() * 0.5) * has_command
```

#### roll_penalty（V12対策）

```python
def _reward_roll_penalty(self):
    """Roll角ペナルティ（V12の-20°傾斜対策）"""
    roll_rad = self.base_euler[:, 0] * 3.14159 / 180.0
    return torch.square(roll_rad)
```

### 実行方法

```bash
cd rl_ws
uv run python scripts/biped_train_v13.py --max_iterations 500
```

### 数値目標

| 指標 | V11 | V12 | V13目標 |
|------|-----|-----|---------|
| hip_pitch相関 | 0.495 | 0.936 | **< 0.5** |
| X移動距離(10s) | 2.53 m | 0.66 m | **> 2.0 m** |
| Roll角 | - | -20° | **< 10°** |
| Base高さ | 0.434m | 0.326m | **> 0.40m** |

### 評価結果（過去最高）

**V13は過去最高の歩行品質を達成！**

#### 定量評価（10秒間headless評価）

| 指標 | V11 | V12 | V13目標 | V13結果 | 達成 |
|------|-----|-----|---------|---------|------|
| X移動距離(10s) | 2.53 m | 0.66 m | > 2.0 m | **2.385 m** | ○ |
| hip_pitch相関 | 0.495 | 0.936 | < 0.5 | 0.797 | △ |
| Roll角 | - | -20° | < 10° | **~5°** | ○ |
| Base高さ | 0.434m | 0.326m | > 0.40m | **0.446 m** | ○ |
| Pitch角 | -11.3° | 6° | - | **-9°** | ○ |

#### 詳細統計

```
Travel distance: X=2.385m, Y=0.633m, Total=2.468m
Average velocity: X=0.230 m/s (target: 0.300)  ← 目標の77%達成
Base height: Mean=0.446m, Std=0.0040m  ← 安定した高さ維持

DOF position range:
  hip_pitch: L=[-0.082, 0.219](0.301) R=[-0.019, 0.526](0.545)  ← 正の領域も使用！
  hip_roll:  L=[-0.098, 0.071](0.169) R=[-0.060, 0.101](0.160)  ← 適度な範囲
  knee:      L=[0.000, 0.751](0.751)  R=[-0.144, 0.820](0.964)

Left-Right hip_pitch correlation: 0.797
Total DOF range sum: 5.628 rad
```

#### 定性評価

**過去最高の歩行品質：**

- **自然な歩行動作**: 目視で「良い歩き方」と評価
- **hip_pitchが正の領域を使用**: V11/V12と異なり、前方スイング（正のhip_pitch）を実現
  - 左hip_pitch: [-0.082, **0.219**]（正の領域を使用）
  - 右hip_pitch: [-0.019, **0.526**]（正の領域を使用）
- **安定した高さ維持**: Base高さ0.446m、標準偏差0.004mと非常に安定
- **Roll角の改善**: V12の-20°から約5°に大幅改善
- **脚が固定されていない**: DOF velocity RMS 1.980で活発に動いている

#### V11/V12との比較

| 指標 | V11 | V12 | V13 | 評価 |
|------|-----|-----|-----|------|
| X移動距離(10s) | 2.53 m | 0.66 m | 2.39 m | V11に近い |
| hip_pitch相関 | 0.495 | 0.936 | 0.797 | V11より劣る |
| 左hip_pitch最大 | 0.000 | 0.037 | **0.219** | **大幅改善** |
| 右hip_pitch最大 | 0.000 | 0.000 | **0.526** | **大幅改善** |
| Base高さ | 0.434m | 0.326m | **0.446m** | **最高** |
| Roll角 | - | -20° | ~5° | **大幅改善** |
| 目視評価 | 生き物っぽい | 開脚固定 | **過去最高** | **最高** |

#### 成功要因分析

1. **静的報酬の削除**
   - stride_length, hip_pitch_opposite_sign, hip_pitch_range を削除
   - 固定姿勢への収束を防止

2. **動的報酬の強化**
   - hip_pitch_alternation: 1.0 → 2.0（速度ベース）
   - hip_pitch_velocity: 新規追加（動いていることを報酬）
   - contact_alternation: 新規追加（接地タイミングの交互性）

3. **similar_to_defaultの復元**
   - 極端な姿勢への収束を防止
   - 適度なdefault姿勢への回帰

4. **roll_penaltyの追加**
   - V12の横傾斜問題を解決

#### 残存課題

- **hip_pitch相関0.797**: まだ両脚同期傾向が残る（目標は負の相関）
- **Y方向のドリフト**: 0.633mの横ずれ
- **前傾姿勢**: Pitch角-9°程度の前傾が残存

#### 今後の改善方針

1. hip_pitch_alternation をさらに強化して相関を負に
2. 横方向移動のペナルティ追加
3. Pitch角ペナルティの調整


## V14: 低重心歩行（失敗）

### 目的

V13の歩行品質を維持しつつ、より低い重心で歩行させる。膝と腰のピッチを深く曲げた姿勢での歩行を目指す。

### 変更点

| パラメータ | V13 | V14 | 目的 |
|-----------|-----|-----|------|
| base_height_target | 0.40 m | 0.35 m | 目標重心を下げる |
| base_height報酬 | -15.0 | -20.0 | 高さ制約を強化 |
| default_knee_pitch | -0.52 rad | -0.78 rad | 膝を深く曲げた初期姿勢 |
| default_ankle_pitch | 0.52 rad | 0.78 rad | バランス維持 |
| base_init_pos Z | 0.45 m | 0.40 m | 初期高さを下げる |

### 評価結果（失敗）

**V14は脚が固定された歩行に退化**

#### 定量評価（10秒間headless評価）

| 指標 | V13 | V14 | 評価 |
|------|-----|-----|------|
| X移動距離(10s) | 2.385 m | 2.107 m | 低下 |
| Y移動距離 | 0.633 m | -0.595 m | 逆方向にドリフト |
| hip_pitch相関 | 0.797 | **-0.557** | 改善（交互歩行化） |
| Base高さ | 0.446 m | **0.384 m** | 目標達成 |
| Roll角 | ~5° | ~6° | 同等 |
| 平均X速度 | 0.230 m/s | 0.202 m/s | 低下 |

#### 詳細統計

```
Travel distance: X=2.107m, Y=-0.595m, Total=2.189m
Average velocity: X=0.202 m/s (target: 0.300) ← 目標の67%
Base height: Mean=0.384m, Std=0.0079m ← 目標0.35mに近い

DOF position range:
  hip_pitch: L=[-0.526, 0.000](0.526) R=[-0.164, 0.420](0.584)
             ↑ 左脚は負の領域のみ使用（後方スイングのみ）
  knee:      L=[0.000, 1.322](1.323)  R=[-0.233, 0.993](1.226)
             ↑ 大きな可動域だが左右非対称

Left-Right hip_pitch correlation: -0.557 ← 交互歩行の兆候！
Total DOF range sum: 7.159 rad ← V13の5.628より大きい
```

#### 定性評価（目視確認）

**問題点：**
- 脚が開いた状態で固定される傾向
- 左脚のhip_pitchが負の領域のみ使用（前方スイングなし）
- 左右の動きが非対称

**ポジティブな点：**
- hip_pitch相関が-0.557と初めて負の値に（交互歩行の兆候）
- 目標の低重心（0.384m）はほぼ達成
- DOF可動域の総和は増加

### 失敗原因の分析

1. **目標高さの急激な変更**
   - 0.40m → 0.35m は15%の低下
   - 初期姿勢と目標姿勢のギャップが大きすぎた可能性

2. **base_height報酬の過度な強化**
   - -15.0 → -20.0 は高さ維持を優先しすぎ
   - 歩行動作よりも姿勢維持が優先された

3. **similar_to_defaultとの矛盾**
   - default姿勢を深く曲げた状態に変更
   - similar_to_default報酬が新しい固定姿勢への収束を促進した可能性

### V12との類似点

| 問題 | V12 | V14 |
|------|-----|-----|
| 原因 | 静的な開脚を報酬 | 静的な低姿勢を強制 |
| 結果 | 脚を開いたまま固定 | 脚を曲げたまま固定 |
| 教訓 | 静的位置報酬は危険 | 姿勢制約の急激な変更は危険 |

### V15への改善方針

1. **段階的なアプローチ**
   - base_height_targetを0.40m → 0.38m → 0.35mと段階的に下げる
   - 一度に大きな変更を避ける

2. **base_height報酬の調整**
   - -20.0は強すぎる、-15.0のまま維持
   - または目標達成時にボーナスを与える形式に変更

3. **default姿勢の維持**
   - default_joint_anglesはV13のまま維持
   - 学習で自然に低姿勢を獲得させる

4. **代替アプローチ：knee_bent報酬**
   - 膝の曲げを直接報酬する
   - 静的位置ではなく、適度な範囲での曲げを報酬

```python
# V15候補：段階的低重心化
reward_cfg = {
    "base_height_target": 0.38,  # 0.40 → 0.38 (段階的)
    "base_height": -15.0,        # 維持
    # default_joint_anglesはV13のまま
}
```


## V15: 低重心歩行（V13ベース・目標高さのみ変更）

### 目的

V14の失敗を踏まえ、V13をベースに`base_height_target`のみを変更。
学習で自然に低姿勢を獲得させるアプローチ。

### 変更点

| パラメータ | V13 | V14 | V15 | 
|-----------|-----|-----|-----|
| base_height_target | 0.40 m | 0.35 m | 0.35 m |
| base_height報酬 | -15.0 | -20.0 | **-15.0** |
| default_knee_pitch | -0.52 rad | -0.78 rad | **-0.52 rad** |
| base_init_pos Z | 0.45 m | 0.40 m | **0.45 m** |

V14との違い：base_height_target以外はV13と同一。

### 評価結果（部分的成功）

**歩行品質は維持されたが、目標高さには到達せず**

#### 定量評価（10秒間headless評価）

| 指標 | V13 | V14 | V15 | 目標 | 評価 |
|------|-----|-----|-----|------|------|
| X移動距離(10s) | 2.385 m | 2.107 m | 2.140 m | - | V13に近い |
| Base高さ | 0.446 m | 0.384 m | **0.452 m** | 0.35 m | **未達成** |
| hip_pitch相関 | 0.797 | -0.557 | **-0.395** | < 0 | 改善 |
| Roll角 | ~5° | ~6° | ~5° | - | 良好 |
| Yaw角 | - | - | **-26°** | - | **問題** |

#### 詳細統計

```
Travel distance: X=2.140m, Y=-0.764m, Total=2.272m
Average velocity: X=0.225 m/s (target: 0.300) ← 目標の75%
Base height: Mean=0.452m, Std=0.0040m ← 目標0.35mに全く近づかない

DOF position range:
  hip_pitch: L=[-0.299, 0.000](0.299) R=[-0.086, 0.413](0.499)
  knee:      L=[0.000, 0.725](0.725)  R=[-0.134, 0.632](0.766)

Left-Right hip_pitch correlation: -0.395 ← 交互歩行の兆候
Total DOF range sum: 5.281 rad
```

#### 定性評価（目視確認）

**ポジティブな点：**
- 「足先の運びはよくなった」との評価
- hip_pitch相関が-0.395と負の値（V13の0.797から大幅改善）
- V13と同等の移動距離を維持

**問題点：**
- 胴体位置が全く下がっていない（0.452m ≈ V13の0.446m）
- Yaw角が-26°と大きく回転してしまっている
- Y方向のドリフト（-0.764m）

### 失敗原因の分析

1. **base_height報酬の強度不足**
   - -15.0では目標高さ0.35mへの収束が不十分
   - 高さ0.45mでも歩行できるため、わざわざ低くならない

2. **初期高さと目標高さのギャップ**
   - 初期高さ0.45m → 目標0.35m（23%の低下）
   - 学習初期に低い姿勢を探索する機会がない

3. **similar_to_defaultの影響**
   - default姿勢（膝-0.52rad）への回帰が優先される
   - 低姿勢（膝をもっと曲げる）への探索が抑制される

### V13/V14/V15の比較

| 指標 | V13 | V14 | V15 |
|------|-----|-----|-----|
| Base高さ | 0.446 m | 0.384 m | 0.452 m |
| X移動距離 | 2.385 m | 2.107 m | 2.140 m |
| hip_pitch相関 | 0.797 | -0.557 | -0.395 |
| 歩行品質 | ○ | × | ○ |
| 低重心達成 | × | △ | × |

**観察：**
- V14は低重心に近づいたが歩行品質が低下
- V15は歩行品質を維持したが低重心に近づかない
- トレードオフの関係にある

### V16への改善方針

目標高さを維持しつつ、ペナルティを強化するか、別のアプローチが必要。

**案1: base_height報酬を強化**
```python
"base_height": -25.0,  # -15.0 → -25.0
```

**案2: 高さに応じたボーナス/ペナルティ**
```python
# 目標より高い場合のみペナルティを強化
def _reward_base_height(self):
    height_error = self.base_pos[:, 2] - self.reward_cfg["base_height_target"]
    # 高すぎる場合はペナルティ強化
    penalty = torch.where(height_error > 0, 
                          height_error ** 2 * 2.0,  # 高すぎる場合は2倍
                          height_error ** 2)
    return penalty
```

**案3: similar_to_defaultを弱化**
```python
"similar_to_default": -0.01,  # -0.02 → -0.01
```

**推奨: 案1（base_height報酬強化）を試す**


## V16: 低重心歩行（base_height報酬強化）- 失敗

### 目的

V15でbase_height報酬-15.0では目標高さに収束しなかったため、報酬を強化して収束を促進。

### 変更点

| パラメータ | V13 | V15 | V16 |
|-----------|-----|-----|-----|
| base_height_target | 0.40 m | 0.35 m | 0.35 m |
| base_height報酬 | -15.0 | -15.0 | **-30.0** |

### 評価結果（失敗）

**報酬強化しても胴体高さは全く下がらず、歩行品質が低下**

#### 定量評価（10秒間headless評価）

| 指標 | V13 | V15 | V16 | 目標 | 評価 |
|------|-----|-----|-----|------|------|
| X移動距離(10s) | 2.385 m | 2.140 m | **1.588 m** | - | **大幅低下** |
| Base高さ | 0.446 m | 0.452 m | **0.433 m** | 0.35 m | 未達成 |
| hip_pitch相関 | 0.797 | -0.395 | **0.155** | < 0 | 退化 |
| Yaw角 | - | -26° | **-29°** | ~0° | 悪化 |
| 平均X速度 | 0.230 m/s | 0.225 m/s | **0.176 m/s** | 0.3 m/s | 低下 |

#### 詳細統計

```
Travel distance: X=1.588m, Y=-0.799m, Total=1.778m
Average velocity: X=0.176 m/s (target: 0.300) ← 目標の59%に低下
Base height: Mean=0.433m, Std=0.003m ← 目標0.35mに全く近づかない

DOF position range:
  hip_pitch: L=[-0.179, 0.000](0.179) R=[0.000, 0.520](0.520)
             ↑ 左脚の可動域が極端に狭い
  knee:      L=[0.000, 0.907](0.907)  R=[-0.083, 0.811](0.893)

Left-Right hip_pitch correlation: 0.155 ← V15の-0.395から退化
Total DOF range sum: 5.099 rad
```

#### 定性評価（目視確認）

**問題点：**
- 「足先の交差がまだやっぱり甘い」
- 「胴体が全然下がってない」
- 「今の半分の高さくらいまで下げたい」（現在0.433m → 目標0.22m程度）
- Yaw角が-29°と大きく回転

**観察：**
- base_height報酬を2倍（-15.0 → -30.0）にしても効果なし
- むしろ歩行品質（移動距離、交互歩行）が低下

### 失敗原因の分析

1. **報酬強度の問題ではない**
   - -15.0 → -30.0 でも胴体高さは0.452m → 0.433mとわずか2cm低下
   - 報酬の形式自体に問題がある可能性

2. **初期姿勢と目標姿勢のギャップが大きすぎる**
   - 初期高さ0.45m、default姿勢（膝-0.52rad）から
   - 目標0.35mに到達するには膝を大幅に曲げる必要がある
   - 探索空間が広すぎて最適解に到達できない

3. **similar_to_defaultとの矛盾**
   - default姿勢への回帰報酬が低姿勢探索を阻害

4. **報酬設計の根本的問題**
   - 現在の`base_height`報酬は二乗誤差
   - 高さ0.45mと0.35mの差（0.1m）の二乗 = 0.01
   - ペナルティ-30.0 × 0.01 = -0.3 は他の報酬に比べて小さい

### V13-V16の比較総括

| 指標 | V13 | V14 | V15 | V16 |
|------|-----|-----|-----|-----|
| Base高さ | 0.446 m | 0.384 m | 0.452 m | 0.433 m |
| X移動距離 | 2.385 m | 2.107 m | 2.140 m | 1.588 m |
| hip_pitch相関 | 0.797 | -0.557 | -0.395 | 0.155 |
| base_height報酬 | -15.0 | -20.0 | -15.0 | -30.0 |
| default姿勢変更 | × | ○ | × | × |
| 歩行品質 | ○ | × | ○ | △ |

**観察：**
- V14は低重心（0.384m）に近づいたが、default姿勢変更で歩行崩壊
- V15/V16は報酬のみ変更したが、胴体高さがほぼ変わらない
- 単純な報酬強化では解決しない

### V17への改善方針

**根本的なアプローチ変更が必要**

**案1: 非対称報酬（高すぎる場合のみ強ペナルティ）**
```python
def _reward_base_height(self):
    height_error = self.base_pos[:, 2] - self.reward_cfg["base_height_target"]
    # 目標より高い場合は10倍のペナルティ
    penalty = torch.where(height_error > 0, 
                          height_error ** 2 * 10.0,
                          height_error ** 2)
    return penalty
```

**案2: 目標高さに近い初期状態から開始**
```python
"base_init_pos": [0.0, 0.0, 0.35],  # 目標と同じ高さから開始
```

**案3: カリキュラム学習**
- まず0.40mで学習 → 0.38m → 0.35m と段階的に

**案4: 目標高さを現実的な値に変更**
- 現在の半分（0.22m）は物理的に困難な可能性
- まず0.38m達成を目指す

**推奨: 案2（初期高さを目標と同じに）または案1（非対称報酬）を試す**

---

## V17: 低重心歩行（3方針統合）- 学習中

### 目的

V15/V16の失敗（報酬強化だけでは胴体高さが下がらない）を踏まえ、**3つの方針を統合**して低重心歩行を実現する。

### 設計方針

V16までの分析から、単一のアプローチでは不十分と判断。以下の3方針を同時に適用：

1. **初期姿勢を低く設定（案2）**
   - hip_pitch: -0.35 rad（-20°、前傾）
   - knee_pitch: -0.78 rad（-45°、深く曲げる）
   - ankle_pitch: 0.43 rad（バランス補正）
   - base_init_pos: 0.35m（目標高さと同じ）

2. **参照軌道に低姿勢を組み込む**
   - base_height_target: 0.35m

3. **非対称報酬（案1）**
   - 高すぎる（error > 0）: -40.0 × error²（強いペナルティ）
   - 低すぎる（error < 0）: -5.0 × error²（弱いペナルティ）
   - **比率8:1**で低い方向への探索を許容

### 変更点

| パラメータ | V13 | V16 | V17 |
|-----------|-----|-----|-----|
| base_init_pos | 0.45m | 0.45m | **0.35m** |
| hip_pitch (default) | 0.0 | 0.0 | **-0.35 rad** |
| knee_pitch (default) | -0.52 rad | -0.52 rad | **-0.78 rad** |
| ankle_pitch (default) | 0.52 rad | 0.52 rad | **0.43 rad** |
| base_height_target | 0.40m | 0.35m | **0.35m** |
| base_height報酬 | -15.0 (対称) | -30.0 (対称) | **非対称** (-40/-5) |

### 非対称報酬の実装

```python
def _reward_base_height(self):
    """V17: 非対称ベース高さペナルティ

    高すぎる場合は強いペナルティ、低すぎる場合は弱いペナルティ。
    これにより、低い姿勢への探索を促進する。

    - 高すぎる（error > 0）: -40.0 * error²
    - 低すぎる（error < 0）: -5.0 * error²

    比率: 8:1で低い方向への探索を許容
    """
    height = self.base_pos[:, 2]
    error = height - self.base_height_target

    penalty = torch.where(
        error > 0,
        self.height_penalty_high * torch.square(error),  # 高い: 強いペナルティ
        self.height_penalty_low * torch.square(error)    # 低い: 弱いペナルティ
    )

    return penalty
```

### 学習設定

```bash
cd rl_ws
uv run python scripts/biped_train_v17.py --max_iterations 500
```

| パラメータ | 値 |
|-----------|-----|
| num_envs | 4096（デフォルト） |
| max_iterations | 500 |
| num_steps_per_env | 24 |
| precision | FP32 |

### リソース使用状況（学習中の測定）

M3 Pro MacBook Pro（36GB統合メモリ、GPU 14コア）での実測値：

| 指標 | 値 | 解釈 |
|------|-----|------|
| CPU使用率（プロセス） | 43.3% | 11コア中約4〜5コア使用 |
| メモリ使用量 | 3.2GB | 36GBの約9%、**大幅な余裕あり** |
| GPU使用率 | **81%** | 高負荷だがまだ余裕あり |
| GPU割り当てメモリ | 6.8GB | 統合メモリで柔軟に利用 |
| システム全体CPU idle | 75% | CPUはボトルネックではない |

### V18への申し送り事項（学習高速化）

**1. 並列環境数（num_envs）の増加を推奨**

現在4096だが、メモリに大幅な余裕があるため増加可能：

| num_envs | 推定メモリ | 推奨度 |
|----------|-----------|--------|
| 4096（現在） | 3.2GB | - |
| 8192 | ~6.4GB | **推奨** |
| 10240 | ~8GB | 試行可 |
| 12288 | ~10GB | 試行可 |

```bash
# V18での実行例
uv run python scripts/biped_train_v18.py -B 8192 --max_iterations 500
```

**期待効果**: サンプル収集速度が2倍 → 学習時間が約半分に短縮

**2. その他の高速化オプション**

| パラメータ | 現在値 | 推奨変更 | 効果 |
|-----------|--------|---------|------|
| num_steps_per_env | 24 | 32 | GPU転送オーバーヘッド削減 |
| num_mini_batches | 4 | 2 | バッチサイズ増加でGPU効率向上 |
| substeps | 2 | 1 | 物理計算半減（安定性に注意） |

**3. 注意事項**

- GPU使用率81%は既に高いが、num_envs増加でバッチ処理効率が上がるため総スループットは向上する可能性がある
- M3 Proの統合メモリアーキテクチャはCPU/GPU間のデータ転送が高速なため、大きなnum_envsが有利
- 極端な設定変更は1つずつ試すこと（複数同時変更は問題切り分けが困難）

### 評価結果

**V17は低重心の達成に成功したが、進行方向が後方かつ斜めという重大な問題が発生。**

#### 定量評価（10秒間headless評価）

| 指標 | V13 | V16 | V17目標 | V17結果 | 達成 |
|------|-----|-----|---------|---------|------|
| Base高さ | 0.446 m | 0.433 m | **0.35 m** | **0.371 m** | **◎** |
| X移動距離(10s) | 2.385 m | 1.588 m | > 2.0 m | **-2.832 m** | ✗✗ |
| Y移動距離(10s) | 0.633 m | -0.799 m | < 1.0 m | **9.642 m** | ✗✗ |
| hip_pitch相関 | 0.797 | 0.155 | < 0.5 | **0.092** | ○ |
| Yaw角(10s) | - | -29° | < 20° | **-63°** | ✗✗ |
| DOF velocity RMS | 2.338 | 2.445 | - | 2.666 | - |

#### 詳細統計

```
Travel distance: X=-2.832m, Y=9.642m, Total=10.049m
Average velocity: X=-0.773 m/s (target: 0.300), Y=0.893 m/s
Base height: Mean=0.371m, Std=0.0110m  ← 目標0.35mに近づいた！

DOF position range (rad):
  hip_pitch:    L=[-0.571, 0.000](0.571)  R=[-0.530, 0.000](0.530)
  hip_roll:     L=[-0.386, 0.029](0.414)  R=[-0.276, 0.339](0.616)
  knee:         L=[-0.429, 1.033](1.461)  R=[-0.772, 0.933](1.705)
  ankle_pitch:  L=[-1.701,-0.787](0.914)  R=[-2.067,-0.567](1.500)
  ankle_roll:   L=[ 0.436, 1.353](0.917)  R=[ 0.419, 1.156](0.737)

Left-Right hip_pitch correlation: 0.092
Total DOF range sum: 9.365 rad
```

#### 定性評価

**成功点：**
- **Base高さ 0.371m**: V13(0.446m)から**75mm低下**、目標0.35mまであと21mm
- **hip_pitch相関 0.092**: V13(0.797)から大幅改善、ほぼ無相関に近い
- **DOF range sum 9.365 rad**: V13(5.628rad)より大きく、関節の動きが活発

**問題点：**
- **後方移動（X=-2.832m）**: 前進指令（target: 0.3 m/s）に対し**後退**している
- **横方向移動（Y=9.642m）**: 大きく横にずれる
- **Yaw角 -63°**: 10秒間で63°左に旋回、方向安定性が崩壊

#### 失敗の原因分析

1. **default姿勢の変更が観測に影響**
   - `default_joint_angles`を変更したことで、`_get_obs()`内の`(self.dof_pos - self.default_dof_pos)`が変化
   - ポリシーが「どの方向に進むべきか」を誤って学習した可能性

2. **hip_pitchが負の領域のみを使用**
   - L=[-0.571, 0.000], R=[-0.530, 0.000]
   - 正の領域（前方スイング）を使用していない → 後方移動の原因

3. **Yaw報酬の不足**
   - V13と同じ`heading_alignment`報酬があるが、低姿勢での動特性変化に対応できていない

4. **非対称報酬の副作用**
   - 高さを下げることに成功したが、歩行方向の制御が犠牲に
   - base_height報酬に注力しすぎて、tracking_lin_vel報酬の相対的重要度が低下

### V17の成果と教訓

**成果（部分的成功）：**
| 目標 | 達成度 | 詳細 |
|------|--------|------|
| 低重心化 | **◎** | 0.446m → 0.371m（-75mm） |
| 交互歩行 | **○** | 相関0.797 → 0.092 |
| 関節可動域活用 | **○** | 5.628rad → 9.365rad |

**問題（要改善）：**
| 問題 | 深刻度 | 原因 |
|------|--------|------|
| 後方移動 | **✗✗** | hip_pitchが負の領域のみ |
| 横移動 | **✗✗** | Yaw制御の不足 |
| 方向安定性 | **✗✗** | heading報酬の相対的低下 |

### V18への改善方針

1. **default姿勢を元に戻す**
   - default_joint_anglesはV13と同じにする
   - 初期姿勢（base_init_pos）のみ低く設定
   - 観測空間の一貫性を維持

2. **前進方向の報酬を強化**
   - `tracking_lin_vel`のスケールを増加（1.5 → 2.5）
   - 後方移動に明示的ペナルティを追加

3. **heading報酬の強化**
   - `heading_alignment`のスケールを増加
   - Yaw角速度へのペナルティを追加

4. **非対称報酬の調整**
   - 高さペナルティの比率を調整（8:1 → 4:1など）
   - 他の報酬とのバランスを再検討

---

## V18: 低重心＋前進方向修正

### 目的

V17で達成した低重心（0.371m）を維持しつつ、**後方移動・横移動の問題を修正**する。

### V17の問題点と対策

| V17の問題 | 原因 | V18での対策 |
|-----------|------|-------------|
| 後方移動（X=-2.8m） | default姿勢変更で観測空間が変化 | default姿勢をV13に戻す |
| 横移動（Y=+9.6m） | Yaw制御の不足 | yaw_penalty報酬を追加 |
| Yaw角-63° | heading報酬の相対的低下 | tracking_lin_vel強化（1.5→2.5） |

### 設計変更

#### 1. default_joint_anglesをV13に戻す

```python
# V17（問題あり）
"left_hip_pitch_joint": -0.35,   # 観測に影響
"left_knee_pitch_joint": -0.78,

# V18（V13と同じ）
"left_hip_pitch_joint": 0.0,     # 元に戻す
"left_knee_pitch_joint": -0.52,  # 元に戻す
```

**理由**: `_get_obs()`内で`(self.dof_pos - self.default_dof_pos)`を計算しているため、default姿勢を変更すると観測空間が変化し、ポリシーが進行方向を誤学習する。

#### 2. base_init_posは低いまま維持

```python
"base_init_pos": [0.0, 0.0, 0.35],  # V17と同じ（低い位置から開始）
```

**理由**: 初期位置は観測に影響しないため、低い位置から開始しても問題ない。

#### 3. tracking_lin_vel報酬を強化

```python
# V17
"tracking_lin_vel": 1.5,

# V18
"tracking_lin_vel": 2.5,  # 67%増加
```

**理由**: base_height報酬（非対称）が強いため、前進報酬の相対的重要度を上げる。

#### 4. yaw_penalty報酬を追加

```python
def _reward_yaw_penalty(self):
    """Yaw角速度へのペナルティ（直進性向上）
    
    V17の問題: 10秒で-63°旋回
    解決策: Yaw角速度にペナルティを与えて直進を促進
    """
    yaw_rate = self.base_ang_vel[:, 2]  # Z軸周りの角速度
    return torch.square(yaw_rate)

# 報酬スケール
"yaw_penalty": -2.0,
```

#### 5. 非対称高さ報酬の比率を調整

```python
# V17: 8:1（高すぎるペナルティ >> 低すぎるペナルティ）
"height_penalty_high": -40.0,
"height_penalty_low": -5.0,

# V18: 6:1（少し緩和）
"height_penalty_high": -30.0,
"height_penalty_low": -5.0,
```

**理由**: 高さペナルティが強すぎると他の報酬（tracking_lin_vel等）が相対的に弱くなる。

### 報酬構造の比較

| 報酬名 | V13 | V17 | V18 | 変更理由 |
|--------|-----|-----|-----|----------|
| tracking_lin_vel | 1.5 | 1.5 | **2.5** | 前進報酬強化 |
| base_height | -15.0（対称） | 非対称(-40/-5) | **非対称(-30/-5)** | 比率緩和 |
| yaw_penalty | - | - | **-2.0** | 直進性向上 |
| similar_to_default | -0.02 | -0.02 | -0.02 | 維持 |

### default姿勢とbase_init_posの分離

| パラメータ | 役割 | V17 | V18 |
|-----------|------|-----|-----|
| default_joint_angles | 観測の基準・姿勢維持報酬の目標 | 変更（問題） | **V13に戻す** |
| base_init_pos | シミュレーション開始位置 | 0.35m（低い） | **0.35m（維持）** |

**キーポイント**: 観測空間に影響する`default_joint_angles`は変更せず、観測に影響しない`base_init_pos`のみで初期状態を制御する。

### 学習設定

```bash
cd rl_ws
uv run python scripts/biped_train_v18.py --max_iterations 500
```

| パラメータ | V17 | V18 | 変更理由 |
|-----------|-----|-----|----------|
| num_envs | 4096 | **12288** | サンプル多様性向上（下記参照） |
| max_iterations | 500 | 500 | 維持 |
| num_steps_per_env | 24 | 24 | 維持 |

### 並列環境数（num_envs）の設定と影響

V17（num_envs=4096）からV18（num_envs=12288）へ変更した。

#### num_envsとは

`num_envs`はGPU上で同時にシミュレートする並列環境の数。PPOは「ベクトル化アーキテクチャ」を使用し、複数のロボットを同時に動かしてデータを収集する。

```
1イテレーションのバッチサイズ = num_envs × num_steps_per_env (horizon_length)

V17: 4096 × 24 = 98,304 サンプル
V18: 12288 × 24 = 294,912 サンプル（3倍）
```

#### num_envsを増やす影響

| 項目 | num_envs↑ の効果 |
|------|------------------|
| サンプル多様性 | 🟢 向上（多様な状況を同時経験） |
| 勾配推定の分散 | 🟢 低下（安定した学習） |
| 1イテレーションの計算時間 | 🔴 **増加（遅くなる）** |
| サンプル効率 | 🟡 低下の可能性あり |
| GPUメモリ使用量 | 🔴 増加 |

#### 実測結果

| 指標 | V17 (4096) | V18 (12288) | 変化 |
|------|------------|-------------|------|
| 100iterあたりの時間 | **3分** | **8分** | **2.7倍遅い** |
| 500iter総時間 | 約15分 | 約40分 | 2.7倍遅い |
| GPU使用率 | 81% | 93% | +12% |
| GPU割り当てメモリ | 6.8GB | 8.7GB | +1.9GB |

#### 結論: num_envsを増やしても「高速化」にはならない

- **num_envs↑ → ウォールクロック時間は増加**（V18はV17の2.7倍遅い）
- **利点はサンプル多様性の向上のみ**
- Andrychowicz et al. (2021)の知見: 「Nを増やすとスループットは上がるが性能は悪化する傾向」
- Isaac Gymの推奨値は4096〜8192程度
- **V19ではnum_envs=4096に戻すことを推奨**

#### 今後の学習時間短縮オプション（num_envs以外）

| 方法 | 変更内容 | リスク |
|------|---------|--------|
| 学習率増加 | 0.001 → 0.002 | 収束不安定の可能性 |
| num_learning_epochs削減 | 5 → 3 | 学習効率低下の可能性 |
| early stopping | 報酬収束時に終了 | 実装が必要 |

### 数値目標

| 指標 | V13 | V17 | V18目標 |
|------|-----|-----|---------|
| Base高さ | 0.446 m | 0.371 m | **< 0.40 m** |
| X移動距離(10s) | +2.385 m | -2.832 m | **> +2.0 m** |
| Y移動距離(10s) | +0.633 m | +9.642 m | **< 1.5 m** |
| hip_pitch相関 | 0.797 | 0.092 | **< 0.5** |
| Yaw角(10s) | - | -63° | **< 30°** |

### 評価結果

| 指標 | V13 | V17 | V18目標 | V18結果 | 達成 |
|------|-----|-----|---------|---------|------|
| Base高さ | 0.446 m | 0.371 m | < 0.40 m | **0.457 m** | ❌ |
| Base高さStd | 0.004 m | - | - | **0.053 m** | - |
| X移動距離(10s) | +2.385 m | -2.832 m | > +2.0 m | **+0.009 m** | ❌ |
| Y移動距離(10s) | +0.633 m | +9.642 m | < 1.5 m | **+0.000 m** | ✅ |
| hip_pitch相関 | 0.797 | 0.092 | < 0.5 | **0.859** | ❌ |
| DOF velocity RMS | 1.98 | - | - | **4.79** | - |

### 評価結果の分析：完全な失敗

V18は**その場でジャンプを繰り返すだけ**の挙動を示し、歩行には全く成功していない。

#### 観測された問題パターン

時系列データから明確な周期的パターンが確認された：

```
t=0s: pos=(x, y, z=0.387) | vel=(-0.43, 0)
t=1s: pos=(x, y, z=0.444) | vel=(-0.11, 0)
t=2s: pos=(x, y, z=0.387) | vel=(-0.43, 0)  ← 完全に繰り返し
t=3s: pos=(x, y, z=0.444) | vel=(-0.11, 0)
...
```

- **Z座標が0.387m〜0.444mを往復**（Std=0.053m、V13の13倍）
- **X移動距離がほぼゼロ**（10秒で0.009m）
- **hip_pitch相関が0.859**（左右同期＝ジャンプ）
- **DOF velocity RMSが4.79**（V13の2.4倍、激しい振動）

#### 失敗の根本原因分析

**1. 報酬の競合・過剰ペナルティ**

V18で追加した方向制御ペナルティが過剰だった可能性：
- `backward_penalty: -2.0`
- `yaw_penalty: -3.0`
- `lateral_velocity_penalty: -1.5`

これらが`tracking_lin_vel`や`forward_progress`と競合し、**「動かないことが最も安全」**という局所最適解に陥った。

**2. 初期位置の問題**

`base_init_pos = 0.35m`は物理的に低すぎる可能性：
- default_joint_anglesはV13の立位姿勢（高さ≈0.45m相当）
- 初期位置0.35mから始まると、シミュレーション開始直後に落下・バウンド
- この不安定な開始状態が学習を阻害

**3. 非対称高さ報酬の悪影響**

`height_penalty_high: 40.0` vs `height_penalty_low: 5.0`の8:1比率が、
ジャンプ動作を許容しすぎた可能性：
- 低い位置（0.387m）への遷移は軽いペナルティ
- 高い位置（0.444m）への遷移は重いペナルティ
- → 上下動を繰り返すことで「平均的に低い」状態を維持する戦略

### V19への改善提案

#### 方針A: V13ベースで最小限の変更（推奨）

V13は良好な歩行を達成している。V13をベースに**1つずつ**変更を加える：

```python
# V19A: V13 + 低い目標高さのみ
base_height_target: 0.40  # V13の0.45から少し下げる
height_penalty_high: 15.0  # V17の40.0ほど極端でなく
height_penalty_low: 10.0   # 非対称比率を2:1程度に緩和

# 方向制御ペナルティは追加しない（V13で十分機能している）
```

#### 方針B: 初期位置の修正

```python
# base_init_posをdefault姿勢と整合させる
base_init_pos: [0.0, 0.0, 0.45]  # V13と同じ（0.35は低すぎる）
```

#### 方針C: 段階的な目標高さ低下

```python
# カリキュラム学習的アプローチ
# Phase 1: 0.43m目標で学習（V13から-0.02m）
# Phase 2: 収束後、0.40m目標で継続学習
# Phase 3: さらに0.37mへ
```

#### V19の推奨設定

| パラメータ | V18（失敗） | V19（提案） | 理由 |
|-----------|-------------|-------------|------|
| base_init_pos | 0.35m | **0.45m** | V13と同じ安定した開始位置 |
| base_height_target | 0.35m | **0.40m** | 段階的に下げる |
| height_penalty_high | 40.0 | **15.0** | 極端な非対称を緩和 |
| height_penalty_low | 5.0 | **10.0** | 比率を2:1に |
| backward_penalty | -2.0 | **削除** | V13で不要だった |
| yaw_penalty | -3.0 | **削除** | V13で不要だった |
| lateral_velocity_penalty | -1.5 | **削除** | V13で不要だった |
| num_envs | 12288 | **4096** | V13と同じ（高速化） |

**原則**: 「一度に1つだけ変える」ことで、失敗原因を特定しやすくする。

---

## V19: V13ベース + 穏やかな非対称高さ報酬（実験結果）

### 設計方針

V18の完全失敗を受け、**V13ベースに戻り最小限の変更のみ**加える：

1. **V13の動的報酬構造を維持** - 歩行成功実績あり
2. **base_init_pos = 0.45m**（V13と同じ）- 安定した開始位置
3. **base_height_target = 0.35m** - V18と同じ目標（低重心化）
4. **非対称比率を2:1に緩和** - V17の8:1は極端すぎた
5. **方向制御ペナルティは追加しない** - V18の失敗原因

#### 変更サマリー

| パラメータ | V13 | V18（失敗） | V19 |
|-----------|-----|-------------|-----|
| ベース | - | V17改変 | **V13** |
| base_init_pos | 0.45m | 0.35m | **0.45m** |
| base_height_target | 0.40m | 0.35m | **0.35m** |
| height_penalty_high | なし | 40.0 | **15.0** |
| height_penalty_low | なし | 5.0 | **10.0** |
| 非対称比率 | 対称 | 8:1 | **2:1** |
| 方向ペナルティ | なし | あり | **なし** |
| num_envs | 4096 | 12288 | **4096** |

### 評価結果

```
=== BSL-Droid Biped Walking Evaluation ===
Experiment: biped-walking-v19
Checkpoint: model_499.pt

t= 0.00s | pos=(-0.000,  0.000, 0.447)
t= 1.00s | pos=(-0.297,  0.010, 0.358) | rpy=(  5.1,  -0.9,  -8.8)°
t= 2.00s | pos=(-0.618,  0.028, 0.345) | rpy=(  0.2,  -2.4, -11.4)°
t= 3.00s | pos=(-0.929,  0.063, 0.345) | rpy=( -4.9,  -2.2, -15.4)°
t= 5.00s | pos=(-1.553,  0.178, 0.345) | rpy=(-12.2,  -1.3, -20.8)°
t= 7.00s | pos=(-2.165,  0.361, 0.341) | rpy=(-12.8,   0.3, -26.4)°
t= 9.00s | pos=(-2.740,  0.601, 0.328) | rpy=(-11.6,  -0.1, -32.3)°

Travel distance:
  X: -3.011 m  ← 後退！
  Y: 0.723 m

Average velocity:
  X: -0.308 m/s (target: 0.300)  ← 目標と逆方向

Base height:
  Mean: 0.347 m  ← 目標0.35mにほぼ一致！
  Std: 0.0140 m  ← 非常に安定

Left-Right hip_pitch correlation: -0.428
Total DOF range sum: 7.462 rad
```

### 評価分析

#### 成功点 ✅

1. **目標高さ達成**: 平均0.347m（目標0.35m）、標準偏差わずか0.014m
2. **安定した姿勢**: V18のようなジャンプなし、滑らかな歩行
3. **良好な関節動作**: DOF range sum = 7.462rad（歩行として十分）
4. **交互歩行の兆候**: hip_pitch相関 -0.428

#### 問題点 ❌

1. **進行方向が逆**: X = -3.011m（後退）、目標は+X方向（前進）
2. **Yaw回転**: 0° → -32.3°（左旋回しながら後退）
3. **横方向ドリフト**: Y = +0.723m

### V17/V19比較

| 指標 | V17 | V19 | 評価 |
|------|-----|-----|------|
| X移動距離 | -2.832m | **-3.011m** | 両方後退 |
| Y移動距離 | -2.195m | **+0.723m** | V19改善 |
| 高さ平均 | 0.371m | **0.347m** | V19が目標達成 |
| 高さStd | 0.0251m | **0.0140m** | V19が安定 |

### 結論

**姿勢は素晴らしいが、進行方向が逆**

V19は低重心歩行（0.347m）を非常に安定して実現したが、後退してしまう。
V17も後退しており、低重心化の報酬設計自体が後退を誘発している可能性がある。

---

## V20: V13ベース + 後退ペナルティ + 段階的低重心化

### 設計方針

V19の後退問題を解決するため、V13をベースに最小限の変更を加える：

1. **V13をベースにする**（前進成功実績あり: X=+2.385m）
2. **目標高さを段階的に下げる**: 0.40m → 0.38m（一気に0.35mにしない）
3. **後退ペナルティを追加**: `backward_velocity`（後退速度の2乗）
4. **旋回抑制を追加**: `yaw_rate_penalty`（V19で-32°旋回したため）
5. **非対称高さ報酬は追加しない**（V19で問題を起こしたため）

#### V19の問題分析

1. **非対称高さ報酬の二重ペナルティバグ**
   - `_reward_base_height`が内部でペナルティ値（`15.0 * error²`）を返却
   - さらにスケール`-15.0`で乗算 → `-225.0 * error²`の過剰ペナルティ
   
2. **目標高さ0.35mが低すぎる**
   - V13は0.40mで前進成功
   - 急激に0.35mにすると姿勢が崩れて後退

#### 変更サマリー

| パラメータ | V13 | V19（後退） | V20 |
|-----------|-----|-------------|-----|
| ベース | - | V13 | **V13** |
| base_height_target | 0.40m | 0.35m | **0.38m** |
| 非対称高さ報酬 | なし | あり（バグ） | **なし** |
| backward_velocity | なし | なし | **-2.0** |
| yaw_rate_penalty | なし | なし | **-0.5** |

### 訓練実行

```bash
cd rl_ws
uv run python scripts/biped_train_v20.py --max_iterations 500
```

### 評価結果

```
=== BSL-Droid Biped Walking Evaluation ===
Experiment: biped-walking-v20
Checkpoint: model_499.pt

t= 0.00s | pos=(-0.000,  0.000, 0.447)
t= 1.00s | pos=( 0.121,  0.024, 0.432) | rpy=( -3.5,  -3.3,  -9.7)°
t= 3.00s | pos=( 0.486,  0.046, 0.433) | rpy=(-11.6,  -4.5,  -5.0)°
t= 5.00s | pos=( 0.818,  0.124, 0.430) | rpy=( -3.2,  -4.2,   5.1)°
t= 7.00s | pos=( 1.160,  0.245, 0.435) | rpy=( -9.1,  -8.5,   9.7)°
t= 9.00s | pos=( 1.493,  0.406, 0.433) | rpy=( -7.6,  -4.4,  17.3)°

Travel distance:
  X: 1.631 m  ← 前進成功！
  Y: 0.496 m

Average velocity:
  X: 0.168 m/s (target: 0.300)  ← V13より遅い

Base height:
  Mean: 0.434 m  ← V13(0.446m)より少し低下
  Std: 0.0045 m

Left-Right hip_pitch correlation: 0.815  ← 同期！交互歩行失敗
  (-1.0 = perfect alternating, +1.0 = perfect synchronized)

Total DOF range sum: 5.398 rad
```

### 評価分析

#### 成功点 ✅

1. **前進成功**: X = +1.631m（後退問題は解決）
2. **高さ低下**: 0.434m（V13の0.446mから0.012m低下）
3. **安定した姿勢**: 高さStd = 0.0045m

#### 問題点 ❌

1. **交互歩行の喪失**: hip_pitch correlation = 0.815（左右が同期）
   - V13: 0.797（同様に同期）
   - V19: -0.428（交互歩行の兆候あり）
2. **速度低下**: 0.168 m/s（V13: 0.230 m/s）
3. **旋回**: Yaw 0° → +17.3°

### V13/V19/V20比較

| 指標 | V13 | V19（後退） | V20 |
|------|-----|-------------|-----|
| X移動距離 | +2.385m | **-3.011m** | **+1.631m** |
| 高さ平均 | 0.446m | 0.347m | **0.434m** |
| hip_pitch相関 | 0.797 | **-0.428** | 0.815 |
| 歩行品質 | 同期歩行 | **交互歩行** | 同期歩行 |

### 重要な発見

**V19は後退していたが、交互歩行（hip_pitch correlation = -0.428）を獲得していた！**

V13/V20は前進しているが、両脚が同期して動く「ホッピング」に近い歩行。
V19の非対称高さ報酬が交互歩行を誘導したが、同時に後退も引き起こした。

### V21への提案

**V19の交互歩行 + V20の前進を組み合わせる**

1. **V19をベースにする**（交互歩行獲得済み）
2. **非対称高さ報酬のバグを修正**
   - `_reward_base_height`はerror²のみを返す
   - スケールで非対称性を制御
3. **明示的な前進報酬を追加**
   - `forward_velocity`報酬を強化
   - `backward_velocity`ペナルティを追加

---

## V21: V19交互歩行 + 前進制御（バグ修正版）

### 設計方針

V19の交互歩行を維持しつつ、後退問題を解決する：

1. **V19をベースにする**（交互歩行獲得済み: hip_pitch相関 = -0.428）
2. **非対称高さ報酬のバグを修正**
   - V19: `_reward_base_height`が`15.0 * error²`を返し、スケール`-15.0`で二重ペナルティ
   - V21: `_reward_base_height`は`error²`のみを返す
   - 非対称性は`_reward_base_height_high`（高い時のみペナルティ）で実現
3. **後退ペナルティを追加**
   - `backward_velocity`: 後退速度の2乗にペナルティ

#### 報酬スケール設計

| 報酬 | V19 | V21 | 説明 |
|------|-----|-----|------|
| `base_height` | -15.0（×15.0内部）= -225 | **-10.0** | 基本ペナルティ（対称） |
| `base_height_high` | なし | **-5.0** | 高い時の追加ペナルティ |
| 合計（高い時） | -225 * error² | **-15.0 * error²** | 大幅に緩和 |
| 合計（低い時） | -225 * error² | **-10.0 * error²** | 大幅に緩和 |
| `backward_velocity` | なし | **-2.0** | 後退速度ペナルティ |

### 訓練実行

```bash
cd rl_ws
uv run python scripts/biped_train_v21.py --max_iterations 500
```

### 評価結果

```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 2.186 m
  Y: 0.532 m
  Total: 2.249 m

Average velocity:
  X: 0.221 m/s (target: 0.300)
  Y: 0.025 m/s

Base height:
  Mean: 0.449 m
  Std: 0.0036 m

DOF velocity RMS: 1.845 rad/s
Action RMS: 0.911

Left-Right hip_pitch correlation: -0.752
```

#### 分析

| 指標 | V19 | V20 | V21 | 評価 |
|------|-----|-----|-----|------|
| X方向移動 | -3.011m（後退） | +1.631m | **+2.186m** | ✅ 前進成功（V20より改善） |
| 胴体高さ | 0.347m | 0.434m | **0.449m** | ❌ 目標0.35mより高い |
| hip_pitch相関 | -0.428 | +0.815 | **-0.752** | ✅ 交互歩行回復 |

**成果**:
- 前進方向: V20より改善（+2.186m > +1.631m）
- 交互歩行: V19の交互パターンを維持し、さらに改善（-0.752 < -0.428）
- バグ修正: 二重ペナルティ問題を解消

**課題**:
- 胴体高さ: 0.449mとV20より悪化（目標0.35mから乖離）
- 報酬の大幅緩和（-225 → -15）により高さペナルティが弱すぎる
- 関節可動域が小さい: hip_pitch範囲 ≈ 0.3rad（V13: 0.6rad）

### V22への提案

**問題**: 高さペナルティが緩和されすぎて、関節を深く曲げるインセンティブが弱い

**アプローチ**: 高さペナルティを再強化しつつ、バグは修正したまま維持

1. **高さペナルティの再強化**
   - `base_height`: -10.0 → -30.0（3倍）
   - `base_height_high`: -5.0 → -15.0（3倍）
   - 合計: high=-45, low=-30（V19の-225より穏やかだが、V21より強い）

2. **関節屈曲を促す報酬の追加検討**
   - 関節角度の目標値を直接報酬に含める
   - または膝の屈曲角度に対する報酬

---

## V22: V21 + 高さペナルティ再強化

### 設計方針

V21の交互歩行・前進を維持しつつ、低い姿勢を強制する：

1. **V21をベースにする**（交互歩行＋前進獲得済み: X=+2.186m, hip_pitch相関=-0.752）
2. **高さペナルティを再強化**（バグ修正は維持）
   - `base_height`: -10.0 → -30.0（3倍）
   - `base_height_high`: -5.0 → -15.0（3倍）
   - 合計: 高い時-45.0*error², 低い時-30.0*error²
3. **後退ペナルティは維持**（-2.0）

#### 報酬スケール比較

| 報酬 | V19 | V21 | V22 | 説明 |
|------|-----|-----|-----|------|
| `base_height` | -225（バグ） | -10.0 | **-30.0** | 基本ペナルティ |
| `base_height_high` | - | -5.0 | **-15.0** | 高い時追加 |
| 合計（高い時） | -225 | -15.0 | **-45.0** | - |
| 合計（低い時） | -225 | -10.0 | **-30.0** | - |
| `backward_velocity` | - | -2.0 | -2.0 | 後退ペナルティ |

### 訓練実行

```bash
cd rl_ws
uv run python scripts/biped_train_v22.py --max_iterations 500
```

### 初回訓練（修正漏れ）

**問題発覚**: 訓練スクリプトの報酬スケールがV21のままだった（-10.0, -5.0）

```
=== V22 初回訓練時の設定（修正漏れ） ===
base_height: -10.0  # 本来-30.0
base_height_high: -5.0  # 本来-15.0
```

結果としてV21と同一の挙動となった（高さ0.449m）。

**対応**: 訓練スクリプトを修正し、再訓練を実施。

### 再訓練後の評価結果

```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 1.831 m
  Y: 0.639 m
  Total: 1.939 m

Average velocity:
  X: 0.191 m/s (target: 0.300)
  Y: -0.025 m/s

Base height:
  Mean: 0.435 m
  Std: 0.0047 m

DOF velocity RMS: 1.789 rad/s
Action RMS: 1.076

Left-Right hip_pitch correlation: 0.742
```

#### 分析

| 指標 | V21 | V22 | 評価 |
|------|-----|-----|------|
| X方向移動 | +2.186m | **+1.831m** | ❌ 悪化（-16%） |
| 胴体高さ | 0.449m | **0.435m** | △ 微改善（-1.4cm） |
| hip_pitch相関 | -0.752 | **+0.742** | ❌ 交互歩行喪失（同期化） |
| Yaw角 | ≈6° | **≈19°** | ❌ 大きく旋回 |

**結果**: V22は失敗
- 高さペナルティの強化（3倍）により、交互歩行パターンが完全に崩壊
- 両足が同期（hip_pitch相関=+0.742）し、V20と同様の問題が再発
- 前進距離も減少、大きな旋回（Yaw≈19°）が発生

---

## 総括

### 実験結果まとめ

| Ver | 主な変更 | X移動 | 高さ | hip_pitch相関 | 評価 |
|-----|---------|-------|------|---------------|------|
| V13 | ベースライン | +2.385m | 0.446m | - | ✅ 良好な歩行 |
| V19 | 非対称高さ報酬 | -3.011m | 0.347m | -0.428 | ❌ 後退 |
| V20 | V13+後退ペナルティ | +1.631m | 0.434m | +0.815 | ❌ 同期歩行 |
| **V21** | V19バグ修正+後退ペナルティ | **+2.186m** | 0.449m | **-0.752** | **✅ 最良** |
| V22 | V21+高さペナルティ3倍 | +1.831m | 0.435m | +0.742 | ❌ 同期歩行 |

### 主要な発見

1. **交互歩行と低姿勢はトレードオフ関係にある**
   - 高さペナルティを強くすると交互歩行が崩壊（V19, V22）
   - 交互歩行を維持すると高さが下がらない（V21）

2. **報酬バグの重要性**
   - V19の二重ペナルティバグ（-225×error²）が低姿勢の原因
   - バグ修正後は意図的に同等の効果を得ることが困難

3. **最適解はV21**
   - 前進（+2.186m）と交互歩行（-0.752）のバランスが最良
   - 高さ（0.449m）は目標（0.35m）未達だが許容範囲

### 今後の課題

1. **低姿勢歩行の実現**
   - 現行の報酬設計では交互歩行を維持しつつ低姿勢を実現できない
   - 関節角度の直接指定、参照軌道の導入など別アプローチが必要

2. **URDFの見直し**
   - 現在のDigitigrade構造では低姿勢歩行に限界がある可能性
   - 関節可動範囲、リンク長の最適化を検討

### 結論

**V21を現時点の最良モデルとして採用**する。目標の低姿勢（0.35m）は達成できなかったが、前進歩行と交互歩行パターンの両立という点で最も実用的な結果を得た。

---

## exp004: BSL-Droid Simplified 歩容獲得実験（詳細記録）

### V12: 同期歩行修正バージョン

#### 背景（V11評価結果）

V11ではハイブリッドアプローチ（V9ペナルティ + V10 Phase-based + 接地強制）を試みたが、深刻な退行が発生：

| 指標 | V9 | V10 | V11 |
|------|-----|------|------|
| X方向移動 | 2.95m | 2.81m | 2.54m |
| Y方向移動 | 0.01m | -0.56m | 0.19m |
| Yawドリフト | -4.6° | -8.0° | **+19.1°（最悪）** |
| hip_pitch相関 | -0.516 | +0.355 | **+0.852（同期歩行）** |
| hip_roll L-R差 | 0.023 | 0.137 | **0.000（完璧）** |
| 重心高さStd | 0.014m | 0.009m | **0.007m（最良）** |

**V11の問題点分析**:
- `smooth_action` (1.5) と `ground_contact_bonus` (1.5) が間接的に同期を促進
- これらの報酬が `hip_pitch_sync_penalty` (-2.0) を上回り、両足同時動作が有利に
- 結果として hip_pitch 相関が +0.852（V8の+0.772より悪化）

#### V12の設計方針

V11の同期問題を修正し、交互歩行を確実に達成する：

| 変更項目 | V11 | V12 | 理由 |
|---------|-----|-----|------|
| `smooth_action` | 1.5 | **0.3** | 同期促進を大幅削減 |
| `ground_contact_bonus` | 1.5 | **削除** | 両足同時接地を促進していた |
| `hip_pitch_sync_penalty` | -2.0 | **-4.0** | 同期ペナルティを2倍強化 |
| `phase_hip_pitch_tracking` | 2.0 | **3.0** | CPG参照軌道追従を強化 |
| `strict_alternating_contact` | なし | **2.0（新規）** | 接地交互を直接報酬化 |

#### 新規報酬関数: `strict_alternating_contact`

```python
def _reward_strict_alternating_contact(self):
    """厳格な交互接地報酬（V12新規）

    接地状態が厳密に交互（片足のみ接地）であることを報酬化。
    XOR演算により、片足だけが接地している場合に報酬。
    """
    left_contact = contacts[:, 0].float()
    right_contact = contacts[:, 1].float()

    # XOR: 片足のみ接地の場合に1
    alternating = torch.abs(left_contact - right_contact)
    return alternating * has_command
```

**期待される状態遷移:**
| 状態 | 報酬 |
|------|------|
| 左足のみ接地 | 2.0 |
| 右足のみ接地 | 2.0 |
| 両足接地 | 0 |
| 両足宙浮き | 0 |

#### 期待効果

- hip_pitch相関を負（< -0.3）に戻す
- V9相当の交互歩行を達成
- V11で得られたhip_roll対称性は維持

#### 訓練実行

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_v12.py --max_iterations 500
```

#### 評価結果

```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 1.796 m
  Y: -0.850 m
  Total: 1.987 m

Average velocity:
  X: 0.204 m/s (target: 0.300)
  Y: -0.000 m/s

Base height:
  Mean: 0.271 m
  Std: 0.0073 m

Orientation (deg):
  Roll:  mean=  2.93, std= 2.49
  Pitch: mean=  0.12, std= 1.42
  Yaw:   start=  0.00, end=-47.43, drift=-47.43

DOF velocity RMS: 1.721 rad/s
Action RMS: 0.674

=== Joint Movement Analysis ===
Left-Right hip_pitch correlation: 0.597
  (-1.0 = perfect alternating, +1.0 = perfect synchronized)

=== Contact Pattern Analysis ===
Both feet grounded:       0 steps (  0.0%)
Single foot grounded:     0 steps (  0.0%)
Both feet airborne:     500 steps (100.0%)  ← 滑走歩行
```

#### 5バージョン比較

| 指標 | V8 | V9 | V10 | V11 | V12 |
|------|-----|-----|------|------|------|
| X方向移動 | 2.36m | 2.95m | 2.81m | 2.54m | **1.80m** |
| Y方向移動 | -0.15m | 0.01m | -0.56m | 0.19m | **-0.85m** |
| Yawドリフト | -1.6° | -4.6° | -8.0° | +19.1° | **-47.4°** |
| hip_pitch相関 | +0.772 | **-0.516** | +0.355 | +0.852 | **+0.597** |
| 重心高さ | 0.27m | 0.28m | 0.26m | 0.27m | 0.27m |
| 重心高さStd | 0.014m | 0.014m | 0.009m | 0.007m | 0.007m |

#### 考察：V12は失敗

**問題1: 交互歩行が達成されていない**
- hip_pitch相関 = +0.597（同期歩行）
- V9の-0.516（交互）から大幅に退行
- 修正が効いていない

**問題2: Yawドリフトが悪化**
- -47.4°は全バージョン中最悪
- V11の+19.1°から更に悪化
- 大きく左旋回しながら斜行

**問題3: 前進距離の低下**
- 1.80mはV8-V11の全てより悪い
- 速度追従も0.204 m/s（目標0.3の68%）

**問題4: 滑走歩行100%継続**
- 接地判定が全く機能していない
- `strict_alternating_contact`報酬が効果なし

#### 根本原因分析

V12の修正方針は正しかったが、以下の問題が複合している：

1. **`smooth_action`削減の副作用**
   - 0.3への削減で動作が粗くなりYawドリフト増大
   - 滑らかさと安定性のトレードオフ

2. **`hip_pitch_sync_penalty`強化の不発**
   - -4.0に強化したが、同期歩行（+0.597）は改善されず
   - Phase-based報酬との相互作用が不明

3. **`strict_alternating_contact`の無効化**
   - 接地判定が全て「宙浮き」のため報酬が常に0
   - `contact_threshold: 0.04m`でも足の高さが検出限界外

4. **報酬バランスの崩壊**
   - `ground_contact_bonus`削除で接地インセンティブ喪失
   - 宙浮き状態が最適解に収束

#### V13への提案

**アプローチ変更**: Phase-based報酬を一旦放棄し、V9のペナルティベース設計に回帰

1. **V9をベースにする**（hip_pitch相関-0.516の実績）
2. **Yaw対策を追加**
   - `yaw_rate`: -1.5 → -3.0
   - `symmetry`: -0.8 → -1.5
3. **接地判定の根本修正**
   - 足の高さベースではなく、接触力ベースに変更
   - またはシミュレータの接地情報を直接使用
4. **Phase-based報酬は削除**
   - CPG参照軌道が同期を助長している可能性

---

## ゲームパッド操縦を見据えた報酬設計の検討

### 問題提起

現在の訓練では速度コマンドが固定値（`lin_vel_x: [0.3, 0.3]`）だが、最終ゴールは「ゲームパッドで前後左右自在に操縦すること」。この場合、現行のCPGインスパイア正弦波参照軌道（Phase-based報酬）は使いづらい可能性がある。

### 先行事例調査

#### 1. ETH Legged Gym (ANYmal, A1, Cassie)

[legged_gym](https://github.com/leggedrobotics/legged_gym)の実装を調査した結果：

**速度コマンド追従報酬（標準手法）:**
```python
def _reward_tracking_lin_vel(self):
    # Tracking of linear velocity commands (xy axes)
    lin_vel_error = torch.sum(torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1)
    return torch.exp(-lin_vel_error / self.cfg.rewards.tracking_sigma)

def _reward_tracking_ang_vel(self):
    # Tracking of angular velocity commands (yaw)
    ang_vel_error = torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2])
    return torch.exp(-ang_vel_error / self.cfg.rewards.tracking_sigma)
```

**コマンドレンジ設定:**
```python
class commands:
    lin_vel_x = [-1.0, 1.0]  # 前後 [m/s]
    lin_vel_y = [-1.0, 1.0]  # 左右 [m/s]
    ang_vel_yaw = [-1, 1]    # 旋回 [rad/s]
    heading = [-3.14, 3.14]  # 方位角
```

**特徴:**
- コマンドは訓練中にランダムにリサンプル（`resampling_time: 10秒`）
- ポリシーは多様な速度・方向への対応を学習
- 歩容パターンの明示的指定なし（ポリシーに任せる）

#### 2. Robot Parkour Learning (CoRL 2023)

[robot-parkour.github.io](https://robot-parkour.github.io/)の手法：

**特徴:**
- 固定的な歩容パターンを指定しない
- 視覚情報から適切なスキルを選択
- End-to-endでポリシーが歩容を自己組織化

**教訓:** 複雑な操縦タスクでは、歩容パターンを明示的に強制するより、タスク達成報酬を与えてポリシーに歩容を発見させる方が柔軟性が高い。

#### 3. MIT Cheetah Software

[cheetah-software](https://github.com/mit-biomimetics/cheetah-software)（MPC + WBC手法）：

**特徴:**
- 歩容（trot, bound, gallop）は明示的にスケジューリング
- ただしこれは古典的制御であり、RLでは異なるアプローチが主流

### Phase-based報酬とゲームパッド操縦の相性

#### 問題点

現在のPhase-based報酬（`_reward_phase_hip_pitch_tracking`など）は以下の前提に基づく：

1. **固定周波数歩行**: `gait_frequency: 1.0 Hz`で常に歩行
2. **前進のみ**: 正弦波位相は前進動作を想定
3. **停止が難しい**: コマンド0でも位相は進み続ける

```python
# 現行の問題
left_ref = amplitude * sin(phase)        # phaseは常に増加
right_ref = amplitude * sin(phase + π)   # 止まれない
```

**ゲームパッド操縦で必要な動作:**
- **停止**: スティック中立で静止（Phase-basedでは困難）
- **方向転換**: 瞬時に左右に移動開始（位相リセットが必要？）
- **後退**: 現在の正弦波では非対応
- **速度変化**: 遅い→速いへの滑らかな遷移

#### 解決アプローチ

| アプローチ | 説明 | メリット | デメリット |
|-----------|------|---------|-----------|
| **A. コマンド適応型Phase** | コマンド速度に応じて周波数・振幅を動的に変更 | 既存報酬を活用可能 | 実装が複雑、停止時の挙動が不安定 |
| **B. 速度追従報酬のみ** | ETH方式。Phase-based削除、速度追従のみ | シンプル、柔軟性高 | 歩容品質の保証なし、「かわいさ」喪失の恐れ |
| **C. 二段階学習** | Phase 1: 固定速度で歩容獲得 → Phase 2: 多様なコマンドで微調整 | 歩容品質を維持しつつ汎化 | 訓練時間増加 |
| **D. コマンド条件付きPhase** | コマンド速度>閾値のときのみPhase報酬有効 | 停止問題を解決 | 歩行開始時の遷移が不自然 |

### 推奨アプローチ: C + D のハイブリッド

#### Phase 1: 歩容獲得（現在の作業）

現行のV12設計を継続。固定速度（0.3 m/s）で「かわいい交互歩行」を確立。

**報酬構成:**
- Phase-based報酬（歩容品質確保）
- 交互歩行ペナルティ（同期防止）
- 速度追従報酬（前進動機）

#### Phase 2: コマンド汎化

歩容獲得後、以下の変更を行う：

1. **コマンドレンジを拡大:**
   ```python
   command_cfg = {
       "lin_vel_x_range": [-0.3, 0.5],  # 後退〜前進
       "lin_vel_y_range": [-0.3, 0.3],  # 横移動
       "ang_vel_range": [-0.5, 0.5],    # 旋回
   }
   ```

2. **コマンド条件付きPhase報酬:**
   ```python
   def _reward_phase_hip_pitch_tracking(self):
       # コマンド速度が閾値以下なら報酬0
       cmd_magnitude = torch.norm(self.commands[:, :2], dim=1)
       active = (cmd_magnitude > 0.05).float()
       
       # 周波数もコマンド速度に比例させる
       adaptive_freq = self.gait_frequency * torch.clamp(cmd_magnitude / 0.3, 0.3, 1.5)
       
       # 参照軌道計算...
       return reward * active
   ```

3. **停止報酬の追加:**
   ```python
   def _reward_stand_still(self):
       """コマンド0の時、静止を報酬"""
       cmd_magnitude = torch.norm(self.commands[:, :2], dim=1)
       standing = (cmd_magnitude < 0.05).float()
       motion_penalty = torch.sum(torch.abs(self.dof_vel), dim=1)
       return standing * torch.exp(-motion_penalty * 0.1)
   ```

### 結論

**現行のPhase-based報酬はゲームパッド操縦と完全には互換性がない**が、以下の理由で今は維持すべき：

1. **歩容品質の確保が最優先**: 現段階では「かわいく歩く」ことが重要
2. **段階的アプローチが有効**: 歩容獲得→コマンド汎化の二段階学習
3. **ETH方式への移行は容易**: Phase-based報酬を削除し、速度追従のみにすれば良い

**次のマイルストーン:**
1. V12で交互歩行を確立
2. V13以降でコマンドレンジを拡大
3. 評価時にゲームパッド入力をシミュレートしてテスト
4. 必要に応じてPhase報酬をコマンド条件付きに修正