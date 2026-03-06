# EXP004: BSL-Droid Simplified二脚ロボットモデルの強化学習による歩容獲得実験

## 概要

BSL-Droid Simplified二脚ロボットモデルに対して、Genesis物理シミュレータで強化学習により歩容を獲得する。exp002で確立した訓練パイプラインを新しいロボットモデル（`bsl_droid_simplified.urdf.xacro`）に適用する。

## 実験の進め方

実験手順・コマンド・ルールは [exp004_rules.md](exp004_rules.md) を参照。

## 背景

- **前実験（exp002）**: `biped_digitigrade.urdf.xacro`を使用した二脚歩行実験。V22までの反復により交互歩行と前進を獲得。
- **本実験**: 新しく作成された`bsl_droid_simplified.urdf.xacro`を使用。ロボットの外観と物理パラメータが異なる。

## ロボット仕様

### BSL-Droid Simplified

| 項目 | 値 |
|------|-----|
| 脚数 | 2脚 |
| DOF | 10 (片脚5関節×2) |
| 関節構成 | hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch |
| 膝構造 | 逆関節（負角度で後方屈曲、Unitree/ANYmal規約） |
| 胴体サイズ | 0.14m(D) × 0.18m(W) × 0.16m(H) |
| 総質量 | 約5.8kg |
| 脚長 | 約0.31m (thigh: 0.11m + shank: 0.12m + foot: 0.08m) |

### 関節詳細

```
左脚 (left):
  - left_hip_yaw_joint    (Z軸, ±30°)
  - left_hip_roll_joint   (X軸, ±25°)
  - left_hip_pitch_joint  (Y軸, -120°〜+60°)
  - left_knee_pitch_joint (Y軸, -135°〜0°, 逆関節：負で後方屈曲)
  - left_ankle_pitch_joint(Y軸, ±60°)

右脚 (right): 同上

追加要素:
  - imu_link (base_link上面中央, fixed joint)
  - neck_link, head_link（頭部、fixed）
  - left_eye_link, right_eye_link（目、fixed）
```

### exp002との主な差異

| 項目 | exp002 (biped_digitigrade) | exp004 (bsl_droid_simplified) |
|------|---------------------------|-------------------------------|
| 膝関節可動範囲 | -120°〜0° | -135°〜0° |
| 膝の曲がり方向 | 負角度で後方屈曲 | **負角度で後方屈曲（同じ規約）** |
| 総質量 | 約6.2kg | 約5.8kg |
| 脚長 | 約0.53m | **約0.31m（短い）** |
| 頭部 | なし | あり（neck + head） |

**重要**: 膝関節の規約はexp002と同じ（負角度で屈曲、Unitree/ANYmalと同様）。軸は他のピッチ関節と統一されている。

## 実験フェーズ

### Phase 1: URDF準備

#### 1.1 xacroとURDFの関係

- **xacro**: ROS 2で使用するマクロ付きURDF記述形式。パラメータ化や再利用が可能。
  - ソースファイル: `ros2_ws/src/biped_description/urdf/bsl_droid_simplified.urdf.xacro`
- **URDF**: 物理シミュレータ（Genesis等）が直接読み込める純粋なXML形式。
  - 出力ファイル: `rl_ws/assets/bsl_droid_simplified.urdf`

Genesisは純粋なURDFのみを読み込めるため、xacroからURDFへの変換（エクスポート）が必要。

#### 1.2 URDFエクスポート手順

xacroからURDFをエクスポート（ros2_ws側でpixi経由で実行）:

```bash
cd ros2_ws
pixi run xacro src/biped_description/urdf/bsl_droid_simplified.urdf.xacro > ../rl_ws/assets/bsl_droid_simplified.urdf
```

**注意**: 
- `pixi run xacro`はros2_ws配下のpixi環境でのみ実行可能
- xacroファイルを変更した場合は、再度エクスポートが必要
- エクスポート済みURDFはgit管理対象

#### 1.3 成果物

- `rl_ws/assets/bsl_droid_simplified.urdf` (エクスポート済みURDF)

---

### Phase 2: Genesis訓練環境構築

#### 2.1 環境クラス作成

`rl_ws/biped_walking/envs/droid_env_v1.py`を新規作成:

```python
# 主要な変更点（biped_env_v22からの差分）:
# - urdf_path: bsl_droid_simplified.urdf
# - default_joint_angles: 膝の角度定義が逆転
# - base_init_pos: [0, 0, 0.30] (脚が短いため低く)
# - feet_names: ["left_foot_link", "right_foot_link"]
```

#### 2.2 初期姿勢設定

逆関節の特性を考慮した立位姿勢（膝正角度で後方屈曲）:

```python
default_joint_angles = {
    "left_hip_yaw_joint": 0.0,
    "left_hip_roll_joint": 0.0,
    "left_hip_pitch_joint": -0.3,    # 股関節を少し前傾
    "left_knee_pitch_joint": 0.6,    # 約35°（正で後方屈曲）
    "left_ankle_pitch_joint": -0.3,  # 膝と相殺
    "right_hip_yaw_joint": 0.0,
    "right_hip_roll_joint": 0.0,
    "right_hip_pitch_joint": -0.3,
    "right_knee_pitch_joint": 0.6,
    "right_ankle_pitch_joint": -0.3,
}
```

#### 2.3 訓練スクリプト作成

`rl_ws/biped_walking/train/droid_train_v1.py`を新規作成

---

### Phase 3: 訓練実行

#### 3.1 訓練パラメータ

exp002 V22と同様の設定をベースとする：

```python
train_cfg = {
    "max_iterations": 500,      # 初期検証
    "num_envs": 4096,           # 並列環境数
    "num_steps_per_env": 24,
    "save_interval": 100,
}
```

#### 3.2 訓練実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_v1.py --max_iterations 500
```

#### 3.3 推論評価

訓練済みモデルの評価には`biped_eval.py`を使用する。このスクリプトは実験名から環境クラスを動的に選択するため、droid用環境の対応が必要。

##### 3.3.1 評価スクリプトの対応

`biped_eval.py`の`get_env_class()`関数を拡張し、`droid-walking-*`に対応させる必要がある：

```python
# biped_eval.py の get_env_class() を以下のように拡張
def get_env_class(exp_name: str):
    # droid-walking系の場合
    if exp_name.startswith("droid-walking"):
        match = re.search(r'-(v\d+)$', exp_name)
        version = match.group(1) if match else "v1"
        module_name = f"biped_walking.envs.droid_env_{version}"
        class_name = f"DroidEnv{version.upper()}"
        module = importlib.import_module(module_name)
        return getattr(module, class_name)
    
    # biped-walking系の場合（既存）
    ...
```

また、URDFパスの上書き部分も修正が必要：

```python
# URDFパスを実験名に応じて決定
if args.exp_name.startswith("droid-walking"):
    urdf_path = rl_ws_dir / "assets" / "bsl_droid_simplified.urdf"
else:
    urdf_path = rl_ws_dir / "assets" / "biped_digitigrade.urdf"
env_cfg["urdf_path"] = str(urdf_path)
```

##### 3.3.2 評価コマンド

```bash
cd rl_ws

# GUIビューア付き評価（リアルタイム表示）
uv run python biped_walking/biped_eval.py -e droid-walking-v1

# 特定チェックポイントを使用
uv run python biped_walking/biped_eval.py -e droid-walking-v1 --ckpt 400

# ヘッドレス評価（統計のみ）
uv run python biped_walking/biped_eval.py -e droid-walking-v1 --no-viewer --duration 10

# MP4録画
uv run python biped_walking/biped_eval.py -e droid-walking-v1 --record droid_v1_walking.mp4 --duration 10
```

##### 3.3.3 評価出力

評価スクリプトは以下の統計を出力する：

| 項目 | 説明 |
|------|------|
| Final Position | エピソード終了時の位置 [X, Y, Z] |
| Mean Velocity | 平均速度（ボディローカル） |
| Mean Body Height | 平均胴体高さ |
| hip_pitch correlation | 左右hip_pitchの相関係数（-1に近いほど交互歩行）|

---

### Phase 4: 結果分析

#### 4.1 評価指標（目標値）

| 指標 | 目標値 |
|------|--------|
| 前進速度 [m/s] | 0.3〜0.5 |
| 胴体高さ [m] | 0.25〜0.30 |
| 転倒率 [%] | <10 |
| hip_pitch相関 | <-0.5 |

#### 4.2 V1評価結果（500イテレーション）

**評価条件**: `--no-viewer --duration 10`（ヘッドレス10秒評価）

##### 移動性能

| 指標 | 結果 | 目標 | 判定 |
|------|------|------|------|
| 前進距離 (X) | 2.559 m | - | - |
| 横移動距離 (Y) | -0.400 m | 0 | △ やや左にドリフト |
| 平均前進速度 | 0.257 m/s | 0.3 m/s | △ やや低い |
| 胴体高さ（平均） | 0.266 m | 0.25〜0.30 m | ✓ 目標範囲内 |
| 胴体高さ（標準偏差） | 0.0076 m | - | ✓ 安定 |

##### 歩容品質

| 指標 | 結果 | 目標 | 判定 |
|------|------|------|------|
| hip_pitch相関 | **+0.530** | <-0.5 | ✗ **両脚同期（問題）** |
| Total DOF range | 5.795 rad | >2.0 | ✓ 十分な関節運動 |
| DOF velocity RMS | 1.975 rad/s | - | - |
| Action RMS | 1.089 | - | - |

##### 姿勢

| 指標 | 結果 |
|------|------|
| Roll角 | 約2.5°（右傾斜） |
| Pitch角 | 0〜3°（微小前傾） |
| Yaw角 | -6°（左回転） |

##### 関節可動域 [rad]

| 関節 | 左脚 | 右脚 | 設計可動範囲 |
|------|------|------|-------------|
| hip_yaw | [-0.007, 0.490] (0.498) | [-0.296, 0.226] (0.522) | ±0.52 (±30°) |
| hip_roll | [-0.214, 0.000] (0.214) | [-0.401, 0.000] (0.401) | ±0.44 (±25°) |
| knee_pitch | **[-0.932, -0.146]** (0.786) | **[-0.907, -0.252]** (0.655) | **[0, +2.36] (0〜+135°)** |
| ankle_pitch | [0.509, 1.526] (1.016) | [0.603, 1.413] (0.810) | ±1.05 (±60°) |

#### 4.3 V1の課題

##### 課題1: 膝関節が逆関節構造を無視している（最重要）

**問題**: 膝関節が設計意図と逆方向に動作している

- **設計**: 膝は0°〜+135°（正角度で後方屈曲 = 鳥脚型逆関節）
- **実際**: 膝は-0.93〜-0.15 rad（**負角度 = 膝が前方に出ている**）
- **結果**: 逆関節（Digitigrade）ではなく、通常の膝（Plantigrade）のような動作

これは**ロボットの基本設計思想に反する重大な問題**である。

**原因分析**:
1. 初期姿勢（膝+0.6 rad = 約35°）が浅すぎる
2. 学習中に膝が可動範囲外（負の角度）に逸脱している
3. デフォルト姿勢維持の報酬が弱く、逆関節形状を保持できていない

##### 課題2: 両脚同期（副次的）

- hip_pitch相関が+0.530（目標は-0.5以下）
- 左右の脚が同じタイミングで動いている
- ただし、これは課題1が解決されれば改善する可能性がある

##### 課題3: 横方向ドリフト

- Y方向に-0.4m移動（10秒間）
- Yaw角が-6°に傾いている

#### 4.4 次バージョン（V2）への改善方針

##### 方針1: 初期姿勢を深く曲げた逆関節状態にする

現在の初期姿勢は関節がほぼ一直線に近い。逆関節の特性を活かすため、**膝を深く曲げた状態**から開始する。

**幾何学的考察**: 足裏を水平に保つには、各関節角度が連動する必要がある。

```
足裏水平条件: ankle_pitch ≈ hip_pitch - knee_pitch

例: hip_pitch = -0.6, knee_pitch = +1.2 の場合
    ankle_pitch = -0.6 - 1.2 = -1.8 (可動範囲外)
    
    → 膝の曲げを緩和: knee_pitch = +1.0
    ankle_pitch = -0.6 - 1.0 = -1.6 (まだ可動範囲外)
    
    → さらに調整: hip_pitch = -0.4, knee_pitch = +0.9
    ankle_pitch = -0.4 - 0.9 = -1.3 (可動範囲±1.05なので、まだ厳しい)
    
    → 現実的な値: hip_pitch = -0.3, knee_pitch = +0.8
    ankle_pitch = -0.3 - 0.8 = -1.1 (可動範囲ぎりぎり)
```

**V2提案値**（足裏水平を維持しつつ、明確な逆関節形状）:

```python
# V1（現在）: 浅い屈曲、足裏の水平が考慮されていない
"left_hip_pitch_joint": -0.3,
"left_knee_pitch_joint": 0.6,     # 約35°
"left_ankle_pitch_joint": -0.3,

# V2（提案）: 深い逆関節形状、足裏水平
"left_hip_pitch_joint": -0.35,    # 股関節前傾
"left_knee_pitch_joint": 0.9,     # 約52°（後方に深く曲げる）
"left_ankle_pitch_joint": -0.55,  # 足裏を水平に（≈ -0.35 - 0.9 = -1.25だが、可動範囲内に収める）
```

**注意**: ankle_pitchの可動範囲が±1.05 rad（±60°）のため、深く曲げすぎると足裏を水平にできない。膝の曲げ角度は0.9 rad程度が限界。

初期高さも調整（深く曲げると低くなる）:
```python
"base_init_pos": [0.0, 0.0, 0.22],  # 0.30m → 0.22m
"base_height_target": 0.20,         # 0.25m → 0.20m
```

##### 方針2: 膝関節の可動範囲をソフトに制約

膝が負の角度にならないよう報酬でペナルティを追加：

```python
def _reward_knee_positive(self):
    """膝関節が正の角度を維持することを報酬"""
    left_knee = self.dof_pos[:, 3]   # left_knee_pitch_joint
    right_knee = self.dof_pos[:, 8]  # right_knee_pitch_joint
    
    # 負の角度にペナルティ
    penalty = torch.clamp(-left_knee, min=0) + torch.clamp(-right_knee, min=0)
    return penalty
```

##### 方針3: デフォルト姿勢維持を強化

```python
"similar_to_default": -0.05,  # -0.02 → -0.05 に強化
```

##### 方針4: 膝の最小角度を設定

デフォルト角度に対して大きく負方向に動かないよう制約：

```python
def _reward_knee_min_angle(self):
    """膝が最小角度（約30°）を下回らないようにする"""
    min_knee_angle = 0.5  # 約30°
    left_knee = self.dof_pos[:, 3]
    right_knee = self.dof_pos[:, 8]
    
    penalty = torch.clamp(min_knee_angle - left_knee, min=0) + \
              torch.clamp(min_knee_angle - right_knee, min=0)
    return penalty
```

---

## 5. V2: 初期姿勢調整

### 5.1 変更内容

V1の課題（膝関節が逆方向に動作）を解決するため、ユーザーがGUIで視覚的に確認しながら初期姿勢を決定した。

**変更箇所**（`droid_train_v2.py`のみ、`droid_env_v2.py`はクラス名のみ変更）:

| パラメータ | V1 | V2 | 備考 |
|-----------|-----|-----|------|
| `hip_pitch` | -0.3 rad (-17°) | **0.9599 rad (55°)** | 前方傾斜→大きく後傾 |
| `knee_pitch` | 0.6 rad (34°) | **-1.7453 rad (-100°)** | 浅い曲げ→深い逆関節 |
| `ankle_pitch` | -0.3 rad (-17°) | **0.7854 rad (45°)** | 足首調整 |
| `base_init_pos[2]` | 0.30 m | **0.22 m** | 深い屈曲に合わせて低く |
| `base_height_target` | 0.25 m | **0.18 m** | 目標高さも低く |

**完全コピー原則の適用**:
- `droid_env_v1.py` → `droid_env_v2.py`: `cp`コマンドでコピー、クラス名のみ変更
- `droid_train_v1.py` → `droid_train_v2.py`: `cp`コマンドでコピー、上記パラメータのみ変更

### 5.2 訓練

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_v2.py --max_iterations 500
```

### 5.3 評価結果

#### 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-v2 --no-viewer --duration 10
```

#### 定量的結果

| 指標 | 結果 | 評価 |
|------|------|------|
| X方向移動距離 | **0.009 m** | ❌ ほぼゼロ |
| Y方向移動距離 | 0.000 m | - |
| 平均速度（報告値） | 0.485 m/s | ※位置データと矛盾 |
| DOF velocity RMS | 11.140 rad/s | ❌ 非常に高い（暴れている） |
| Action RMS | 0.842 | - |
| Base height mean | 0.261 m | 目標0.18mより高い |
| Base height std | 0.0415 m | ❌ 大きな上下動 |

#### 交互歩行の検証

| 指標 | 結果 | 評価 |
|------|------|------|
| hip_pitch相関 | **+0.846** | ❌ 両脚同期（ジャンプ） |
| ankle_pitch相関 | +0.786 | ❌ 両脚同期 |

**致命的問題**: hip_pitch相関が**+0.846**は、両脚が**完全に同期**して動いていることを示す。これは**その場でジャンプ**している状態。

#### 位置軌跡

```
t=0s:  pos=(-0.009, -0.000, 0.303)
t=5s:  pos=(-0.009, -0.000, 0.303)  # 変化なし
t=10s: pos=(-0.009, -0.000, 0.303)  # 変化なし
```

10秒間で位置が**全く変化していない**。ロボットはその場でジャンプし続けている。

### 5.4 V2の失敗分析

#### exp002 V18との類似性

exp002でも全く同じ失敗パターンが観察された（[exp002 V18分析](../exp002_biped_rl_walking/exp002_biped_rl_walking.md#v18-報酬再設計失敗)）:

| 指標 | exp004 V2 | exp002 V18 |
|------|-----------|------------|
| hip_pitch相関 | +0.846 | +0.859 |
| 移動距離(10s) | 0.009 m | 0.07 m |
| 挙動 | その場ジャンプ | その場ジャンプ |

両者は**ほぼ同一の失敗モード**に陥っている。

#### 根本原因

1. **報酬の競合**: 前進報酬と姿勢維持報酬が競合し、「動かない」局所最適解に収束
2. **初期姿勢の急激な変更**: V1からV2で初期姿勢を大幅に変更したため、学習済みの方策が適合しない
3. **ペナルティ過剰**: 転倒ペナルティ等が強すぎて、「安全に立っている」状態で停滞

#### exp002からの教訓

> V18で実装した報酬設計は理論的には正しかったが、変更が急激すぎて「動かない」局所最適解に陥った。V19では、動いていたV10をベースにして「小さな変更」で改善を試みる方針に転換した。

この教訓に従い、V3では以下の方針を取る必要がある。

### 5.5 V3への改善方針

#### 方針: 段階的改善（exp002 V19の方針を踏襲）

**やらないこと**:
- 報酬関数の大幅な変更
- 初期姿勢のさらなる変更
- 新しい報酬項の追加

**やること**:
1. **V1に戻る**: V1は前進していた（X=0.74m/10s）ので、まずV1を基点とする
2. **膝方向のみ修正**: 膝が正方向に動くよう、報酬関数に**最小限の**制約を追加
3. **一つずつ変更**: 複数の変更を同時に行わない

#### 具体的なV3設計

```python
# V3: V1ベース + 膝正角度維持のみ追加

# 報酬関数（追加分のみ）
"knee_positive": -0.1,  # 膝が負角度になったらペナルティ

def _reward_knee_positive(self):
    """膝関節が正の角度を維持することを報酬"""
    left_knee = self.dof_pos[:, 3]   # left_knee_pitch_joint
    right_knee = self.dof_pos[:, 8]  # right_knee_pitch_joint
    # 負の角度にペナルティ
    penalty = torch.clamp(-left_knee, min=0) + torch.clamp(-right_knee, min=0)
    return penalty
```

初期姿勢は**V1のまま**（変更しない）。学習中に膝が正方向に動くようになることを期待する。

---

## 6. V3: V1ベース + V21要素 + 膝正角度維持

### 6.1 設計方針

V2の失敗（その場ジャンプ）とexp002の教訓から、**段階的改善**の原則に従う。

**基本方針**:
1. **V1をベースとする**: V1は前進していた（X=0.74m/10s）
2. **exp002 V21の成功要素を取り入れる**: 穏やかな高さペナルティで交互歩行成功
3. **膝正角度維持を追加**: 逆関節の特性を活かすための最小限の制約

### 6.2 変更内容

#### 変更点（V1からの差分）

| パラメータ | V1 | V3 | 理由 |
|-----------|-----|-----|------|
| `base_height` | -30.0 | **-10.0** | V21レベルに緩和（動きやすく） |
| `base_height_high` | -15.0 | **-5.0** | V21レベルに緩和 |
| `knee_positive` | なし | **-0.5** | 膝が負角度になったらペナルティ |

**変更しないもの**（V1と同一）:
- 初期姿勢（hip_pitch=-0.3, knee_pitch=0.6, ankle_pitch=-0.3）
- 初期高さ（base_init_pos=0.30m）
- 高さ目標（base_height_target=0.25m）
- その他すべての報酬スケール

#### 高さペナルティの比較

| バージョン | base_height | base_height_high | 合計（高い時） |
|-----------|-------------|------------------|---------------|
| V1 (droid) | -30.0 | -15.0 | **-45.0** |
| V21 (biped) | -10.0 | -5.0 | **-15.0** |
| **V3 (droid)** | -10.0 | -5.0 | **-15.0** |

V1の高さペナルティは強すぎて「動きにくい」状態だった可能性がある。V21と同じ緩やかなペナルティに変更。

#### 膝正角度維持報酬

```python
def _reward_knee_positive(self):
    """膝関節が正の角度を維持することを促す報酬
    
    BSL-Droid Simplifiedの膝関節は正角度で後方屈曲（逆関節）。
    負の角度になると通常の膝のように前方に曲がってしまう。
    """
    left_knee = self.dof_pos[:, 3]   # left_knee_pitch_joint
    right_knee = self.dof_pos[:, 8]  # right_knee_pitch_joint
    
    # 負の角度にペナルティ（正で0、負で|角度|を返す）
    penalty = torch.clamp(-left_knee, min=0) + torch.clamp(-right_knee, min=0)
    return penalty
```

### 6.3 訓練

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_v3.py --max_iterations 500
```

### 6.4 評価結果

#### 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-v3 --no-viewer --duration 10
```

#### 定量的結果

| 指標 | V1 | V2 | V3 | 評価 |
|------|-----|-----|-----|------|
| X方向移動距離 | 0.74 m | 0.009 m | **2.972 m** | ✅ 大幅改善 |
| Y方向移動距離 | -0.40 m | 0.000 m | -0.334 m | △ 左ドリフト |
| 平均速度X | 0.080 m/s | - | **0.298 m/s** | ✅ 目標0.3に近い |
| Base height mean | 0.325 m | 0.261 m | **0.311 m** | △ 目標0.25mより高い |
| Base height std | 0.0082 m | 0.0415 m | **0.0022 m** | ✅ 非常に安定 |
| DOF velocity RMS | 1.975 rad/s | 11.140 rad/s | **2.097 rad/s** | ✅ 正常範囲 |

#### 交互歩行の検証

| 指標 | V1 | V2 | V3 | 評価 |
|------|-----|-----|-----|------|
| hip_pitch相関 | +0.530 | +0.846 | **+0.335** | △ 改善したが交互ではない |

**hip_pitch相関の解釈**:
- V2の+0.846（ジャンプ）から**+0.335**に改善
- しかし目標の**-0.5以下**（交互歩行）には到達していない
- ユーザー観察:「前足と後足の固定が生じている」→ 片方の脚が前、もう片方が後ろで固定

#### 関節可動域 [rad]

| 関節 | 左脚 | 右脚 | 備考 |
|------|------|------|------|
| hip_pitch | [-0.381, 0.111] (0.491) | [-0.503, 0.000] (0.503) | 左右非対称 |
| hip_roll | [-0.349, 0.026] (0.375) | [-0.014, 0.334] (0.348) | - |
| **knee** | **[-0.300, 0.385]** (0.685) | **[-0.412, 0.239]** (0.651) | ❌ **負領域あり** |
| ankle_pitch | [0.115, 0.850] (0.735) | [0.139, 0.809] (0.671) | - |

**膝関節の問題**: 膝が**負の角度**（-0.300〜-0.412 rad）に入っている。これはユーザー観察の「膝が前に突き出す」状態に対応する。`knee_positive`ペナルティ（-0.5）が弱すぎて、膝の負角度を完全に抑制できていない。

#### exp002 V21との比較

| 指標 | exp002 V21 | exp004 V3 | 備考 |
|------|-----------|-----------|------|
| X方向移動距離 | +2.186 m | **+2.972 m** | ✅ V3が優秀 |
| 平均速度X | 0.221 m/s | **0.298 m/s** | ✅ V3が目標に近い |
| Base height | 0.449 m | 0.311 m | V3の方が低い（目標に近い） |
| hip_pitch相関 | **-0.752** | +0.335 | ❌ V21は交互歩行成功 |
| DOF velocity RMS | 1.845 rad/s | 2.097 rad/s | ほぼ同等 |

**重要な差異**: exp002 V21は**交互歩行（hip_pitch相関-0.752）**を達成しているが、exp004 V3は交互歩行に失敗している（+0.335）。

### 6.5 V3の分析

#### 成功点

1. **ジャンプ問題の解消**: V2の「その場ジャンプ」（hip_pitch相関+0.846）から脱却
2. **前進成功**: 10秒で約3m移動、目標速度0.3m/sにほぼ到達
3. **安定性**: Base height stdが0.0022mと非常に安定

#### 残存課題

1. **膝が負角度に入る**: `knee_positive`ペナルティ（-0.5）が弱すぎる
2. **交互歩行の欠如**: hip_pitch相関+0.335は「片足固定歩行」を示唆
3. **左ドリフト**: Y方向に-0.334m、Yaw角が-5°

#### 「片足固定歩行」の考察

ユーザー観察と定量データから、V3は以下の歩行パターンと推測される：

```
[片足固定歩行]
- 左脚: 前方に出たまま固定（hip_pitch負）
- 右脚: 後方に出たまま固定（hip_pitch負）
- 推進力: 足首の蹴り出しのみで前進

[正常な交互歩行（V21）]
- 左右の脚が交互に前後入れ替わる
- hip_pitch相関が-0.5以下
```

**原因仮説**: 
1. 膝が負角度に入ることで、脚を振り出す余裕がなくなる
2. `knee_positive`ペナルティが弱く、負角度を十分に抑制できていない
3. 高さペナルティの緩和（V21レベル）と膝制約の組み合わせが不適切

### 6.6 V4への改善方針

#### 方針1: 膝正角度ペナルティを強化

```python
# V3
"knee_positive": -0.5,

# V4（提案）
"knee_positive": -2.0,  # 4倍に強化
```

#### 方針2: 膝の最小角度を設定

デフォルト姿勢の膝角度（0.6 rad）に対して、大きく下回らないよう制約：

```python
def _reward_knee_min_angle(self):
    """膝が最小角度（0.3 rad ≈ 17°）を下回らないようにする"""
    min_knee_angle = 0.3  # 最小値
    left_knee = self.dof_pos[:, 3]
    right_knee = self.dof_pos[:, 8]
    
    penalty = torch.clamp(min_knee_angle - left_knee, min=0) + \
              torch.clamp(min_knee_angle - right_knee, min=0)
    return penalty

# 報酬スケール
"knee_min_angle": -1.0,
```

#### 方針3: exp002 V21の完全な報酬構造を採用

V3では高さペナルティのみV21から採用したが、V21には他にも交互歩行を促進する報酬がある：

- `hip_pitch_alternation`: 2.0（左右hip_pitchの逆相運動報酬）
- `hip_pitch_velocity`: 0.5（hip_pitchの速度報酬）
- `contact_alternation`: 0.8（接地タイミングの交互性報酬）

これらはV1/V3でも同じ値だが、膝制約との相互作用で効果が変わった可能性がある。

---

## 7. V4: V3ベース + 膝制約強化

### 7.1 設計方針

V3の「膝が負角度に入る」「片足固定歩行」問題を解決するため、膝制約を強化する。

### 7.2 変更内容

#### 変更点（V3からの差分）

| パラメータ | V3 | V4 | 理由 |
|-----------|-----|-----|------|
| `knee_positive` | -0.5 | **-2.0** | 4倍に強化 |
| `knee_min_angle` | なし | **-1.0** | 0.3rad以下でペナルティ（新規追加） |

**変更しないもの**（V3と同一）:
- 高さペナルティ（base_height: -10.0, base_height_high: -5.0）
- 初期姿勢（hip_pitch=-0.3, knee_pitch=0.6, ankle_pitch=-0.3）
- その他すべての報酬スケール

### 7.3 訓練

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_v4.py --max_iterations 500
```

### 7.4 評価結果

#### 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-v4 --no-viewer --duration 10
```

#### 定量的結果

| 指標 | V3 | V4 | 評価 |
|------|-----|-----|------|
| X方向移動距離 | 2.972 m | **2.977 m** | ≈ 同等 |
| Y方向移動距離 | -0.334 m | **+0.228 m** | △ ドリフト方向が逆転 |
| 平均速度X | 0.298 m/s | **0.298 m/s** | ≈ 同等 |
| Base height mean | 0.311 m | **0.307 m** | ≈ 同等 |
| Base height std | 0.0022 m | **0.0030 m** | ≈ 同等 |
| DOF velocity RMS | 2.097 rad/s | **1.624 rad/s** | ✅ 改善（より滑らか） |
| Yaw角（10秒後） | -5° | **+17°** | ❌ 右旋回が増加 |

#### 交互歩行の検証

| 指標 | V3 | V4 | 評価 |
|------|-----|-----|------|
| hip_pitch相関 | +0.335 | **+0.167** | △ 改善したが交互ではない |

**改善**: hip_pitch相関が+0.335から**+0.167**に低下。片足固定歩行の傾向が**やや緩和**された。

#### 関節可動域 [rad]

| 関節 | V3 左脚 | V4 左脚 | V3 右脚 | V4 右脚 | 備考 |
|------|---------|---------|---------|---------|------|
| hip_pitch | [-0.381, 0.111] (0.491) | **[-0.146, 0.000]** (0.146) | [-0.503, 0.000] (0.503) | **[-0.220, 0.000]** (0.220) | ❌ 可動域が大幅減少 |
| knee | [-0.300, 0.385] (0.685) | **[-0.380, 0.180]** (0.561) | [-0.412, 0.239] (0.651) | **[-0.485, 0.063]** (0.548) | ❌ まだ負領域 |
| ankle_pitch | [0.115, 0.850] (0.735) | **[0.196, 0.806]** (0.609) | [0.139, 0.809] (0.671) | **[0.270, 0.883]** (0.612) | ≈ 同等 |

**膝関節の状況**:
- V4でも膝が**負の角度**（-0.380〜-0.485 rad）に入っている
- V3より**悪化**（V3: -0.300〜-0.412 → V4: -0.380〜-0.485）
- ペナルティを強化しても膝の負角度侵入を防げていない

**hip_pitchの可動域縮小（副作用）**:
- V3: 0.491〜0.503 rad → V4: **0.146〜0.220 rad**
- 可動域が**1/3以下**に縮小
- 膝ペナルティが強すぎて、ポリシーが「動かない」方向に収束した可能性

#### Total DOF range sum

| バージョン | Total DOF range sum | 評価 |
|-----------|---------------------|------|
| V3 | 4.860 rad | - |
| V4 | **3.875 rad** | ❌ 減少（動きが小さくなった） |

### 7.5 V4の分析

#### 改善点

1. **hip_pitch相関の低下**: +0.335 → +0.167（片足固定がやや緩和）
2. **DOF velocity RMS低下**: 2.097 → 1.624 rad/s（動きが滑らかに）

#### 残存課題

1. **膝が負角度に入ったまま**: ペナルティ強化しても膝の負角度侵入を防げていない
2. **hip_pitch可動域の縮小**: 膝ペナルティの副作用で全体の動きが抑制された
3. **右旋回の増加**: Yaw角が-5° → +17°と悪化

#### 根本的な問題の考察

##### V4で膝ペナルティが機能しなかった理由

膝ペナルティ（`knee_positive`, `knee_min_angle`）の**方向は正しい**（負角度にペナルティ）。しかし、以下の理由で効果が限定的だった：

1. **報酬間の競合**: 前進報酬を最大化するために、ポリシーは膝を負角度にせざるを得ない構造になっている可能性
2. **物理的制約**: 現在の初期姿勢（膝0.6 rad = 約34°）では、歩行中に膝が負角度に入らざるを得ない
3. **副作用の発生**: ペナルティを強化した結果、「動かない」方向に収束し、hip_pitch可動域が縮小

##### exp002とexp004の膝角度定義の違い

| 項目 | exp002 (biped_digitigrade) | exp004 (droid_simplified) |
|------|---------------------------|---------------------------|
| 膝の逆関節方向 | **負角度**（-120°〜0°） | **正角度**（0°〜+135°） |
| 膝が前に出る | 正角度で前に出る | **負角度で前に出る** |
| V21での膝可動域 | [-0.429, 1.033] rad | - |

**重要な発見**: exp002 V21では膝が**負領域（-0.429 rad）にも入っている**が、それは逆関節として正常な動作。一方、exp004では膝が負領域に入ると**逆関節ではなく通常の膝**のように動いてしまう。

つまり、**同じ「負角度」でも意味が逆**であり、exp002の成功パターンをそのまま適用できない。

##### exp002 V21の参照度合いについて

V3/V4では以下の要素のみV21から参照した：

| 要素 | V21から参照 | 備考 |
|------|------------|------|
| 高さペナルティ緩和 | ✅ | base_height: -10.0, base_height_high: -5.0 |
| backward_velocity | ✅ | -2.0 |
| 膝正角度維持 | ❌ | exp004独自（膝角度符号が逆のため） |
| hip_pitch_alternation | ✅ | 2.0（V1から継承） |
| hip_pitch_velocity | ✅ | 0.5（V1から継承） |
| contact_alternation | ✅ | 0.8（V1から継承） |

**参照しなかった/できなかった要素**:
- V21の**初期姿勢**（膝角度の符号が逆のため直接使えない）
- V21の**膝可動域設計**（exp002では負角度も正常動作のため参考にならない）

### 7.6 V5への改善方針

#### 問題の整理

現在の問題は「膝ペナルティの方向が間違っている」ではなく、「**ペナルティだけでは膝の負角度侵入を防げない**」である。

物理的に考えると：
- 初期膝角度0.6 rad（約34°）は**浅すぎる**
- 歩行中に脚を振り出すと、膝が伸びて0°を超え、負角度に入ってしまう
- ペナルティで抑制しても、前進するためには負角度が必要な構造になっている

#### 方針: 初期姿勢で膝を深く曲げる

**根本解決**: 膝の初期角度を大きくすることで、歩行中に負角度に入る余地を減らす。

```python
# V5: 初期姿勢を深く曲げる
"left_knee_pitch_joint": 1.2,    # V1: 0.6 → V5: 1.2（約69°）
"right_knee_pitch_joint": 1.2,

# 膝を深く曲げるので、股関節と足首も調整
"left_hip_pitch_joint": -0.5,    # V1: -0.3 → V5: -0.5
"left_ankle_pitch_joint": -0.7,  # V1: -0.3 → V5: -0.7

# 初期高さも下げる
"base_init_pos": [0.0, 0.0, 0.25],  # V1: 0.30 → V5: 0.25
```

#### 膝ペナルティの扱い

膝ペナルティは**維持**するが、強化はしない（V3レベルに戻す）：

```python
"knee_positive": -0.5,      # V4: -2.0 → V5: -0.5（V3と同じ）
"knee_min_angle": 削除,     # V4で追加したが効果なし
```

#### 期待される効果

1. **膝が深く曲がった状態からスタート**: 歩行中に伸びても0°を超えにくい
2. **ペナルティの負担軽減**: 初期状態で既に正角度が確保されているため、ペナルティに頼らずに済む
3. **hip_pitch可動域の回復**: ペナルティを緩和することで動きが回復

---

## 8. V5: 深い膝初期姿勢

### 8.1 変更内容

V4からの変更点：

| 項目 | V4 | V5 | 変更理由 |
|------|-----|-----|---------|
| `knee_pitch`初期角度 | 0.6 rad | 1.2 rad | 深く曲げて負角度に入りにくくする |
| `hip_pitch`初期角度 | -0.3 rad | -0.5 rad | 膝を深く曲げるため股関節を調整 |
| `ankle_pitch`初期角度 | -0.3 rad | -0.7 rad | 膝と相殺して足裏を地面に接地 |
| `base_init_pos[2]` | 0.30 m | 0.25 m | 膝を深く曲げるため高さを下げる |
| `base_height_target` | 0.25 m | 0.22 m | 高さ目標も下げる |
| `knee_positive` | -2.0 | -0.5 | V3レベルに戻す（副作用回避） |
| `knee_min_angle` | -1.0 | 削除 | 効果なしのため削除 |

### 8.2 訓練結果

- イテレーション: 499
- 最終報酬: 約0.7〜0.8

### 8.3 評価結果

```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 3.018 m
  Y: 0.077 m
  Total: 3.019 m

Average velocity:
  X: 0.302 m/s (target: 0.300)

Base height:
  Mean: 0.247 m
  Std: 0.0064 m

Orientation (deg):
  Roll:  mean= -0.88, std= 1.89
  Pitch: mean=  0.82, std= 0.88
  Yaw:   start=  0.00, end= -0.64, drift= -0.64

DOF velocity RMS: 1.593 rad/s

=== Joint Movement Analysis ===
DOF position range (rad) [min, max, range]:
  hip_pitch   : L=[-0.474, -0.000] (0.474)  R=[-0.540,  0.051] (0.592)
  knee        : L=[-1.198, -0.345] (0.853)  R=[-0.898, -0.406] (0.492)
  ankle_pitch : L=[ 1.161,  1.744] (0.583)  R=[ 1.215,  1.745] (0.529)

Left-Right hip_pitch correlation: 0.610
  (-1.0 = perfect alternating, +1.0 = perfect synchronized)

=== Knee Angle Analysis ===
Left knee:  min=-1.198, max=-0.345, mean=-0.813 rad   ← 評価スクリプトのバグ（後述）
Right knee: min=-0.898, max=-0.406, mean=-0.652 rad

Negative angle ratio:
  Left knee:  100.0%   ← 常に負角度
  Right knee: 100.0%

=== Contact Pattern Analysis ===
Both feet grounded:     0 steps (  0.0%)
Single foot grounded:   0 steps (  0.0%)
Both feet airborne:   500 steps (100.0%)   ← 常に浮いている
```

### 8.4 V5の分析

#### 深刻な問題

1. **膝が常に負角度（前方に突き出し）**
   - 設定: 初期膝角度 1.2 rad（正）
   - 結果: 膝角度 -1.198 〜 -0.345 rad（**常に負**）
   - **初期姿勢の設定が物理シミュレーションに反映されていない可能性**

2. **常に両足離地（浮いている）**
   - 接地判定が全ステップで失敗
   - 「スライド歩行」ではなく「浮遊歩行」状態

3. **hip_pitch相関が正（+0.610）**
   - 交互歩行ではなく同期歩行（両脚が同じ方向に動く）
   - V4（+0.167）より悪化

#### 目視観察との整合

ユーザー報告:
- 「膝は前に突き出すように曲がったまま」→ **膝が負角度（確認）**
- 「脚全体が鉛直ではなく斜め」→ **前傾姿勢（hip_pitch=-0.5, ankle_pitch=+1.7）**

#### 原因の考察

**仮説1: 初期姿勢が反映されていない**

train側で設定した`default_joint_angles`がenv側に正しく渡されていない可能性：
- V5のenv: V4からコピー（docstring/クラス名のみ変更）
- V5のtrain: 初期姿勢を1.2 radに設定

**仮説2: 物理的に不安定な初期姿勢**

膝1.2 rad + hip_pitch -0.5 rad + ankle_pitch -0.7 radの組み合わせが物理的に不安定で、シミュレーション開始直後に崩れた可能性。

**仮説3: 足首角度の設定ミス**

評価ログで`ankle_pitch`が+1.16〜+1.74 radと表示されているが、train設定は-0.7 rad。
**関節インデックスの不整合**の可能性。

### 8.5 緊急調査: 関節インデックスの確認

評価スクリプトの関節インデックスを確認する必要がある。

現在の評価スクリプトの想定:
```python
# droid-walking系: インデックス3, 8 (left_knee_pitch, right_knee_pitch)
left_knee_idx, right_knee_idx = 3, 8
```

実際のjoint_names順序（train側）:
```python
"joint_names": [
    "left_hip_yaw_joint",      # 0
    "left_hip_roll_joint",     # 1
    "left_hip_pitch_joint",    # 2
    "left_knee_pitch_joint",   # 3  ← 膝
    "left_ankle_pitch_joint",  # 4
    "right_hip_yaw_joint",     # 5
    "right_hip_roll_joint",    # 6
    "right_hip_pitch_joint",   # 7
    "right_knee_pitch_joint",  # 8  ← 膝
    "right_ankle_pitch_joint", # 9
]
```

**評価スクリプトのインデックスは正しい**。

では「Knee Angle Analysis」で表示された値（min=-1.198等）と「DOF position range」のknee（L=[-1.198, -0.345]）が一致しているので、**膝が本当に負角度に入っている**。

### 8.6 V6への改善方針

#### 問題の根本原因

初期姿勢として設定した膝角度（+1.2 rad）が維持されず、学習過程で負角度（-1.2 rad程度）に収束してしまった。

考えられる原因：
1. **similar_to_default報酬が弱すぎる**: -0.02では初期姿勢維持の圧力が不足
2. **forward_progress報酬との競合**: 前進するために負角度の方が有利と学習した
3. **膝ペナルティ（knee_positive）の効果不足**: -0.5では抑止力が足りない

#### 調査: RL理論とLegged Gym実装の確認

`ref/rl_ppo_lesson.md`およびETH ZurichのLegged Gym実装を調査した結果：

1. **報酬ハッキングへの対策**（rl_ppo_lesson.md）:
   - 「複数の報酬項のバランス、制約の追加」が推奨される
   - 単一のペナルティ強化では局所解から脱出困難

2. **Legged Gymの`_reward_dof_pos_limits`実装**:
   ```python
   def _reward_dof_pos_limits(self):
       # Penalize dof positions too close to the limit
       out_of_limits = -(self.dof_pos - self.dof_pos_limits[:, 0]).clip(max=0.)  # lower
       out_of_limits += (self.dof_pos - self.dof_pos_limits[:, 1]).clip(min=0.)  # upper
       return torch.sum(out_of_limits, dim=1)
   ```
   - 関節可動域のソフトリミットとして広く使用されている標準的手法
   - `soft_dof_pos_limit`パラメータで可動範囲を内側に絞り込む

#### 検討した選択肢

| オプション | 内容 | 採否 |
|-----------|------|------|
| A | `_reward_dof_pos_limits`を追加（Legged Gym方式） | **採用** |
| B | アクション空間を物理的にクリッピング | 不採用 |
| C | 既存報酬の大幅強化（similar_to_default, knee_positive） | **採用** |

**オプションB不採用の理由**:
1. 勾配情報が正しく伝播しない（clampの勾配は0になる領域がある）
2. 方策が「なぜその行動が悪いのか」を学習できない
3. Sim-to-Real転移時に安全系の二重管理が必要になる
4. Legged Gymが採用していない（報酬ベースのソフト制約を採用）

#### V6実装方針: オプションA + C

1. **`_reward_dof_pos_limits`報酬関数を新規追加**
   - 膝関節（index 3, 8）の下限を0.2 radに設定
   - 下限を下回った場合に線形ペナルティ
   - スケール: -5.0

2. **既存報酬の強化**
   - `similar_to_default`: -0.02 → **-0.1**（5倍）
   - `knee_positive`: -0.5 → **-3.0**（6倍）

3. **初期姿勢はV5を維持**
   - knee_pitch: 1.2 rad（深い膝曲げ）
   - base_init_pos: [0, 0, 0.25]

```python
# V6報酬スケール
"dof_pos_limits": -5.0,       # 新規追加（Legged Gym方式）
"similar_to_default": -0.1,   # V5: -0.02 → V6: -0.1（5倍）
"knee_positive": -3.0,        # V5: -0.5 → V6: -3.0（6倍）
```

---

## V6: 関節可動域ソフトリミット導入

### 9.1 目的

Legged Gymで実績のある`_reward_dof_pos_limits`報酬関数を導入し、膝関節が負角度に入ることを強く抑制する。

### 9.2 変更内容

**droid_env_v6.py**:
- `_reward_dof_pos_limits`関数を新規追加
- 膝関節（index 3, 8）の下限を0.2 radに設定

**droid_train_v6.py**:
- `dof_pos_limits`: -5.0（新規）
- `similar_to_default`: -0.02 → -0.1（5倍強化）
- `knee_positive`: -0.5 → -3.0（6倍強化）

### 9.3 訓練コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_v6.py --max_iterations 500
```

### 9.4 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-v6
uv run python biped_walking/biped_eval.py -e droid-walking-v6 --no-viewer --duration 10
```

### 9.5 結果

#### 定量評価（10秒間）

```
Travel distance:
  X: 3.239 m
  Y: 0.089 m

Average velocity:
  X: 0.324 m/s (target: 0.300) ✓

Base height:
  Mean: 0.280 m (target: 0.22m → 高めで安定)

Orientation (deg):
  Roll:  mean= -1.15, std= 2.36
  Pitch: mean=  0.15, std= 0.53
  Yaw drift: -1.01° (10秒で)

DOF velocity RMS: 1.238 rad/s

=== Knee Angle Analysis ===
Left knee:  min= 1.036, max= 1.344, mean= 1.227 rad
Right knee: min= 1.075, max= 1.330, mean= 1.188 rad

Negative angle ratio:
  Left knee:    0.0%  ✓✓✓
  Right knee:   0.0%  ✓✓✓

=== Contact Pattern Analysis ===
Both feet grounded:       0 steps (  0.0%)
Single foot grounded:     0 steps (  0.0%)
Both feet airborne:     500 steps (100.0%) ✗

Left-Right hip_pitch correlation: 0.561
  (V5: +0.610 → V6: +0.561 やや改善)
```

#### V5との比較

| 指標 | V5 | V6 | 評価 |
|------|-----|-----|------|
| X移動距離 | 3.018 m | 3.239 m | ✓改善 |
| Y移動距離 | 0.077 m | 0.089 m | 同等 |
| 膝負角度比率 | 100% | **0%** | ✓✓大幅改善 |
| 膝角度範囲 | -1.2〜-0.3 rad | +1.0〜+1.3 rad | ✓✓正常化 |
| 両足宙浮き | 100% | 100% | ✗未解決 |
| hip_pitch相関 | +0.610 | +0.561 | △やや改善 |

### 9.6 考察

#### 成功点

1. **膝の負角度侵入を完全に防止**: `dof_pos_limits`報酬とペナルティ強化により、膝が負角度に入ることを100%防止できた。これはV1〜V5で解決できなかった最大の課題。

2. **直進性の改善**: Y方向のドリフトが小さく（0.089m/10s）、Yaw drift も-1°程度と良好。

3. **速度追従**: 目標0.3 m/sに対して0.324 m/sと良好な追従。

#### 残存課題: 膝が曲がったまま

ユーザーの目視観察「膝が前に突き出すように曲がったまま」と一致する現象:

- 膝角度が1.0〜1.3 rad（約57°〜75°）の範囲で**常に曲がった状態**
- デフォルト姿勢の膝角度（1.2 rad）付近で固定
- **膝の伸展動作がほとんど発生していない**

原因分析:
1. `similar_to_default: -0.1`が強すぎ、デフォルト姿勢から離れることを過度に抑制
2. 結果として膝が1.2 rad付近に「ロック」される
3. 歩行に必要な膝の伸展（角度を小さくする）動作が発生しない

#### Contact Pattern（両足宙浮き100%）について

「両足宙浮き100%」は**滑走歩行**を示唆:
- 足が地面と摩擦接触しながら滑っている
- 明確な離地・着地サイクルがない
- 膝が曲がったまま固定されているため、正常な歩行サイクルが形成されない

### 9.7 URDF・報酬設計の修正（V6後の発見）

#### 問題の根本原因

RViz2 + joint_state_publisher_gui で膝関節の動作を確認したところ、以下の問題が判明：

1. **joint_gui.pyのリミット定義ミス**: 膝の可動域が `-120°〜0°` と設定されていたが、URDFは `0°〜+135°` だった
2. **URDFの軸定義問題**: 膝だけ `axis xyz="0 1 0"` で正角度→前方屈曲となっており、「逆関節」ではなく通常の膝と同じ動き

#### 採用した修正方針

**「軸は統一、リミットで逆関節を表現」（Unitree/ANYmal規約）**

| 関節 | 軸 | リミット | 意味 |
|------|-----|----------|------|
| hip_pitch | (0, 1, 0) | -120°〜+60° | 正で前方振り出し |
| **knee_pitch** | **(0, 1, 0)** | **-135°〜0°** | **負で後方屈曲（逆関節）** |
| ankle_pitch | (0, 1, 0) | -60°〜+60° | 正でつま先上げ |

**メリット**:
1. 全ピッチ軸が同じ規約（一貫性）
2. 「膝を曲げる = 負角度」という明確な意味
3. 四脚ロボット（Unitree Go1/2, ANYmal）やLegged Gymの標準実装と整合

#### 修正内容

1. **xacroファイル**: 膝リミットを `0°〜+135°` → `-135°〜0°` に変更
2. **joint_gui.py**: 膝リミットを `-135°〜0°` に修正
3. **rl_ws/assets/bsl_droid_simplified.urdf**: 再エクスポート
4. **droid_env.py**: 
   - `_reward_knee_positive` → `_reward_knee_negative` にリネーム（正角度にペナルティ→負角度に維持）
   - `_reward_knee_min_angle`, `_reward_dof_pos_limits` も符号反転
5. **droid_train_v6.py**: デフォルト姿勢の膝角度を `+1.2` → `-1.2` に変更

#### V7での再学習が必要

V1〜V6の学習結果は**旧規約（正角度で屈曲）**に基づいているため、**全て無効**となる。V7以降は新規約で再学習が必要。

### 9.8 V7: 新規約での再学習

> **注意**: 評価コマンド（`biped_eval.py`）は実行後、結果が出力されるまでに時間がかかる場合があります。モデルのロードやシミュレーション初期化に数秒〜数十秒を要することがあるため、即座に結果が表示されなくても正常です。

#### V7概要

- **実験名**: `droid-walking-v7`
- **変更内容**: URDF負角度規約（Unitree/ANYmal標準）への移行後の初回学習
- **訓練設定**:
  - デフォルト姿勢: hip_pitch=-0.5, knee=-1.2, ankle=0.7 (rad)
  - 初期高さ: 0.25m
  - base_height_target: 0.22m
  - 報酬: `knee_negative`（正角度侵入にペナルティ）
- **訓練**: 500イテレーション

#### V7評価結果

```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: -0.013 m     ← ほぼ前進せず
  Y: -0.034 m
  Total: 0.037 m

Average velocity:
  X: -0.001 m/s (target: 0.300)  ← 目標0.3 m/sに対して0

Base height:
  Mean: 0.083 m   ← 0.25mから0.08mに低下（座り込み）
  Std: 0.0162 m

DOF velocity RMS: 1.259 rad/s

=== Joint Movement Analysis ===
DOF position range (rad) [min, max, range]:
  knee: L=[-2.093, -0.456] (1.637)  R=[-2.089, -0.330] (1.759)
        ← 膝が-2 rad（-115°）近くまで深く曲がる

=== Contact Pattern Analysis ===
Both feet grounded:       0 steps (  0.0%)
Single foot grounded:     0 steps (  0.0%)
Both feet airborne:     500 steps (100.0%) ← 両足が地面に接触していない
```

#### V7考察

**深刻な問題**:
1. **即座に崩れ落ちている**: z座標が0.247m→0.080m（0.6秒で最低点に到達し地面に座り込み）
2. **前進ゼロ**: X方向移動が-0.013m（10秒間で実質静止）
3. **両足宙浮き100%**: 足裏が地面に接触せず、お尻が地面についた状態
4. **膝が過度に深く曲がる**: knee=-2.0 rad（-115°）まで曲がっている

**原因分析**:
- 初期高さ0.25mが、hip_pitch=-0.5, knee=-1.2の姿勢に対して低すぎる
- base_height_target=0.22mも低すぎて、「しゃがんで座り込む」ことが報酬的に有利になっている
- 膝の深い屈曲（-2 rad）が許容されており、完全にしゃがみ込んだ状態

**V8への改善方針**:
1. **初期姿勢を「立っている」姿勢に変更**: より高い立位姿勢
   - hip_pitch: 60° (≈1.047 rad) - 脚を後ろに引いた姿勢
   - knee_pitch: -100° (≈-1.745 rad) - 適度な屈曲
   - ankle_pitch: 45° (≈0.785 rad) - 足裏水平に近づける
2. **初期高さを上げる**: 0.25m → 0.35m以上
3. **base_height_targetを上げる**: 0.22m → 0.30m以上
4. **膝の最小角度制限**: 膝が-2 rad以下に曲がりすぎないよう制約

### 9.10 V8: 立位姿勢と幾何学的高さ計算

#### V8概要

- **実験名**: `droid-walking-v8`
- **目的**: V7の「座り込み」問題を解決し、適切な立位姿勢で歩行を学習させる

#### 初期姿勢の設計

ユーザー指定の初期角度:
- hip_pitch: 60° (1.047 rad) - 脚を後ろに引いた姿勢
- knee_pitch: -100° (-1.745 rad) - 適度な屈曲（-2 radまでは想定動作範囲内）
- ankle_pitch: 45° (0.785 rad) - 足裏を水平に近づける

#### 幾何学的高さ計算

URDFのリンク寸法から、指定姿勢でのbase_link高さを計算:

**パラメータ（xacroより）:**
| パラメータ | 値 |
|-----------|-----|
| thigh_length | 0.11m |
| shank_length | 0.12m |
| foot_height | 0.035m |
| torso_height | 0.16m |
| 胴体中心→股関節 | 0.08m (= torso_height/2) |

**計算:**
```
1. 大腿の垂直成分: thigh × cos(hip_pitch)
   = 0.11 × cos(60°) = 0.11 × 0.5 = 0.055m

2. 下腿の累積角度: hip_pitch + knee_pitch = 60° + (-100°) = -40°
   下腿の垂直成分: shank × cos(|-40°|)
   = 0.12 × cos(40°) = 0.12 × 0.766 = 0.092m

3. 股関節→足首: 0.055 + 0.092 = 0.147m

4. 足首→地面: foot_height = 0.035m

5. 股関節→地面: 0.147 + 0.035 = 0.182m

6. base_link中心→地面: 0.182 + 0.08 = 0.262m
```

#### V8設定

| パラメータ | V7 | V8 | 備考 |
|-----------|-----|-----|------|
| hip_pitch | -0.5 rad | 1.047 rad (60°) | 脚を後ろに引く |
| knee_pitch | -1.2 rad | -1.745 rad (-100°) | 適度な屈曲 |
| ankle_pitch | 0.7 rad | 0.785 rad (45°) | 足裏水平 |
| base_init_pos[2] | 0.25m | 0.35m | 落下して安定するため余裕 |
| base_height_target | 0.22m | **0.26m** | 幾何学的計算値 |
| knee_max_angle | なし | -2.4 rad | URDFリミット近傍のソフト制限 |
| similar_to_default | -0.1 | -0.05 | デフォルト姿勢固着を緩和 |

#### 補足: 膝角度-2 radは想定動作範囲内

V7考察で「膝が-2 rad（-115°）まで曲がる」を問題視したが、これは**想定動作範囲内**。
真の問題は`base_height_target=0.22m`が低すぎて「座り込み」が報酬的に有利だったこと。

V8では:
- 膝の深い屈曲（-2 rad程度）は許容
- URDFリミット(-150° = -2.618 rad)に近づきすぎた場合のみ`knee_max_angle`でソフトペナルティ
- `base_height_target=0.26m`により「立っている高さ」を維持することを報酬で誘導

#### URDF可動範囲の拡大

V8に合わせてxacroの可動範囲も拡大:

| 関節 | 変更前 | 変更後 |
|------|--------|--------|
| hip_pitch | -120°〜+60° | -120°〜**+90°** |
| knee_pitch | -135°〜0° | **-150°**〜0° |
| ankle_pitch | ±60° | **±90°** |

#### V8評価結果

```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 2.583 m        ← V7の-0.013mから大幅改善！
  Y: -0.199 m
  Total: 2.591 m

Average velocity:
  X: 0.259 m/s (target: 0.300)  ← 目標の86%達成
  Y: 0.008 m/s

Base height:
  Mean: 0.214 m     ← 目標0.26mより低い（後述）
  Std: 0.0124 m

Orientation (deg):
  Roll:  mean= -0.00, std= 1.01   ← 安定
  Pitch: mean=  3.00, std= 1.15   ← やや前傾
  Yaw:   start=  0.00, end=-15.70, drift=-15.70  ← 右方向にドリフト

DOF velocity RMS: 1.743 rad/s
Action RMS: 0.757

=== Joint Movement Analysis ===
DOF position range (rad) [min, max, range]:
  hip_pitch   : L=[-0.140,  0.049] (0.189)  R=[-0.178,  0.513] (0.691)
  hip_roll    : L=[-0.453,  0.008] (0.460)  R=[-0.444,  0.000] (0.444)
  knee        : L=[ 1.016,  1.441] (0.425)  R=[ 0.806,  1.382] (0.576)
  ankle_pitch : L=[-2.142, -1.698] (0.444)  R=[-1.770, -1.584] (0.186)

Left-Right hip_pitch correlation: 0.772
  (-1.0 = perfect alternating, +1.0 = perfect synchronized)
  ← 左右が同期（並行移動）している！交互歩行になっていない

=== Knee Angle Analysis ===
Left knee:  min=-2.142, max=-1.698, mean=-2.003 rad  ← 約-115°
Right knee: min=-1.770, max=-1.584, mean=-1.704 rad  ← 約-98°

=== Contact Pattern Analysis ===
Both feet grounded:       0 steps (  0.0%)
Single foot grounded:     0 steps (  0.0%)
Both feet airborne:     500 steps (100.0%)  ← 滑走歩行（両足が常に接地判定外）
```

#### V8考察

**成功点**:
1. **前進を達成**: V7の静止状態から、10秒で2.58m前進（目標速度の86%）
2. **姿勢安定**: Roll/Pitch変動が小さく、転倒せずに歩行継続
3. **膝角度正常**: 膝が負角度（-1.7〜-2.1 rad）を維持し、逆関節らしい姿勢

**残存課題**:

| 課題 | 定量データ | 原因分析 |
|------|-----------|---------|
| 左右同期（並行移動） | hip_pitch相関 +0.772 | 交互歩行報酬が不十分、または同期が報酬的に有利 |
| 小さいストライド | hip_pitch range: L=0.19, R=0.69 rad | hip_pitch動作範囲が小さい |
| 足の持ち上げ量不足 | 両足宙浮き100%（滑走） | 明確な離着地サイクルがない |
| Yawドリフト | -15.7°/10s | 左右非対称な動作 |
| 目標高さ未達 | 実測0.214m vs 目標0.26m | 低い姿勢が報酬的に有利になっている可能性 |

#### 問題の深堀り分析

**1. 「並行移動」問題の原因**

hip_pitch相関が+0.772（理想は-1.0）であることから、左右の脚が**同じ方向に同時に動いている**:
- 両脚を同時に前に振る→両脚を同時に後ろに振る
- これは「スキップ」や「ホッピング」に近い動き
- 交互歩行（片足を前に出しながら反対足を後ろに引く）ができていない

考えられる原因:
1. `hip_pitch_alternation`報酬が弱すぎる（現在2.0）
2. `similar_to_default`ペナルティが左右同期を誘発
3. 接地判定の閾値が低すぎて、常に「両足宙浮き」と判定され交互歩行報酬が機能しない

**2. 足の持ち上げ量問題**

「両足宙浮き100%」は実際には**足が地面に接触しながら滑っている**状態:
- 接地判定閾値（0.025m）が足の高さに対して低すぎる可能性
- または足が常に地面スレスレを滑走している

足を持ち上げるには:
1. `feet_air_time`報酬を強化（明確な離地→着地サイクルを誘導）
2. `foot_clearance`報酬を追加（足の最低高さを報酬化）
3. 接地判定閾値の調整

**3. base_height_target未達問題**

目標0.26mに対して実測0.214m（-0.046m、18%低い）:
- 幾何学的計算では0.262mだが、実際には膝がより深く曲がっている
- 膝の平均角度: L=-2.0 rad, R=-1.7 rad（デフォルト-1.745 radより深い）
- 低い姿勢の方が安定性が高く、報酬的に有利になっている可能性

---

### 9.12 今後の検討事項（V9以降への示唆）

#### A. 交互歩行の強化

| 対策 | 詳細 |
|------|------|
| `hip_pitch_alternation`報酬強化 | 2.0 → 4.0〜5.0 に増加 |
| 位相ベース報酬の追加 | 左右脚に180°位相差を持つ参照軌道を与える |
| `contact_alternation`報酬強化 | 片足ずつの離着地に強いボーナス |
| 両足同時離着地ペナルティ | 両足が同時に接地/離地した場合にペナルティ |

#### B. 足の持ち上げ量の改善

| 対策 | 詳細 |
|------|------|
| `foot_clearance`報酬追加 | スイング中の足の最低高さを報酬化 |
| `feet_air_time`報酬強化 | 1.0 → 2.0〜3.0 に増加 |
| 接地判定閾値の調整 | 0.025m → 0.03〜0.04m に上げる |
| `swing_height`報酬追加 | 足のZ座標の最大値を報酬化 |

#### C. ストライド長の改善

| 対策 | 詳細 |
|------|------|
| hip_pitch動作範囲の報酬化 | 各ステップでのhip_pitch振幅を報酬化 |
| `step_length`報酬追加 | 前進距離/ステップ数を報酬化 |
| `similar_to_default`緩和 | -0.05 → -0.02 に減少（動作範囲を広げる） |

#### D. 高さ維持の改善

| 対策 | 詳細 |
|------|------|
| `base_height`ペナルティ強化 | -10.0 → -15.0〜-20.0 |
| 目標高さの調整 | 0.26m → 0.22〜0.24m（現実的な値に） |
| 低すぎペナルティの追加 | 0.20m以下で追加ペナルティ |

#### E. Yawドリフト対策

| 対策 | 詳細 |
|------|------|
| `yaw_rate`ペナルティ追加 | Yaw角速度にペナルティ |
| 左右対称性報酬 | 左右の関節角度/速度の差にペナルティ |

---

### 9.13 設計ノウハウまとめ

#### 1. 初期高さと目標高さの関係

| パラメータ | 役割 | 設定指針 |
|-----------|------|----------|
| `base_init_pos[2]` | シミュレーション開始時の配置高さ | 目標高さより高めに設定（落下して安定する） |
| `base_height_target` | 報酬で維持させる目標高さ | 幾何学的計算値、または実際に安定する高さ |

**落とし穴**: 目標高さが高すぎると、ロボットが「背伸び」して不安定になる。低すぎると「しゃがみ込み」が有利になる。

#### 2. 交互歩行報酬の設計

**hip_pitch相関係数の解釈**:
- -1.0: 完璧な交互歩行（片足前進、反対足後退）
- 0.0: 独立した動き
- +1.0: 完璧な同期（両足同時に同方向）

**V8では+0.772**であり、両足が同期して動いている。これは:
- 「スキップ」や「ホッピング」に近い動き
- 前進はできるが、効率が悪く不安定

#### 3. 接地判定の重要性

接地判定閾値が報酬関数に与える影響:
- 閾値が低すぎる: 滑走していても「宙浮き」と判定され、`feet_air_time`報酬が機能しない
- 閾値が高すぎる: 足を持ち上げても「接地」と判定され、同様に機能しない

BSL-Droid Simplified（foot_height=0.035m）の場合:
- 現在: 0.025m（低すぎる可能性）
- 推奨: 0.03〜0.04m

#### 4. 報酬バランスの原則

| カテゴリ | 報酬例 | バランス指針 |
|---------|--------|-------------|
| 主タスク | tracking_lin_vel, forward_progress | 最も強く（1.0〜2.0） |
| 歩容形成 | alternating_gait, feet_air_time | 中程度（0.5〜1.5） |
| 姿勢維持 | base_height, orientation | ペナルティとして強め（-5.0〜-15.0） |
| 振動抑制 | action_rate, dof_vel | 弱め（-0.01〜-0.1） |

**落とし穴**: 姿勢維持ペナルティが強すぎると、ロボットが「動かない」方が報酬的に有利になる。

#### 5. 膝角度の範囲と想定動作

BSL-Droid Simplified（逆関節）の膝角度解釈:
- 0° = 脚まっすぐ（伸展）
- -100°〜-120° = 通常の立位・歩行
- -150° = URDFリミット（過度な屈曲）

V8では膝が-1.7〜-2.1 rad（約-97°〜-120°）で動作しており、**想定範囲内**。

### 9.14 V1-V6学習済みモデルの再生について

#### 現状

V7でURDF規約を変更（膝: `0°〜+135°` → `-135°〜0°`）したため、V1-V6の学習済み重みは**現在のURDFでは正常に再生できない**。

ログデータ（`logs/droid-walking-v{1-6}/`）は学習過程の記録として保持する。

#### V1-V6を再生する方法（必要な場合）

旧規約のURDFを復元すればV1-V6の重みを再生可能：

```bash
# 1. 旧規約のURDFを生成（一時的）
cd ros2_ws

# xacroの膝リミットを一時的に旧規約に戻す
# bsl_droid_simplified.urdf.xacro の knee_pitch_joint を編集:
#   変更前: <limit lower="${-135 * pi / 180}" upper="${0 * pi / 180}" .../>
#   変更後: <limit lower="${0 * pi / 180}" upper="${135 * pi / 180}" .../>

# 2. 旧規約URDFをエクスポート
pixi run xacro src/biped_description/urdf/bsl_droid_simplified.urdf.xacro \
    > ../rl_ws/assets/bsl_droid_simplified_legacy.urdf

# 3. 評価時に旧URDFを指定（droid_train_v{1-6}.py の urdf_path を変更）
# または cfgs.pkl を編集して urdf_path を legacy ファイルに向ける

# 4. 評価実行
cd ../rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-v6

# 5. 終了後、xacroを新規約に戻すことを忘れずに
```

**注意**: 旧規約での再生は過去の実験結果の検証目的のみ。新規開発はV7以降の新規約で行うこと。

---

## ファイル構成

```
rl_ws/
├── assets/
│   ├── bsl_droid_simplified.urdf   # エクスポート済みURDF
│   └── ...
├── biped_walking/
│   ├── biped_eval.py               # 統一評価スクリプト（droid対応済み）
│   ├── envs/
│   │   ├── droid_env_v1.py         # Droid用環境クラスV1
│   │   ├── droid_env_v2.py         # Droid用環境クラスV2（V1からコピー、初期姿勢変更）
│   │   ├── droid_env_v3.py         # Droid用環境クラスV3（V1からコピー、膝報酬追加）
│   │   ├── droid_env_v4.py         # Droid用環境クラスV4（V3からコピー、膝制約強化）
│   │   └── ...
│   └── train/
│       ├── droid_train_v1.py       # Droid用訓練スクリプトV1
│       ├── droid_train_v2.py       # Droid用訓練スクリプトV2（初期姿勢変更）
│       ├── droid_train_v3.py       # Droid用訓練スクリプトV3（高さ緩和+膝制約）
│       ├── droid_train_v4.py       # Droid用訓練スクリプトV4（膝制約強化）
│       └── ...
└── logs/
    ├── droid-walking-v1/           # V1訓練ログ
    ├── droid-walking-v2/           # V2訓練ログ
    ├── droid-walking-v3/           # V3訓練ログ
    └── droid-walking-v4/           # V4訓練ログ
        ├── cfgs.pkl
        └── model_*.pt
```

---

## リスクと対策

### R1: 膝関節角度定義の統一

- **解決済み**: URDF修正により膝関節の規約をUnitree/ANYmal標準（負角度で屈曲）に統一
- **対策**: 
  - xacro, joint_gui.py, droid_env.py全てで負角度規約に修正
  - デフォルト姿勢の膝角度を `-1.2 rad` に設定

### R2: 脚長の違い

- **リスク**: 脚が短くなったことで歩幅や歩行周波数が変化
- **対策**: 
  - `base_init_pos`を低く設定（0.45m → 0.30m）
  - `base_height_target`を調整（0.35m → 0.25m）
  - 歩行周波数を調整する可能性

### R3: 頭部追加による重心変化

- **リスク**: 頭部（neck + head）の追加により重心位置が変化
- **対策**: 
  - 頭部は固定関節のため制御には影響しない
  - 重心変化は物理シミュレーションで自動的に考慮される

---

## V9: 交互歩行強化実験

V8の評価で判明した「並行脚運動」問題に対処するため、V9では交互歩行を明示的に促進する報酬を追加した。

### V9.1 V8からの課題

V8評価で明らかになった問題点：

| 項目 | V8結果 | 問題 |
|------|--------|------|
| hip_pitch相関 | +0.772 | 左右同期（交互でない） |
| hip_pitch範囲 | L=0.19, R=0.69 rad | ストライド不足 |
| 両足宙浮き率 | 100% | 滑走歩行 |
| Yawドリフト | -15.7°/10s | 直進不安定 |

### V9.2 報酬関数の変更

V8をベースに、交互歩行を促す報酬を強化：

```python
reward_cfg = {
    "gait_frequency": 1.5,  # Hz

    "reward_scales": {
        # ========== 主タスク（V8から継続） ==========
        "tracking_lin_vel": 1.5,
        "tracking_ang_vel": 0.5,
        "alive": 0.1,
        "forward_progress": 0.3,

        # ========== 交互歩行報酬（V9強化） ==========
        "alternating_gait": 1.5,       # V8: 0.5 → V9: 1.5
        "foot_swing": 0.3,             # 新規
        "feet_air_time": 2.0,          # V8: 1.0 → V9: 2.0
        "single_stance": 0.3,          # 新規
        "no_fly": -0.5,                # 新規（両足宙浮きペナルティ）

        # ========== hip_pitch動作報酬（V9新規） ==========
        "hip_pitch_alternation": 4.0,  # 新規：hip_pitchの位相差報酬
        "hip_pitch_velocity": 0.5,     # 新規：hip_pitch速度報酬
        "contact_alternation": 1.5,    # 新規：接地タイミング交互報酬

        # ========== 同期ペナルティ（V9新規） ==========
        "hip_pitch_sync_penalty": -3.0,  # 新規：hip_pitch同期ペナルティ

        # （姿勢維持、振動抑制等はV8から継続）
    },
}
```

### V9.3 新規報酬関数の詳細

#### `hip_pitch_alternation` (weight: 4.0)

左右hip_pitchの位相差に基づく報酬：

```python
def _reward_hip_pitch_alternation(self):
    """hip_pitchの交互動作を報酬化"""
    left_hp = self.dof_pos[:, self.left_hip_pitch_idx]
    right_hp = self.dof_pos[:, self.right_hip_pitch_idx]
    
    # 位相差が180°に近いほど高報酬
    phase_diff = torch.abs(left_hp - right_hp)
    reward = phase_diff / 0.5  # 0.5 rad差で報酬1.0
    return torch.clamp(reward, 0.0, 1.0)
```

#### `hip_pitch_sync_penalty` (weight: -3.0)

左右hip_pitchの同期にペナルティ：

```python
def _reward_hip_pitch_sync_penalty(self):
    """hip_pitchの同期動作にペナルティ"""
    left_hp = self.dof_pos[:, self.left_hip_pitch_idx]
    right_hp = self.dof_pos[:, self.right_hip_pitch_idx]
    
    # 同方向に動いている場合にペナルティ
    left_vel = self.dof_vel[:, self.left_hip_pitch_idx]
    right_vel = self.dof_vel[:, self.right_hip_pitch_idx]
    
    sync = (left_vel * right_vel > 0).float()  # 同方向=1, 逆方向=0
    return sync
```

#### `no_fly` (weight: -0.5)

両足が同時に宙に浮いている場合にペナルティ：

```python
def _reward_no_fly(self):
    """両足宙浮きペナルティ（Cassieから借用）"""
    contacts = self._get_foot_contacts()
    both_air = (contacts.sum(dim=1) == 0).float()
    return both_air
```

### V9.4 V9の期待効果

| 報酬 | 期待効果 |
|------|---------|
| `hip_pitch_alternation` | 左右脚の位相差を増大 |
| `hip_pitch_sync_penalty` | 両脚同時動作を抑制 |
| `feet_air_time` 強化 | 足の持ち上げを促進 |
| `no_fly` | 両足宙浮きを抑制 |

### V9.5 V9の限界と反省

V9は交互歩行を「ペナルティベース」で誘導するアプローチであり、以下の限界があった：

1. **明確な目標がない**: 「同期しない」ことは分かるが「どう動くべきか」の指針がない
2. **ペナルティの蓄積**: 多くのペナルティが学習を不安定にする可能性
3. **局所解への収束**: ペナルティを避けるため「動かない」解に収束するリスク

これらの反省を踏まえ、V10では**ポジティブな参照軌道**を導入することにした。

### V9.6 V9評価結果

#### V9.6.1 ヘッドレス評価結果（10秒間）

```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 2.948 m
  Y: 0.014 m
  Total: 2.948 m

Average velocity:
  X: 0.293 m/s (target: 0.300)
  Y: 0.013 m/s

Base height:
  Mean: 0.219 m
  Std: 0.0122 m

Orientation (deg):
  Roll:  mean=  1.31, std= 2.48
  Pitch: mean= -4.83, std= 1.93
  Yaw:   drift= -4.57°/10s

DOF velocity RMS: 2.112 rad/s

=== Joint Movement Analysis ===
  hip_pitch   : L=0.477 rad  R=0.529 rad
  hip_roll    : L=0.441 rad  R=0.418 rad
  knee        : L=0.498 rad  R=0.701 rad
  ankle_pitch : L=0.505 rad  R=0.433 rad
  ankle_roll  : L=0.380 rad  R=0.569 rad

Left-Right hip_pitch correlation: -0.516

=== Contact Pattern Analysis ===
Both feet airborne: 100.0%
```

#### V9.6.2 V9の評価考察

1. **hip_pitch相関の改善**: +0.772（V8）→ -0.516（V9）
   - V9の交互歩行ペナルティが効果を発揮し、**交互歩行傾向が強化**された
   - 負の相関値は左右の脚が逆位相で動いていることを示す

2. **Yawドリフトの大幅改善**: -15.7°（V8）→ -4.57°（V9）
   - 70%以上の改善、ほぼ直進に近い

3. **前進距離の向上**: 2.58m（V8）→ 2.95m（V9）
   - **14%向上**、目標速度（0.3 m/s）にほぼ到達

4. **残存課題: 両足宙浮き100%**
   - 滑走歩行は改善されず

#### V9.6.3 目視観察所見

> 「脚がまだ斜めになっている」

- hip_rollの動作（L=0.441, R=0.418 rad）により、脚が横方向に傾いている
- この傾斜が滑走歩行の一因となっている可能性

---

## Phase 4: 生物的な歩行スタイルの追求（V10）

### 4.1 背景と動機

V8では前進移動（10秒で2.58m）を達成したが、以下の問題が残存していた：

1. **並行脚運動**: 左右のhip_pitchが同期して動き（相関+0.772）、交互歩行になっていない
2. **小さいストライド**: hip_pitch range L=0.19, R=0.69 rad
3. **足の持ち上げ不足**: 両足宙浮き100%（滑走歩行）
4. **Yawドリフト**: -15.7°/10s

V9では交互歩行を促すペナルティを追加したが、より根本的なアプローチとして**「かわいく生き物のように歩く」**を目標に設定し、Phase-based Reference Trajectoryを導入することにした。

### 4.2 現状分析: Phase同期機構の有無

#### 調査対象

現在のコード（droid_env.py）にphase同期やreference trajectory追従の要素があるか確認した。

#### 調査結果

```python
# droid_env.py より抜粋
self.gait_phase = torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device)
self.gait_frequency = self.reward_cfg.get("gait_frequency", 1.5)  # Hz

# step() 内
self.gait_phase = (self.gait_phase + self.dt * self.gait_frequency * 2 * math.pi) % (2 * math.pi)
```

**結論**: `gait_phase`変数は存在していたが、**実際の関節角度目標との関連付けがなかった**。位相は更新されているだけで、参照軌道への追従メカニズムは未実装だった。

### 4.3 先行事例の調査

> **注**: 先行研究の詳細調査（Legged Gym, Cassie, Energy Minimization, Walk These Ways, CPG理論等）は独立したドキュメントに記載している。
>
> **参照**: [exp004_reward_design_survey.md](./exp004_reward_design_survey.md)
>
> #### 調査した主要研究
>
> | 研究 | 主要手法 | 本実験への示唆 |
> |------|---------|---------------|
> | Legged Gym (ETH) | `feet_air_time`報酬 | 周期的な足上げの促進 |
> | Cassie | `no_fly`報酬 | 単脚接地の促進 |
> | Energy Minimization | CoT最小化 | 自然な歩容創発 |
> | CaT | 終了条件ベースの制約 | 報酬ハッキング回避 |

### 4.4 設計方針: Phase-based Reference Trajectory

調査結果を踏まえ、**CPG（Central Pattern Generator）インスパイアの正弦波参照軌道**を導入することに決定した。

#### コンセプト

生物の歩行は中枢パターン発生器（CPG）という神経回路によって周期的なリズムが生成されている。これを模倣し、正弦波の参照軌道を関節に与え、その追従を報酬化する。

#### 数学的定義

```
left_hip_pitch_ref  = offset + amplitude × sin(ωt)
right_hip_pitch_ref = offset + amplitude × sin(ωt + π)
```

- `amplitude`: hip_pitch振幅（0.25 rad ≈ 14°）
- `offset`: オフセット（0 rad）
- `ω = 2πf`: 角周波数（f = 1.2 Hz）

左右で180°（π）の位相差を設けることで、自然な交互歩行パターンを誘導する。

#### 期待される効果

1. **リズミカルな動き**: 正弦波による滑らかな周期運動
2. **交互歩行の誘導**: 180°位相差による左右交互動作
3. **接地タイミングの同期**: 位相と接地タイミングの対応
4. **滑らかさの向上**: 正弦波微分（余弦波）に沿った速度変化

### 4.6 V10実装

#### 4.6.1 新規報酬関数（6つ）

droid_env.pyに以下の報酬関数を追加した:

##### 1. `_reward_phase_hip_pitch_tracking`

正弦波参照軌道への追従を報酬化。

```python
def _reward_phase_hip_pitch_tracking(self):
    """位相同期hip_pitch参照軌道追従報酬"""
    amplitude = self.reward_cfg.get("ref_hip_pitch_amplitude", 0.3)
    offset = self.reward_cfg.get("ref_hip_pitch_offset", 0.0)

    # 左右の参照軌道（180°位相差）
    left_ref = offset + amplitude * torch.sin(self.gait_phase)
    right_ref = offset + amplitude * torch.sin(self.gait_phase + math.pi)

    # 現在のhip_pitch角度（default_dof_posからの偏差）
    left_hp = self.dof_pos[:, self.left_hip_pitch_idx] - self.default_dof_pos[self.left_hip_pitch_idx]
    right_hp = self.dof_pos[:, self.right_hip_pitch_idx] - self.default_dof_pos[self.right_hip_pitch_idx]

    # 追従誤差
    left_error = torch.square(left_hp - left_ref)
    right_error = torch.square(right_hp - right_ref)

    # 指数関数的報酬
    tracking_sigma = self.reward_cfg.get("phase_tracking_sigma", 0.1)
    reward = torch.exp(-(left_error + right_error) / tracking_sigma)

    has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
    return reward * has_command
```

##### 2. `_reward_phase_contact_sync`

歩行位相と接地タイミングの同期を報酬化。

```python
def _reward_phase_contact_sync(self):
    """位相-接地同期報酬
    
    phase ∈ [0, π): 左足スイング期待、右足接地期待
    phase ∈ [π, 2π): 右足スイング期待、左足接地期待
    """
    contacts = self._get_foot_contacts()
    left_contact = contacts[:, 0].float()
    right_contact = contacts[:, 1].float()

    phase_sin = torch.sin(self.gait_phase)
    expected_right = (phase_sin > 0).float()
    expected_left = (phase_sin < 0).float()

    match_left = 1.0 - torch.abs(left_contact - expected_left)
    match_right = 1.0 - torch.abs(right_contact - expected_right)

    reward = (match_left + match_right) / 2.0
    has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
    return reward * has_command
```

##### 3. `_reward_phase_velocity_sync`

hip_pitch速度と位相微分（余弦波）の同期を報酬化。

```python
def _reward_phase_velocity_sync(self):
    """位相-hip_pitch速度同期報酬
    
    d/dt[sin(ωt)] = ω*cos(ωt)
    """
    left_vel = self.dof_vel[:, self.left_hip_pitch_idx]
    right_vel = self.dof_vel[:, self.right_hip_pitch_idx]

    expected_left_sign = torch.cos(self.gait_phase)
    expected_right_sign = -torch.cos(self.gait_phase)

    left_sign_match = (left_vel * expected_left_sign > 0).float()
    right_sign_match = (right_vel * expected_right_sign > 0).float()

    reward = (left_sign_match + right_sign_match) / 2.0
    has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
    return reward * has_command
```

##### 4. `_reward_smooth_action`

滑らかなアクション変化を正の報酬として付与。

```python
def _reward_smooth_action(self):
    """滑らかなアクション報酬"""
    action_diff = self.actions - self.last_actions
    action_smoothness = torch.exp(-torch.sum(torch.square(action_diff), dim=1) * 10.0)
    return action_smoothness
```

##### 5. `_reward_periodic_foot_lift`

位相に同期した足の持ち上げを報酬化。

```python
def _reward_periodic_foot_lift(self):
    """周期的な足の持ち上げ報酬"""
    link_pos = self.robot.get_links_pos()
    feet_z = link_pos[:, self.feet_indices, 2]

    phase_sin = torch.sin(self.gait_phase)
    left_swing = (phase_sin > 0).float()
    right_swing = (phase_sin < 0).float()

    target_lift = 0.03
    left_lift_reward = torch.clamp(feet_z[:, 0] - target_lift, min=0.0, max=0.05) * left_swing
    right_lift_reward = torch.clamp(feet_z[:, 1] - target_lift, min=0.0, max=0.05) * right_swing

    reward = left_lift_reward + right_lift_reward
    has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
    return reward * has_command
```

##### 6. `_reward_natural_rhythm`

hip_pitchのゼロクロスを検出し、自然な周期運動を報酬化。

```python
def _reward_natural_rhythm(self):
    """自然なリズム報酬"""
    left_hp = self.dof_pos[:, self.left_hip_pitch_idx] - self.default_dof_pos[self.left_hip_pitch_idx]
    sign_change = (self.last_left_hip_pitch * left_hp < 0).float()
    reward = sign_change * 0.5
    has_command = (torch.norm(self.commands[:, :2], dim=1) > 0.1).float()
    return reward * has_command
```

#### 4.6.2 V10報酬スケール設定

```python
reward_cfg = {
    "tracking_sigma": 0.25,
    "base_height_target": 0.22,
    "feet_air_time_target": 0.25,
    "gait_frequency": 1.2,  # Hz（ゆったりとしたリズム）

    # Phase-based trajectory parameters
    "ref_hip_pitch_amplitude": 0.25,  # rad ≈ 14°
    "ref_hip_pitch_offset": 0.0,
    "phase_tracking_sigma": 0.1,

    "reward_scales": {
        # ========== Phase-based 主報酬（V10新規） ==========
        "phase_hip_pitch_tracking": 3.0,
        "phase_contact_sync": 1.5,
        "phase_velocity_sync": 1.0,
        "smooth_action": 1.0,
        "periodic_foot_lift": 2.0,
        "natural_rhythm": 0.5,

        # ========== 主タスク報酬 ==========
        "tracking_lin_vel": 1.5,
        "tracking_ang_vel": 0.5,
        "alive": 0.1,
        "forward_progress": 0.3,

        # ========== 従来の交互歩行報酬（補助的） ==========
        "alternating_gait": 0.5,      # V9: 1.5 → V10: 0.5
        "foot_swing": 0.3,
        "feet_air_time": 1.0,         # V9: 2.0 → V10: 1.0
        "single_stance": 0.3,
        "no_fly": -0.5,

        # ========== hip_pitch動作報酬（補助的） ==========
        "hip_pitch_alternation": 1.0,  # V9: 4.0 → V10: 1.0
        "hip_pitch_velocity": 0.3,
        "contact_alternation": 0.5,    # V9: 1.5 → V10: 0.5

        # ========== V9からの同期ペナルティ ==========
        "hip_pitch_sync_penalty": -1.0,  # V9: -3.0 → V10: -1.0

        # （以下省略）
    },
}
```

#### 4.6.3 設計パラメータの選択理由

| パラメータ | V10値 | 選択理由 |
|-----------|-------|---------|
| `gait_frequency` | 1.2 Hz | V9の1.5Hzより遅く、ゆったりとした「かわいい」リズム |
| `ref_hip_pitch_amplitude` | 0.25 rad | 約14°、大きすぎない控えめな振幅 |
| `phase_tracking_sigma` | 0.1 | 追従精度と探索のバランス |

#### 4.6.4 従来報酬とのバランス

V10では、Phase-based報酬を主報酬とし、V9以前の交互歩行報酬は補助的な役割に変更:

| 報酬 | V9 | V10 | 変更理由 |
|------|-----|-----|---------|
| `hip_pitch_alternation` | 4.0 | 1.0 | phase報酬に移行 |
| `contact_alternation` | 1.5 | 0.5 | phase_contact_syncに移行 |
| `feet_air_time` | 2.0 | 1.0 | periodic_foot_liftに移行 |
| `hip_pitch_sync_penalty` | -3.0 | -1.0 | phase報酬で自然に抑制 |

### 4.7 実装時のバグと修正

#### 4.7.1 IndexError: too many indices for tensor of dimension 1

**エラー内容**:
```
IndexError: too many indices for tensor of dimension 1
File "droid_env.py", line 870, in _reward_phase_hip_pitch_tracking
    left_hp = self.dof_pos[:, self.left_hip_pitch_idx] - self.default_dof_pos[0, self.left_hip_pitch_idx]
```

**原因**: `self.default_dof_pos`は1次元テンソル（shape: [10]）だが、2次元としてアクセス（`[0, idx]`）していた。

**修正**:
```python
# 修正前
left_hp = self.dof_pos[:, self.left_hip_pitch_idx] - self.default_dof_pos[0, self.left_hip_pitch_idx]
right_hp = self.dof_pos[:, self.right_hip_pitch_idx] - self.default_dof_pos[0, self.right_hip_pitch_idx]

# 修正後
left_hp = self.dof_pos[:, self.left_hip_pitch_idx] - self.default_dof_pos[self.left_hip_pitch_idx]
right_hp = self.dof_pos[:, self.right_hip_pitch_idx] - self.default_dof_pos[self.right_hip_pitch_idx]
```

同様の修正を`_reward_natural_rhythm`にも適用。

### 4.8 V10訓練・評価コマンド

```bash
# 訓練
cd rl_ws
uv run python biped_walking/train/droid_train_v10.py --max_iterations 500

# 評価
uv run python biped_walking/biped_eval.py -e droid-walking-v10
```

### 4.9 V10評価結果

#### 4.9.1 ヘッドレス評価結果（10秒間）

```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 2.811 m
  Y: -0.557 m
  Total: 2.866 m

Average velocity:
  X: 0.285 m/s (target: 0.300)
  Y: -0.023 m/s

Base height:
  Mean: 0.253 m
  Std: 0.0094 m

Orientation (deg):
  Roll:  mean=  1.96, std= 2.40
  Pitch: mean= -0.08, std= 1.43
  Yaw:   drift= -8.01°/10s

DOF velocity RMS: 1.728 rad/s

=== Joint Movement Analysis ===
  hip_pitch   : L=0.531 rad  R=0.475 rad
  hip_roll    : L=0.501 rad  R=0.364 rad
  knee        : L=0.607 rad  R=0.625 rad
  ankle_pitch : L=0.366 rad  R=0.483 rad
  ankle_roll  : L=0.565 rad  R=0.512 rad

Left-Right hip_pitch correlation: 0.355

=== Contact Pattern Analysis ===
Both feet airborne: 100.0%
```

#### 4.9.2 V8 vs V9 vs V10 比較

| 指標 | V8 | V9 | V10 | 備考 |
|------|-----|-----|------|------|
| X移動距離 | 2.58 m | **2.95 m** | 2.81 m | V9が最良 |
| Y移動距離 | 0.06 m | **0.01 m** | -0.56 m | V10で斜行発生 |
| 平均X速度 | 0.26 m/s | **0.29 m/s** | 0.29 m/s | V9/V10で目標に近接 |
| 重心高さ | 0.248 m | 0.219 m | **0.253 m** | V10が最も高い |
| 高さ標準偏差 | 0.015 m | 0.012 m | **0.009 m** | V10が最も安定 |
| Yawドリフト | -15.7° | **-4.6°** | -8.0° | V9が最良 |
| hip_pitch相関 | +0.772 | **-0.516** | +0.355 | V9が最も交互歩行 |
| hip_pitch range (L) | 0.19 rad | 0.48 rad | **0.53 rad** | V10が最大 |
| hip_pitch range (R) | 0.69 rad | 0.53 rad | 0.48 rad | V8が最大 |
| 両足宙浮き率 | 100% | 100% | 100% | 全バージョンで滑走 |

#### 4.9.3 考察

##### V9の成功点

1. **交互歩行の達成**: hip_pitch相関 -0.516（負の値 = 交互歩行）
   - V9のペナルティベース報酬が効果的に左右交互動作を誘導
   - V10（+0.355）よりも交互性が高い

2. **直進性の改善**: Yawドリフト -4.6°（V8の70%以上改善）
   - Y方向ずれも0.01mとほぼ直進

3. **前進距離の向上**: 2.95m（V8比+14%）

##### V10の特徴

1. **重心高さの安定化**: std 0.009m（最小）
   - Phase-based報酬による滑らかな動作

2. **脚の傾斜改善**: 目視で「脚が斜めになり過ぎていたのは良い感じになった」
   - V9では「脚がまだ斜め」と指摘されていた

3. **斜行の発生**: Y方向に-0.56m移動
   - Phase-based報酬への追従が左右で非対称だった可能性

##### V9 vs V10: アプローチの違い

| 観点 | V9 (ペナルティベース) | V10 (参照軌道ベース) |
|------|---------------------|---------------------|
| 交互歩行 | ✅ 強い交互性 (-0.516) | △ 中程度 (+0.355) |
| 直進性 | ✅ 優秀 (Y=0.01m) | ❌ 斜行 (Y=-0.56m) |
| 安定性 | ○ 良好 | ✅ より安定 |
| 脚の傾斜 | △ 斜め傾向あり | ✅ 改善 |
| 動きの滑らかさ | ○ 普通 | △ 細かい動き |

##### 目視観察所見

**V9観察**:
> 「脚がまだ斜めになっている」

**V10観察**:
> 「動きがちょっと細かいね．あと斜めに進んでいる．でも脚が斜めになり過ぎていたのは良い感じになったね．」

##### 問題点と課題

1. **両足宙浮き（全バージョン共通）**: 100%のまま変化なし
   - 滑走歩行が解消されていない
   - 接地判定閾値の調整または`no_fly`ペナルティ強化が必要

2. **V10の斜行**: 
   - hip_roll rangeの左右差（L=0.501, R=0.364 rad）が原因か
   - `symmetry`報酬の強化が必要

##### V11への改善方針

1. **斜行対策**:
   - `symmetry`報酬強化（左右hip_roll差にペナルティ）
   - Yaw角速度ペナルティ強化

2. **細かい動き対策**:
   - `smooth_action`報酬強化
   - `action_rate`ペナルティ強化
   - `gait_frequency`を下げる（1.2Hz → 1.0Hz）

3. **両足宙浮き対策**:
   - `no_fly`ペナルティ強化（-0.5 → -2.0）
   - 接地判定閾値を上げる（0.025m → 0.035m）

4. **V9の長所を活かす**: 
   - V9のペナルティベース報酬は交互歩行誘導に効果的だった
   - V10のPhase-based報酬とV9のペナルティを組み合わせるハイブリッドアプローチを検討

### 4.10 V10設計の理論的背景

#### 4.10.1 CPG（Central Pattern Generator）

生物の歩行は脳からのトップダウン指令だけでなく、脊髄にあるCPGという神経回路によって周期的なリズムが自律的に生成されている。CPGの特徴:

1. **自律的発振**: 外部入力なしでも周期的パターンを生成
2. **位相結合**: 複数のCPGが相互に結合し、協調したパターンを生成
3. **感覚フィードバック**: 接地などの感覚入力でタイミングを調整

V10の設計はこのCPGを簡略化したもので:
- 正弦波発振器 → `gait_phase`の更新
- 位相結合 → 左右180°位相差
- 感覚フィードバック → `phase_contact_sync`報酬

#### 4.10.2 参照軌道追従 vs. 自由探索

強化学習における歩行制御には大きく2つのアプローチがある:

1. **参照軌道追従型**: 事前定義の軌道に追従するよう学習
   - 利点: 学習が安定、望ましい動作を誘導しやすい
   - 欠点: 軌道設計が必要、最適でない可能性

2. **自由探索型**: タスク報酬のみで自由に探索
   - 利点: 最適な動作を発見できる可能性
   - 欠点: 学習が不安定、望ましくない動作が創発する可能性

V10は両者のハイブリッドで:
- Phase-based報酬で大まかなパターンを誘導（参照軌道型）
- 追従の厳密さは`tracking_sigma`で調整し、ある程度の自由度を許容（自由探索型）

---

## 参考資料

### 関連実験・リソース

- [exp002_biped_rl_walking](../exp002_biped_rl_walking/exp002_biped_rl_walking.md) - 前実験
- [bsl_droid_simplified.urdf.xacro](../../../ros2_ws/src/biped_description/urdf/bsl_droid_simplified.urdf.xacro) - ロボットURDF

### 先行研究（V10設計時に調査）

#### 学術論文

| 論文 | 著者/所属 | 年 | 主要手法 | URL |
|------|----------|-----|---------|-----|
| Learning to Walk in Minutes Using Massively Parallel Deep RL | Rudin et al. (ETH Zurich) | CoRL 2021 | feet_air_time報酬、大規模並列学習 | https://arxiv.org/abs/2109.11978 |
| Learning Agile Soccer Skills for a Bipedal Robot | Haarnoja et al. (DeepMind) | Science Robotics 2024 | End-to-end RL、高周波制御 | https://arxiv.org/abs/2304.13653 |
| Minimizing Energy Consumption Leads to Gaits | Siekmann et al. (Berkeley) | CoRL 2021 | エネルギー最小化による歩容創発 | https://arxiv.org/abs/2111.01674 |
| RL for Versatile, Dynamic Bipedal Locomotion | Li et al. (Berkeley) | IJRR 2024 | Dual-history architecture | https://arxiv.org/abs/2401.16889 |
| Learning Low-Frequency Motion Control | Gangapurwala et al. (Oxford) | ICRA 2023 | 低周波制御（8Hz）| https://arxiv.org/abs/2209.14887 |
| SAC for Real-World Humanoid Walking | Xie et al. (Google) | ICRA 2019 | SAC + mimic reward | https://arxiv.org/abs/1812.11103 |

#### オープンソース実装

| リポジトリ | 説明 | URL |
|-----------|------|-----|
| legged_gym | ETH Zurich公式、IsaacGymベース | https://github.com/leggedrobotics/legged_gym |
| unitree_rl_gym | Unitree公式、legged_gymベース | https://github.com/unitreerobotics/unitree_rl_gym |

### 用語集

| 用語 | 説明 |
|------|------|
| CPG (Central Pattern Generator) | 中枢パターン発生器。脊髄にある神経回路で、周期的な運動パターンを自律的に生成する |
| feet_air_time | 足が地面から離れている時間。周期的な足上げを促す報酬として使用 |
| no_fly | 両足が同時に宙に浮いていないことを報酬化。単脚接地を促進 |
| Phase-based reward | 歩行位相に基づく報酬。特定の位相で特定の動作を期待する |
| Domain randomization | シミュレーションパラメータをランダム化し、Sim-to-Real転移を改善する手法 |
| Digitigrade | 趾行性。つま先立ちで歩く形態（鳥類、恐竜等） |

---

## Phase 5: ハイブリッドアプローチ（V11）

### 5.1 V11設計の背景

V9/V10の評価結果と先行研究サーベイを踏まえ、V11では**ハイブリッドアプローチ**を採用する。

#### 5.1.1 V9/V10の長所・短所まとめ

| 観点 | V9 (ペナルティベース) | V10 (参照軌道ベース) |
|------|---------------------|---------------------|
| **交互歩行** | ✅ 優秀 (相関-0.516) | △ 不十分 (+0.355) |
| **直進性** | ✅ 優秀 (Y=0.01m) | ❌ 斜行 (Y=-0.56m) |
| **重心安定性** | ○ 良好 (std=0.012m) | ✅ 最良 (std=0.009m) |
| **脚の傾斜** | ❌ 斜め傾向 | ✅ 改善 |
| **接地** | ❌ 滑走100% | ❌ 滑走100% |

#### 5.1.2 サーベイ結果からの採用項目

| 先行研究 | 採用する知見 | V11での適用 |
|---------|-------------|-------------|
| **Legged Gym (ETH)** | `feet_air_time`報酬、大規模並列学習 | 継続使用、接地判定修正 |
| **Cassie Biped** | `no_fly`報酬で単脚接地促進 | ペナルティ強化 (-0.5 → -3.0) |
| **Low-Frequency Control (Oxford)** | 低周波制御で滑らかな動作 | gait_frequency低下 (1.2→1.0Hz) |

**不採用項目**:
- Energy Minimization: 計算コスト高、現段階では不採用
- SAC + mimic reward: 参照モーション必要、PPOで継続
- Dual-history architecture: ネットワーク変更必要、将来検討

### 5.2 V11設計方針

#### コンセプト: ハイブリッドアプローチ + 接地強制

**V9の長所**（ペナルティによる交互歩行誘導）と**V10の長所**（Phase-based参照軌道による滑らかさ）を組み合わせ、さらに**接地問題を根本解決**する。

#### 5.2.1 根本原因分析と対策

##### 問題1: 両足宙浮き100%（滑走歩行）

**原因**: 接地判定閾値（0.025m）がfoot_height（0.035m）より低く、「常に宙浮き」と判定

**対策**: 
```python
contact_threshold = 0.04  # 0.025m → 0.04m
```

##### 問題2: V10で交互性が低下

**原因**: Phase-based報酬のみでは同期動作への直接抑制が弱い

**対策**: V9の`hip_pitch_sync_penalty`を復活・強化

##### 問題3: V10で斜行発生

**原因**: hip_rollの左右差（L=0.501, R=0.364 rad）

**対策**: `symmetry_hip_roll`報酬を新規追加

### 5.3 V11新規報酬関数

#### 5.3.1 `_reward_symmetry_hip_roll`

左右hip_rollの対称性を報酬化。斜行対策。

```python
def _reward_symmetry_hip_roll(self):
    """左右hip_rollの対称性報酬"""
    left_hr = self.dof_pos[:, self.left_hip_roll_idx]
    right_hr = self.dof_pos[:, self.right_hip_roll_idx]
    # 左右で符号反転が正常（外転・内転が対称）
    diff = torch.abs(left_hr + right_hr)
    return torch.exp(-diff / 0.1)
```

#### 5.3.2 `_reward_ground_contact_bonus`

少なくとも片足が接地している時に正の報酬。

```python
def _reward_ground_contact_bonus(self):
    """接地ボーナス"""
    contacts = self.contact_forces[:, self.feet_indices, 2] > 1.0
    at_least_one = (contacts.sum(dim=1) >= 1).float()
    return at_least_one
```

### 5.4 V11報酬スケール設計

```python
"reward_scales": {
    # ========== Phase-based（V10から継承、調整） ==========
    "phase_hip_pitch_tracking": 2.0,   # V10: 3.0 → 2.0
    "phase_contact_sync": 1.5,
    "phase_velocity_sync": 0.5,
    "periodic_foot_lift": 1.5,
    "natural_rhythm": 0.3,
    
    # ========== ペナルティ（V9から継承、強化） ==========
    "hip_pitch_sync_penalty": -2.0,    # V10: -1.0 → -2.0
    "no_fly": -3.0,                    # V10: -0.5 → -3.0
    
    # ========== V11新規 ==========
    "symmetry_hip_roll": 1.0,          # 斜行対策
    "ground_contact_bonus": 1.5,       # 接地促進
    
    # ========== 主タスク ==========
    "tracking_lin_vel": 1.5,
    "forward_progress": 0.3,
    
    # ========== 動作品質 ==========
    "smooth_action": 1.5,              # V10: 1.0 → 1.5
    "action_rate": -0.05,              # 強化
}
```

### 5.5 V11パラメータ変更

| パラメータ | V10 | V11 | 理由 |
|-----------|-----|-----|------|
| `gait_frequency` | 1.2 Hz | **1.0 Hz** | 細かい動き抑制 |
| `ref_hip_pitch_amplitude` | 0.25 rad | **0.30 rad** | より大きなストライド |
| `contact_threshold` | 0.025m | **0.04m** | 正しい接地判定 |

### 5.6 期待される効果

| 指標 | V9 | V10 | V11期待 |
|------|-----|-----|---------|
| X移動距離 | 2.95m | 2.81m | 2.8m以上 |
| Y移動距離 | 0.01m | -0.56m | **<0.1m** |
| hip_pitch相関 | -0.516 | +0.355 | **<0** (交互) |
| 両足宙浮き率 | 100% | 100% | **<50%** |

### 5.7 V11評価結果

#### 5.7.1 ヘッドレス評価結果（10秒間）

```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 2.539 m
  Y: 0.194 m
  Total: 2.546 m

Average velocity:
  X: 0.258 m/s (target: 0.300)
  Y: -0.005 m/s

Base height:
  Mean: 0.269 m
  Std: 0.0072 m

Orientation (deg):
  Roll:  mean=  0.38, std= 2.22
  Pitch: mean=  1.81, std= 0.87
  Yaw:   drift= +19.10°/10s

DOF velocity RMS: 1.773 rad/s

=== Joint Movement Analysis ===
  hip_pitch   : L=0.340 rad  R=0.542 rad
  hip_roll    : L=0.082 rad  R=0.082 rad （大幅に対称化）
  knee        : L=0.331 rad  R=0.375 rad
  ankle_pitch : L=0.387 rad  R=0.489 rad
  ankle_roll  : L=0.231 rad  R=0.463 rad

Left-Right hip_pitch correlation: +0.852

=== Contact Pattern Analysis ===
Both feet airborne: 100.0%
```

#### 5.7.2 V8 vs V9 vs V10 vs V11 比較

| 指標 | V8 | V9 | V10 | V11 | 備考 |
|------|-----|-----|------|------|------|
| X移動距離 | 2.58m | **2.95m** | 2.81m | 2.54m | V9が最良、V11悪化 |
| Y移動距離 | 0.06m | **0.01m** | -0.56m | 0.19m | V9最良、V11改善 |
| 平均X速度 | 0.26m/s | **0.29m/s** | 0.29m/s | 0.26m/s | V11で低下 |
| 重心高さ | 0.248m | 0.219m | 0.253m | **0.269m** | V11最高 |
| 高さstd | 0.015m | 0.012m | 0.009m | **0.007m** | V11最も安定 |
| Yawドリフト | -15.7° | **-4.6°** | -8.0° | +19.1° | V11で大幅悪化 |
| hip_pitch相関 | +0.772 | **-0.516** | +0.355 | +0.852 | V11で同期歩行に逆戻り |
| hip_roll L-R差 | - | 0.023 | 0.137 | **0.000** | V11で完全対称化 |
| 両足宙浮き | 100% | 100% | 100% | 100% | 変化なし |

#### 5.7.3 目視観察所見

> 「脚がまだ斜め．足の持ち上げとストライドは良化．しかし前足と後足の固定による剣道すり足が再発している．」

##### 改善点

1. **hip_rollの完全対称化**: L=0.082, R=0.082 rad
   - `symmetry_hip_roll`報酬が効果的に作用
   - V10の斜行問題（hip_roll L-R差=0.137）を解消

2. **重心高さの安定化**: mean=0.269m, std=0.007m
   - 全バージョン中最も安定した姿勢維持
   - `ground_contact_bonus`と`no_fly`強化の影響か

3. **足の持ち上げ改善**: 目視で確認
   - 接地閾値修正（0.025→0.04m）の効果

##### 問題点

1. **hip_pitch同期の深刻な悪化**: 相関+0.852（V8の+0.772より悪化）
   - **剣道すり足**の再発：左右の脚が同期して動く
   - Phase-based報酬とペナルティのバランスが逆効果
   - `hip_pitch_sync_penalty`(-2.0)より他の報酬が強い可能性

2. **Yawドリフトの大幅悪化**: +19.1°（V9は-4.6°）
   - hip_roll対称化の副作用で回転トルクのバランスが崩れた
   - 同期歩行による非対称な推進力

3. **前進距離の低下**: 2.54m（V9比-14%）
   - 同期歩行による効率低下

#### 5.7.4 考察：なぜV11は同期歩行に逆戻りしたか

V11の報酬設計を分析すると、以下の問題が浮かび上がる：

##### 報酬の競合

| 報酬 | スケール | 効果 | 競合 |
|------|---------|------|------|
| `phase_hip_pitch_tracking` | 2.0 | 左右180°位相差を誘導 | ○ |
| `hip_pitch_sync_penalty` | -2.0 | 同期を抑制 | ○ |
| `symmetry_hip_roll` | 1.0 | hip_rollを対称化 | **△ 間接的に同期を促進?** |
| `smooth_action` | 1.5 | 滑らかな動作 | **× 同期しやすい?** |
| `ground_contact_bonus` | 1.5 | 接地を促進 | **× 両足同時接地?** |

##### 仮説：`smooth_action`と`ground_contact_bonus`の副作用

- `smooth_action`（1.5）：アクション変化を抑制 → 両脚が同じ動きをする方が「滑らか」
- `ground_contact_bonus`（1.5）：接地を報酬 → 両足接地を維持しようとする
- 結果として、Phase-based報酬の交互歩行誘導を打ち消している可能性

---

## Phase 6: 同期歩行問題の根本解決（V12）

### 6.1 V11の失敗分析

V11は以下の点で期待を裏切った：

| 期待 | 結果 | 原因 |
|------|------|------|
| hip_pitch相関 < 0 | **+0.852** | 報酬競合による同期歩行 |
| Y移動 < 0.1m | 0.19m | 改善したが不十分 |
| Yawドリフト改善 | **+19.1°** | 同期歩行の副作用 |

### 6.2 サーベイ知見の再検討

先行研究サーベイから、交互歩行の強制に有効なアプローチを再検討：

#### 6.2.1 Legged Gym (ETH) の `feet_air_time` 報酬

```python
# 足が接地した瞬間に、空中時間に応じた報酬
rew = (self.feet_air_time - 0.5) * first_contact
```

**ポイント**: 「接地した瞬間」に報酬を与えることで、周期的な足上げ→接地のサイクルを促進。

#### 6.2.2 Cassie Biped の位相ベース接地報酬

```python
# 位相に応じて期待する接地状態
# phase=0~0.5: 左足接地、右足遊脚
# phase=0.5~1: 右足接地、左足遊脚
```

**ポイント**: 位相と接地状態を直接結合。

### 6.3 V12設計方針

#### コンセプト: 「交互接地の直接強制」

V11の失敗から、間接的な報酬（smooth_action等）を減らし、**交互歩行を直接強制する報酬を強化**する。

#### 6.3.1 報酬の取捨選択

| 報酬 | V11 | V12 | 理由 |
|------|-----|-----|------|
| `smooth_action` | 1.5 | **0.3** | 同期を促進する副作用を抑制 |
| `ground_contact_bonus` | 1.5 | **削除** | 両足同時接地を誘導 |
| `hip_pitch_sync_penalty` | -2.0 | **-4.0** | 同期抑制を大幅強化 |
| `phase_hip_pitch_tracking` | 2.0 | **3.0** | 参照軌道追従を強化 |
| `alternating_gait` | 1.0 | **2.0** | 交互歩行を直接報酬 |

#### 6.3.2 新規報酬: `_reward_strict_alternating_contact`

片足接地を厳密に報酬化する新しい報酬関数：

```python
def _reward_strict_alternating_contact(self):
    """厳密な交互接地報酬
    
    - 片足だけ接地: +1.0
    - 両足接地: 0.0
    - 両足宙浮き: -0.5
    """
    contacts = self._get_foot_contacts()
    single_contact = (contacts.sum(dim=1) == 1).float()
    both_air = (contacts.sum(dim=1) == 0).float()
    return single_contact - 0.5 * both_air
```

### 6.4 V12期待効果

| 指標 | V11 | V12期待 |
|------|-----|---------|
| hip_pitch相関 | +0.852 | **< 0** (交互歩行) |
| Yawドリフト | +19.1° | **< 10°** |
| 両足宙浮き率 | 100% | **< 80%** |

### 6.5 V12評価結果

```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 1.796 m
  Y: -0.850 m
  Total: 1.987 m

Average velocity:
  X: 0.204 m/s (target: 0.300)

Base height:
  Mean: 0.271 m
  Std: 0.0073 m

Orientation (deg):
  Roll:  mean=  2.93, std= 2.49
  Pitch: mean=  0.12, std= 1.42
  Yaw:   drift=-47.43°

Left-Right hip_pitch correlation: 0.597
```

### 6.6 V12の失敗分析

**V12は失敗**: 期待した効果は得られなかった。

| 指標 | V11 | V12期待 | V12結果 | 評価 |
|------|-----|---------|---------|------|
| X移動距離 | 2.54m | - | **1.80m** | ❌ 悪化 |
| hip_pitch相関 | +0.852 | < 0 | **+0.597** | ❌ まだ同期 |
| Yawドリフト | +19.1° | < 10° | **-47.4°** | ❌ 大幅悪化 |
| Y移動距離 | 0.19m | - | **-0.85m** | ❌ 斜行悪化 |

**失敗の原因分析**:

1. **報酬関数の過剰複雑化**: V10-V12で28項目以上の報酬を追加
   - 報酬間の競合が発生し、学習が不安定に
   - Phase-based報酬とペナルティベース報酬が打ち消し合い

2. **smooth_actionの副作用**: アクション変化を抑制 → 両脚同期が「滑らか」と評価される

3. **ground_contact_bonus/strict_alternating_contactの効果薄い**: 接地判定が機能していない状況で効果なし

---

## Phase 7: V9回帰による抜本的シンプル化（V13）

### 7.1 V13設計の背景

V10-V12の失敗から、以下の教訓を得た：

1. **V9がdroid-walkingで最良**: hip_pitch相関-0.516で交互歩行を達成した唯一のバージョン
2. **複雑化は悪化を招く**: Phase-based報酬の追加は同期促進という副作用を生んだ
3. **シンプルな報酬設計が有効**: V9のペナルティベース報酬が最も効果的だった

### 7.2 V13設計原則

1. **V9をベースにする**: 交互歩行を達成した唯一の設計
2. **Phase-based報酬を全削除**: 同期促進の原因を排除
3. **報酬項目を最小限に**: 28項目 → 18項目に厳選
4. **contact_threshold修正は維持**: 0.025m → 0.04m（V11の改善点）

### 7.3 V13報酬設計

```python
reward_cfg = {
    "tracking_sigma": 0.25,
    "base_height_target": 0.22,
    "feet_air_time_target": 0.25,
    "gait_frequency": 1.5,
    "contact_threshold": 0.04,

    "reward_scales": {
        # 主タスク報酬（4項目）
        "tracking_lin_vel": 1.5,
        "tracking_ang_vel": 0.5,
        "alive": 0.1,
        "forward_progress": 0.3,

        # 交互歩行報酬（6項目）- V9の核心
        "hip_pitch_alternation": 4.0,      # V9成功要因
        "hip_pitch_sync_penalty": -3.0,    # V9成功要因
        "contact_alternation": 1.5,
        "feet_air_time": 2.0,
        "single_stance": 0.5,
        "no_fly": -2.0,                    # V9の-1.0を強化

        # 姿勢・安定性ペナルティ（4項目）
        "orientation": -2.5,
        "base_height": -10.0,
        "yaw_rate": -1.5,                  # V9の-1.0を強化
        "backward_velocity": -2.0,

        # 膝角度制約
        "dof_pos_limits": -5.0,
        "knee_negative": -3.0,
        "knee_max_angle": -3.0,

        # 最小限の振動抑制
        "action_rate": -0.03,
        "dof_vel": -1e-3,
    },
}
```

### 7.4 削除した報酬（V10-V12で追加したもの）

| 報酬 | 追加バージョン | 削除理由 |
|------|--------------|---------|
| phase_hip_pitch_tracking | V10 | 同期促進の原因 |
| phase_contact_sync | V10 | 同期促進の原因 |
| phase_velocity_sync | V10 | 同期促進の原因 |
| smooth_action | V10 | 同期促進の原因 |
| periodic_foot_lift | V10 | 複雑化 |
| natural_rhythm | V10 | 複雑化 |
| symmetry_hip_roll | V11 | 効果薄い |
| ground_contact_bonus | V11 | 両足同時接地を促進 |
| strict_alternating_contact | V12 | 効果薄い |

### 7.5 V13評価結果

```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 2.578 m
  Y: -1.024 m
  Total: 2.773 m

Average velocity:
  X: 0.280 m/s (target: 0.300)

Base height:
  Mean: 0.222 m
  Std: 0.0169 m

Orientation (deg):
  Roll:  mean=  0.27, std= 6.12
  Pitch: mean= -1.43, std= 4.12
  Yaw:   drift=-45.52°

Left-Right hip_pitch correlation: -0.152

=== Joint Movement Analysis ===
DOF position range (rad):
  hip_pitch   : L=0.484  R=0.366
  knee        : L=0.608  R=0.641
  ankle_pitch : L=0.830  R=0.666

Total DOF range sum: 5.896 rad
```

### 7.6 V9 vs V12 vs V13 比較

| 指標 | V9 | V12 | **V13** | 評価 |
|------|-----|-----|---------|------|
| X移動距離 | **2.95m** | 1.80m | 2.58m | △ V9より劣るがV12から改善 |
| Y移動距離 | **0.01m** | -0.85m | -1.02m | ❌ 斜行悪化 |
| 平均X速度 | **0.29m/s** | 0.20m/s | 0.28m/s | ✓ 改善 |
| Yawドリフト | **-4.6°** | -47.4° | -45.5° | ❌ 改善せず |
| **hip_pitch相関** | **-0.516** | +0.597 | **-0.152** | ✓ **同期→交互へ改善** |
| DOF range sum | 5.34 | 3.78 | 5.90 | ✓ 動作範囲拡大 |

### 7.7 V13の考察

#### 改善点

1. **hip_pitch相関が負になった**: V12の+0.597からV13の**-0.152**へ改善
   - 完全な交互歩行（-0.5以下）ではないが、同期歩行からは脱却
   - V9回帰アプローチの有効性を実証

2. **前進距離の回復**: V12の1.8mからV13の2.58mへ大幅改善
   - シンプルな報酬設計が学習の安定性を向上

3. **DOF range sumの増加**: 3.78 → 5.90 rad
   - より大きな関節可動域で歩行

#### 残存課題

1. **Yawドリフト**: -45.5°（V9は-4.6°）
   - `yaw_rate`を-1.5に強化したが効果不十分
   - 根本的な左右非対称の原因調査が必要

2. **斜行**: Y方向に-1.02m移動
   - hip_rollの左右差が原因と推測
   - `symmetry`報酬の追加を検討

3. **両足宙浮き100%**: 接地判定の問題は未解決
   - 物理的には滑走歩行と推測

### 7.8 V14への改善方針

V13でhip_pitch相関の改善を確認できたため、以下の段階的改善を検討：

1. **Yawドリフト対策**:
   - `yaw_rate`: -1.5 → -5.0 に強化

2. **斜行対策**:
   - `symmetry`: -1.0 に復活（V9から）

3. **脚の傾斜対策**:
   - `base_height_target`: 0.22m → 0.26m（幾何学的計算値に補正）

**重要**: 一度に多くの変更を加えないこと。V10-V12の失敗を繰り返さない。

---

## Phase 8: 高さ補正とYaw/斜行対策（V14）

### 8.1 V14設計の背景

V13の評価で以下の問題が判明した：

| 問題 | V13の値 | 原因分析 |
|------|---------|---------|
| **脚が斜め** | Base height=0.222m | 目標高さ0.22mが幾何学的計算値0.262mより40mm低い |
| **Yawドリフト** | -45.52° | `yaw_rate`ペナルティ-1.5が不十分 |
| **斜行** | Y=-1.024m | 左右hip_rollの非対称 (L=0.445, R=0.483 rad) |

### 8.2 V14での改善内容

#### 改善1: 目標高さの補正（脚の斜め対策）

V8の幾何学的計算に基づき、適切な目標高さに修正：

```python
base_height_target: 0.22m → 0.26m
```

**理論的背景**（V8での計算）:
```
1. 大腿の垂直成分: 0.11 × cos(60°) = 0.055m
2. 下腿の垂直成分: 0.12 × cos(40°) = 0.092m
3. 股関節→足首: 0.055 + 0.092 = 0.147m
4. 足首→地面: foot_height = 0.035m
5. 股関節→地面: 0.147 + 0.035 = 0.182m
6. base_link中心→地面: 0.182 + 0.08 = 0.262m
```

V13の`base_height_target=0.22m`は幾何学的計算値より**40mm低い**。
この低い目標高さが膝の過屈曲と脚の傾斜を引き起こしていた。

#### 改善2: Yawドリフト対策の強化

```python
yaw_rate: -1.5 → -5.0 (3.3倍強化)
```

Yaw角速度へのペナルティを大幅に強化し、回転を抑制。

#### 改善3: 斜行対策（左右対称性報酬の復活）

```python
symmetry: -1.0 (V13で削除していたが復活)
```

V9にあった左右対称性ペナルティを復活：
- hip_roll, knee, ankle_pitchの左右差にペナルティ
- hip_pitchは交互であるべきなので除外

### 8.3 V14報酬設計

```python
reward_cfg = {
    "base_height_target": 0.26,  # V13: 0.22 → V14: 0.26

    "reward_scales": {
        # 主タスク報酬（V13から継続）
        "tracking_lin_vel": 1.5,
        "tracking_ang_vel": 0.5,
        "alive": 0.1,
        "forward_progress": 0.3,

        # 交互歩行報酬（V13から継続）
        "hip_pitch_alternation": 4.0,
        "hip_pitch_sync_penalty": -3.0,
        "contact_alternation": 1.5,
        "feet_air_time": 2.0,
        "single_stance": 0.5,
        "no_fly": -2.0,

        # 姿勢・安定性ペナルティ
        "orientation": -2.5,
        "base_height": -10.0,
        "yaw_rate": -5.0,            # V13: -1.5 → V14: -5.0 (★強化)
        "backward_velocity": -2.0,
        "symmetry": -1.0,            # V13: 削除 → V14: 復活 (★追加)

        # 膝角度制約、振動抑制（V13から継続）
        # ...
    },
}
```

### 8.4 V14評価結果

```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 2.428 m
  Y: -0.403 m
  Total: 2.461 m

Average velocity:
  X: 0.240 m/s (target: 0.300)

Base height:
  Mean: 0.252 m
  Std: 0.0185 m

Orientation (deg):
  Roll:  mean=-11.75, std= 4.81    ← 深刻な左傾斜！
  Pitch: mean= -0.47, std= 2.55
  Yaw:   drift=-27.22°

Left-Right hip_pitch correlation: -0.388

=== Joint Movement Analysis ===
DOF position range (rad):
  hip_pitch   : L=0.368  R=0.395  (小さい、ストライド不足)
  hip_roll    : L=0.378  R=0.559  (右が大きい、左右非対称)

DOF velocity std (rad/s):
  hip_pitch   : L=0.343  R=0.834  (右が2.4倍速い！)
  hip_roll    : L=1.575  R=2.567  (右が1.6倍速い)
```

### 8.5 V9 vs V13 vs V14 比較

| 指標 | V9 | V13 | V14 | 評価 |
|------|-----|-----|-----|------|
| X移動距離 | **2.95m** | 2.58m | 2.43m | ❌ V14で悪化 |
| Y移動距離 | **0.01m** | -1.02m | -0.40m | △ V14で改善 |
| Yawドリフト | **-4.6°** | -45.5° | -27.2° | △ V14で改善 |
| hip_pitch相関 | **-0.516** | -0.152 | -0.388 | △ V14で改善 |
| **Roll mean** | **1.3°** | 0.3° | **-11.8°** | ❌ V14で大幅悪化 |
| Base height | 0.219m | 0.222m | **0.252m** | ✓ 目標に近づいた |

### 8.6 V14の問題分析

#### 深刻な問題: Roll傾斜 (-11.8°)

V14では**左に11.8°傾いた状態**で歩行している。これは：
- V9（1.3°）、V13（0.3°）と比較して大幅に悪化
- 「片足で支えて反対の片足で飛び跳ね」という観察と一致

#### 原因分析

**1. hip_roll rangeの左右非対称**
```
V9:  L=0.441 rad, R=0.418 rad (差: 0.02)
V14: L=0.378 rad, R=0.559 rad (差: 0.18) ← 大きな左右差！
```
右脚のhip_rollが左脚より大きい → 左に傾く

**2. DOF velocity stdの非対称**
```
hip_pitch: L=0.343, R=0.834 rad/s (右が2.4倍速い!)
hip_roll:  L=1.575, R=2.567 rad/s (右が1.6倍速い)
```
右脚がより活発に動き、左脚で支えている状態

**3. V14での変更の副作用**
- `symmetry: -1.0`: 強すぎて学習が不安定になった可能性
- `base_height_target: 0.26m`: 高い姿勢を維持しようとして不安定化
- `yaw_rate: -5.0`: Yaw回転を過度に抑制しようとして、代わりにRoll方向に傾いた

#### ストライド不足

hip_pitch rangeがV9より小さい：
```
V9:  L=0.477 rad, R=0.529 rad
V14: L=0.368 rad, R=0.395 rad (約25%減少)
```

V14では`hip_pitch_velocity`報酬が含まれていないため、ストライドが不足している。

---

## Phase 9: Roll傾斜修正とV9安定性への回帰（V15）

### 9.1 V15設計の背景

V14の評価で判明した問題：

| 問題 | V9 | V14 | 分析 |
|------|-----|-----|------|
| **Roll傾斜** | 1.3° | -11.8° | 深刻、「片足飛び跳ね」の原因 |
| **hip_pitch range** | L=0.477 | L=0.368 | ストライド不足 |
| **DOF velocity非対称** | 均等 | R=2.4倍 | 右脚だけが活発に動く |
| **X移動距離** | 2.95m | 2.43m | Roll傾斜による効率低下 |

### 9.2 V15設計方針: V9への回帰

V9が最も安定していた（Roll=1.3°, Yaw=-4.6°, X=2.95m）ことを踏まえ、
**V9に近い設定に戻しつつ、Roll傾斜への直接対策を追加**する。

### 9.3 V15での改善内容

#### 改善1: base_height_target中間値

```python
V14: 0.26m → V15: 0.24m (V9: 0.22m との中間)
```

高すぎると不安定、低すぎると脚が斜めになる。中間値で安定性向上を狙う。

#### 改善2: yaw_rate緩和

```python
V14: -5.0 → V15: -2.0 (V9: -1.0 より少し強化)
```

過度なYaw抑制がRoll傾斜を誘発した可能性があるため、V9に近い値に緩和。

#### 改善3: symmetry削除

```python
V14: -1.0 → V15: 削除
```

V13で削除して問題なかった。V14で復活させたが逆効果だったため、削除してV13と同様の状態に戻す。

#### 改善4: roll_penalty追加（新規）

```python
roll_penalty: -5.0 (新規)
```

V14でRoll=-11.8°という深刻な傾斜が発生。Roll角を直接ペナルティ化してバランスを改善。

#### 改善5: hip_pitch_velocity復活

```python
V14: なし → V15: 0.8 (V9と同じ)
```

V14ではhip_pitch rangeが小さかった（ストライド不足）。
hip_pitch velocityを報酬化してストライド拡大を促進。

#### 改善6: pitch_penalty復活

```python
V14: なし → V15: -3.0 (V9から復活)
```

姿勢安定化のため、V9で使用していたpitch_penaltyも復活。

### 9.4 V15報酬設計

```python
reward_cfg = {
    "base_height_target": 0.24,  # V14: 0.26 → V15: 0.24 (中間値)
    "contact_threshold": 0.04,

    "reward_scales": {
        # ========== 主タスク報酬（4項目） ==========
        "tracking_lin_vel": 1.5,
        "tracking_ang_vel": 0.5,
        "alive": 0.1,
        "forward_progress": 0.3,

        # ========== 交互歩行報酬（7項目） ==========
        "hip_pitch_alternation": 4.0,
        "hip_pitch_sync_penalty": -3.0,
        "contact_alternation": 1.5,
        "feet_air_time": 2.0,
        "single_stance": 0.5,
        "no_fly": -2.0,
        "hip_pitch_velocity": 0.8,  # V14: なし → V15: 復活 (★改善5)

        # ========== 姿勢・安定性ペナルティ ==========
        "orientation": -2.5,
        "base_height": -10.0,
        "yaw_rate": -2.0,            # V14: -5.0 → V15: -2.0 (★改善2)
        "backward_velocity": -2.0,
        "roll_penalty": -5.0,        # 新規追加 (★改善4)
        "pitch_penalty": -3.0,       # V9から復活 (★改善6)
        # symmetry: 削除 (★改善3)

        # ========== 膝角度制約 ==========
        "dof_pos_limits": -5.0,
        "knee_negative": -3.0,
        "knee_max_angle": -3.0,

        # ========== 振動抑制 ==========
        "action_rate": -0.03,
        "dof_vel": -1e-3,
    },
}
```

### 9.5 V14 vs V15 変更一覧

| パラメータ | V14 | V15 | 変更理由 |
|-----------|-----|-----|---------|
| `base_height_target` | 0.26m | **0.24m** | 高すぎて不安定化、中間値に |
| `yaw_rate` | -5.0 | **-2.0** | 過度な抑制がRoll傾斜誘発 |
| `symmetry` | -1.0 | **削除** | 逆効果だった |
| `roll_penalty` | なし | **-5.0** | Roll傾斜直接対策（新規） |
| `hip_pitch_velocity` | なし | **0.8** | ストライド拡大（V9復活） |
| `pitch_penalty` | なし | **-3.0** | 姿勢安定化（V9復活） |

### 9.6 期待効果

| 指標 | V9 | V14 | V15期待 |
|------|-----|-----|---------|
| X移動距離 | 2.95m | 2.43m | > 2.7m |
| Roll mean | 1.3° | -11.8° | < 5° |
| Yawドリフト | -4.6° | -27.2° | < 15° |
| hip_pitch相関 | -0.516 | -0.388 | < -0.4 |
| Y移動距離 | 0.01m | -0.40m | < 0.2m |

### 9.7 訓練・評価コマンド

```bash
# 訓練
cd rl_ws
uv run python biped_walking/train/droid_train_v15.py --max_iterations 500

# 評価（ヘッドレス）
uv run python biped_walking/biped_eval.py -e droid-walking-v15 --no-viewer --duration 10

# 評価（ビューア付き）
uv run python biped_walking/biped_eval.py -e droid-walking-v15
```

---

## 設計ノウハウまとめ（V9〜V15の教訓）

### 1. 報酬設計の原則

| 原則 | 説明 | 事例 |
|------|------|------|
| **シンプルさを保つ** | 報酬項目は最小限に | V10-V12で28項目→失敗、V13で18項目→改善 |
| **一度に多くを変えない** | 変更は段階的に | V14で3項目変更→予期せぬ副作用 |
| **効果確認後に次へ** | 各変更の効果を検証 | V9の成功要因を維持しながら改善 |

### 2. パラメータ調整の知見

| パラメータ | 低すぎる影響 | 高すぎる影響 | 推奨 |
|-----------|-------------|-------------|------|
| `base_height_target` | 脚が斜め、膝過屈曲 | 不安定、転倒 | 幾何学的計算値の90-100% |
| `yaw_rate` | Yawドリフト | Roll傾斜誘発 | -1.0〜-2.0 |
| `symmetry` | 左右非対称 | 学習不安定 | 使用しないか弱く(-0.5) |

### 3. 問題と対策の対応

| 問題 | 有効な対策 | 無効/逆効果な対策 |
|------|-----------|-----------------|
| 同期歩行 | `hip_pitch_sync_penalty`, `hip_pitch_alternation` | Phase-based報酬（V10-V12） |
| Yawドリフト | `yaw_rate`（適度に）、左右対称な動作 | `yaw_rate`過度強化（Roll誘発） |
| Roll傾斜 | `roll_penalty`直接ペナルティ | `symmetry`強化（逆効果） |
| ストライド不足 | `hip_pitch_velocity` | - |

### 4. V9が安定していた理由

V9はdroid-walkingで最も安定した結果（Roll=1.3°, Yaw=-4.6°, X=2.95m）を達成：

1. **シンプルな報酬設計**: 交互歩行に必要な報酬のみ
2. **適度なペナルティ**: 過度でない姿勢維持ペナルティ
3. **hip_pitch_velocity**: ストライド確保
4. **控えめな高さ目標**: 0.22mで安定

V15はV9の成功要因を維持しつつ、Roll傾斜への直接対策を追加した設計となっている。
### 9.8 V15評価結果

#### 基本指標

| 指標 | V15結果 | V9参考 | V14参考 | 評価 |
|------|---------|--------|---------|------|
| X移動距離 | **3.050m** | 2.95m | 2.43m | ✅ 最高記録！ |
| Y移動距離 | **-0.039m** | 0.01m | -0.40m | ✅ ほぼ直進 |
| Yawドリフト | **-18.11°** | -4.6° | -27.2° | △ 改善 |
| 平均速度 | **0.298m/s** | - | - | ✅ 目標0.3m/sにほぼ到達 |
| 基準高さ | 0.228m | - | - | ✅ 目標0.24mに近い |

#### 交互歩行指標

| 指標 | V15結果 | V9参考 | V14参考 | 評価 |
|------|---------|--------|---------|------|
| hip_pitch相関 | **-0.242** | -0.516 | -0.388 | ❌ V9より弱い |
| Contact Pattern | 両足空中: 100% | - | - | ⚠ 滑走歩行持続 |

#### 姿勢指標

| 指標 | V15結果 | V9参考 | V14参考 | 評価 |
|------|---------|--------|---------|------|
| Roll mean | **-2.9°** | 1.3° | -11.8° | ✅ 大幅改善 |
| Roll std | 5.2° | - | - | △ やや大きい |
| Pitch mean | -3.7° | - | - | △ 前傾気味 |
| Pitch std | 2.9° | - | - | ○ 安定 |

#### 関節動作

```
DOF Velocity (rad/s):
  left_hip_pitch_joint   mean: 1.053, std: 0.894
  right_hip_pitch_joint  mean: 0.929, std: 0.831
  left_knee_pitch_joint  mean: 0.766, std: 0.646
  right_knee_pitch_joint mean: 0.775, std: 0.696

hip_pitch range: L=0.469 rad, R=0.378 rad
knee_pitch range: L=0.256 rad, R=0.276 rad
```

### 9.9 V15分析

#### 成功点

1. **前進性能が最高記録**: X=3.05m、目標速度0.3m/sにほぼ到達
2. **直進性が大幅改善**: Y=-0.04m、ほぼ真っ直ぐ前進
3. **Roll傾斜が修正**: -11.8°（V14）→-2.9°（V15）、`roll_penalty`が効果的
4. **Yawドリフト改善**: -27.2°（V14）→-18.11°（V15）

#### 問題点

1. **交互歩行の質がV9より劣る**
   - hip_pitch相関: -0.242（V15） vs -0.516（V9）
   - 左右のhip_pitchが逆位相で動いているが、振幅が弱い

2. **「剣道すり足」パターンの持続**
   - ユーザー報告: 「前足と後足の固定による剣道すり足が残っています」
   - Contact Pattern: 100%が両足空中（滑走歩行）
   - 視覚的には片脚が常に前、もう片脚が常に後ろにいる状態

3. **hip_pitch範囲の左右差**
   - L=0.469 rad, R=0.378 rad（左が約24%大きい）
   - 右脚の振りが小さく、左脚が主導する歩行

### 9.10 V9 vs V13 vs V14 vs V15 総合比較

| 指標 | V9 | V13 | V14 | V15 | 最良 |
|------|-----|------|------|------|------|
| X移動距離 | 2.95m | 2.58m | 2.43m | **3.05m** | V15 |
| Y移動距離 | **0.01m** | -1.02m | -0.40m | -0.04m | V9 |
| Yawドリフト | **-4.6°** | -45.5° | -27.2° | -18.1° | V9 |
| hip_pitch相関 | **-0.516** | -0.152 | -0.388 | -0.242 | V9 |
| Roll mean | **1.3°** | 0.3° | -11.8° | -2.9° | V9 |

### 9.11 「剣道すり足」問題の分析

V15は数値指標では優秀だが、視覚的には「剣道すり足」が解消されていない。

#### 問題の本質

「剣道すり足」とは：
- 片脚が常に**前脚**として機能
- もう片脚が常に**後脚**として機能
- 脚の役割（前/後）が入れ替わらない

これは`hip_pitch相関`が弱い（-0.242）ことと一致：
- 理想の交互歩行: 相関 ≈ -1.0（完全逆位相）
- V9の交互歩行: 相関 = -0.516（中程度の逆位相）
- V15の状態: 相関 = -0.242（弱い逆位相）

#### Contact Detection問題

Contact Patternが100%「両足空中」を示しているのは、接触検出の問題がある可能性：
- 物理シミュレーションでは足が地面に触れているはず
- `contact_threshold = 0.04`が適切でない可能性
- あるいは「滑走歩行」（両足が常に軽く接地）している可能性

### 9.12 V16への改善方針

V15の課題を踏まえ、以下の方針でV16を設計する：

#### 方針1: 交互歩行報酬の強化（V9回帰）

V9が最も高い`hip_pitch相関`を達成した設計に近づける：
- `hip_pitch_alternation`のスケールをV9と同等に
- `hip_pitch_sync_penalty`をV9レベルに

#### 方針2: 脚役割スイッチング報酬

「前脚/後脚の固定」を防ぐ新しい報酬の検討：
- hip_pitchの符号変化を報酬化（＋から−、−から＋への遷移）
- 一定時間ごとの脚役割スイッチを強制

#### 方針3: Contact検出の改善

- `contact_threshold`の調整（0.04 → 0.02 等）
- 接触力ベースの検出に変更検討

#### 優先度

1. まずV9の交互歩行成功要因を再分析
2. V9とV15の報酬設計を詳細比較
3. 脚役割スイッチング報酬の設計

---

## Phase 10: 脚役割スイッチングによる「剣道すり足」解消（V16）

### 10.1 V16設計の背景

V15の評価結果から、以下の問題が明確になった：

| 問題 | V15の状態 | V9参考 | 分析 |
|------|----------|--------|------|
| **hip_pitch相関** | -0.242 | -0.516 | 交互歩行が弱い |
| **剣道すり足** | 持続 | なし | 前脚/後脚の役割が固定 |
| **hip_pitch範囲** | L=0.47, R=0.38 | 均等 | 左脚主導で右脚が弱い |

#### 「剣道すり足」の本質

「剣道すり足」とは、歩行中に脚の役割（前脚/後脚）が入れ替わらない状態を指す：

```
理想の歩行:
  t=0: 左脚=前, 右脚=後
  t=T: 左脚=後, 右脚=前  ← 役割が入れ替わる
  t=2T: 左脚=前, 右脚=後
  ...

剣道すり足:
  t=0: 左脚=前, 右脚=後
  t=T: 左脚=前, 右脚=後  ← 役割が固定
  t=2T: 左脚=前, 右脚=後
  ...
```

hip_pitchの観点では、「役割の入れ替わり」は「hip_pitchの符号変化」として検出できる：
- 前脚: hip_pitch < default（脚を前に出している）
- 後脚: hip_pitch > default（脚を後ろに引いている）
- 役割入れ替わり = hip_pitchがdefaultを跨いで符号変化

### 10.2 V16設計方針

V15の良い点（前進性能、直進性、Roll修正）を維持しつつ、「剣道すり足」を解消するため、以下の設計変更を行う。

#### 設計原則

1. **V9の交互歩行成功要因を復活**: V9が-0.516を達成した報酬構成を参考
2. **脚役割スイッチングの直接報酬化**: hip_pitchの符号変化を検出・報酬
3. **V15で有効だった報酬は維持**: roll_penalty, yaw_rate等

### 10.3 V16での改善内容

#### 改善1: hip_pitch_sign_change報酬（新規）

```python
hip_pitch_sign_change: 3.0 (新規)
```

**目的**: 脚役割の入れ替わりを直接報酬化

**実装ロジック**:
```python
def _reward_hip_pitch_sign_change(self):
    """hip_pitchの符号変化報酬（V16新規）

    hip_pitchが正から負、または負から正に変化するタイミングを報酬化。
    これにより「常に前脚」「常に後脚」というパターンを崩し、
    脚の役割（前/後）の入れ替わりを促進する。
    """
    # 現在のhip_pitch（デフォルトからの相対値）
    left_hp = dof_pos[left_hip_pitch_idx] - default[left_hip_pitch_idx]
    right_hp = dof_pos[right_hip_pitch_idx] - default[right_hip_pitch_idx]

    # 前ステップとの積が負 = 符号変化
    left_sign_change = (last_left_hp * left_hp < 0).float()
    right_sign_change = (last_right_hp * right_hp < 0).float()

    # 両脚の符号変化を合計
    reward = left_sign_change + right_sign_change
    return reward * has_command
```

**期待効果**: 
- 歩行周期ごとに脚役割が入れ替わることを促進
- 「常に前脚」「常に後脚」のパターンを崩す

#### 改善2: hip_pitch_alternation強化

```python
V15: 4.0 → V16: 6.0
```

V9では`hip_pitch_alternation`が交互歩行の主要因だった。V15より強化してV9の成功を再現。

#### 改善3: hip_pitch_sync_penalty強化

```python
V15: -3.0 → V16: -5.0
```

同期歩行（両脚が同方向に動く）へのペナルティを強化。V9では-3.0だったが、さらに強化。

#### 改善4: V9から交互歩行関連報酬を復活

| 報酬 | V15 | V16 | V9参考 | 目的 |
|------|-----|-----|--------|------|
| `foot_clearance` | なし | **2.0** | 2.0 | 足の持ち上げ促進 |
| `hip_pitch_range` | なし | **1.0** | 1.0 | ストライド拡大 |
| `alternating_gait` | なし | **1.5** | 1.5 | 交互歩行基本報酬 |
| `foot_swing` | なし | **0.8** | 0.8 | 足の振り動作 |

#### 改善5: symmetry復活（弱く）

```python
V15: なし → V16: -0.3 (V9: -0.5, V14: -1.0)
```

V14で-1.0にしたら逆効果だったため、V9より弱い-0.3で試す。左右対称性を緩やかに促進。

### 10.4 V16報酬設計

```python
reward_cfg = {
    "base_height_target": 0.24,  # V15から継承
    "contact_threshold": 0.04,

    "reward_scales": {
        # ========== 主タスク報酬（4項目） ==========
        "tracking_lin_vel": 1.5,
        "tracking_ang_vel": 0.5,
        "alive": 0.1,
        "forward_progress": 0.3,

        # ========== 交互歩行報酬（V9復活 + V16強化） ==========
        "hip_pitch_alternation": 6.0,   # V15: 4.0 → V16: 6.0 (★強化)
        "hip_pitch_sync_penalty": -5.0, # V15: -3.0 → V16: -5.0 (★強化)
        "contact_alternation": 1.5,
        "feet_air_time": 2.0,
        "single_stance": 0.5,
        "no_fly": -2.0,
        "hip_pitch_velocity": 0.8,

        # ★ V16新規: 脚役割スイッチング報酬
        "hip_pitch_sign_change": 3.0,   # 符号変化を報酬化

        # ★ V16: V9から復活
        "foot_clearance": 2.0,
        "hip_pitch_range": 1.0,
        "alternating_gait": 1.5,
        "foot_swing": 0.8,

        # ========== 姿勢・安定性ペナルティ ==========
        "orientation": -2.5,
        "base_height": -10.0,
        "yaw_rate": -2.0,               # V15から継承
        "backward_velocity": -2.0,
        "roll_penalty": -5.0,           # V15から継承
        "pitch_penalty": -3.0,
        "symmetry": -0.3,               # V16復活（弱く）

        # ========== 膝角度制約 ==========
        "dof_pos_limits": -5.0,
        "knee_negative": -3.0,
        "knee_max_angle": -3.0,

        # ========== 振動抑制 ==========
        "action_rate": -0.03,
        "dof_vel": -1e-3,
    },
}
```

### 10.5 V15 vs V16 変更一覧

| パラメータ | V15 | V16 | 変更理由 |
|-----------|-----|-----|---------|
| `hip_pitch_alternation` | 4.0 | **6.0** | 交互歩行強化 |
| `hip_pitch_sync_penalty` | -3.0 | **-5.0** | 同期ペナルティ強化 |
| `hip_pitch_sign_change` | なし | **3.0** | 脚役割スイッチング（新規） |
| `foot_clearance` | なし | **2.0** | V9から復活 |
| `hip_pitch_range` | なし | **1.0** | V9から復活 |
| `alternating_gait` | なし | **1.5** | V9から復活 |
| `foot_swing` | なし | **0.8** | V9から復活 |
| `symmetry` | なし | **-0.3** | 弱く復活 |

### 10.6 報酬項目数の比較

| バージョン | 報酬項目数 | 結果 |
|-----------|-----------|------|
| V9 | 26項目 | hip_pitch相関 -0.516（最良） |
| V13 | 18項目 | hip_pitch相関 -0.152（不十分） |
| V15 | 18項目 | hip_pitch相関 -0.242（不十分） |
| V16 | **23項目** | 目標: hip_pitch相関 < -0.4 |

V16はV15より5項目増加したが、V9の26項目より少なく抑えている。
増加した項目はすべてV9で有効だった交互歩行関連報酬。

### 10.7 期待効果

| 指標 | V9 | V15 | V16期待 |
|------|-----|-----|---------|
| X移動距離 | 2.95m | 3.05m | > 2.8m |
| Y移動距離 | 0.01m | -0.04m | < 0.1m |
| Yawドリフト | -4.6° | -18.1° | < 15° |
| hip_pitch相関 | **-0.516** | -0.242 | **< -0.4** |
| Roll mean | 1.3° | -2.9° | < 5° |

**主目標**: hip_pitch相関を-0.4以下（V9の-0.516に近づける）に改善し、「剣道すり足」を解消。

### 10.8 訓練・評価コマンド

```bash
# 訓練
cd rl_ws
uv run python biped_walking/train/droid_train_v16.py --max_iterations 500

# 評価（ヘッドレス）
uv run python biped_walking/biped_eval.py -e droid-walking-v16 --no-viewer --duration 10

# 評価（ビューア付き）
uv run python biped_walking/biped_eval.py -e droid-walking-v16
```

### 10.9 設計上の考慮事項

#### なぜhip_pitch_sign_changeが有効と考えるか

1. **直接的なアプローチ**: 「脚役割の入れ替わり」という目標を直接報酬化
2. **時間的な報酬**: 瞬間的な状態ではなく、状態の「変化」を検出
3. **V9との差異分析**: V9は`hip_pitch_range`と`foot_clearance`があったが、V15にはなかった

#### リスク要因

1. **報酬項目の増加**: 23項目に増加したことで学習が不安定になる可能性
2. **過度な交互動作**: `hip_pitch_alternation`を6.0に強化したことで、不自然な動きになる可能性
3. **symmetryの復活**: V14で逆効果だったが、V16では-0.3と弱くしている

#### 失敗時の次のステップ

1. `hip_pitch_sign_change`のスケール調整（3.0 → 5.0 or 2.0）
2. V9の報酬設計に完全回帰（V16の変更を段階的に追加）
3. Contact検出の改善（`contact_threshold`調整）
### 10.10 V16評価結果

#### 基本指標

| 指標 | V16結果 | V15参考 | V9参考 | 評価 |
|------|---------|---------|--------|------|
| X移動距離 | **3.234m** | 3.05m | 2.95m | ✅ 最高記録更新 |
| Y移動距離 | **-0.346m** | -0.04m | 0.01m | ❌ 大幅悪化（斜行） |
| Yawドリフト | **-19.15°** | -18.1° | -4.6° | △ 横ばい |
| 平均速度 | **0.321m/s** | 0.298m/s | - | ✅ 目標超過 |
| 基準高さ | 0.262m | 0.228m | - | △ 高め |

#### 交互歩行指標（最重要）

| 指標 | V16結果 | V15参考 | V9参考 | 評価 |
|------|---------|---------|--------|------|
| **hip_pitch相関** | **+0.697** | -0.242 | -0.516 | ❌❌ **致命的悪化！同期歩行** |
| Contact Pattern | 両足空中: 100% | 両足空中: 100% | - | ⚠ 変化なし |

#### 姿勢指標

| 指標 | V16結果 | V15参考 | V9参考 | 評価 |
|------|---------|---------|--------|------|
| Roll mean | **-1.55°** | -2.9° | 1.3° | ✅ 良好 |
| Roll std | 5.17° | 5.2° | - | △ 横ばい |
| Pitch mean | -3.40° | -3.7° | - | △ 前傾気味 |
| Pitch std | 1.47° | 2.9° | - | ✅ 安定 |

#### 関節動作分析

```
DOF position range (rad):
  hip_pitch   : L=0.191  R=0.434  ← 右脚が2.3倍大きい（左右非対称）
  hip_roll    : L=0.258  R=0.297
  knee        : L=0.492  R=0.546

DOF velocity std (rad/s):
  hip_pitch   : L=1.114  R=2.592  ← 右脚が2.3倍速い（びっこ）
  hip_roll    : L=1.609  R=2.030
  knee        : L=3.664  R=3.648
```

### 10.11 V16分析：なぜ完全に逆効果になったか

#### 致命的な問題：hip_pitch相関が+0.697（同期歩行）

V16の設計意図とは**完全に逆**の結果になった：

| バージョン | hip_pitch相関 | 状態 |
|-----------|---------------|------|
| V9 | **-0.516** | 交互歩行（最良） |
| V15 | -0.242 | 弱い交互歩行 |
| V16 | **+0.697** | **同期歩行（最悪）** |

**解釈**:
- 負の相関 = 交互歩行（左脚が前のとき右脚が後ろ）
- 正の相関 = 同期歩行（両脚が同じ方向に動く）
- +0.697は「両脚がほぼ同時に同方向に動く」状態

#### 「びっこを引く」問題

ユーザー報告の「びっこを引いている」状態はデータでも確認できる：

```
hip_pitch velocity std:
  Left:  1.114 rad/s
  Right: 2.592 rad/s  ← 右脚が2.3倍速く動く

hip_pitch range:
  Left:  0.191 rad
  Right: 0.434 rad    ← 右脚が2.3倍大きく振れる
```

**片脚（右）だけが大きく速く動き、反対脚（左）は小さくゆっくり動く**
→ 重心が片側に残り「びっこ」状態

#### 斜行の発生

Y移動距離が-0.04m（V15）→ -0.346m（V16）に大幅悪化：
- 左右非対称な動きにより斜めに歩行
- 左脚主導から右脚主導に変わったが、バランスが取れていない

### 10.12 V16失敗の原因分析

#### 仮説1: hip_pitch_sign_change報酬の逆効果

`hip_pitch_sign_change`は「符号変化を報酬化」する設計だったが：

```python
# 符号変化検出
left_sign_change = (last_left_hp * left_hp < 0).float()
right_sign_change = (last_right_hp * right_hp < 0).float()
reward = left_sign_change + right_sign_change
```

**問題点**:
- **同時符号変化が最も報酬が高い**: 両脚が同時に符号変化すると報酬が2倍
- これが同期歩行を誘発した可能性

**本来の意図**: 交互に符号変化することで脚役割を入れ替える
**実際の学習結果**: 同時に符号変化することで報酬最大化

#### 仮説2: 交互歩行報酬の過剰強化

```python
hip_pitch_alternation: 4.0 → 6.0  # 50%増加
hip_pitch_sync_penalty: -3.0 → -5.0  # 67%増加
```

これらを強化したにもかかわらず同期歩行になったのは、
`hip_pitch_sign_change`の報酬が他の交互歩行報酬を上回った可能性。

#### 仮説3: 報酬項目の競合

V16では23項目の報酬があり、以下の競合が発生した可能性：

| 報酬 | 目的 | 実際の効果 |
|------|------|-----------|
| `hip_pitch_sign_change` | 脚役割スイッチ | **同期歩行を誘発** |
| `hip_pitch_alternation` | 交互動作 | 効果打ち消し |
| `hip_pitch_sync_penalty` | 同期ペナルティ | 効果不足 |

### 10.13 V9 vs V15 vs V16 総合比較

| 指標 | V9 | V15 | V16 | 最良 | V16評価 |
|------|-----|------|------|------|---------|
| X移動距離 | 2.95m | 3.05m | **3.23m** | V16 | ✅ |
| Y移動距離 | **0.01m** | -0.04m | -0.35m | V9 | ❌ |
| Yawドリフト | **-4.6°** | -18.1° | -19.2° | V9 | ❌ |
| **hip_pitch相関** | **-0.516** | -0.242 | +0.697 | V9 | ❌❌ |
| Roll mean | **1.3°** | -2.9° | -1.6° | V9 | ○ |
| hip_pitch L/R差 | 均等 | L主導 | **R主導** | V9 | ❌ |

### 10.14 V17への改善方針

V16の完全な失敗を踏まえ、根本的な方針転換が必要：

#### 方針1: hip_pitch_sign_change報酬の削除または修正

**オプションA: 削除**
- V16で追加した`hip_pitch_sign_change`を削除
- V15の設定に戻す

**オプションB: 修正（交互符号変化を報酬化）**
```python
# 左右が「交互に」符号変化したときのみ報酬
# 同時変化は報酬なし（または小さいペナルティ）
alternating_change = left_sign_change ^ right_sign_change  # XOR
reward = alternating_change.float()
```

#### 方針2: V9への完全回帰

V9が唯一hip_pitch相関-0.516を達成したバージョン。
V9の報酬設計に完全に戻し、そこから最小限の変更で改善を試みる。

#### 方針3: 報酬設計の簡素化

V16では23項目の報酬があり複雑すぎた。
報酬項目を減らしてシンプルな設計に回帰：

| 削除候補 | 理由 |
|---------|------|
| `hip_pitch_sign_change` | 同期歩行を誘発 |
| `alternating_gait` | `hip_pitch_alternation`と重複 |
| `foot_swing` | 効果不明 |
| `symmetry` | 過去に逆効果 |

#### 推奨: V17設計案

```python
# V9ベース + 最小限の変更
reward_scales = {
    # V9の核心報酬のみ
    "tracking_lin_vel": 1.5,
    "hip_pitch_alternation": 4.0,  # V9と同じ
    "hip_pitch_sync_penalty": -3.0,  # V9と同じ
    "feet_air_time": 2.0,
    "orientation": -2.5,
    "base_height": -10.0,
    "roll_penalty": -5.0,  # V15で有効だった
    # ... V9の他の報酬
}
```

---

## Phase 11: V16失敗の詳細分析とV17設計（独自考察）

### 11.1 V16失敗の根本原因：報酬関数設計の論理的欠陥

V16でhip_pitch相関が-0.242（V15）から+0.697（V16）へ劇的に悪化した。この失敗は単なるパラメータ調整の問題ではなく、**報酬関数設計の論理的欠陥**に起因する。

#### 11.1.1 `hip_pitch_sign_change`報酬の致命的欠陥

V16で追加した`hip_pitch_sign_change`報酬の実装を詳細に分析する：

```python
# V16の実装
left_sign_change = (last_left_hp * left_hp < 0).float()  # 左脚の符号変化: 0 or 1
right_sign_change = (last_right_hp * right_hp < 0).float()  # 右脚の符号変化: 0 or 1
reward = left_sign_change + right_sign_change  # 合計: 0, 1, or 2
```

**問題点の数学的分析**:

| 状態 | left_sign_change | right_sign_change | reward | 意味 |
|------|------------------|-------------------|--------|------|
| 両脚静止 | 0 | 0 | **0** | 報酬なし |
| 左脚のみ変化 | 1 | 0 | **1** | 小さい報酬 |
| 右脚のみ変化 | 0 | 1 | **1** | 小さい報酬 |
| **両脚同時変化** | 1 | 1 | **2** | **最大報酬** |

**致命的な設計ミス**:
- **同時符号変化が最大報酬**となる設計
- 理想的な交互歩行（片脚ずつ符号変化）よりも、同期歩行（両脚同時変化）の方が報酬が高い
- 結果として、ポリシーは**同期歩行に最適化**された

#### 11.1.2 設計意図と実際の効果の乖離

| 項目 | 設計意図 | 実際の効果 |
|------|---------|-----------|
| **報酬最大化条件** | 脚役割が交互に入れ替わる | 両脚が同時に符号変化 |
| **促進される動作** | 交互歩行 | 同期歩行（ホッピング） |
| **hip_pitch相関への影響** | 負（交互） | **正（同期）** |

#### 11.1.3 強化学習の最適化観点からの分析

PPOアルゴリズムは期待報酬を最大化するようにポリシーを更新する。V16の報酬設計では：

1. **同時符号変化**: reward = 2（最大）
2. **交互符号変化**: reward = 1（同時変化の半分）
3. **変化なし**: reward = 0

PPOは当然ながら**reward = 2となる同時変化を学習**した。これは設計者の意図とは逆だが、報酬関数の数学的帰結として正しい。

### 11.2 独自考察：なぜV9が成功しV16が失敗したか

#### 11.2.1 V9の成功要因の再分析

V9がhip_pitch相関-0.516を達成できた理由：

| 報酬 | スケール | 効果 |
|------|---------|------|
| `hip_pitch_alternation` | 4.0 | 左右hip_pitchの**瞬間的な位相差**を報酬化 |
| `hip_pitch_sync_penalty` | -3.0 | 同方向移動に**直接ペナルティ** |
| `contact_alternation` | 1.5 | 接地タイミングの交互性 |

**V9の設計哲学**: 「状態の差異」を報酬化（左右の位相差、接地状態の差）

#### 11.2.2 V16の失敗要因

V16は「状態の変化」を報酬化しようとしたが：

| 報酬 | 問題点 |
|------|--------|
| `hip_pitch_sign_change` | 変化の「同時性」に報酬を与えてしまった |
| V9報酬の復活 | 競合する報酬信号が学習を不安定化 |

**V16の設計哲学の失敗**: 「時間的変化」の報酬化は、「同時変化」が最適解になりやすい

### 11.3 V17設計方針

V16の失敗分析から、V17では以下の方針を採用する。

#### 11.3.1 基本方針: V9完全回帰 + 最小限の改善

1. **V9の報酬設計に完全回帰**: hip_pitch相関-0.516を達成した唯一の設計
2. **V16の`hip_pitch_sign_change`は採用しない**: 論理的欠陥が明確
3. **V15で有効だった`roll_penalty`のみ追加**: Roll傾斜対策として実績あり

#### 11.3.2 代替案: XOR方式の符号変化報酬（オプション）

「同時変化」ではなく「交互変化」を報酬化する修正版：

```python
def _reward_hip_pitch_alternating_sign_change(self):
    """交互符号変化報酬（V17オプション）

    XOR演算により、片脚のみ符号変化したときに報酬を与える。
    両脚同時変化は報酬なし。
    """
    left_sign_change = (last_left_hp * left_hp < 0).float()
    right_sign_change = (last_right_hp * right_hp < 0).float()

    # XOR: 片方だけ変化したときに1、両方同時変化は0
    alternating_change = (left_sign_change + right_sign_change == 1).float()

    # オプション: 両脚同時変化にペナルティ
    both_change = (left_sign_change * right_sign_change).float()

    reward = alternating_change - 0.5 * both_change
    return reward * has_command
```

**XOR方式の数学的特性**:

| 状態 | left | right | alternating_change | 意味 |
|------|------|-------|-------------------|------|
| 両脚静止 | 0 | 0 | 0 | 報酬なし |
| 左脚のみ変化 | 1 | 0 | **1** | **最大報酬** |
| 右脚のみ変化 | 0 | 1 | **1** | **最大報酬** |
| 両脚同時変化 | 1 | 1 | **0** | 報酬なし |

この設計なら交互変化が最適となる。

#### 11.3.3 V17設計の選択

**選択案A: 保守的アプローチ（推奨）**
- V9完全回帰 + `roll_penalty`のみ追加
- 新しい報酬関数は追加しない
- 最も安全で効果が予測可能

**選択案B: 改良アプローチ**
- V9回帰 + `roll_penalty` + XOR方式符号変化報酬
- V16の設計意図を正しく実装
- ただし新しい報酬関数のリスクあり

**V17では選択案Aを採用**: V16の失敗から、新しい報酬関数の追加はリスクが高い。まずV9の成功を再現し、そこから段階的に改善する。

### 11.4 V17報酬設計

```python
reward_cfg = {
    "tracking_sigma": 0.25,
    "base_height_target": 0.22,  # V9と同じ
    "feet_air_time_target": 0.25,
    "gait_frequency": 1.5,
    "contact_threshold": 0.04,  # V11以降の改善は維持

    "reward_scales": {
        # ========== 主タスク報酬（4項目）==========
        "tracking_lin_vel": 1.5,
        "tracking_ang_vel": 0.5,
        "alive": 0.1,
        "forward_progress": 0.3,

        # ========== 交互歩行報酬（V9から完全継承）==========
        "hip_pitch_alternation": 4.0,      # V9成功要因
        "hip_pitch_sync_penalty": -3.0,    # V9成功要因
        "contact_alternation": 1.5,
        "feet_air_time": 2.0,
        "single_stance": 0.5,
        "no_fly": -2.0,
        "hip_pitch_velocity": 0.8,         # V9で使用
        "foot_clearance": 2.0,             # V9で使用
        "alternating_gait": 1.5,           # V9で使用
        "foot_swing": 0.8,                 # V9で使用

        # ========== 姿勢・安定性ペナルティ ==========
        "orientation": -2.5,
        "base_height": -10.0,
        "yaw_rate": -1.0,                  # V9と同じ（V15/V16より弱く）
        "backward_velocity": -2.0,
        "roll_penalty": -5.0,              # V15で有効（V17追加）
        "pitch_penalty": -3.0,             # V9で使用
        "symmetry": -0.5,                  # V9と同じ

        # ========== 膝角度制約 ==========
        "dof_pos_limits": -5.0,
        "knee_negative": -3.0,
        "knee_max_angle": -3.0,

        # ========== 振動抑制 ==========
        "action_rate": -0.03,
        "dof_vel": -1e-3,
    },
}
```

### 11.5 V16 vs V17 変更一覧

| パラメータ | V16 | V17 | 変更理由 |
|-----------|-----|-----|---------|
| `base_height_target` | 0.24m | **0.22m** | V9に回帰 |
| `hip_pitch_alternation` | 6.0 | **4.0** | V9に回帰 |
| `hip_pitch_sync_penalty` | -5.0 | **-3.0** | V9に回帰 |
| `hip_pitch_sign_change` | 3.0 | **削除** | 論理的欠陥があるため |
| `hip_pitch_range` | 1.0 | **削除** | 不要（V9でも効果薄い） |
| `yaw_rate` | -2.0 | **-1.0** | V9に回帰 |
| `symmetry` | -0.3 | **-0.5** | V9と同じ |
| `roll_penalty` | -5.0 | **-5.0** | V15で有効、継続 |

### 11.6 期待効果

| 指標 | V9 | V15 | V16 | V17期待 |
|------|-----|------|------|---------|
| X移動距離 | 2.95m | 3.05m | 3.23m | > 2.8m |
| Y移動距離 | 0.01m | -0.04m | -0.35m | < 0.1m |
| Yawドリフト | -4.6° | -18.1° | -19.2° | < 10° |
| **hip_pitch相関** | **-0.516** | -0.242 | +0.697 | **< -0.4** |
| Roll mean | 1.3° | -2.9° | -1.6° | < 3° |

**主目標**: V9のhip_pitch相関-0.516を再現し、「剣道すり足」を解消する。

### 11.7 V17の理論的根拠

#### 11.7.1 なぜV9回帰が最適か

1. **実績**: droid-walkingでhip_pitch相関-0.516を達成した唯一のバージョン
2. **シンプルさ**: 報酬項目が明確で競合が少ない
3. **安定性**: Phase-based報酬などの複雑な設計を含まない
4. **予測可能性**: 類似の設計がexp002でも成功

#### 11.7.2 V10-V16の失敗から学んだこと

| バージョン | 追加した設計 | 結果 | 教訓 |
|-----------|-------------|------|------|
| V10 | Phase-based参照軌道 | 同期促進 | 参照軌道は同期を誘発しやすい |
| V11 | symmetry_hip_roll, ground_contact_bonus | 同期悪化 | 間接的報酬は副作用が多い |
| V12 | strict_alternating_contact | 効果なし | 複雑化は改善しない |
| V13 | V9回帰（一部） | 改善 | シンプル化は有効 |
| V14 | symmetry復活 | Roll悪化 | 過去に失敗した設計は再失敗する |
| V15 | roll_penalty追加 | 部分改善 | 直接的ペナルティは効果的 |
| V16 | hip_pitch_sign_change | **完全失敗** | 論理的欠陥のある報酬は壊滅的 |

**結論**: 新しい報酬関数の追加は高リスク。V9への回帰が最も安全で効果的。

### 11.8 訓練・評価コマンド

```bash
# 訓練
cd rl_ws
uv run python biped_walking/train/droid_train_v17.py --max_iterations 500

# 評価（ヘッドレス）
uv run python biped_walking/biped_eval.py -e droid-walking-v17 --no-viewer --duration 10

# 評価（ビューア付き）
uv run python biped_walking/biped_eval.py -e droid-walking-v17
```

### 11.9 リスク評価と次のステップ

#### 11.9.1 V17のリスク

| リスク | 確率 | 対策 |
|--------|------|------|
| V9と同程度にとどまる | 中 | `roll_penalty`効果で改善の可能性 |
| V9より悪化 | 低 | V9完全回帰のため低リスク |
| 新たな問題発生 | 低 | 新規報酬なし |

#### 11.9.2 V17成功時の次のステップ

1. XOR方式符号変化報酬をV18で試験的に追加
2. `contact_threshold`の最適化
3. より高い目標速度（0.4 m/s）への挑戦

#### 11.9.3 V17失敗時の次のステップ

1. V9の報酬設計を完全にコピー（`roll_penalty`も削除）
2. 初期姿勢やPD制御ゲインなど、報酬以外のパラメータを調査

### 11.10 V17評価結果

#### 基本指標

| 指標 | V17結果 | V9参考 | V15参考 | V16参考 | 評価 |
|------|---------|--------|---------|---------|------|
| X移動距離 | **2.675m** | 2.95m | 3.05m | 3.23m | ❌ 全バージョン中最低 |
| Y移動距離 | **-0.245m** | 0.01m | -0.04m | -0.35m | ❌ 斜行（V15より悪化） |
| Yawドリフト | **-29.09°** | -4.6° | -18.1° | -19.2° | ❌❌ 全バージョン中最悪 |
| 平均速度 | **0.262m/s** | - | 0.298m/s | 0.321m/s | ❌ 目標未達 |
| 基準高さ | 0.226m | - | 0.228m | 0.262m | △ V9目標に近い |

#### 交互歩行指標

| 指標 | V17結果 | V9参考 | V15参考 | V16参考 | 評価 |
|------|---------|--------|---------|---------|------|
| **hip_pitch相関** | **-0.637** | -0.516 | -0.242 | +0.697 | ✅ 最良（交互歩行） |
| Contact Pattern | 両足空中: 100% | - | 100% | 100% | ⚠ 変化なし |

#### 姿勢指標

| 指標 | V17結果 | V9参考 | V15参考 | V16参考 | 評価 |
|------|---------|--------|---------|---------|------|
| Roll mean | **-2.12°** | 1.3° | -2.9° | -1.6° | ○ 許容範囲 |
| Roll std | 4.73° | - | 5.2° | 5.17° | ○ 安定 |
| Pitch mean | **-0.84°** | - | -3.7° | -3.4° | ✅ 最良 |
| Pitch std | 2.93° | - | 2.9° | 1.47° | △ 横ばい |

#### 関節動作分析

```
DOF position range (rad):
  hip_pitch   : L=0.462  R=0.350  ← 左脚が1.3倍大きい
  hip_roll    : L=0.361  R=0.415
  knee        : L=0.559  R=0.737  ← 右脚が大きい

DOF velocity std (rad/s):
  hip_pitch   : L=1.290  R=2.122  ← 右脚が1.6倍速い
  hip_roll    : L=1.763  R=2.282
  ankle_pitch : L=3.939  R=1.366  ← 左脚が2.9倍速い（非対称）
```

### 11.11 V17分析：「過去最低」の理由

#### 問題の整理

| 指標 | V17 | 評価 | 問題の深刻度 |
|------|-----|------|-------------|
| X移動距離 | 2.675m | 全バージョン最低 | ★★★ |
| Yawドリフト | -29.09° | 全バージョン最悪 | ★★★ |
| Y移動距離 | -0.245m | V15より悪化 | ★★ |
| hip_pitch相関 | -0.637 | **最良** | - |

**矛盾**: hip_pitch相関は-0.637で最良（理想的な交互歩行に近い）なのに、前進性能・直進性能が最悪。

#### 仮説1: 交互歩行と前進性能のトレードオフ

V17ではhip_pitch相関-0.637を達成したが、これは**過度な交互動作**を意味する可能性：
- 交互動作を強調しすぎて「その場での足踏み」に近い状態
- 前進推力が犠牲になった

**V9との比較**:
- V9: hip_pitch相関 -0.516、X=2.95m（バランスが取れている）
- V17: hip_pitch相関 -0.637、X=2.675m（交互動作が強すぎ、前進不足）

#### 仮説2: Yawドリフトの悪化要因

V17のYawドリフト-29.09°は全バージョン中最悪：
- `yaw_rate`を-2.0（V15）→-1.0（V17/V9回帰）に緩和した影響
- しかしV9は-4.6°だったので、`yaw_rate`だけが原因ではない

**左右非対称の影響**:
```
ankle_pitch velocity: L=3.939, R=1.366 rad/s（左が2.9倍）
hip_pitch velocity:   L=1.290, R=2.122 rad/s（右が1.6倍）
```
関節ごとに左右で速度が大きく異なり、これが旋回を引き起こしている可能性。

#### 仮説3: V9回帰の不完全性

V17は「V9完全回帰」を目指したが、実際にはV9と異なる点がある可能性：
- `roll_penalty`を追加（V9にはなかった）
- 環境の乱数シードや初期化の違い
- droid_env.pyの他の変更が影響

### 11.12 V9 vs V15 vs V16 vs V17 総合比較

| 指標 | V9 | V15 | V16 | V17 | 最良 |
|------|-----|------|------|------|------|
| X移動距離 | 2.95m | 3.05m | **3.23m** | 2.68m | V16 |
| Y移動距離 | **0.01m** | -0.04m | -0.35m | -0.25m | V9 |
| Yawドリフト | **-4.6°** | -18.1° | -19.2° | -29.1° | V9 |
| hip_pitch相関 | -0.516 | -0.242 | +0.697 | **-0.637** | V17 |
| Roll mean | **1.3°** | -2.9° | -1.6° | -2.1° | V9 |
| Pitch mean | - | -3.7° | -3.4° | **-0.8°** | V17 |
| 平均速度 | - | 0.298 | **0.321** | 0.262 | V16 |

**結論**: V9が総合的に最もバランスが良い。V17は交互歩行指標（hip_pitch相関）だけは改善したが、他の全指標が悪化。

### 11.13 「剣道すり足」問題の再評価

ユーザー報告では「V17も剣道すり足が残っている」とのこと。

しかしhip_pitch相関が-0.637（強い交互歩行）であるにもかかわらず「剣道すり足」と感じられる理由：

1. **前進量が少ない**: 交互に動いているが、歩幅が小さく「その場足踏み」に見える
2. **Yawドリフトが大きい**: まっすぐ歩けていないため、歩行というより「よろめき」に見える
3. **左右非対称**: ankle_pitchの速度差が2.9倍あり、「びっこ」感が残る

**重要な発見**: hip_pitch相関だけでは歩行品質を評価できない。前進量、直進性、左右対称性の総合評価が必要。

### 11.14 V18への改善方針

V9〜V17の実験を通じて明らかになったこと：

| 問題 | V9の状態 | V17で悪化した理由 |
|------|----------|------------------|
| Yawドリフト | -4.6° | 不明（`yaw_rate`回帰したのに悪化） |
| 前進量 | 2.95m | 交互歩行強化の副作用？ |
| 左右非対称 | 良好 | `roll_penalty`追加の影響？ |

#### 方針1: V9の完全コピー（roll_penaltyも削除）

```python
# V9の報酬設計を一切変更せずにコピー
# roll_penaltyも追加しない
```

#### 方針2: パラダイム転換（Phase 12の提案）

Phase 12で検討した「報酬項目の大幅削減」を実行：
- タスク報酬のみ（forward_velocity, alive）
- 制約は終了条件へ
- エネルギー効率の最適化

#### 方針3: 環境パラメータの調査

報酬以外の要因を調査：
- 初期姿勢
- PD制御ゲイン（kp=35.0, kd=2.0）
- action_scale
- 乱数シード

---

## Phase 12-13: 報酬設計パラダイムの再検討と先行研究調査

### 12.1 問題提起

V9〜V17の実験を通じて、**報酬関数で関節の動きを細かく制約する**アプローチを採用してきた。しかし以下の疑問が生じる:

- この「ガチガチに縛る」方針は、強化学習の本来の枠組みとして適切か？
- 先行研究ではどのようなアプローチが採用されているか？

### 12.2 V9〜V17の実験から得られた教訓

#### 教訓1: 関節制約報酬は逆効果になりうる

| バージョン | 交互歩行報酬の強さ | hip_pitch相関 | X移動距離 |
|-----------|------------------|---------------|-----------|
| V9 | 中程度 | -0.516 | 2.95m |
| V15 | やや強い | -0.242 | 3.05m |
| V16 | 非常に強い | +0.697（逆転！） | 3.23m |
| V17 | V9に回帰 | **-0.637** | **2.675m** |

**洞察**: 
- 「交互に動く」という局所的な目標を達成しても、「前進する」というタスク目標が犠牲になる
- 報酬項目間の**トレードオフ**が存在し、調整不可能

#### 教訓2: Dense Reward Shapingの限界

V9〜V17では20〜25項目の報酬を使用。結果として：

| 問題 | 事例 |
|------|------|
| **報酬ハッキング** | V16: 同時符号変化で報酬最大化→同期歩行 |
| **項目間競合** | V17: 交互歩行と前進のトレードオフ |
| **予測不能な相互作用** | V17: V9回帰したのに結果が異なる |

#### 教訓3: hip_pitch相関だけでは歩行品質を評価できない

| hip_pitch相関 | 意味 | 実際の歩行 |
|--------------|------|-----------|
| -0.637 (V17) | 理想的な交互動作 | 前進しない、Yaw大 |
| -0.516 (V9) | 中程度の交互 | バランス良好 |
| +0.697 (V16) | 同期動作 | 前進は良好 |

**結論**: 関節の動きパターンと歩行性能は別物。関節を「正しく」動かしても歩けるとは限らない。

### 12.3 先行研究調査（要約）

> **詳細**: [exp004_reward_design_survey.md](./exp004_reward_design_survey.md)

二脚ロボットの歩容獲得に関する先行研究を調査した結果、成功事例には以下の共通点があった：

| 研究 | アプローチ | 報酬項目数 | 特徴 |
|------|-----------|-----------|------|
| **Legged Gym (ETH)** | タスク+ペナルティ | 5-8 | 転倒を終了条件に、feet_air_time報酬 |
| **Walk These Ways** | 報酬少+終了条件多 | ~10 | 制約を報酬でなく終了条件で |
| **Energy Minimization** | CoT最小化 | 3-5 | エネルギー効率で歩容が創発 |
| **CaT** | Constraint as Termination | 2-3 | 制約違反=即終了 |

#### 本実験との違い

| 項目 | 本実験(V9-V17) | 成功事例 |
|------|---------------|---------|
| 報酬項目数 | 20-25 | 5-10 |
| 関節制約 | 報酬で実装 | 終了条件 |
| 歩容指定 | hip_pitch_alternation等で強制 | 創発に任せる |
| エネルギー考慮 | なし | CoT最小化 |

### 12.4 V18設計方針の根拠

先行研究と本実験の教訓から、以下の設計判断を行った：

#### 判断1: 報酬項目を6項目に削減

**根拠**: 
- Legged Gym、Walk These Waysは10項目以下で成功
- 本実験では項目数増加に伴い報酬ハッキング・項目間競合が発生
- 「Less is More」原則の適用

#### 判断2: 関節レベルの制約を全削除

**根拠**:
- `hip_pitch_alternation`等は報酬ハッキングの原因となった（V16）
- 先行研究は関節レベルの詳細制御を行っていない
- 歩容は**創発**させるべきもの

#### 判断3: 制約を終了条件として実装

**根拠**:
- CaT (Constraint as Termination) の知見
- 報酬ペナルティより明確な学習シグナル
- 探索の自由度を確保しつつ安全性を担保

#### 判断4: エネルギー効率報酬（torques）を追加

**根拠**:
- Energy Minimization研究：CoT最小化で自然な歩行が創発
- トルク最小化はCoTの簡易近似
- 生物もエネルギー効率を最適化している

### 12.5 結論

**現在の「ガチガチに縛る」アプローチは、強化学習の本来の強み（創発的解の発見）を活かせていない。**

V18以降では「ミニマリスト報酬 + 終了条件ベースの制約」というパラダイムシフトを実行し、関節レベルの制御は**学習に任せる**方針を採用する。

---

## Phase 14: V18実装と評価

### 14.1 V18の設計方針

Phase 12-13の調査結果を踏まえ、以下の方針でV18を実装した：

1. **報酬項目を6項目に削減**
2. **関節レベルの制約を全削除**
3. **姿勢制約を終了条件として実装**

#### V18報酬設計

```python
reward_scales = {
    "tracking_lin_vel": 2.0,     # 主目標: 目標速度追従
    "tracking_ang_vel": 0.5,     # 方向維持
    "alive": 1.0,                # 生存報酬
    "torques": -1e-4,            # トルク最小化（CoT代替）
    "dof_acc": -1e-6,            # 加速度抑制
    "orientation": -1.0,         # 姿勢維持
}
```

#### 終了条件

```python
# Roll/Pitch制限
terminated = (roll > 0.5) | (pitch > 0.5)  # ~28.6°
```

### 14.2 V18評価結果

```
=== V18 Evaluation Statistics (10秒間) ===
X移動距離: 2.827 m
Y移動距離: 0.307 m
Yawドリフト: +9.77°

姿勢:
  Roll:  mean= 8.56°, std= 3.74°
  Pitch: mean=13.70°, std= 2.76°
  Base height: 0.212m (初期: 0.347m)

hip_pitch相関: +0.751（同期歩行）
DOF range sum: 3.588 rad
```

### 14.3 V17との比較

| 指標 | V17 | V18 | 変化 | 評価 |
|------|-----|-----|------|------|
| X移動距離 | 2.68m | **2.83m** | +5.6% | ✅ 改善 |
| Yawドリフト | -29.1° | **+9.8°** | 大幅改善 | ✅ 改善 |
| Base height | 0.226m | 0.212m | -6.2% | △ やや悪化 |
| Pitch mean | -0.84° | **13.70°** | 大幅前傾 | ❌ 悪化 |
| hip_pitch相関 | -0.637 | +0.751 | 同期化 | ❌ 悪化 |
| DOF range | 4.96 rad | 3.59 rad | -28% | ❌ 動き減少 |

### 14.4 V18の評価

#### 成功点（ユーザーフィードバック）

- 「びっこを引いていない」
- 「前脚と後脚の固定が改善された」

#### 問題点

1. **歩幅が小刻みすぎる**: DOF range減少
2. **胴体が前傾している**: Pitch mean = 13.7°
3. **胴体が下がりすぎている**: Base height = 0.212m（初期の61%）
4. **同期歩行**: hip_pitch相関 +0.751

### 14.5 V19への改善方針

V18の問題を解決するため、以下を実施：

#### 14.5.1 問題と対策の対応

| 問題 | 数値 | 対策 | 手法 |
|------|------|------|------|
| 前傾 | Pitch=13.7° | pitch_penalty追加 | 報酬 |
| 沈み込み | Height=0.212m | base_height追加 | 報酬 |
| 小刻み | DOF range=3.59 | hip_pitch_velocity追加 | 報酬 |
| 同期 | 相関=+0.751 | feet_air_time + single_stance追加 | 報酬 |
| 姿勢崩れ | 全般 | 終了条件強化 | CaT |

#### 14.5.2 V19報酬設計案

```python
reward_scales = {
    # タスク報酬（V18維持）
    "tracking_lin_vel": 2.0,
    "tracking_ang_vel": 0.5,
    "alive": 1.0,

    # エネルギー効率（V18維持）
    "torques": -1e-4,
    "dof_acc": -1e-6,

    # 姿勢制御（強化）★V19追加
    "orientation": -3.0,          # V18: -1.0 → -3.0
    "base_height": -3.0,          # 新規（-5.0は強すぎるため-3.0で様子見）
    "pitch_penalty": -2.0,        # 新規

    # 歩行パターン（新規）★V19追加
    "feet_air_time": 1.0,         # 交互歩行促進
    "single_stance": 0.3,         # 片足接地報酬（Cassie方式）
    "hip_pitch_velocity": 0.5,    # 動き促進
}
```

#### 14.5.3 V19終了条件

```python
env_cfg = {
    "termination_if_roll_greater_than": 25,    # 30° → 25°
    "termination_if_pitch_greater_than": 20,   # 30° → 20°（前傾禁止）
    "termination_if_height_lower_than": 0.15,  # V18: 0.12m → 0.15m（0.18mは厳しすぎ）
}
```

#### 14.5.4 期待効果

| 指標 | V18 | V19期待 | 根拠 |
|------|-----|---------|------|
| Pitch | 13.7° | < 10° | pitch_penalty + 終了条件 |
| Height | 0.212m | > 0.22m | base_height + 終了条件 |
| DOF range | 3.59 rad | > 4.0 rad | hip_pitch_velocity |
| hip_pitch相関 | +0.751 | < 0 | feet_air_time |

詳細な設計根拠は [exp004_reward_design_survey.md](exp004_reward_design_survey.md) を参照。
---

## Phase 15: V19実装と評価（失敗分析）

### 15.1 V19評価結果

```
=== V19 Evaluation Statistics (10秒間) ===
X移動距離: 0.047 m  ← 【致命的】ほぼ静止
平均速度: 0.005 m/s (目標0.3の1.7%)

姿勢:
  Roll:  mean=-11.03°, std= 2.33°  ← 左に傾き（八の字姿勢）
  Pitch: mean= -5.00°, std= 1.74°  ← 後傾気味
  Base height: 0.191 m             ← V18より更に低下

hip_pitch相関: -0.979（交互）
DOF range sum: 4.998 rad
Contact: Single foot 91%

関節動作:
  hip_pitch: L=0.533 rad, R=0.679 rad  ← 大きな動作範囲
  hip_roll:  L=0.480 rad, R=0.552 rad  ← 八の字開脚
```

### 15.2 V18との比較

| 指標 | V18 | V19 | 変化 | 評価 |
|------|-----|-----|------|------|
| **X移動距離** | 2.827m | **0.047m** | **-98.3%** | ❌ 壊滅的劣化 |
| 平均速度 | 0.274 m/s | 0.005 m/s | -98.2% | ❌ 壊滅的劣化 |
| Base height | 0.212m | 0.191m | -10% | △ やや悪化 |
| Pitch | 13.70° | -5.00° | 改善? | △ 後傾に反転 |
| Roll | 8.56° | -11.03° | 悪化 | × 左傾斜 |
| hip_pitch相関 | +0.751 | **-0.979** | 交互化 | ○ 数値上は改善 |
| DOF range | 3.59 rad | 4.998 rad | +39% | ○ 動きは増加 |
| Contact single | 0% | 91% | 機能 | ○ 接地検出成功 |

### 15.3 ユーザーフィードバック

**良い点**:
- なし

**悪い点**:
- 脚を八の字に開いたままその場で静止
- 胴体位置は下がり過ぎている

### 15.4 失敗原因分析

#### 15.4.1 根本原因

**「制約が厳しすぎて、動かないことが最適解になった」**

V19では以下の変更を同時に適用した結果、エージェントは「動かない」という局所最適解に収束した。

#### 15.4.2 終了条件の過剰な厳格化

```python
# V18 → V19の終了条件変更
termination_if_pitch_greater_than: 30° → 20°
termination_if_height_lower_than: 0.12m → 0.15m
```

- V18のPitch mean = 13.7°であり、動けば20°を超えるリスク
- 積極的に動くと沈み込んでHeight < 0.15mになるリスク
- 結果: 「動かないのが最も安全」と学習

#### 15.4.3 姿勢ペナルティの過剰強化

```python
# V18 → V19のペナルティ変更
orientation: -1.0 → -3.0        # 3倍に強化
base_height: 0 → -3.0           # 新規追加
pitch_penalty: 0 → -2.0         # 新規追加
# 合計ペナルティ係数: 1.0 → 8.0（8倍）
```

#### 15.4.4 報酬バランスの崩壊

```
V18: tracking_lin_vel(2.0) vs 姿勢ペナルティ(-1.0) = 2:1（前進優先）
V19: tracking_lin_vel(2.0) vs 姿勢ペナルティ(-8.0) = 2:8 = 1:4（静止優先）
```

前進するインセンティブが姿勢維持のペナルティに圧倒された。

#### 15.4.5 皮肉な「成功」

数値上は以下の点が改善された：
- hip_pitch相関: +0.751 → -0.979（完璧な交互動作）
- 接地検出: 0% → 91%（片足接地）

しかし、これは「その場で足踏み」する動作であり、前進しないため意味がない。

### 15.5 教訓

> **「制約を強化すれば問題が解決する」は誤り**
> 
> 制約（終了条件・ペナルティ）を強化すると、エージェントは「制約を満たす最も簡単な方法」を学習する。
> 多くの場合、それは「何もしない」ことである。

#### 得られた知見

1. **制約強化は逆効果になりうる**: 厳しい終了条件 → 「動かない」が最適解
2. **報酬バランスが最重要**: 主報酬（前進）を圧倒しないペナルティ設計が必要
3. **複数変更の同時適用は危険**: 問題の切り分けが困難になる
4. **段階的改善が必要**: 一度に1つの問題だけを解決する

### 15.6 学習ログ分析：追加学習の可能性

#### 15.6.1 V18とV19の学習曲線比較

| 指標 | V18 | V19 | 評価 |
|------|-----|-----|------|
| **最終報酬** | 68.71 | 54.95 | V19は20%低い |
| **報酬の収束** | std=0.013（収束） | std=1.54（変動中） | V19は未収束 |
| **エピソード長** | 1001（上限） | 878 | V19は早期終了が多い |
| **報酬上昇傾向** | 200step以降横ばい | 500stepでも上昇中 | V19はまだ学習途中 |

#### 15.6.2 V19の学習曲線詳細

```
Quarter-wise reward average:
  Steps 0-125:   22.50
  Steps 125-250: 41.01 (+18.51)
  Steps 250-375: 48.03 (+7.01)
  Steps 375-500: 54.95 (+6.92)  ← まだ上昇中
```

- 最大報酬: 59.02 (step 438)
- 最終報酬: 54.95
- Last 50 steps std: 1.54（まだ変動中）

#### 15.6.3 個別報酬項目の比較（最終ステップ）

| 報酬項目 | V18 | V19 | 差分 | 解釈 |
|---------|-----|-----|------|------|
| **tracking_lin_vel** | 1.899 | 1.466 | -0.433 | 前進が不十分 |
| **tracking_ang_vel** | 0.467 | 0.112 | -0.355 | 旋回も少ない |
| **alive** | 0.959 | 0.874 | -0.085 | 早期終了が多い |
| orientation | -0.002 | -0.001 | +0.001 | 姿勢は良好 |
| torques | -0.000 | -0.014 | -0.014 | トルク使用が多い |
| dof_acc | -0.017 | -0.103 | -0.086 | 加速度が大きい |
| base_height | - | -0.007 | - | 沈み込みペナルティ |
| pitch_penalty | - | -0.016 | - | 前傾ペナルティ |
| feet_air_time | - | -0.012 | - | **負の値**（問題） |
| single_stance | - | +0.023 | - | 片足接地は成功 |
| hip_pitch_velocity | - | +0.410 | - | 動きは促進されている |

#### 15.6.4 追加学習の見込み分析

**結論: 500step以上回しても根本的改善は見込めない**

理由：

1. **報酬設計自体が問題**:
   - `feet_air_time`が**負の値**（-0.012）になっている
   - これは「足を上げると罰せられる」状態であり、設計意図と逆
   - 追加学習しても「足を上げない」方向に最適化が進む

2. **tracking_lin_velの伸び悩み**:
   - V18: 1.899 vs V19: 1.466
   - V19は姿勢ペナルティが強すぎて前進を抑制
   - 追加学習しても姿勢維持に報酬が消費され、前進は改善しない

3. **V18との収束状態の違い**:
   - V18: step 200で収束、以降は微調整
   - V19: step 500でもstd=1.54と変動中 → 不安定な状態
   - 不安定 = 「どちらに転んでも良い」状態ではなく「良い解が見つからない」状態

4. **エピソード長の差**:
   - V18: 1001（上限到達 = 終了条件に引っかからない）
   - V19: 878（早期終了が12%発生 = 厳しい終了条件に抵触）
   - 追加学習しても「動かない」戦略がさらに強化される可能性

#### 15.6.5 追加学習よりも報酬設計の見直しが必要

V19の学習は「間違った目標に向かって正しく学習している」状態：
- 姿勢ペナルティを回避 → 動かない
- 終了条件を回避 → 動かない
- feet_air_timeが負 → 足を上げない

**追加学習は逆効果**になる可能性が高く、報酬設計の根本的見直し（V20）が必要。

### 15.6 V20への改善方針

#### 15.6.1 基本方針

**V18をベースに、最小限の変更で改善を試みる**

V19の失敗から、以下の方針を採用：
1. 終了条件はV18レベルに戻す
2. 姿勢ペナルティを削減（orientation -1.0のみ）
3. 前傾・沈み込みは一旦許容（歩行確立後に対処）
4. feet_air_timeのみ控えめに追加

#### 15.6.2 V20報酬設計案

```python
# V20提案: 7項目（V18の6項目 + feet_air_time）
reward_scales = {
    # === V18から維持（バランス変更なし）===
    "tracking_lin_vel": 2.0,      # 前進報酬（主報酬）
    "tracking_ang_vel": 0.5,      # 旋回報酬
    "alive": 1.0,                 # 生存報酬
    "orientation": -1.0,          # 姿勢ペナルティ（V18と同じ）
    "torques": -1e-4,             # トルクペナルティ
    "dof_acc": -1e-6,             # 加速度ペナルティ
    
    # === V20で追加（控えめに）===
    "feet_air_time": 0.3,         # V19の1.0→0.3に減少（補助的役割）
}
```

#### 15.6.3 V20終了条件

```python
env_cfg = {
    "termination_if_roll_greater_than": 30,   # V19: 25 → V20: 30（緩和）
    "termination_if_pitch_greater_than": 30,  # V19: 20 → V20: 30（緩和）
    "termination_if_height_lower_than": 0.10, # V19: 0.15 → V20: 0.10（緩和）
}
```

#### 15.6.4 優先順位の明確化

1. 🥇 **前進**: まず歩けることが最重要
2. 🥈 **交互歩行**: 足を交互に動かす
3. 🥉 **姿勢**: 歩けた後に改善

#### 15.6.5 段階的改善戦略

```
V20: V18 + feet_air_time(0.3) → 交互歩行の誘導
 ↓
V21: V20 + 成功なら base_height(軽め) → 沈み込み改善
 ↓
V22: V21 + 成功なら orientation強化 → 姿勢改善
```

**一度に複数の問題を解決しようとしない**

---

## Phase 16: V19失敗からの設計指針確立

### 16.1 V19失敗の総括

V19は「制約を強化すれば問題が解決する」という仮説に基づいて設計されたが、結果は壊滅的であった。

#### 16.1.1 評価結果の再確認（2026-02-01実施）

```
=== V19 Evaluation Statistics (10秒間) ===
Travel distance:
  X: 0.047 m  ← 【致命的】ほぼ静止
  Y: 0.065 m

Average velocity:
  X: 0.005 m/s (target: 0.300)  ← 目標の1.7%
  Y: 0.006 m/s

Base height:
  Mean: 0.191 m  ← V18(0.212m)より悪化
  Std: 0.0143 m

Orientation (deg):
  Roll:  mean=-11.03, std= 2.33  ← 左に傾き（八の字姿勢）
  Pitch: mean= -5.00, std= 1.74  ← 後傾気味
  Yaw:   drift=+13.41°

DOF position range (rad):
  hip_pitch   : L=0.533  R=0.679  ← 大きな動作範囲
  hip_roll    : L=0.480  R=0.552  ← 八の字開脚の原因
  knee        : L=0.544  R=0.293
  ankle_pitch : L=0.657  R=0.412
  ankle_roll  : L=0.469  R=0.378

Left-Right hip_pitch correlation: -0.979  ← 完璧な交互動作

Contact Pattern:
  Single foot grounded: 91.0%  ← 片足接地は成功
  Both feet airborne:    8.6%
```

#### 16.1.2 ユーザーフィードバック

**良い点**: なし

**悪い点**:
- 脚を八の字に開いたままその場で静止
- 胴体位置は下がり過ぎている

### 16.2 失敗の根本原因分析

#### 16.2.1 報酬バランスの崩壊

| 項目 | V18（成功） | V19（失敗） | 変化 |
|------|-------------|-------------|------|
| tracking_lin_vel | 2.0 | 2.0 | 維持 |
| 姿勢ペナルティ合計 | -1.0 | **-8.0** | **8倍** |
| 報酬バランス | 2:1（前進優先） | **1:4（静止優先）** | 逆転 |

V19では姿勢ペナルティが主報酬を圧倒し、「動かない」が最適解となった。

#### 16.2.2 終了条件の過剰な厳格化

| 終了条件 | V18 | V19 | 実測値（V18） |
|---------|-----|-----|--------------|
| pitch | 30° | **20°** | 13.7° |
| height | 0.12m | **0.15m** | 0.212m |

V18の実測値を考慮すると、動けばpitch 20°を超えるリスクがあり「動かない」が安全策となった。

#### 16.2.3 feet_air_time報酬の逆効果

学習ログ分析結果：
```
feet_air_time: -0.012 ⚠️ 負の値
```

`target_air_time=0.2秒`に対し、V19の小刻みな動きでは滞空時間が不足し、「足を上げると罰せられる」状態になった。

### 16.3 確立された設計指針

V9〜V19の実験を通じて、以下の設計指針が確立された。

#### 16.3.1 報酬バランスの原則

```
【必須】主報酬（前進）≧ ペナルティ合計 × 2

V18（成功）: 2.0 / 1.0 = 2.0 ✓
V19（失敗）: 2.0 / 8.0 = 0.25 ✗
```

| 報酬バランス比 | 結果 | 推奨 |
|---------------|------|------|
| > 2.0 | 前進を優先しつつ制約を守る | ✓ 推奨 |
| 1.0〜2.0 | バランス取れるが注意が必要 | △ 要検証 |
| < 1.0 | **「動かない」が最適解になる** | ✗ 禁止 |

#### 16.3.2 終了条件の調整原則

```
【禁止】一度に10°以上の厳格化
【推奨】5°刻みで段階的に調整
【必須】実測値 + マージン20%以上を確保
```

例：V18のPitch mean=13.7°の場合
- 安全な終了条件: 13.7° × 1.2 + 5° ≈ **22°** 以上
- V19の20°は危険領域

#### 16.3.3 複数変更の同時適用の禁止

```
【原則】1回の実験で変更するのは1カテゴリのみ
```

V19で同時に変更した項目（失敗パターン）：
1. 終了条件（roll, pitch, height）
2. 姿勢ペナルティ（orientation, base_height, pitch_penalty）
3. 歩行パターン報酬（feet_air_time, single_stance, hip_pitch_velocity）

→ 問題の切り分けが不可能になった

#### 16.3.4 feet_air_time報酬の使用条件

```
【条件1】target_air_time ≤ 実際に達成可能な滞空時間
【条件2】スケールは控えめに開始（0.3以下）
【条件3】まず歩行が確立してから追加
```

### 16.4 V20への改善方針（再確認）

V19の失敗を踏まえ、以下の方針を堅持する。

#### 16.4.1 基本戦略

```
V18をベースに、最小限の変更で1つずつ改善を試みる
```

#### 16.4.2 V20の具体的設計

**報酬設計（7項目）**:
```python
reward_scales = {
    # V18から維持（バランス変更なし）
    "tracking_lin_vel": 2.0,      # 主報酬
    "tracking_ang_vel": 0.5,
    "alive": 1.0,
    "orientation": -1.0,          # V18と同じ（-3.0にしない）
    "torques": -1e-4,
    "dof_acc": -1e-6,

    # V20で追加（控えめに）
    "feet_air_time": 0.3,         # V19の1.0→0.3
}
# 報酬バランス: 2.0 / 1.0 = 2.0 ✓（V18と同等）
```

**終了条件（V18と同じ）**:
```python
env_cfg = {
    "termination_if_roll_greater_than": 30,   # V19の25→30に戻す
    "termination_if_pitch_greater_than": 30,  # V19の20→30に戻す
    "termination_if_height_lower_than": 0.10, # V19の0.15→0.10に戻す
}
```

**feet_air_time設定**:
```python
reward_cfg = {
    "feet_air_time_target": 0.10,  # V19の0.20→0.10（達成容易に）
}
```

#### 16.4.3 期待効果

| 指標 | V18 | V19 | V20期待 |
|------|-----|-----|---------|
| X移動距離 | 2.83m | 0.05m | > 2.5m |
| hip_pitch相関 | +0.75 | -0.98 | < 0（交互） |
| Base height | 0.212m | 0.191m | > 0.20m |

#### 16.4.4 成功基準

1. X移動距離 ≥ 2.0m（V18の70%以上）
2. hip_pitch相関 < 0（交互歩行の兆候）
3. 早期終了率 < 20%

### 16.5 今後の段階的改善計画

```
Phase A: 歩行確立（最優先）
  V20: V18 + feet_air_time(0.3, target=0.1s)
  成功基準: X移動 ≥ 2.0m, hip相関 < 0

Phase B: 姿勢改善（歩行確立後）
  V21: V20成功時 + base_height(-1.0)
  成功基準: Height > 0.22m, X移動維持

Phase C: 歩行品質向上
  V22: V21成功時 + orientation(-2.0)
  V23: V22成功時 + hip_pitch_velocity(0.3)
```

**重要**: 各フェーズで前フェーズの成果を維持できることを確認してから次に進む。後退が見られた場合は、変更を取り消して原因を分析する。

---

## Phase 17: V20実装と評価

### 17.1 V20実験結果

#### 17.1.1 学習ログ分析

```
=== droid-walking-v20 Training Log Analysis ===

Train/mean_reward:
  Step    0:     0.938
  Step  100:    48.406
  Step  250:    67.738
  Step  499:    68.390
  Trend: ↑ 上昇 (収束)

Train/mean_episode_length:
  Step  100:   978.000
  Step  150〜: 1001.000（上限到達、早期終了なし）

Last 50 steps std: 0.0447 → ほぼ収束
```

**個別報酬項目（最終ステップ）**:
```
action_rate              :    -0.0153 ⚠️
alive                    :     1.0010
dof_acc                  :    -0.0204 ⚠️
feet_air_time            :     0.0000  ← 全く報酬が得られていない
orientation              :    -0.0003 ⚠️
torques                  :    -0.0006 ⚠️
tracking_ang_vel         :     0.4865
tracking_lin_vel         :     1.9684
```

**重要な発見**: `feet_air_time = 0.0000` → 足が地面から離れていない、または接地検出が機能していない。

#### 17.1.2 定量評価結果（10秒間）

```
=== V20 Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 2.719 m         ← V18(2.83m)より減少
  Y: 0.004 m
  Total: 2.719 m

Average velocity:
  X: 0.268 m/s (target: 0.300)  ← 目標の89%

Base height:
  Mean: 0.183 m      ← V18(0.212m)より29mm低下
  Std: 0.0169 m

Orientation (deg):
  Roll:  mean=  2.00, std= 3.59   ← 良好
  Pitch: mean=  9.39, std= 1.75   ← 前傾（V18: 13.7°よりは改善）
  Yaw:   drift= +1.12°            ← 良好（V18: +9.77°より改善）

hip_pitch相関: 0.408   ← 同期歩行（V18: +0.751より改善だが依然正）
DOF range sum: 3.981 rad  ← V18(3.588)より増加
DOF velocity RMS: 1.293 rad/s

Contact Pattern:
  Both feet airborne: 500 steps (100%)  ← 接地検出が機能していない
```

#### 17.1.3 V18との比較

| 指標 | V18 | V20 | 変化 | 評価 |
|------|-----|-----|------|------|
| X移動距離 | 2.83m | 2.72m | -4% | △ やや悪化 |
| Yawドリフト | +9.77° | +1.12° | **大幅改善** | ○ 改善 |
| Base height | 0.212m | **0.183m** | **-14%** | × 悪化 |
| Pitch mean | 13.70° | 9.39° | **改善** | ○ 改善 |
| hip_pitch相関 | +0.751 | +0.408 | 同期→やや改善 | △ 依然同期 |
| DOF range sum | 3.59 rad | 3.98 rad | +11% | ○ 増加 |

### 17.2 ユーザーフィードバック（目視評価）

**良い点**:
- 前に進んだ

**悪い点**:
- 脚を動かす周期が小刻みすぎる
- 脚を動かすストライドと上げ幅がどちらも細かすぎる
- 胴体位置が下がり過ぎている

### 17.3 問題分析

#### 17.3.1 接地検出の問題

V20で最も深刻な問題は**接地検出が機能していない**こと：
- `feet_air_time` 報酬が常に0
- `Both feet airborne: 100%` という異常な結果

これはGenesisシミュレータの接触力取得方法がLegged Gym（IsaacGym）と異なる可能性がある。現在のdroid_env.pyでの接地判定実装を確認する必要がある。

#### 17.3.2 小刻み歩行の原因

| 症状 | 推定原因 |
|------|---------|
| ストライドが小さい | hip_pitchの可動範囲が狭い（L: 0.254 rad, R: 0.314 rad） |
| 周期が速い | 足を上げるインセンティブがない（feet_air_time無効） |
| 足上げ幅が小さい | 同上 |

#### 17.3.3 胴体沈み込みの原因

- Base height: 0.183m（初期0.35mの52%）
- `base_height` 報酬を設定していないため、沈み込んでも問題ない
- 低い姿勢の方が安定性が高く、エネルギー効率も良い傾向

### 17.4 V21への改善方針

#### 17.4.1 優先課題

1. **接地検出の修正**（最優先）: feet_air_time報酬が機能するよう、droid_env.pyの接地判定を確認・修正
2. **胴体高さの改善**: `base_height` 報酬を追加（target: 0.25m）
3. **ストライド拡大**: 現時点では保留（接地検出が機能すれば自然に改善される可能性）

#### 17.4.2 技術的調査事項

1. Genesis物理シミュレータでの接触力取得方法
2. `contact_forces`テンソルの形状と値の確認
3. 接地閾値（現在: 0.04）の妥当性検証

#### 17.4.3 V21設計案（暫定）

```python
# 接地検出が機能する前提での設計
reward_scales = {
    # V18ベース
    "tracking_lin_vel": 2.0,
    "tracking_ang_vel": 0.5,
    "alive": 1.0,
    "orientation": -1.0,
    "torques": -1e-4,
    "dof_acc": -1e-6,
    "action_rate": -0.02,

    # 新規追加
    "feet_air_time": 0.5,      # V20の0.3→0.5（接地検出修正後）
    "base_height": -1.0,       # 沈み込み防止
}

reward_cfg = {
    "base_height_target": 0.25,  # 目標高さを明示
    "feet_air_time_target": 0.15,  # 0.10→0.15
}
```

### 17.5 次のステップ

1. droid_env.pyの接地検出ロジックを調査
2. 接地判定が正しく機能するよう修正
3. 修正後にV21を実装・訓練

### 17.6 接地検出問題の詳細調査

droid_env.pyを確認した結果、接地検出の実装は以下の通り：

```python
def _get_foot_contacts(self):
    """足の接地状態を取得（簡易版：Z座標ベース）"""
    contact_threshold = self.reward_cfg.get("contact_threshold", 0.025)  # デフォルト0.025m
    link_pos = self.robot.get_links_pos()
    feet_z = link_pos[:, self.feet_indices, 2]
    return feet_z < contact_threshold  # Z座標が閾値未満なら接地
```

**問題点**:
- V20では`contact_threshold = 0.04`を設定しているが、足のZ座標がこれより大きい可能性
- 実際の歩行中、胴体高さが0.178〜0.183m程度で、足が地面より高い位置にある
- Genesisの座標系や足リンクの原点位置を確認する必要がある

**調査項目**:
1. 歩行中の`feet_z`の実際の値をログ出力して確認
2. 閾値を0.08〜0.10mに引き上げてテスト
3. 接触力ベースの判定への切り替えを検討

### 17.7 V21設計の最終案

接地検出は実機デプロイ時にもトルク推定等の実装が必要となり、コストが高い。**接地検出なしで進める方針**を採用する。

#### 17.7.1 設計方針

```python
reward_scales = {
    # V18ベース（維持）
    "tracking_lin_vel": 2.0,
    "tracking_ang_vel": 0.5,
    "alive": 1.0,
    "orientation": -1.0,
    "torques": -1e-4,
    "dof_acc": -1e-6,
    "action_rate": -0.02,

    # 新規追加: 胴体高さ維持
    "base_height": -2.0,      # 沈み込み防止

    # 接地検出に依存する報酬は使用しない
    # "feet_air_time": 削除
}

reward_cfg = {
    "base_height_target": 0.25,  # 目標高さ
}
```

#### 17.7.2 接地検出なしの理由

1. **シミュレーションでの問題**: 現在の座標ベース判定が機能していない
2. **実機での問題**: トルク推定方式でも実装コストが発生
3. **代替手段**: 胴体高さ報酬で間接的に歩行品質を改善可能

#### 17.7.3 期待効果

| 指標 | V20 | V21期待 |
|------|-----|---------|
| Base height | 0.183m | > 0.22m |
| X移動距離 | 2.72m | ≥ 2.5m |
| Pitch | 9.39° | < 10° |

交互歩行の誘導は、胴体高さが改善された後に別のアプローチ（hip_pitch速度報酬など）で検討する。

---

## Phase 18: V21評価と足先空間版V22への移行

### 18.1 V21実験結果

#### 18.1.1 V21設計概要

V21は「接地検出なし + base_height報酬」アプローチ：

```python
reward_scales = {
    "tracking_lin_vel": 2.0,
    "tracking_ang_vel": 0.5,
    "alive": 1.0,
    "torques": -1e-4,
    "dof_acc": -1e-6,
    "orientation": -1.0,
    "base_height": -2.0,       # V21追加：沈み込み防止
    "action_rate": -0.02,
}

reward_cfg = {
    "base_height_target": 0.25,  # V20の0.22 → 0.25
}
```

#### 18.1.2 定量評価結果（10秒間）

```
=== V21 Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 2.698 m         ← V20(2.72m)とほぼ同等
  Y: 0.451 m         ← 横方向ドリフト増加
  Total: 2.735 m

Average velocity:
  X: 0.269 m/s (target: 0.300)  ← 目標の90%

Base height:
  Mean: 0.191 m      ← V20(0.183m)から+8mm【改善】
  Std: 0.0155 m

Orientation (deg):
  Roll:  mean=  0.70, std= 3.36   ← 良好
  Pitch: mean= 11.48, std= 2.61   ← V20(9.39°)より悪化
  Yaw:   drift= +15.75°           ← V20(+1.12°)より大幅悪化

hip_pitch相関: 0.575   ← V20(+0.408)より悪化（より同期的）
DOF range sum: 4.179 rad  ← V20(3.981)より増加
DOF velocity RMS: 1.232 rad/s
```

#### 18.1.3 V20との比較

| 指標 | V20 | V21 | 変化 | 評価 |
|------|-----|-----|------|------|
| X移動距離 | 2.72m | 2.70m | -1% | △ 同等 |
| Yawドリフト | +1.12° | **+15.75°** | **悪化** | × 悪化 |
| Base height | 0.183m | **0.191m** | **+4%** | ○ 微改善 |
| Pitch mean | 9.39° | 11.48° | +2° | △ やや悪化 |
| hip_pitch相関 | +0.408 | +0.575 | 同期化 | × 悪化 |
| DOF range sum | 3.98 rad | 4.18 rad | +5% | ○ 増加 |

### 18.2 ユーザーフィードバック（目視評価）

**良い点**:
- 前に進んだ

**悪い点**:
- 脚を動かす周期が小刻みすぎる
- 脚を動かすストライドと上げ幅がどちらも細かすぎる
- 胴体位置が下がり過ぎている（V20から変わらず）

### 18.3 V21の評価

#### 18.3.1 base_height報酬の効果

- 胴体高さ: 0.183m → 0.191m（+8mm）
- 目標0.25mに対して依然として大幅に不足
- **報酬スケール-2.0では効果が弱すぎる**

#### 18.3.2 関節空間アプローチの限界

V18〜V21で試みた関節空間での報酬設計では：
- 前進は達成（X: 2.7〜2.8m）
- 胴体高さ維持に失敗（目標0.25mに対して0.18〜0.19m）
- 小刻み歩行が解消されない
- 交互歩行にならない（hip_pitch相関 > 0.4）

**結論**: 関節空間での探索は「どの関節をどう動かすか」を学習させるため、歩行の本質的なタスク（足をどこに置くか）から乖離している。

### 18.4 足先空間アプローチへの移行

#### 18.4.1 方針転換の理由

1. **タスク整合性**: 歩行の本質は「足先をどこに配置するか」
2. **次元削減**: 10D関節空間 → 6D足先空間（または4D: XZ×2）
3. **直感的制御**: 足先位置は人間にとって理解しやすい
4. **先行研究の成功**: Residual Policy Learning、階層型RLでの実績

#### 18.4.2 V22設計概要

**アプローチ**: Residual Policy + 基本軌道

```
行動空間: 4次元（左足XZ偏差 + 右足XZ偏差）
          ↓ 残差として加算
基本軌道: 正弦波ベースの足先XZ軌道
          ↓ IK変換
関節指令: 10次元
```

**設計理由**:
1. Y方向は固定（横移動は今回不要）
2. 基本軌道で最低限の歩行パターンを保証
3. 残差で適応的な調整を学習
4. 解析的IKが利用可能（BSL-Droidの構造上）

#### 18.4.3 実装計画

1. **運動学モジュール**: `DroidKinematics`クラス（FK/IK）
2. **足先空間環境**: `DroidEnvTaskSpace`クラス
3. **基本軌道生成**: 正弦波ベースのスイング軌道
4. **V22訓練スクリプト**: 新しい行動空間に対応

### 18.5 次のステップ

> **Note**: 足先空間（Task-Space）アプローチの実験は [exp006_droid_rl_walking_taskspace](../exp006_droid_rl_walking_taskspace/exp006_droid_rl_walking_taskspace.md) に移行しました。

---

## 実験継続先

本実験（exp004）での関節空間（Joint-Space）アプローチは V21 で区切りとし、以下の実験に継続しています：

- **足先空間アプローチ**: [exp006_droid_rl_walking_taskspace](../exp006_droid_rl_walking_taskspace/exp006_droid_rl_walking_taskspace.md)
  - V22: Residual Policy（失敗）
  - V23: Residual Policy改善版
  - V24: End-to-End（静止最適問題発生）
  - V25: 報酬バランス改善版
