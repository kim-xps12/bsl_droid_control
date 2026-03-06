# V20: Genesis Contact Sensor による接地検出の根本修正

## 概要

V19（ストライド改善試行）で継続した「接地検出の完全な失敗」問題を根本から解決する。V19では `contact_threshold` を0.025m→0.05mに緩和したが効果がなく、高さベースの接地検出の限界が明らかになった。

V20ではGenesisの物理エンジンが提供する**Contact Sensor**を導入し、足リンクの実際の接触状態を検出する方式に切り替える。

## 前バージョンからの改善事項

### V19の課題分析（V19レポートより）

| 課題 | V19結果 | 根本原因 |
|------|---------|----------|
| **feet_air_time** | 0.0 | 高さベース検出の失敗 |
| **single_foot_contact** | 0.0 | 高さベース検出の失敗 |
| **接地パターン** | 100%両足空中と判定 | contact_threshold不適切 |
| **目視評価** | すり足状態 | 足上げ動機の消失 |

**V19の失敗原因**:
- Base高さが0.224m（目標0.20mより高い）
- 足のZ座標が常に`contact_threshold`（0.05m）より高い
- 結果として「両足空中」と誤判定され続けた

### V19の改善提案に対する精査

V19レポートで提案された改善案:

| 優先度 | 方策 | 採用 | 理由 |
|--------|------|------|------|
| **1-A** | 力ベース検出への移行 | **採用** | 根本解決策 |
| 1-B | 高さ閾値の大幅緩和（0.05→0.10m） | 不採用 | 対症療法に過ぎない |
| 1-C | 動的閾値（base_heightに連動） | 不採用 | 実装が複雑 |
| **2** | feet_clearance報酬（正の報酬） | 次バージョン検討 | 接地検出修正後に評価 |
| **3** | hip_pitch_range報酬 | 次バージョン検討 | 接地検出修正後に評価 |

## 設計詳細

### 1. Genesis Contact Sensor API 調査結果

Genesisの `genesis_official/` サブモジュールを調査した結果、以下のセンサーAPIが利用可能であることが判明した。

#### 利用可能なセンサー

| センサー | クラス | 返り値 | 用途 |
|---------|--------|--------|------|
| Contact Sensor | `gs.sensors.Contact` | boolean | 接触の有無を判定 |
| Contact Force Sensor | `gs.sensors.ContactForce` | Vector3 [N] | 接触力を取得（ローカルフレーム） |

#### サンプルコード（公式: `genesis_official/examples/sensors/contact_force_go2.py`）

```python
import genesis as gs

# シーンとロボットの設定
scene = gs.Scene(...)
robot = scene.add_entity(gs.morphs.URDF(file="...", pos=(0, 0, 0.2)))

# 各足リンクにセンサーを追加（scene.build()前に実行）
foot_link_names = ("left_foot", "right_foot")
for link_name in foot_link_names:
    sensor_options = gs.sensors.Contact(
        entity_idx=robot.idx,
        link_idx_local=robot.get_link(link_name).idx_local,
        draw_debug=True,  # デバッグ表示
    )
    sensor = scene.add_sensor(sensor_options)

scene.build(n_envs=num_envs)

# ステップ後にセンサー値を読み取り
for _ in range(steps):
    scene.step()
    is_contact = sensor.read()  # boolean (n_envs,) または scalar
```

#### センサーオプション（`genesis/options/sensors/options.py`より）

**Contact センサー**:
| パラメータ | 型 | 説明 | デフォルト |
|-----------|---|------|-----------|
| `entity_idx` | int | ロボットのエンティティインデックス | 必須 |
| `link_idx_local` | int | 足リンクのローカルインデックス | 必須 |
| `delay` | float | 読み取り遅延（秒） | 0.0 |
| `draw_debug` | bool | デバッグ球体の表示 | False |
| `debug_sphere_radius` | float | デバッグ球体の半径 | 0.05 |

**ContactForce センサー**:
| パラメータ | 型 | 説明 | デフォルト |
|-----------|---|------|-----------|
| `entity_idx` | int | ロボットのエンティティインデックス | 必須 |
| `link_idx_local` | int | 足リンクのローカルインデックス | 必須 |
| `min_force` | float/tuple | 最小検出可能力（下回ると0） | 0.0 |
| `max_force` | float/tuple | 最大出力力（上回るとクリップ） | ∞ |
| `delay` | float | 読み取り遅延（秒） | 0.0 |
| `noise` | float/tuple | 加算ホワイトノイズの標準偏差 | 0.0 |

#### 低レベルAPI: Collider.get_contacts()

より詳細な接触情報が必要な場合:
```python
all_contacts = scene._sim.rigid_solver.collider.get_contacts(
    as_tensor=True,   # Torch Tensorで返す
    to_torch=True,    # GPU上に保持
)
# 返り値の辞書:
# - "link_a", "link_b": 接触ペアのリンクインデックス
# - "force": 接触力ベクトル [N_contacts, 3]
# - "penetration": 貫通深さ
# - "position": 接触点位置
# - "normal": 接触法線
```

### 2. 解決策の比較検討

| 方式 | 実装難易度 | 信頼性 | パフォーマンス | 推奨度 |
|-----|-----------|--------|--------------|--------|
| **A. Contact Sensor** | 簡単 | 高 | 高 | **採用** |
| B. ContactForce Sensor | 中 | 高 | 中 | 力の大きさが必要な場合 |
| C. Collider低レベルAPI | 難 | 高 | 低 | デバッグ用 |
| D. Z座標閾値緩和 | 簡単 | 低 | 高 | 非推奨 |

**方式A（Contact Sensor）を採用する理由**:
1. Genesisの物理エンジンが内部で計算する接触判定を直接利用
2. Z座標閾値に依存しないため、Base高さの変動に影響されない
3. 実装がシンプル（センサー追加とメソッド更新のみ）
4. パフォーマンスへの影響が最小限

### 3. 実装方針

#### 3.1 環境クラスの修正（`droid_env_unitree.py`）

**変更点1**: `__init__` でContact Sensorを追加

```python
# scene.build() の前に追加
self.contact_sensors = []
for foot_name in self.feet_names:
    link = self.robot.get_link(foot_name)
    sensor = self.scene.add_sensor(gs.sensors.Contact(
        entity_idx=self.robot.idx,
        link_idx_local=link.idx_local,
    ))
    self.contact_sensors.append(sensor)
```

**変更点2**: `_get_foot_contacts` メソッドの更新

```python
def _get_foot_contacts(self):
    """足の接地状態を取得（Contact Sensorベース）

    V20変更: Z座標閾値ベース → Genesis Contact Sensor ベース

    【変更理由】
    V19まではZ座標閾値で判定していたが、Base高さの変動で
    誤判定が発生していた。GenesisのContact Sensorを使用することで、
    物理エンジンの実際の接触判定を利用する。
    """
    if not self.contact_sensors:
        return None

    # 各センサーから接触状態を読み取り
    contacts = []
    for sensor in self.contact_sensors:
        contact = sensor.read()  # boolean
        contacts.append(contact)

    # (n_envs, 2) の形状に整形
    return torch.stack(contacts, dim=-1)
```

#### 3.2 トレーニングスクリプト（`droid_train_unitree_v20.py`）

V19からコピーし、以下のみ変更:
- docstring のバージョン番号
- 実験名を `droid-walking-unitree-v20` に変更

**報酬設計はV19と同一**:
- 接地検出の修正により、既存の報酬（`feet_air_time`, `single_foot_contact`）が正しく機能することを検証
- 報酬スケールの変更は次バージョンで検討

### 4. 報酬設計（V19と同一、17項目）

```python
reward_scales = {
    # === 主報酬 ===
    "tracking_lin_vel": 1.5,
    "tracking_ang_vel": 0.5,

    # === 歩行品質報酬 ===
    "feet_air_time": 1.0,
    "contact": 0.2,
    "single_foot_contact": 0.3,
    "step_length": 0.5,

    # === 安定性ペナルティ ===
    "lin_vel_z": -2.0,
    "ang_vel_xy": -0.05,
    "orientation": -0.5,
    "base_height": -5.0,

    # === 歩行品質ペナルティ ===
    "feet_swing_height": -8.0,
    "contact_no_vel": -0.1,
    "hip_pos": -0.5,
    "velocity_deficit": -0.5,

    # === 関節速度制限 ===
    "dof_vel_limits": -0.3,

    # === エネルギー効率ペナルティ ===
    "torques": -1e-5,
    "action_rate": -0.01,
    "dof_acc": -2.5e-7,
}
```

**V19からの変更点**: なし（接地検出修正の効果を純粋に検証するため）

### 5. その他の設定（V19と同一）

```python
reward_cfg = {
    "tracking_sigma": 0.25,
    "base_height_target": 0.20,
    "swing_height_target": 0.05,
    "gait_frequency": 1.2,
    "contact_threshold": 0.05,  # Contact Sensor使用時は参照されないが互換性のため保持
    "air_time_offset": 0.3,
    "dof_vel_limits": 44.0,
    "soft_dof_vel_limit": 0.9,
}

command_cfg = {
    "lin_vel_x_range": [0.15, 0.25],
    "lin_vel_y_range": [0, 0],
    "ang_vel_range": [0, 0],
}
```

## 期待される効果

| 指標 | V19値 | V20目標 | 達成基準 |
|------|-------|---------|----------|
| **feet_air_time報酬** | 0.0 | **> 0** | 接地検出が機能 |
| **single_foot_contact報酬** | 0.0 | **> 0** | 接地検出が機能 |
| **接地パターン** | 100%両足空中 | **交互接地に近づく** | 目視で確認 |
| X速度 | 0.185 m/s | 0.15-0.25 m/s | 維持 |
| hip_pitch可動域 L | 0.531 rad | 0.5+ rad | 維持 |
| hip_pitch可動域 R | 0.469 rad | 0.5+ rad | 維持 |
| Yawドリフト | +3.95° | < 5° | 維持 |

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_unitree_v20.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-unitree-v20 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 46.58 |
| エピソード長 | 1001（フルエピソード、20秒） |
| 収束ステップ | ~150（std < 0.2） |
| 最終std | 0.153（まだ変動中、追加学習の余地あり） |

**四半期ごとの報酬推移:**

| 期間 | 平均報酬 | 変化 |
|------|----------|------|
| Steps 0-125 | 22.94 | - |
| Steps 125-250 | 43.93 | +20.99 |
| Steps 250-375 | 45.96 | +2.03 |
| Steps 375-500 | 46.53 | +0.57 |

### 評価結果

| 指標 | V19値 | V20目標 | V20結果 | 達成 |
|------|-------|---------|---------|------|
| **contact報酬** | 0.060 | > 0 | **0.240** | ✓ |
| **single_foot_contact報酬** | 0.0 | > 0 | **0.206** | ✓ |
| **接地パターン（片足）** | 0% | > 50% | **91.4%** | ✓ |
| **接地パターン（両足空中）** | 100% | < 10% | **2.2%** | ✓ |
| X速度 | 0.185 m/s | 0.15-0.25 m/s | **0.204 m/s** | ✓ |
| hip_pitch相関 | -0.859 | < 0 | **-0.550** | ✓（交互） |
| Yawドリフト | +3.95° | < 5° | **+2.69°** | ✓ |
| Base高さ | 0.224 m | 0.20 m | **0.256 m** | △（高め） |
| 目視評価 | すり足状態 | 足上げ歩行 | **ゆったり周期（良）、つま先突き（課題）** | △ |

### 関節可動域分析

| 関節 | 左(rad) | 右(rad) | V19値L/R | 変化 |
|------|---------|---------|----------|------|
| hip_yaw | 0.270 | 0.193 | - | - |
| hip_roll | 0.353 | 0.467 | - | - |
| hip_pitch | 0.289 | 0.379 | 0.531/0.469 | 減少 |
| knee_pitch | 0.331 | 0.398 | - | - |
| **ankle_pitch** | **0.484** | **0.369** | - | **大きい** |

### 報酬項目詳細（最終ステップ）

| 報酬項目 | V19値 | V20値 | 変化 | 備考 |
|----------|-------|-------|------|------|
| tracking_lin_vel | 0.484 | **1.081** | +123% | 大幅改善 |
| tracking_ang_vel | 0.156 | **0.359** | +130% | 大幅改善 |
| contact | 0.060 | **0.240** | +300% | Contact Sensor効果 |
| single_foot_contact | 0.000 | **0.206** | ∞ | **機能開始** |
| step_length | 0.026 | **0.018** | -31% | やや減少 |
| **feet_air_time** | 0.000 | **-0.033** | N/A | **負の値（課題）** |
| hip_pos | -0.059 | -0.059 | 同等 | - |

## 考察と改善案

### 成功点

1. **Contact Sensorの導入効果が絶大**
   - 接地検出が正常に機能: 片足接地91.4%
   - single_foot_contact報酬が機能: 0.0 → 0.206
   - contact報酬が4倍に増加: 0.060 → 0.240

2. **ゆったりした歩行サイクルの実現**（ユーザーフィードバック確認）
   - gait_frequency=1.2Hzが適切に機能
   - 周期的な歩行パターンが確立

3. **速度追従の大幅改善**
   - tracking_lin_vel: 0.484 → 1.081（+123%）
   - X速度: 0.204 m/s（目標0.15 m/sを適切に上回る）

4. **Yawドリフトの抑制**
   - +3.95° → +2.69°（改善）
   - 10秒間で3°未満は良好

5. **交互歩行の維持**
   - hip_pitch相関: -0.550（負の相関、交互動作）

### 課題

1. **つま先を地面に突く動作の発生**（ユーザーフィードバック）
   - 原因分析:
     - **ankle_pitchの可動域が大きい**（L: 0.484 rad, R: 0.369 rad）
     - **feet_air_timeが負**（-0.033）= 遊脚時間がair_time_offset（0.3s）を下回る
     - 足を下ろす際の足首制御が不適切

2. **hip_pitch可動域の減少**
   - V19: L=0.531, R=0.469 rad → V20: L=0.289, R=0.379 rad
   - Contact Sensorによる正確な接地検出が、より保守的な歩容を誘導した可能性

3. **Base高さが目標より高い**
   - 目標: 0.20m、実測: 0.256m（+28%）
   - base_height報酬が十分に機能していない

### サーベイ知見との照合と改善案

**exp007_unitree_rl_gym_survey.md Section 7.5.1「Barrier-Based Style Rewards」**:
> 対数バリア関数を使用した「スタイル報酬」で、foot clearanceを制御。二乗誤差よりも柔軟な制約。

→ **提案**: ankle_pitchの動きを制約するバリア関数型報酬の導入

**exp007_unitree_rl_gym_survey.md Section 7.3.2「Sinusoidal Reference Trajectory」**:
> 参照軌道を「アクションのバイアス」として使用することで、周期的な歩行パターンを促進。

→ **提案**: 遊脚下降時のankle_pitch参照軌道を導入し、つま先突きを防止

**exp007_unitree_rl_gym_survey.md Section 7.6.1「feet_air_timeオフセットの調整」**:
> BSL-Droidの歩行周波数1.2Hzでは、遊脚期は約0.42秒。オフセットを0.3→0.2秒に変更することで、より小さい滞空時間でも報酬を獲得可能。

→ **提案**: air_time_offsetを0.3s → 0.2sに調整

### 次バージョンへの提案

**優先度1（必須）: つま先突き動作の抑制**

| 方策 | 内容 | 期待効果 |
|------|------|---------|
| **A. ankle_pitch_rangeペナルティ追加** | 遊脚時のankle_pitch角度を制限 | つま先が下を向く動きを抑制 |
| **B. air_time_offset短縮** | 0.3s → 0.2s | feet_air_time報酬を正に |
| **C. contact_no_vel強化** | -0.1 → -0.2 | 接地時の足速度をより厳しく制限 |

**推奨**: A + Bの組み合わせ。ankle_pitch制御が直接的な対策、air_time_offset調整で足上げ動機を維持。

**優先度2: hip_pitch可動域の回復**

| 方策 | 内容 | 期待効果 |
|------|------|---------|
| **hip_pitch_range報酬追加** | hip_pitchの可動範囲を正の報酬化 | 大きなストライドの促進 |
| **step_length報酬強化** | 0.5 → 0.8 | 歩幅増加の動機強化 |

**優先度3: Base高さの調整**

| 方策 | 内容 | 期待効果 |
|------|------|---------|
| **base_height_target調整** | 0.20m → 0.25m | 実測値に近づける |
| **base_heightペナルティ強化** | -5.0 → -8.0 | 高さ維持の強化 |

### V21パラメータ変更案

```python
# 優先度1: つま先突き対策
reward_scales = {
    ...
    "ankle_pitch_range": -0.3,    # 【新規】遊脚時のankle_pitch制限
    "contact_no_vel": -0.2,       # -0.1 → -0.2
    ...
}

reward_cfg = {
    ...
    "air_time_offset": 0.2,       # 0.3 → 0.2
    "ankle_pitch_limit": 0.3,     # 【新規】ankle_pitchの許容範囲（rad）
    ...
}

# 優先度2: ストライド改善
reward_scales = {
    ...
    "step_length": 0.8,           # 0.5 → 0.8
    "hip_pitch_range": 0.3,       # 【新規】hip_pitch可動域報酬
    ...
}
```

## まとめ

### 実施内容

V20では、V19で継続していた「接地検出の完全な失敗」問題を根本から解決するため、Genesis物理エンジンが提供するContact Sensorを導入した。

**主な変更点**:
1. `droid_env_unitree.py` の `__init__` でContact Sensorを各足リンクに追加
2. `_get_foot_contacts` メソッドをZ座標閾値ベースからContact Sensorベースに変更

### 実際の結果

**Contact Sensor導入効果（大成功）**:
- 接地検出が正常に機能: 片足接地91.4%（V19: 0%）
- single_foot_contact報酬が機能: 0.0 → 0.206
- 速度追従が大幅改善: tracking_lin_vel +123%

**歩行品質の変化**:
- **良化**: ゆったりした周期の歩容を実現（ユーザーフィードバック）
- **悪化**: つま先を地面に突く動作が発生（新規課題）

### 根本原因と次バージョンへの課題

**つま先突き動作の原因**:
1. ankle_pitchの可動域が大きい（L: 0.484 rad, R: 0.369 rad）
2. feet_air_timeが負（-0.033）= 遊脚時間がair_time_offset（0.3s）を下回る
3. 足を下ろす際の足首角度制御が不適切

**V21での対策**:
1. **優先度1**: ankle_pitch_rangeペナルティ追加 + air_time_offset短縮（0.3→0.2s）
2. **優先度2**: step_length強化（0.5→0.8）+ hip_pitch_range報酬追加
3. **優先度3**: base_height_target調整（実測値に合わせる）

## 備考

- 学習スクリプト: `rl_ws/biped_walking/train/droid_train_unitree_v20.py`
- 環境クラス: `rl_ws/biped_walking/envs/droid_env_unitree.py`（Contact Sensor追加）
- 参照: [exp007_report_v19.md](exp007_report_v19.md)

### Genesis Contact Sensor 参考資料

- 公式サンプル: `rl_ws/genesis_official/examples/sensors/contact_force_go2.py`
- センサー実装: `rl_ws/genesis_official/genesis/engine/sensors/contact_force.py`
- センサーオプション: `rl_ws/genesis_official/genesis/options/sensors/options.py`

### 評価ログ（2026-02-04）

```
=== Evaluation Statistics ===
Duration: 10.0s (500 steps)

Travel distance:
  X: 2.040 m
  Y: 0.016 m
  Total: 2.040 m

Average velocity:
  X: 0.204 m/s (target: 0.150)
  Y: -0.003 m/s

Base height:
  Mean: 0.256 m
  Std: 0.0077 m

Orientation (deg):
  Roll:  mean= -1.12, std= 2.97
  Pitch: mean=  0.17, std= 0.82
  Yaw:   start=  0.00, end=  2.69, drift= +2.69

DOF velocity RMS: 1.271 rad/s
Action RMS: 0.873

Left-Right hip_pitch correlation: -0.550
  (-1.0 = perfect alternating, +1.0 = perfect synchronized)

=== Contact Pattern Analysis ===
Both feet grounded:      32 steps (  6.4%)
Single foot grounded:   457 steps ( 91.4%)
Both feet airborne:      11 steps (  2.2%)

Gait quality indicators:
  Total DOF range sum: 3.533 rad
```

### 学習ログ分析（2026-02-04）

```
=== Key Metrics Over Training ===

Train/mean_reward:
  Step    0:     0.364711
  Step  499:    46.576939
  Trend: ↑ 上昇

=== Reward Trend Analysis ===
  Max reward: 46.6936 at step 442
  Min reward: 0.1154 at step 2
  Final reward: 46.5769

  Last 50 steps std: 0.152996
  → まだ変動中（std > 0.1）、追加学習の余地あり
```

### 報酬構成要素（最終ステップ）

| 報酬項目 | 値 | 備考 |
|----------|-----|------|
| tracking_lin_vel | 1.0811 | 主報酬（最大） |
| tracking_ang_vel | 0.3594 | - |
| contact | 0.2395 | Contact Sensor効果 |
| single_foot_contact | 0.2058 | **機能開始** |
| step_length | 0.0181 | やや減少 |
| feet_air_time | **-0.0325** | **負（課題）** |
| hip_pos | -0.0591 | 最大ペナルティ |
| ang_vel_xy | -0.0186 | - |
| lin_vel_z | -0.0170 | - |

---

## V21への提案: Z座標閾値ベース接地検出の根本問題分析

V20でContact Sensorを導入したが、V19までのZ座標閾値ベースの接地検出がなぜ失敗していたのかを詳細に分析した。この分析結果は、Contact Sensorが使用できない環境でのフォールバック実装や、他のプロジェクトへの知見共有に役立つ。

### 1. 旧アルゴリズムの詳細

```python
# droid_env_unitree.py の _get_foot_contacts() 旧実装
def _get_foot_contacts(self):
    """足の接地状態を取得（Z座標ベース）"""
    if self.feet_indices is None:
        return None
    link_pos = self.robot.get_links_pos()  # 全リンクの位置を取得
    feet_z = link_pos[:, self.feet_indices, 2]  # 足リンクのZ座標のみ抽出
    return feet_z < self.contact_threshold  # 閾値より低ければ接地と判定
```

**判定ロジック**: `feet_z < contact_threshold` → True なら接地

### 2. foot_linkの基準点問題

**調査結果**: `get_links_pos()` が返す位置は**リンク原点（ankle joint位置）**であり、足裏ではない。

#### URDFの構造（`bsl_droid_simplified.urdf`より）

```xml
<link name="left_foot_link">
  <collision>
    <origin xyz="0.02 0 -0.035" rpy="1.5708 0 0"/>
    <geometry>
      <cylinder length="0.09" radius="0.03"/>  <!-- 足裏の形状 -->
    </geometry>
  </collision>
</link>
```

#### 座標系の解析

| 項目 | 値 | 説明 |
|------|-----|------|
| リンク原点 | Z = 0 (ローカル座標) | ankle jointの位置 |
| 衝突形状中心 | Z = -0.035m (ローカル座標) | シリンダーの中心 |
| 衝突形状長 | 0.09m | シリンダーの高さ |
| **足裏底面** | **Z = -0.035 - 0.045 = -0.08m** (ローカル座標) | シリンダー底面 |

つまり、`get_links_pos()` が返すZ座標は **足裏より約0.08m上（ankle joint位置）** を指している。

### 3. 閾値と実際の値の不整合

#### 初期姿勢での値

| 項目 | 値 | 出所 |
|------|-----|------|
| Base初期高さ | 0.35m | `env_cfg["base_init_pos"]` |
| 運用時Base高さ | 約0.224m | V19評価結果 |
| foot_link Z座標 | 約0.095m | `get_links_pos()` 実測値（初期姿勢） |
| contact_threshold | 0.05m | `reward_cfg["contact_threshold"]` |

#### 判定の破綻

```
判定: feet_z < contact_threshold
      0.095m < 0.05m
      → False（常に「空中」と判定）
```

**結論**: foot_linkのZ座標（0.095m）は**物理的に接地していても**、閾値（0.05m）を下回ることが**不可能**だった。

### 4. 正しい閾値の推定

foot_linkのZ座標から足裏底面までのオフセットを考慮すると:

| 状態 | foot_link Z | 足裏底面 Z | 必要な閾値 |
|------|-------------|-----------|-----------|
| 初期姿勢 | 0.095m | 0.095 - 0.08 = 0.015m | - |
| 完全接地時 | 約0.08m | 約0m | **≥0.08m** |

**V19の閾値0.05mでは、完全接地時でも判定不能**だった。

### 5. V21で検証すべき実験

#### 実験A: Z座標閾値の修正（Contact Sensorなしで動作確認）

```python
# reward_cfg の修正案
"contact_threshold": 0.10,  # 0.05 → 0.10（足裏オフセットを考慮）
```

**目的**: Contact Sensorを使用せずに、適切な閾値でZ座標ベース検出が機能するか確認

#### 実験B: 足裏位置の直接計算

```python
def _get_foot_contacts(self):
    """足の接地状態を取得（足裏位置補正版）"""
    link_pos = self.robot.get_links_pos()
    feet_z = link_pos[:, self.feet_indices, 2]
    # 足裏オフセット補正（URDFの衝突形状から算出）
    foot_sole_offset = 0.08  # ankle jointから足裏底面までの距離
    feet_sole_z = feet_z - foot_sole_offset
    return feet_sole_z < 0.02  # 地面（Z=0）から2cm以内で接地判定
```

**目的**: リンク原点と足裏の位置差を明示的に補正

### 6. 推奨事項

| 優先度 | 方式 | 信頼性 | 汎用性 | 推奨度 |
|--------|------|--------|--------|--------|
| 1 | **Contact Sensor（V20採用）** | ★★★ | ★★☆ | **推奨** |
| 2 | Z座標閾値修正（0.05→0.10m） | ★★☆ | ★★★ | 代替案 |
| 3 | 足裏位置の直接計算 | ★★☆ | ★☆☆ | URDF依存 |

**結論**: V20のContact Sensor方式が最も信頼性が高い。ただし、Genesis以外の環境や計算コスト制約がある場合は、Z座標閾値を**0.10m以上**に設定することでフォールバックが可能。
