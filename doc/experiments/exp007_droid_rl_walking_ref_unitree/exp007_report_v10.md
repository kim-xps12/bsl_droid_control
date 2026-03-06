# V10: 足上げ改善と滑らかな歩行の実現

## 概要

V9は左右対称性とYawドリフトを大幅に改善したが、symmetry報酬の強化によりhip_pitch相関が同期方向に悪化し、足引きずりや断続的な速度推移が発生した。

V10では、V9レポートの考察と提案に基づき、以下の方針で改善を行う：

1. **symmetry報酬の緩和**：hip_pitch同期の副作用を軽減
2. **足上げ高さの増加**：swing_height_targetを引き上げ、引きずりを防止
3. **足引きずりペナルティの新規追加**：接地中の足の水平速度をペナルティ化
4. **動作滑らかさの改善**：action_rateペナルティを強化

## 前バージョンからの改善事項

V9レポート（exp007_report_v9.md）の「次バージョンへの提案」セクションに基づく改善：

### V9の根本原因分析

1. **symmetry報酬の副作用**
   - V9でsymmetryを0.3→1.0に強化したが、hip_pitch相関が-0.382→+0.605に悪化
   - 左右の関節角度が「同じ」であることを報酬化したため、hip_pitchの同期動作が強化された
   - 結果として「対称的だが同期」という状態に収束

2. **足上げ高さ不足**
   - swing_height_target: 0.03m（Unitreeは0.08m）
   - 足上げが不十分で、足を引きずる動作が発生

3. **足引きずり抑制機構の欠如**
   - feet_stumbleペナルティ未実装
   - 接地中の足の水平速度が大きくてもペナルティなし

4. **断続的な速度推移**
   - action_rate: -0.005（弱い）
   - アクション変化が抑制されず、カクカクした動作に

### V10パラメータ変更一覧

| パラメータ | V9値 | V10値 | 変更理由 |
|-----------|------|-------|---------|
| symmetry | 1.0 | **0.5** | hip_pitch同期を緩和 |
| swing_height_target | 0.03 | **0.05** | 足上げ高さ増加 |
| feet_swing_height | -5.0 | **-10.0** | 足上げペナルティ強化 |
| **feet_stumble** | (なし) | **-0.5** | 【新規】足引きずり抑制 |
| action_rate | -0.005 | **-0.01** | 滑らかさ向上 |

**報酬項目数**: 21 → 22（新規追加1）

## 設計詳細

### 1. symmetry報酬の緩和

V9ではsymmetryを1.0に強化したが、hip_pitchの同期動作を誘発した。V10では0.5に緩和し、hip_pitch逆相関の維持を優先する。

```python
"symmetry": 0.5,  # V9: 1.0 → V10: 0.5
```

**設計原理**：
- symmetry報酬は「左右の関節角度が同じ」を報酬化
- hip_pitchは交互歩行で逆位相が理想なので、強すぎると逆効果
- 0.5に緩和することで、hip_pitch_antiphase_v2との バランスを取る

### 2. 足上げ高さの増加

サーベイ（exp007_unitree_rl_gym_survey.md セクション3.2）の推奨に基づき、swing_height_targetを増加。

```python
"swing_height_target": 0.05,  # V9: 0.03 → V10: 0.05m
"feet_swing_height": -10.0,   # V9: -5.0 → V10: -10.0（2倍強化）
```

**設計原理**：
- Unitreeでは目標高さ0.08mを使用
- BSL-Droidは小型なので0.05mに設定
- ペナルティを2倍に強化し、足上げを強制

### 3. feet_stumbleペナルティの新規追加

接地中の足が水平方向に大きな速度を持つ場合にペナルティを与える。

```python
def _reward_feet_stumble(self):
    """足引きずりペナルティ（V10追加）"""
    contacts = self._get_foot_contacts()
    if contacts is None:
        return torch.zeros(...)

    # 水平方向の足速度（XY平面）
    vel_xy = torch.sqrt(self.feet_vel[:, :, 0]**2 + self.feet_vel[:, :, 1]**2)

    # 接地中の足のみを対象
    return torch.sum(vel_xy * contacts.float(), dim=1)
```

**reward_scales追加**：
```python
"feet_stumble": -0.5,  # 新規
```

**設計原理**：
- 接地中の足は地面に対して静止しているべき
- 水平速度が大きい = 足を引きずっている
- サーベイ2.5.4「feet_stumble」ペナルティを参考に実装

### 4. 動作滑らかさの改善

action_rateペナルティを強化し、アクション変化を抑制。

```python
"action_rate": -0.01,  # V9: -0.005 → V10: -0.01（2倍強化）
```

**設計原理**：
- V9では速度推移がカクカクしていた
- アクション変化率を強く抑制することで滑らかな動作を促進
- 急激なアクション変化を減らし、連続的な歩行サイクルを誘導

### V10報酬スケール一覧

```python
reward_scales = {
    # 主報酬
    "tracking_lin_vel": 1.5,
    "tracking_ang_vel": 1.0,
    # 歩行品質報酬
    "feet_air_time": 1.5,
    "contact": 0.3,
    "alive": 0.03,
    "single_foot_contact": 1.0,
    "symmetry": 0.5,               # ★緩和
    "hip_pitch_antiphase_v2": 0.8,
    "both_legs_active": 0.5,
    # 安定性ペナルティ
    "lin_vel_z": -2.0,
    "ang_vel_xy": -0.1,
    "orientation": -1.0,
    "base_height": -5.0,
    # 歩行品質ペナルティ
    "feet_swing_height": -10.0,    # ★強化
    "contact_no_vel": -0.1,
    "hip_pos": -0.5,
    "velocity_deficit": -2.0,
    "feet_stumble": -0.5,          # ★新規
    # エネルギー効率ペナルティ
    "torques": -1e-5,
    "action_rate": -0.01,          # ★強化
    "dof_acc": -2.5e-7,
    "dof_vel": -0.005,
}
# 報酬項目数: 22
```

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_unitree_v10.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-unitree-v10 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 98.12 |
| エピソード長 | 1001（最大） |
| 収束ステップ | 約200（step 200で93.2、以降は漸増） |
| 収束判定 | Last 50 steps std = 0.08（収束） |

**報酬推移（Quarter-wise）:**

| ステップ範囲 | 平均報酬 | 増分 |
|-------------|---------|------|
| 0-125 | 41.00 | - |
| 125-250 | 91.01 | +50.00 |
| 250-375 | 97.28 | +6.27 |
| 375-500 | 98.06 | +0.78 |

### 評価結果

| 指標 | V9値 | V10値 | 変化 | 評価 |
|------|------|-------|------|------|
| X速度 | 0.092 m/s | **0.006 m/s** | -93% | ❌ **大幅悪化**（静止ポリシー） |
| hip_pitch相関 | +0.605 | **+0.449** | -0.156 | △ 微改善（依然正） |
| hip_pitch L/R比 | 0.91倍 | **3.94倍** | +3.3倍 | ❌ **大幅悪化** |
| Yawドリフト | -30.74° | **+14.52°** | - | ✅ 改善 |
| Roll std | 2.48° | **2.43°** | -2% | → 維持 |
| 単足接地率 | 97.2% | **97.6%** | +0.4% | → 維持 |

### 姿勢・安定性

| 指標 | 値 |
|------|-----|
| Roll | mean=-5.27°, std=2.43° |
| Pitch | mean=0.50°, std=0.87° |
| Yaw drift | +14.52°（10秒間） |
| Base height | 0.220 m (std=0.012) |

### 関節動作分析

| 関節 | L range (rad) | R range (rad) | L vel std (rad/s) | R vel std (rad/s) |
|------|---------------|---------------|-------------------|-------------------|
| hip_pitch | 0.336 | 0.407 | **0.292** | **1.150** |
| hip_roll | 0.242 | 0.231 | 0.695 | 0.507 |
| knee | 0.503 | 0.588 | 0.814 | 0.859 |
| ankle_pitch | 0.347 | 0.306 | 0.487 | 0.769 |
| ankle_roll | 0.253 | 0.310 | 0.384 | 0.529 |

**重要な観察**:
- **hip_pitch速度stdが極端に非対称**（L: 0.292、R: 1.150、約4倍の差）
- V9のL/R比0.91から3.94に大幅悪化
- 動作量が全体的に極めて小さい（DOF range sum: 3.524 rad、V9より低下）

### 接地パターン

| パターン | ステップ数 | 割合 |
|----------|-----------|------|
| Both feet grounded | 6 | 1.2% |
| Single foot grounded | 488 | 97.6% |
| Both feet airborne | 6 | 1.2% |

### 目視観察（ユーザーフィードバック）

- **良い点**: なし
- **悪い点**:
  - その場で胴体をぐねらせるだけで前に進まない
  - 静止ポリシーへの回帰

## 考察と改善案

### V10の失敗原因分析

V10は**静止ポリシーへの回帰**という深刻な問題が発生した。原因を以下に分析する。

#### 1. ペナルティの累積効果（根本原因）

V10では複数のペナルティを同時に強化・追加したことで、**動くこと自体が大きなペナルティを受ける**状態になった。

| ペナルティ | V7（動作OK） | V8 | V9 | V10 | 累積効果 |
|-----------|-------------|-----|-----|-----|---------|
| ang_vel_xy | -0.05 | **-0.1** | -0.1 | -0.1 | 2倍 |
| orientation | -0.5 | **-1.0** | -1.0 | -1.0 | 2倍 |
| feet_swing_height | -5.0 | -5.0 | -5.0 | **-10.0** | 2倍 |
| action_rate | -0.005 | -0.005 | -0.005 | **-0.01** | 2倍 |
| feet_stumble | なし | なし | なし | **-0.5** | 新規 |

V7からV10までの累積で、主要ペナルティが全て**2倍以上に強化**されている。

#### 2. feet_stumbleの副作用

feet_stumbleは「接地中の足の水平速度」をペナルティ化するが、歩行中は接地脚も多少動くため、**歩行動作自体にペナルティがかかる**。結果として、静止が最適解となった。

#### 3. swing_height_targetの過剰引き上げ

0.03m → 0.05mへの引き上げは、BSL-Droidの脚長（約0.31m）に対して比較的大きな変化。達成困難な目標を設定したことで、feet_swing_heightペナルティが常に発生する状態になった。

#### 4. 報酬の絶対値vs歩行品質

V10の最終報酬（98.12）はV7（73.44）より高いが、**報酬最大化 ≠ 歩行品質最大化**である。ペナルティを回避するために静止するポリシーが、総報酬では高得点を獲得している。

### V7-V10のバージョン進化と教訓

| バージョン | X速度 | 報酬 | 教訓 |
|-----------|-------|------|------|
| V7 | **0.214 m/s** | 73.4 | ペナルティ適度、歩行成功 |
| V8 | 0.108 m/s | - | ang_vel_xy/orientation強化で速度低下 |
| V9 | 0.092 m/s | 110.8 | symmetry強化で対称性改善、速度微減 |
| V10 | **0.006 m/s** | 98.1 | ペナルティ累積で静止回帰 |

**重要な教訓**：
- ペナルティの強化は慎重に行う（一度に1-2項目まで）
- 複数のペナルティを同時に強化しない
- 「動いていた頃」の設定をベースに、最小限の調整を行う

### 次バージョン（V11）への提案

**方針**: 「動いていた頃の報酬に基づいて重みを適切に調整する」

V7をベースに、V9で有効だった要素（hip_pitch_antiphase_v2、both_legs_active、tracking_ang_vel強化）のみを追加する保守的なアプローチを提案する。

#### 提案1: V7ベース + V9の成功要素（推奨）

V7の報酬設定をベースに、V9で対称性改善に寄与した要素のみを追加。

```python
# V11: V7ベース + V9の成功要素
reward_scales = {
    # 主報酬（V7同等）
    "tracking_lin_vel": 1.5,
    "tracking_ang_vel": 0.5,  # V7レベルに戻す（Yaw制御はang_vel_rangeで対応）

    # 歩行品質報酬
    "feet_air_time": 1.5,
    "contact": 0.3,
    "alive": 0.03,
    "single_foot_contact": 1.0,
    "symmetry": 0.3,  # V7レベルに戻す
    "hip_pitch_antiphase_v2": 0.8,  # V9から継承（交互歩行改善）
    "both_legs_active": 0.5,  # V9から継承（両脚動作促進）

    # 安定性ペナルティ（V7レベル）
    "lin_vel_z": -2.0,
    "ang_vel_xy": -0.05,  # V7レベルに戻す
    "orientation": -0.5,  # V7レベルに戻す
    "base_height": -5.0,

    # 歩行品質ペナルティ（V7レベル）
    "feet_swing_height": -5.0,  # V7レベルに戻す
    "contact_no_vel": -0.1,
    "hip_pos": -0.5,
    "velocity_deficit": -2.0,
    # feet_stumble: 削除（静止誘発の可能性）

    # エネルギー効率ペナルティ（V7レベル）
    "torques": -1e-5,
    "action_rate": -0.005,  # V7レベルに戻す
    "dof_acc": -2.5e-7,
    "dof_vel": -0.005,
}
# swing_height_target: 0.03（V7レベルに戻す）
# ang_vel_range: [-0.2, 0.2]（V9から継承、Yaw制御訓練）
```

#### V11パラメータ変更一覧（V10からの差分）

| パラメータ | V10値 | V11提案値 | 変更理由 |
|-----------|-------|----------|---------|
| symmetry | 0.5 | **0.3** | V7レベルに戻す |
| tracking_ang_vel | 1.0 | **0.5** | V7レベルに戻す |
| ang_vel_xy | -0.1 | **-0.05** | V7レベルに戻す |
| orientation | -1.0 | **-0.5** | V7レベルに戻す |
| feet_swing_height | -10.0 | **-5.0** | V7レベルに戻す |
| swing_height_target | 0.05 | **0.03** | V7レベルに戻す |
| action_rate | -0.01 | **-0.005** | V7レベルに戻す |
| feet_stumble | -0.5 | **削除** | 静止誘発の可能性 |

**報酬項目数**: 22 → 21（削除1）

#### 提案2: max_iterationsの延長検討

V10が静止ポリシーに収束した一因として、学習が不十分な可能性も検討した。しかし、V10の学習曲線は200ステップ付近で収束しており、500→1000に延長しても改善は期待できない。**根本原因はペナルティ設計**であり、イテレーション数の問題ではない。

#### 報酬設計の階層的原則（サーベイ知見に基づく）

サーベイ（exp007_unitree_rl_gym_survey.md セクション6.2）に基づき、以下の階層構造を遵守する：

1. **主報酬**（最優先）：tracking_lin_vel > tracking_ang_vel
2. **歩行品質報酬**（次優先）：feet_air_time, contact, single_foot_contact
3. **補助報酬**：symmetry, hip_pitch_antiphase_v2, both_legs_active
4. **ペナルティ**（抑制のみ）：絶対値は主報酬より小さく設定

V10の失敗は、ペナルティの累積が主報酬を上回り、「動かない」が最適解になったことに起因する。

### 削除すべき要素の考察

V3-V4（動作成功）とV7-V10（複雑化・劣化）を比較し、削除または緩和すべき要素を分析した。

#### 報酬項目数の推移と成績

| バージョン | 報酬項目数 | X速度 | 状態 |
|-----------|----------|-------|------|
| V3 | 16 | 0.151 m/s | ✓ 動作 |
| V4 | 17 | 0.192 m/s | ✓ 動作 |
| V7 | 22 | 0.214 m/s | ✓ 動作 |
| V10 | 22 | 0.006 m/s | ❌ 静止 |

V3-V4は**16-17項目**で0.15-0.19 m/sを達成しており、V7-V10の22項目は過剰複雑化である。

#### 削除推奨の要素（6項目）

1. **symmetry報酬** → **削除推奨**
   - V3-V4には存在しない
   - V9で1.0に強化時、hip_pitch相関が-0.382→+0.605に悪化
   - 「左右対称」は「hip_pitch同期」を誘発し、交互歩行と矛盾

2. **alternating_gait / hip_pitch_antiphase_v2** → **削除推奨**
   - V7で追加、実装上の問題が報告（hip_pitch相関0.781→0.828と悪化）
   - 複雑な報酬設計で意図した効果が得られていない
   - V4はsingle_foot_contactのみで交互歩行の基礎を達成

3. **both_legs_active** → **削除推奨**
   - V9で追加
   - 効果が不明確で、報酬複雑化の一因

4. **step_length** → **削除推奨**
   - V7で追加
   - 歩幅拡大の効果が実証されていない

5. **foot_flat** → **削除または大幅緩和**
   - V7で-3.0として追加
   - ペナルティ累積の一因

6. **feet_stumble** → **削除**
   - V10で新規追加
   - 静止ポリシー回帰の直接原因の一つ

#### ペナルティ値の緩和推奨

| ペナルティ | V3-V4値 | V10値 | 推奨値 |
|-----------|---------|-------|--------|
| ang_vel_xy | -0.05 | -0.1 | **-0.05** |
| orientation | -0.5 | -1.0 | **-0.5** |
| feet_swing_height | -5.0 | -10.0 | **-5.0** |
| action_rate | -0.005/-0.01 | -0.01 | **-0.005** |
| dof_vel | -0.001 | -0.005 | **-0.001または削除** |

#### 最小有効報酬セット（V3-V4パターン）

V3-V4の成功に基づく**15-16項目**の簡潔な設計:

```python
# 最小有効報酬セット（15項目）
reward_scales = {
    # 主報酬（3項目）
    "tracking_lin_vel": 1.5,
    "tracking_ang_vel": 0.5,
    "velocity_deficit": -2.0,  # V6で効果実証

    # 歩行品質報酬（4項目）
    "feet_air_time": 1.5,
    "contact": 0.2,
    "single_foot_contact": 0.8,  # 交互歩行の核心
    "alive": 0.03,

    # 安定性ペナルティ（4項目、V3値）
    "lin_vel_z": -2.0,
    "ang_vel_xy": -0.05,
    "orientation": -0.5,
    "base_height": -5.0,

    # 歩行品質ペナルティ（2項目）
    "feet_swing_height": -5.0,
    "contact_no_vel": -0.1,

    # エネルギー効率（2項目）
    "torques": -1e-5,
    "dof_acc": -2.5e-7,
}
```

この設計は、V5-V10で追加された以下の要素を全て削除している：
- symmetry（hip_pitch同期誘発）
- alternating_gait / hip_pitch_antiphase_v2（実装不良）
- both_legs_active（効果不明）
- step_length（効果不明）
- foot_flat（ペナルティ過剰）
- feet_stumble（静止誘発）
- dof_vel（V3になく必須ではない）
- hip_pos（V4にあるが、削除可能）
- action_rate（V3値-0.01はやや強いが、V4値-0.005が適切）

#### 結論：シンプル化 vs 複雑化

| アプローチ | 報酬項目数 | 結果 |
|-----------|----------|------|
| V3-V4（シンプル） | 16-17 | 0.15-0.19 m/s |
| V7-V10（複雑） | 22 | 不安定、最終的に0.006 m/s |

**教訓**：報酬項目を増やすほど良くなるわけではない。むしろ、項目間の相互作用が予測困難になり、静止ポリシーや異常動作のリスクが増加する。

V11では、V7ベースの提案1に加え、**V3-V4パターンへの回帰**（提案3）も有力な選択肢として検討すべきである。

### Unitree RL Gymとの徹底比較（追加調査）

`ref/unitree_rl_gym`のコードを精査し、BSL-Droid V10との差異を分析した。

#### Unitree G1/H1の実際の報酬設定

`legged_gym/envs/g1/g1_config.py`および`h1/h1_config.py`から抽出：

| 報酬項目 | G1/H1値 | BSL-Droid V10値 | 差異分析 |
|---------|---------|-----------------|----------|
| tracking_lin_vel | **1.0** | 1.5 | V10がやや高い |
| tracking_ang_vel | **0.5** | 1.0 | **V10が2倍**（過剰） |
| lin_vel_z | -2.0 | -2.0 | 同じ |
| ang_vel_xy | **-0.05** | -0.1 | **V10が2倍**（過剰） |
| orientation | -1.0 | -1.0 | 同じ |
| base_height | -10.0 | -5.0 | V10が緩い |
| dof_acc | -2.5e-7 | -2.5e-7 | 同じ |
| action_rate | **-0.01** | -0.01 | 同じ |
| alive | **0.15** | 0.03 | Unitreeが高い（静止リスク注意） |
| contact | **0.18** | 0.3 | 類似 |
| feet_swing_height | **-20.0** | -10.0 | **Unitreeが2倍** |
| feet_air_time | **0.0（無効）** | 1.5 | **重大な差異** |
| dof_vel | -1e-3 | -0.005 | V10が5倍強い |

#### 重要な発見

```
【発見1】Unitree G1/H1はfeet_air_timeを使用していない
  → G1: feet_air_time = 0.0（無効化）
  → 報酬項目数: 約13-14項目（V10の22項目の約半分）

【発見2】Unitreeは対称性報酬を持たない
  → symmetry: 存在しない
  → alternating_gait: 存在しない
  → hip_pitch_antiphase: 存在しない
  → 交互歩行はcontact報酬（歩行フェーズ整合性）のみで達成

【発見3】Unitreeはcontact報酬で歩行フェーズを学習
  → leg_phase < 0.55: スタンス相（接地期待）
  → leg_phase >= 0.55: スイング相（非接地期待）
  → XORの否定で整合性を報酬化（シンプルで効果的）

【発見4】Unitreeのfeet_swing_heightは-20.0と非常に強い
  → ただしサーベイ6.2節ではBSL-Droidには-5.0を推奨
  → ロボットサイズに応じたスケーリングが必要
```

#### Unitree報酬設計の核心原理

`legged_gym/envs/base/legged_robot.py`の報酬関数実装を分析：

1. **シンプルさ優先**: 13-14項目で実機歩行を達成
2. **contact報酬が交互歩行の核心**: 歩行フェーズとの整合性のみで十分
3. **対称性報酬は不使用**: symmetry/alternating_gaitは不要
4. **feet_air_timeは無効**: ストライド報酬は使わない

### 最新研究からの知見

Web検索およびサーベイから得られた知見：

#### 静止ポリシー問題の既知性

[STRIDE (arXiv 2025)](https://arxiv.org/html/2502.04692v1) より：
> "Traditional reward shaping methods employ a **static, fixed-weight linear combination**. However, such an approach has inherent limitations: **weights remain constant** throughout training."

[Two-Layered Reward RL (MDPI 2025)](https://www.mdpi.com/2227-7390/13/21/3445) より：
> "In complex tasks such as humanoid motion tracking, **traditional static weighted reward functions struggle** to adapt to shifting learning priorities."

→ **核心的問題**: V7-V10で報酬項目を増やし続けたが、項目間の相互作用が予測困難になり、逆効果を招いた。

#### 実世界ヒューマノイド歩行の成功例

[Real-world humanoid locomotion (Science Robotics 2024)](https://www.science.org/doi/10.1126/scirobotics.adi9579) より：
> "The controller is a causal transformer that takes the history of proprioceptive observations and actions as input. The model is trained with **large-scale reinforcement learning** on thousands of randomized environments in simulation and deployed to the real world in a **zero-shot fashion**."

→ **成功の鍵**: シンプルな報酬設計 + 大規模並列訓練 + ドメインランダマイゼーション

#### 片足接地報酬の重要性

[Revisiting Reward Design (arXiv 2024)](https://arxiv.org/html/2404.19173v1) より（サーベイ8.2節に記載）：
> "The most reliable and unconstrained way to produce walking instead of hopping is via the **single foot contact reward**, which does not require tuning."

→ V3-V4の`single_foot_contact`が交互歩行の核心であり、symmetry/alternating_gait/hip_pitch_antiphaseは不要

### 提案3: 最小有効報酬セット（推奨）

Unitree原則 + V3-V4実績に基づく**15項目**の設計：

```python
# V11: 最小有効報酬セット
reward_scales = {
    # ============================================================
    # 【主報酬】Unitreeと同等
    # ============================================================
    "tracking_lin_vel": 1.5,      # V3-V4で実証済み
    "tracking_ang_vel": 0.5,      # Unitreeと同じ（V10の1.0は過剰）

    # ============================================================
    # 【歩行品質報酬】Unitree方式 + V3-V4実証済み要素
    # ============================================================
    "contact": 0.2,               # Unitree: 0.18、歩行フェーズ整合性
    "single_foot_contact": 0.8,   # V4で実証済み、交互歩行の核心
    "feet_air_time": 1.5,         # V3-V4で実証済み
    "alive": 0.03,                # 控えめに設定（Unitreeの0.15は静止誘発リスク）

    # ============================================================
    # 【安定性ペナルティ】Unitree値を使用
    # ============================================================
    "lin_vel_z": -2.0,            # Unitreeと同じ
    "ang_vel_xy": -0.05,          # ★Unitree値に戻す（V10: -0.1は過剰）
    "orientation": -0.5,          # サーベイ6.2推奨値（小型ロボット向け緩和）
    "base_height": -5.0,          # サーベイ6.2推奨値

    # ============================================================
    # 【歩行品質ペナルティ】
    # ============================================================
    "feet_swing_height": -5.0,    # サーベイ6.2推奨値（Unitreeの-20.0は過剰）
    "contact_no_vel": -0.1,       # Unitreeと同等
    "velocity_deficit": -2.0,     # V6で効果実証済み

    # ============================================================
    # 【エネルギー効率ペナルティ】
    # ============================================================
    "torques": -1e-5,             # Unitreeと同等
    "dof_acc": -2.5e-7,           # Unitreeと同等
}

# 報酬項目数: 15（V10の22から7項目削減）
```

#### 削除する要素（7項目）と根拠

| 削除要素 | 理由 | 根拠 |
|---------|------|------|
| **symmetry** | hip_pitch同期誘発 | V9で実証（相関+0.605に悪化）、Unitreeになし |
| **hip_pitch_antiphase_v2** | 効果なし | V7-V9で悪化継続、Unitreeになし |
| **both_legs_active** | 効果不明 | Unitreeになし |
| **step_length** | 効果未実証 | Unitreeになし |
| **foot_flat** | ペナルティ過剰 | Unitreeになし |
| **feet_stumble** | 静止誘発 | V10で実証 |
| **action_rate** | ペナルティ累積 | V3-V4では-0.005または不要 |
| **dof_vel** | V3になく不要 | ペナルティ累積の一因 |
| **hip_pos** | 効果限定的 | V3になし |

#### 追加調整パラメータ

```python
reward_cfg = {
    "tracking_sigma": 0.10,         # V6で効果実証（静止回避）
    "base_height_target": 0.20,     # BSL-Droid向け
    "swing_height_target": 0.03,    # V3-V4値に戻す（V10の0.05は過剰）
    "gait_frequency": 1.0,          # V4値
    "contact_threshold": 0.08,      # V4で修正済み
    "air_time_offset": 0.25,        # デフォルト
}

command_cfg = {
    "lin_vel_x_range": [0.15, 0.25],  # V7値（動作実績あり）
    "ang_vel_range": [0, 0],           # まずは直進のみ
}
```

### V11設計の根拠まとめ

#### 1. 実績に基づく設計

| バージョン | 報酬項目数 | X速度 | 状態 |
|-----------|----------|-------|------|
| V3 | **16** | 0.151 m/s | ✓ 動作 |
| V4 | **17** | 0.192 m/s | ✓ 動作 |
| V7 | 22 | 0.214 m/s | ✓ 動作（ただし以降悪化） |
| V10 | 22 | 0.006 m/s | ❌ 静止 |

→ **16-17項目で十分な性能を達成**

#### 2. Unitreeの成功パターンに準拠

- Unitree G1/H1は**13-14項目**で実機歩行を達成
- 対称性報酬・交互歩行報酬は**不使用**
- `contact`報酬だけで歩行フェーズを学習

#### 3. 最新研究の知見を反映

- [STRIDE (2025)](https://arxiv.org/html/2502.04692v1): 固定重み報酬の限界
- [Science Robotics 2024](https://www.science.org/doi/10.1126/scirobotics.adi9579): シンプルな報酬で実世界歩行
- [IJRR 2025](https://journals.sagepub.com/doi/full/10.1177/02783649241285161): 汎用的二脚制御

#### 4. ペナルティ累積効果の回避

V10の失敗原因は「複数ペナルティの同時強化」。最小構成に戻すことで、この問題を根本的に解決。

### V11の成功基準

| 指標 | V10値 | V11目標 | 判定基準 |
|------|-------|---------|---------|
| X速度 | 0.006 m/s | **> 0.15 m/s** | V3-V4レベルに回復 |
| hip_pitch相関 | +0.449 | **< 0** | 交互歩行の回復 |
| 報酬項目数 | 22 | **15** | 簡素化 |
| エピソード長 | 1001 | **> 900** | 安定性維持 |

## まとめ

V10は**静止ポリシーへの回帰**という深刻な失敗に終わった：

- X速度: 0.092 m/s → **0.006 m/s**（93%減）
- 原因: ペナルティの累積効果（5項目を同時に強化/追加）
- 報酬最大化（98.12）≠ 歩行品質最大化

**重要な教訓**：
1. ペナルティの強化は一度に1-2項目まで
2. 複数のペナルティを同時に強化しない
3. 「動いていた頃」の設定をベースに最小限の調整を行う
4. 報酬の絶対値ではなく、実際の動作で評価する

V11では、V7（0.214 m/s達成）の設定をベースに、V9で有効だった交互歩行改善要素（hip_pitch_antiphase_v2、both_legs_active）のみを追加する保守的なアプローチを推奨する。

## 今後の実験への申し送り（重要）

### 報酬設計の原則

V1-V10の試行錯誤から得られた教訓を、今後の実験で必ず遵守すること。

#### 1. 報酬項目の追加は最終手段

```
❌ 問題発生 → 新しい報酬項目を追加
✅ 問題発生 → 既存項目の重みを調整 → 効果なし → 項目の削除を検討 → 最後に追加を検討
```

**根拠**：
- V3-V4（16-17項目）で0.15-0.19 m/sを達成
- V7-V10（22項目）で不安定化、最終的に静止ポリシー
- Unitree G1/H1は13-14項目で実機歩行を達成
- **項目を増やすほど良くなるわけではない**

#### 2. 新規項目追加前のチェックリスト

新しい報酬項目を追加する前に、以下を必ず確認：

- [ ] **既存項目の重み調整で対応できないか？**
- [ ] **不要な項目を削除できないか？**
- [ ] **Unitree RL Gymに同等の項目が存在するか？**
- [ ] **V3-V4（動作成功時）に存在した項目か？**
- [ ] **追加による副作用（ペナルティ累積、相互干渉）を検討したか？**
- [ ] **一度に追加するのは1項目のみか？**

#### 3. 避けるべきパターン

| パターン | 問題 | 実例 |
|---------|------|------|
| 複数項目の同時追加/強化 | 相互作用が予測困難 | V10: 5項目同時変更 → 静止 |
| 対称性報酬の強化 | hip_pitch同期誘発 | V9: symmetry 1.0 → 相関悪化 |
| ペナルティの累積強化 | 動くこと自体がペナルティに | V7→V10でペナルティ倍増 |
| 複雑な報酬設計 | 意図しない効果 | alternating_gait → 効果なし |

#### 4. 推奨するアプローチ

1. **シンプルさを維持**: 15-17項目を上限とする
2. **実績ベース**: V3-V4、Unitreeの設定を参照
3. **1変更1検証**: 一度に変更するのは1-2項目まで
4. **削除優先**: 追加より削除を優先的に検討
5. **ペナルティ控えめ**: 主報酬を上回らないよう設定

#### 5. 問題別の対処指針

| 問題 | 推奨アプローチ | 避けるべきアプローチ |
|------|--------------|-------------------|
| 交互歩行が不十分 | single_foot_contactの重み調整 | symmetry/alternating_gait追加 |
| 足上げ不足 | swing_height_target調整 | feet_swing_height強化 |
| Yawドリフト | tracking_ang_vel調整 | 新規Yawペナルティ追加 |
| 動作が滑らかでない | action_rate微調整 | 複数ペナルティ強化 |
| 静止ポリシー | ペナルティ緩和、velocity_deficit確認 | 新規報酬追加 |

### この申し送りの背景

V1-V10で報酬項目を16→22に増加させた結果：
- **報酬最大化（98.12）≠ 歩行品質最大化**
- 項目間の相互作用により意図しない挙動が発生
- 最終的に静止ポリシーという最悪の結果に

Unitree RL Gymおよび最新研究（STRIDE 2025, Science Robotics 2024）も、**シンプルな報酬設計**の重要性を強調している。

今後の実験では、**「追加」ではなく「調整」と「削除」**を優先し、報酬設計の複雑化を避けること。

## 備考

- 学習ログ：`rl_ws/logs/droid-walking-unitree-v10/`
- 新規報酬関数の実装：`biped_walking/envs/droid_env_unitree.py`（`_reward_feet_stumble`）
- 参照：[exp007_report_v9.md](exp007_report_v9.md) 「次バージョンへの提案」セクション
- 参照：[exp007_unitree_rl_gym_survey.md](exp007_unitree_rl_gym_survey.md)
