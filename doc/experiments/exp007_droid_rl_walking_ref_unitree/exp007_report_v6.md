# V6: tracking_sigma調整による静止ポリシー回避

## 概要

V5で発生した「静止ポリシー」問題（X速度: 0.003 m/s）を解決するため、ガウシアン報酬関数（tracking_lin_vel）のsigmaパラメータを鋭化し、静止と歩行の報酬差を拡大する。V5の改善点（足首振動抑制、対称性報酬）は継続する。

## 前バージョンからの改善事項

V5レポート（exp007_report_v5.md）の「次バージョンへの提案」セクションより：
- tracking_sigmaを0.25→0.10に変更（ガウシアン鋭化）
- velocity_deficitペナルティを-0.5→-2.0に強化（4倍）

| パラメータ | V5値 | V6値 | 変更理由 |
|-----------|------|------|---------|
| **tracking_sigma** | 0.25 | **0.10** | 静止vs歩行の報酬差を3.5倍に拡大（21%→77%） |
| **velocity_deficit** | -0.5 | **-2.0** | 追加保険として4倍強化 |
| lin_vel_x_range | [0.10, 0.15] | [0.10, 0.15] | 維持 |
| gait_frequency | 0.8 Hz | 0.8 Hz | 維持 |
| dof_vel | -0.005 | -0.005 | 維持 |
| air_time_offset | 0.25 s | 0.25 s | 維持 |
| symmetry | 0.3 | 0.3 | 維持 |

## 設計詳細

### tracking_sigma調整の理論的根拠

tracking_lin_vel報酬は以下のガウシアン関数で計算される：

```
reward = exp(-error² / sigma)
```

**sigma=0.25（V5）の場合**:
- 目標速度0.125 m/s、実速度0.003 m/s（静止）: 報酬 = 0.787（79%）
- 目標速度0.125 m/s、実速度0.125 m/s（歩行達成）: 報酬 = 1.000（100%）
- **差分: 21%**（静止でも高報酬）

**sigma=0.10（V6）の場合**:
- 目標速度0.125 m/s、実速度0.003 m/s（静止）: 報酬 = 0.226（23%）
- 目標速度0.125 m/s、実速度0.125 m/s（歩行達成）: 報酬 = 1.000（100%）
- **差分: 77%**（静止は大幅ペナルティ）

### V6報酬スケール

```python
reward_cfg = {
    "tracking_sigma": 0.10,  # ★核心変更
    "base_height_target": 0.20,
    "swing_height_target": 0.03,
    "gait_frequency": 0.8,
    "contact_threshold": 0.08,
    "air_time_offset": 0.25,
    "reward_scales": {
        # 主報酬
        "tracking_lin_vel": 1.5,
        "tracking_ang_vel": 0.5,
        "feet_air_time": 1.5,
        "contact": 0.2,
        "alive": 0.03,
        "single_foot_contact": 0.8,
        "symmetry": 0.3,
        # ペナルティ
        "lin_vel_z": -2.0,
        "ang_vel_xy": -0.05,
        "orientation": -0.5,
        "base_height": -5.0,
        "feet_swing_height": -5.0,
        "contact_no_vel": -0.1,
        "hip_pos": -0.5,
        "velocity_deficit": -2.0,  # ★4倍強化
        "torques": -1e-5,
        "action_rate": -0.005,
        "dof_acc": -2.5e-7,
        "dof_vel": -0.005,
    },
}
```

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_unitree_v6.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-unitree-v6 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 61.33 |
| エピソード長 | 1001 (最大) |
| 収束ステップ | 約100（安定化は200付近） |
| Last 50 steps std | 0.203（まだ変動中） |

報酬推移:
- Steps 0-125: 24.79
- Steps 125-250: 56.15 (+31.36)
- Steps 250-375: 60.30 (+4.15)
- Steps 375-500: 61.17 (+0.87)

### 評価結果

| 指標 | V5値 | V6値 | 変化 |
|------|------|------|------|
| X速度 | 0.003 m/s | **0.074 m/s** | ✅ +2367%（静止→歩行） |
| Y速度 | - | 0.005 m/s | - |
| X移動距離 | - | 0.736 m | - |
| hip_pitch相関 | +0.653 | **+0.781** | ⚠️ 悪化（両脚同期） |
| DOF range sum | 0.44 rad | **3.330 rad** | ✅ +657%（動作増加） |
| エピソード長 | 1001 | 1001 | = 維持 |
| Base height | - | 0.234 m (std 0.010) | - |

### 姿勢・安定性

| 指標 | 値 |
|------|-----|
| Roll | mean=0.01°, std=1.09° |
| Pitch | mean=0.80°, std=0.90° |
| Yaw drift | +2.06°（10秒間） |

### 関節動作分析

| 関節 | L range (rad) | R range (rad) | L vel std (rad/s) | R vel std (rad/s) |
|------|---------------|---------------|-------------------|-------------------|
| hip_pitch | 0.325 | 0.259 | 0.571 | 0.457 |
| hip_roll | 0.181 | 0.242 | 0.283 | 0.399 |
| knee | 0.345 | 0.525 | 0.578 | 0.345 |
| **ankle_pitch** | 0.194 | 0.278 | 0.514 | 0.707 |
| **ankle_roll** | **0.454** | **0.527** | **0.856** | **0.772** |

### 接地パターン

| パターン | ステップ数 | 割合 |
|----------|-----------|------|
| Both feet grounded | 6 | 1.2% |
| **Single foot grounded** | **488** | **97.6%** | ✅
| Both feet airborne | 6 | 1.2% |

### 個別報酬コンポーネント（最終ステップ）

| 報酬項目 | 値 | 評価 |
|----------|-----|------|
| tracking_lin_vel | **1.056** | ✅ 主報酬として機能 |
| single_foot_contact | 0.593 | ✅ 片足接地達成 |
| tracking_ang_vel | 0.330 | ○ |
| symmetry | 0.222 | ○ |
| contact | 0.147 | ○ |
| alive | 0.023 | ○ |
| dof_vel | **-0.012** | ⚠️ 振動抑制中 |
| feet_swing_height | -0.013 | ⚠️ |
| lin_vel_z | -0.011 | ○ |
| ang_vel_xy | -0.011 | ○ |
| hip_pos | -0.008 | ○ |
| velocity_deficit | **-0.006** | ⚠️ まだ目標未達 |

## 考察と改善案

### 成功点

1. **静止ポリシーの回避**: V5の0.003 m/s → V6の0.074 m/sで、実際に前進する歩行を獲得
2. **片足接地の達成**: 97.6%のステップで片足のみ接地（single_foot_contact報酬が機能）
3. **安定性維持**: エピソード長1001、Roll/Pitch変動は小さい（std < 1.1°）
4. **動作量の増加**: DOF range sumが0.44 rad → 3.330 radと大幅増加

### 課題

1. **両脚同期の悪化**: hip_pitch相関が+0.653 → +0.781に悪化（交互歩行未達成）
2. **足首傾斜問題**: ankle_roll rangeがL=0.454, R=0.527 radと大きく傾斜
   - 目視観察：「爪先だけで接地して歩こうとしている」
3. **小刻みな動作**: hip_pitchのrangeが0.259-0.325 radと小さい
4. **目標速度未達**: 0.074 m/sは目標0.100 m/sの74%

### 根本原因分析

**足首傾斜問題の原因**:
- ankle_roll関節に対するペナルティ（hip_pos）がhip_yaw/hip_rollのみを対象
- ankle_rollの傾斜は許容されており、爪先立ちでの歩行が局所最適解として学習された

**両脚同期の原因**:
- single_foot_contact報酬は「片足接地」を報酬化するが、「交互に」を強制しない
- 両脚が同じ位相で微小な振動をすることで報酬を獲得している可能性

### 次バージョンへの提案

1. **ankle_roll/ankle_pitchペナルティの追加**
   ```python
   "ankle_pos": -1.0,  # 足首関節の過度な変位をペナルティ
   ```
   対象：left_ankle_pitch, left_ankle_roll, right_ankle_pitch, right_ankle_roll

2. **交互歩行の強化**: hip_pitchの位相差報酬
   ```python
   "hip_phase_diff": 0.5,  # 左右hip_pitchの位相差（180°が理想）を報酬化
   ```

3. **歩幅拡大のための調整**:
   - action_rate緩和（-0.005 → -0.002）で大きな動作を許容
   - または正弦波参照軌道の導入（survey 7.3.2参照）

4. **tracking_sigma微調整**: 現在0.10だが、目標未達なので0.15に緩和して学習安定性を向上させる選択肢も

優先順位：
1. **ankle_posペナルティ追加**（必須）- 爪先立ち問題の直接的解決
2. **hip_phase_diff報酬追加** - 交互歩行の誘導
3. action_rate緩和 - 歩幅拡大

## まとめ

V6はtracking_sigma調整により、V5の静止ポリシー問題を解決し、前進する歩行を獲得した（0.003 m/s → 0.074 m/s）。片足接地も97.6%達成。

しかし、新たに以下の課題が顕在化：
- 足首傾斜（爪先立ち歩行）
- 両脚同期の悪化
- 小刻みな動作

V7ではankle_posペナルティを追加し、足首傾斜問題を解決することを最優先とする。

## 備考

- 学習ログ：`rl_ws/logs/droid-walking-unitree-v6/`
- 参照：[exp007_unitree_rl_gym_survey.md](exp007_unitree_rl_gym_survey.md) セクション7.5（Foot Clearance）
