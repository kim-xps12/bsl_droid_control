# EXP007: BSL-Droid Unitree RL Gym参考実装による強化学習歩容獲得実験

## 概要

BSL-Droid Simplified二脚ロボットモデルに対して、Unitree RL Gym（G1/H1）の報酬設計を参考にした強化学習歩容獲得を行う。exp006までの足先空間（Task-Space）アプローチを一旦離れ、Unitreeの関節空間制御に基づくアプローチを試行する。

## 実験の進め方

実験手順・コマンド・ルールは [exp007_rules.md](exp007_rules.md) を参照。

## 背景

### exp006までの課題

exp006（足先空間アプローチ）では以下の問題が発生した：

1. **hip_roll異常**: 脚が八の字に開く（0〜91°）
2. **高周波振動（震え）**: 報酬を微振動でexploit
3. **接地率0%**: 足が常に浮遊状態
4. **Yaw drift**: 左右非対称な動作による回転

### Unitree RL Gymからの学び

[exp007_unitree_rl_gym_survey.md](exp007_unitree_rl_gym_survey.md) の調査から、以下の知見を得た：

1. **関節空間制御**: Unitreeは足先空間ではなく関節角度を直接制御
2. **歩行フェーズ報酬**: `contact`報酬により自然な接地/遊脚パターンを学習
3. **遊脚高さペナルティ**: `feet_swing_height`でつまずき防止
4. **バランスの取れた報酬設計**: 主報酬とペナルティの適切なバランス

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

### Unitree G1/H1との比較

| 項目 | G1/H1 | BSL-Droid Simplified |
|------|-------|---------------------|
| 脚DOF | 12（片脚6） | 10（片脚5） |
| 膝構造 | 通常 | 逆関節 |
| 胴体高さ | 0.78-1.05 m | 0.19 m |
| 質量 | 30-50 kg | 5.8 kg |

---

## バージョン別実験レポート

各バージョンの詳細な設計・実装・結果は以下の個別ファイルを参照：

| バージョン | ファイル | 概要 | 主な結果 |
|-----------|----------|------|----------|
| V1 | [exp007_report_v1.md](exp007_report_v1.md) | Unitree参考実装（関節空間制御） | 静止ポリシーに収束 |
| V2 | [exp007_report_v2.md](exp007_report_v2.md) | 重み・目標速度調整 | 静止ポリシー継続 |
| V3 | [exp007_report_v3.md](exp007_report_v3.md) | 静止ポリシー問題への対策 | 0.151 m/s達成、交互歩行未達成 |
| V4 | [exp007_report_v4.md](exp007_report_v4.md) | 交互歩行・大股歩行への改善 | 0.192 m/s達成、足首振動問題 |
| V5 | [exp007_report_v5.md](exp007_report_v5.md) | 足首振動抑制と対称性改善 | 静止ポリシーに回帰 |
| V6 | [exp007_report_v6.md](exp007_report_v6.md) | tracking_sigma調整による静止回避 | 0.074 m/s達成、足首傾斜問題 |
| V7 | [exp007_report_v7.md](exp007_report_v7.md) | 交互歩行強化と歩幅拡大 | 0.214 m/s達成、報酬設計ミスで交互歩行悪化 |
| V8 | [exp007_report_v8.md](exp007_report_v8.md) | 交互歩行報酬の修正と胴体安定性強化 | hip_pitch相関改善（-0.382）、左右非対称・Yawドリフト113° |
| V9 | [exp007_report_v9.md](exp007_report_v9.md) | 報酬項目の整理と両脚対称性強化 | 対称性・Yaw改善、hip_pitch相関悪化（+0.605）、足引きずり |
| V10 | [exp007_report_v10.md](exp007_report_v10.md) | 足上げ改善と滑らかな歩行の実現 | ❌ **静止ポリシー回帰**（0.006 m/s、ペナルティ累積効果） |
| V11 | [exp007_report_v11.md](exp007_report_v11.md) | 最小有効報酬セットによる静止ポリシー回避 | 0.212 m/s達成、内股・爪先立ち・小刻み歩行問題 |
| V12 | [exp007_report_v12.md](exp007_report_v12.md) | 歩行品質改善（内股・爪先立ち・小刻み歩行対策） | 0.213 m/s、hip_pitch相関改善(-0.208)、目視問題継続 |
| V13 | [exp007_report_v13.md](exp007_report_v13.md) | V1回帰 + ゆっくり大股歩行 | 0.106 m/s、hip_roll改善、速度低下（速度・周波数を下げすぎた） |
| V14 | [exp007_report_v14.md](exp007_report_v14.md) | V1ベース + 速度目標のみ低下 | ❌ **静止ポリシー**（0.003 m/s、V1系統は静止回避機構なし） |
| V15 | [exp007_report_v15.md](exp007_report_v15.md) | V3ベース + ゆったり大股歩行 | 0.198 m/s達成、hip_pitch相関改善(-0.165)、**小刻み問題**（膝動作不足） |

### バージョン進化の概要

```
V1 → V2: 報酬重み調整 → 静止ポリシー継続
     ↓
V3: velocity_deficit追加 → 0.151 m/s達成
     ↓
V4: 接地検出修正 → 0.192 m/s達成、足首振動発生
     ↓
V5: dof_vel強化、symmetry追加、低速度目標 → 静止ポリシー回帰
     ↓
V6: tracking_sigma鋭化（0.25→0.10）→ 0.074 m/s達成、足首傾斜問題発生
     ↓
V7: alternating_gait/foot_flat/step_length追加 → 0.214 m/s達成、報酬設計ミス（hip_pitch相関悪化0.781→0.828）
     ↓
V8: hip_pitch_antiphase（速度逆相関）、胴体安定性強化 → hip_pitch相関改善（-0.382）、**左右非対称**（右脚のみ動く）、Yawドリフト113°
     ↓
V9: 報酬項目整理（23→21）、hip_pitch_antiphase_v2、both_legs_active、symmetry/tracking_ang_vel強化 → 対称性・Yaw大幅改善、**hip_pitch相関悪化（+0.605）**、足引きずり
     ↓
V10: symmetry緩和（1.0→0.5）、feet_stumble追加、swing_height_target増加（0.03→0.05m）、action_rate強化 → **❌ 静止ポリシー回帰**（0.006 m/s、ペナルティ累積効果で動作抑制）
     ↓
V11: **最小有効報酬セット**（22→15項目）、ペナルティ緩和（V3-V4レベル）、不要報酬7項目削除 → 0.212 m/s達成（静止回避成功）、**内股・爪先立ち・小刻み歩行問題**
     ↓
V12: hip_pos/foot_flat復活（15→17項目）、single_foot_contact強化 → 0.213 m/s維持、hip_pitch相関改善(-0.208)、**目視での問題継続**（内股・爪先立ち・胴体傾き）
     ↓
V13: **V1回帰** + ゆっくり大股（action_rate/hip_pos/tracking_sigmaをV1に戻す、gait_frequency 0.8Hz）→ **0.106 m/s**（速度・周波数を下げすぎ、V1再現できず）
     ↓
V14: **V1ベース + 速度のみ低下**（lin_vel_x_range: [0.2,0.3]→[0.15,0.20]、他はV1と同じ15項目）→ ❌ **静止ポリシー**（0.003 m/s、V1系統は静止回避機構なし）

V3から派生:
V3 → V15: **V3ベース + ゆったり大股**（gait_frequency 1.0Hz、feet_air_time 1.5、静止回避機構維持16項目）→ 0.198 m/s達成、**小刻み問題**（膝動作不足）
     ↓
V16案: action_rate緩和(-0.005)、swing_height_target増加(0.04m)、feet_air_time offset調整(0.3秒)
```

---

## 参照ドキュメント

- [exp007_rules.md](exp007_rules.md) - 実験ルール・手順・コマンド
- [exp007_unitree_rl_gym_survey.md](exp007_unitree_rl_gym_survey.md) - Unitree RL Gymの報酬設計サーベイ

## 外部参考文献

- [Unitree RL Gym (GitHub)](https://github.com/unitreerobotics/unitree_rl_gym)
- [Legged Gym (ETH)](https://github.com/leggedrobotics/legged_gym)
- [Genesis公式ドキュメント](https://genesis-world.readthedocs.io/)
- [Revisiting Reward Design and Evaluation for Robust Humanoid Standing and Walking (arXiv 2024)](https://arxiv.org/html/2404.19173v1)
- [Leveraging Symmetry in RL-based Legged Locomotion Control (IROS 2024)](https://ieeexplore.ieee.org/document/10341443)
