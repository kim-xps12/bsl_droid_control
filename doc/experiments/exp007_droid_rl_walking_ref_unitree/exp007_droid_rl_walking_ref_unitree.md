# EXP007: BSL-Droid Unitree RL Gym参考実装による強化学習歩容獲得実験

## 概要

BSL-Droid Simplified二脚ロボットモデルに対して、Unitree RL Gym（G1/H1）の報酬設計を参考にした強化学習歩容獲得を行う。exp006までの足先空間（Task-Space）アプローチを一旦離れ、Unitreeの関節空間制御に基づくアプローチを試行する。

## 実験の進め方

実験ルール・手順は以下を参照:
- [exp007_rules.md](exp007_rules.md) - 原則・ルール（メインエントリ）
- [exp007_workflow.md](exp007_workflow.md) - ワークフロー・レポートテンプレート
- [exp007_commands.md](exp007_commands.md) - コマンドリファレンス
- [exp007_reward_design.md](exp007_reward_design.md) - 報酬設計原則・トラブルシューティング

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
| V16 | [exp007_report_v16.md](exp007_report_v16.md) | V15ベース + 振動抑制・大股歩行強化 | 0.185 m/s、DOF range改善(+48%)、**左右非対称**(hip_pitch L/R=2.7x)、Yawドリフト+13.57° |
| V17 | [exp007_report_v17.md](exp007_report_v17.md) | 「ゆったり大股で歩く」の実現 | ❌ 静止ポリシー回帰 |
| V18 | [exp007_report_v18.md](exp007_report_v18.md) | V3ベース + RobStride RS-02実機パラメータ適合（関節角速度制限） | 0.248 m/s達成、接地検出失敗 |
| V19 | [exp007_report_v19.md](exp007_report_v19.md) | 歩幅報酬（step_length）の追加 | 0.185 m/s、接地検出継続失敗 |
| V20 | [exp007_report_v20.md](exp007_report_v20.md) | Genesis Contact Sensor による接地検出の根本修正 | 0.204 m/s、片足接地91.4%達成、つま先突き問題 |
| V21 | [exp007_report_v21.md](exp007_report_v21.md) | Z座標閾値ベース接地検出の検証（実験A） | すり足に退行（片足接地0.2%）、閾値0.10m高すぎ |
| V22 | [exp007_report_v22.md](exp007_report_v22.md) | Contact Sensor復帰 + A案（ankle_pitch_rangeペナルティ） | 0.209 m/s、片足接地89.6%、タップダンス歩容発生 |
| V23 | [exp007_report_v23.md](exp007_report_v23.md) | V22 + B案（air_time_offset短縮: 0.3→0.2） | 0.210 m/s、片足接地91.2%、**タップダンス解消、内股軌道発生** |
| V24 | [exp007_report_v24.md](exp007_report_v24.md) | V23 + 遊脚横方向速度ペナルティ（内股解消） | 0.204 m/s、片足接地88.2%、**内股解消、タップダンス再発** |
| V25 | [exp007_report_v25.md](exp007_report_v25.md) | V24 + swing_foot_lateral_velocityスケール緩和（-0.5→-0.2） | 0.206 m/s、片足接地91.2%、**内股再発、タップダンス未改善**（悪いとこどり） |
| V26 | [exp007_report_v26.md](exp007_report_v26.md) | V24復帰 + feet_air_time強化（1.0→1.5） | 0.208 m/s、hip_pitch相関-0.318、タップダンス継続 |
| V27 | [exp007_report_v27.md](exp007_report_v27.md) | エネルギーペナルティ緩和 + air_time_offset短縮 | 0.217 m/s、**hip_pitch相関-0.931（大幅改善）**、タップダンス再発 |
| V28 | [exp007_report_v28.md](exp007_report_v28.md) | air_time_offset延長 + contact報酬強化 + 内股対策強化 | 0.214 m/s、hip_pitch相関-0.951、タップダンス継続、**根本原因特定** |
| V29 | [exp007_report_v29.md](exp007_report_v29.md) | swing_duration報酬導入 + 歩行周期延長 + 歩幅強化 | 0.223 m/s、hip_pitch相関-0.971、**swing_duration機能せず**（報酬値0.0000）、タップダンス悪化 |
| V30 | [exp007_report_v30.md](exp007_report_v30.md) | air_time_offset引き下げ + swing_contact_penalty導入 | swing_duration機能化、片足接地率95.4%、**hip_pitch相関悪化**（-0.571） |
| V31 | [exp007_report_v31.md](exp007_report_v31.md) | single_foot_contact強化 + swing_contact_penalty緩和 | hip_pitch相関改善（-0.660）、**Yawドリフト悪化**（+30.37°） |
| V32 | [exp007_report_v32.md](exp007_report_v32.md) | swing_contact_penaltyをV30の値に復元 | **Yawドリフト大幅改善**（-2.00°）、左足タップダンス残存 |
| V33 | [exp007_report_v33.md](exp007_report_v33.md) | symmetry_range報酬の有効化 | 左右対称化達成、振幅縮小副作用、Yaw悪化（-18.69°） |
| V34 | [exp007_report_v34.md](exp007_report_v34.md) | symmetry_rangeの無効化（V32ベース回帰） | X速度0.173 m/s回復、Yaw +7.35°、左足タップダンス残存 |
| V35 | [exp007_report_v35.md](exp007_report_v35.md) | swing_contact_penalty強化（-0.5→-0.7） | X速度0.178 m/s（最高）、両足接地率1.4%、hip_roll増大による横揺れ |
| V36 | [exp007_report_v36.md](exp007_report_v36.md) | negligible報酬項目の削減（20→16項目） | 報酬簡素化達成、Yaw改善(-4.92°)、hip_pitch相関悪化(-0.435) |
| V37 | [exp007_report_v37.md](exp007_report_v37.md) | hip_pos強化（-0.8→-1.0） | タップダンス解消、Yaw -17.35°（過去最悪）、Pitch std +228% |
| V38 | [exp007_report_v38.md](exp007_report_v38.md) | ang_vel_xy強化（-0.05→-0.1） | Roll std -30%、Pitch std -40%、hip_pitch相関-0.801（過去最高）、**Yaw -42.71°（過去最悪更新）** |
| V39 | [exp007_report_v39.md](exp007_report_v39.md) | hip_pos緩和（-1.0→-0.8） | Yaw -42.71°→-19.17°（+55%改善）、Roll std -7.2%、Pitch std -58%、**X速度-27%（0.122）** |
| V40 | [exp007_report_v40.md](exp007_report_v40.md) | hip_pos分解（hip_pos無効化→ankle_roll有効化） | hip_pitch相関-0.743(+26%改善)、Roll std 5.58°(-8%)、base_vel_y -36%。**Yaw -28.11°(悪化)**、Pitch std +91%、hip_pitch非対称-22.9% |

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
V16: dof_vel追加(-0.005)、step_length追加(0.5)、action_rate緩和(-0.005)、swing_height_target増加(0.04m)、air_time_offset調整(0.3秒) → DOF range改善(+48%)、**左右非対称**(L/R=2.7x)、Yawドリフト悪化
     ↓
V17: **step_length→symmetry_range置換(0.5)**、tracking_ang_vel強化(0.8)、contact_threshold緩和(0.035m)、dof_vel強化(-0.01)【18項目維持】→ （訓練待ち）

V3派生（実機展開系統）:
V3 → V18: **V3ベース + RobStride RS-02実機パラメータ**（dof_vel_limits報酬追加、44 rad/s制限、soft_dof_vel_limit=0.9）→ 0.248 m/s達成、**接地検出失敗**（feet_air_time=0, single_foot_contact=0）
     ↓
V19: step_length報酬追加、swing_height_target増加（0.03→0.05m）、gait_frequency低下（1.5→1.2Hz）→ 0.185 m/s、接地検出継続失敗（**100%両足空中と誤判定**）
     ↓
V20: **Genesis Contact Sensor 導入**（Z座標閾値→物理エンジンの接触判定）→ 0.204 m/s、**片足接地91.4%達成**、つま先突き問題発生
     ↓
V21: **Z座標閾値ベース接地検出の検証**（実験A: contact_threshold 0.05→0.10m）→ ❌ **すり足に退行**（片足接地0.2%、閾値0.10m高すぎ）
     ↓
V22系統（つま先突き動作解消の段階的試行）:
V22: **Contact Sensor復帰 + A案**（ankle_pitch_rangeペナルティ）→ 0.209 m/s、片足接地89.6%、**タップダンス歩容発生**（feet_air_time負）
     ↓
V23: **B案追加**（air_time_offset 0.3→0.2s短縮）→ 0.210 m/s、片足接地91.2%、**タップダンス解消、内股軌道発生**
     ↓
V24: **遊脚横方向速度ペナルティ追加**（swing_foot_lateral_velocity -0.5）→ 0.204 m/s、片足接地88.2%、**内股解消、タップダンス再発**（feet_air_time悪化: -0.0158→-0.0203）
     ↓
V25: **swing_foot_lateral_velocityスケール緩和**（-0.5→-0.2）→ 0.206 m/s、片足接地91.2%、❌ **悪いとこどり**（内股再発+タップダンス未改善）
     ↓
     教訓: 内股抑制とタップダンス解消は独立した問題
     ↓
V26: **V24復帰 + feet_air_time強化**（swing_foot_lateral_velocity -0.5復帰、feet_air_time 1.0→1.5）→ 0.208 m/s、hip_pitch相関-0.318、**タップダンス継続**
     ↓
V27: **エネルギーペナルティ緩和 + air_time_offset短縮**（action_rate -0.01→-0.005、dof_acc -2.5e-7→-1.0e-7、air_time_offset 0.2→0.15）→ 0.217 m/s、**hip_pitch相関-0.931（大幅改善）**、タップダンス再発
     ↓
     教訓: エネルギーペナルティ緩和はhip_pitch相関改善に有効、air_time_offset短縮はタップダンス促進
     ↓
V28: **air_time_offset延長 + contact報酬強化 + 内股対策強化**（air_time_offset 0.15→0.25、contact 0.2→0.4、hip_pos -0.5→-0.8）→ 0.214 m/s、hip_pitch相関-0.951、**タップダンス継続、feet_air_time=-0.0427（根本原因特定）**
     ↓
     重要発見: feet_air_time報酬のfirst_contact構造がタップダンスの根本原因
     パラメータ調整では解消不可能 → 報酬計算方式の変更が必要
     ↓
V29: **swing_duration報酬導入 + 歩行周期延長 + 歩幅強化**（feet_air_time 1.5→0、swing_duration 新規1.0、gait_frequency 1.2→0.9、step_length 0.5→0.8）→ 0.223 m/s、hip_pitch相関-0.971（過去最高）、**swing_duration完全機能せず**（報酬値0.0000）、タップダンス悪化（片足接地率-12%）
     ↓
     重要発見: air_time_offset=0.25秒がBSL-Droidでは達成困難
     swing_duration報酬が学習シグナルを提供できず → 閾値引き下げ + 直接抑制が必要
     ↓
V30: **air_time_offset引き下げ + swing_contact_penalty導入**（air_time_offset 0.25→0.10、swing_duration 1.0→2.0、swing_contact_penalty 新規-0.5）→ swing_duration機能化（0.1091）、片足接地率95.4%、**hip_pitch相関悪化**（-0.571）
     ↓
     重要発見: swing_contact_penalty=-0.5が強すぎて交互歩行パターンが乱れた
     single_foot_contact強化 + ペナルティ緩和が必要
     ↓
V31: **single_foot_contact強化 + swing_contact_penalty緩和**（single_foot_contact 0.3→0.5、swing_contact_penalty -0.5→-0.3）→ hip_pitch相関改善（-0.660）、**Yawドリフト悪化**（+30.37°）
     ↓
     教訓: swing_contact_penalty緩和がYawドリフト悪化とタップダンス再発を引き起こした
     V30の成功（swing_contact_penalty=-0.5）とV31の成功（single_foot_contact=0.5）を組み合わせる
     ↓
V32: **swing_contact_penaltyをV30の値に復元**（-0.3→-0.5）→ **Yawドリフト大幅改善**（-2.00°）、右足タップダンス消失、左足タップダンス残存
     ↓
     発見: 左右非対称問題（hip_pitch velocity std 左が82%大きい）
     左右のhip_pitch振幅差を直接ペナルティ化が必要
     ↓
V33: **symmetry_range報酬の有効化**（0→0.3）→ 左右対称化達成（velocity比1.82→0.95）、**振幅縮小副作用**（DOF range -15%）、Yaw悪化（-18.69°）
     ↓
     教訓: symmetry_rangeの"race to bottom"効果（振幅縮小がペナルティ減少に有利）
     報酬shapingによる対称性誘導の構造的限界（サーベイSection 7.2.3と一致）
     ↓
V34: **symmetry_rangeの無効化**（0.3→0）→ X速度0.173 m/s回復、DOF range 6.021 rad、Yaw +7.35°、左足タップダンス残存
     ↓
     発見: swing_contact_penalty=-0.5が左足抑制に不十分、-0.7への強化が必要
     ↓
V35: **swing_contact_penalty強化**（-0.5→-0.7）→ X速度0.178 m/s（最高）、**両足接地率1.4%（大幅改善）**、hip_roll増大による横揺れ
     ↓
     発見: タップダンス大幅改善だがhip_roll運動増大（L: +20%、R: +26%）→ 横揺れ
     hip_posペナルティがトップに浮上（23.8%）→ hip_pos強化が必要
     ↓
V36: **negligible 4項目削除**（dof_vel_limits, torques, ankle_pitch_range, dof_acc）→ 報酬簡素化（16項目）、Yaw改善(-4.92°)、**hip_pitch相関悪化**(-0.435)
     ↓
     教訓: negligible項目削除は真のno-opではない可能性（RL確率的変動との分離困難）
     スイング持続時間短縮（0.49→0.31s）、タップダンス再発
     ↓
V37: **hip_pos強化**（-0.8→-1.0）→ タップダンス解消、スイング回復(0.44s)、L/R均等化。**Yaw -17.35°(過去最悪)、Pitch std +228%**
     ↓
     教訓: hip_pos(hip_yaw+hip_roll同時ペナルティ)の構造的限界
     hip_yaw制約→Yaw修正能力低下→Yawドリフト悪化
     ↓
V38: **ang_vel_xy強化**（-0.05→-0.1）→ **Roll std -30%(6.53°)、Pitch std -40%(3.16°)、hip_pitch相関-0.801(過去最高)**、X速度回復(0.168)。**Yaw -42.71°(過去最悪更新)**
     ↓
     教訓: 直接制御(ang_vel_xy)はRoll/Pitch安定性に有効
     Yaw根本原因=hip_pos=-1.0によるhip_yaw制約。回復フェーズ消失
     ↓
V39: **hip_pos緩和**（-1.0→-0.8）→ **Yaw -19.17°(+55%改善)**、Roll std 6.06°(-7%)、Pitch std 1.34°(-58%、過去最良)。**X速度0.122(-27%)**、hip_pitch相関-0.588(-27%)
     ↓
     教訓: hip_pos緩和でYaw/Roll/Pitch改善だがX速度・hip_pitch相関低下のトレードオフ
     ang_vel_xy=-0.1 + hip_pos=-0.8の組み合わせでRoll安定性維持を実証
     hip_pos構造的欠陥(hip_yaw+hip_roll同時ペナルティ)の解消が急務
     ↓
V40: **hip_pos分解**（hip_pos: -0.8→0無効化、ankle_roll: 0→-1.0有効化）→ hip_pitch相関-0.743(+26%)、Roll std 5.58°(-8%)、base_vel_y -36%。**Yaw -28.11°悪化**、Pitch std +91%、hip_pitch非対称-22.9%
     ↓
     教訓: hip_yaw完全自由化は逆効果。角運動量不均衡でYaw悪化
     ankle_rollのhip_roll制御効果（Roll改善、base_vel_y改善）は有効
     hip_yawの適度な制約が必要（hip_pos維持+ankle_roll追加が正解）
```

---

## 参照ドキュメント

- [exp007_rules.md](exp007_rules.md) - 原則・ルール（メインエントリ）
- [exp007_workflow.md](exp007_workflow.md) - ワークフロー・レポートテンプレート
- [exp007_commands.md](exp007_commands.md) - コマンドリファレンス
- [exp007_reward_design.md](exp007_reward_design.md) - 報酬設計原則・トラブルシューティング
- [exp007_unitree_rl_gym_survey.md](exp007_unitree_rl_gym_survey.md) - Unitree RL Gymの報酬設計サーベイ

## 外部参考文献

- [Unitree RL Gym (GitHub)](https://github.com/unitreerobotics/unitree_rl_gym)
- [Legged Gym (ETH)](https://github.com/leggedrobotics/legged_gym)
- [Genesis公式ドキュメント](https://genesis-world.readthedocs.io/)
- [Revisiting Reward Design and Evaluation for Robust Humanoid Standing and Walking (arXiv 2024)](https://arxiv.org/html/2404.19173v1)
- [Leveraging Symmetry in RL-based Legged Locomotion Control (IROS 2024)](https://ieeexplore.ieee.org/document/10341443)
