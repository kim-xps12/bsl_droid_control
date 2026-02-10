# EXP007 総括: exp008への引き継ぎ資料

## 1. 実験概要

BSL-Droid Simplified二脚ロボット（torso_width=0.18m, hip_offset_y=0.10m, 10DOF）に対し、Unitree RL Gymの報酬設計を参考に強化学習歩容獲得を行った。V1〜V40の40バージョンにわたり報酬チューニングを反復した。

## 2. 最終到達点

| 指標 | ベストバージョン | 値 |
|------|-----------------|-----|
| Yawドリフト | V32 | -2.00° |
| X速度 | V35 | 0.178 m/s |
| hip_pitch相関 | V38 | -0.801 |
| Pitch std | V39 | 1.34° |
| Roll std | V40 | 5.58° |

## 3. 確立された報酬構成（V41案: 17項目）

exp007 V40レポートで提案されたV41推奨案。V39ベース + ankle_roll=-0.5。

| カテゴリ | 報酬名 | スケール |
|---------|--------|---------|
| 主報酬 | tracking_lin_vel | 1.5 |
| 主報酬 | tracking_ang_vel | 0.5 |
| 歩行品質 | swing_duration | 2.0 |
| 歩行品質 | swing_contact_penalty | -0.7 |
| 歩行品質 | contact | 0.4 |
| 歩行品質 | single_foot_contact | 0.5 |
| 歩行品質 | step_length | 0.8 |
| 安定性 | lin_vel_z | -2.0 |
| 安定性 | ang_vel_xy | -0.1 |
| 安定性 | orientation | -0.5 |
| 安定性 | base_height | -5.0 |
| 歩行品質P | feet_swing_height | -8.0 |
| 歩行品質P | contact_no_vel | -0.1 |
| 関節制御 | hip_pos | -0.8 |
| 関節制御 | ankle_roll | -0.5 |
| 静止対策 | velocity_deficit | -0.5 |
| エネルギー | action_rate | -0.005 |
| 遊脚制御 | swing_foot_lateral_velocity | -0.5 |

無効化項目: feet_air_time=0, symmetry_range=0

## 4. 報酬項目の有効性評価

### 有効と確認された項目

| 項目 | 効果 | 確認バージョン |
|------|------|--------------|
| swing_contact_penalty=-0.7 | タップダンス解消 | V30, V35 |
| ang_vel_xy=-0.1 | Roll std -30%, Pitch std -40% | V38 |
| hip_pos=-0.8 | Yaw安定性確保（hip_yaw間接制約） | V39, V40 |
| ankle_roll | Roll改善、base_vel_y改善 | V40 |

### Negligibleと確認された項目（V34-V35で除去済み）

dof_vel_limits(0%), torques(0%), ankle_pitch_range(0.6%), dof_acc(1.1%)

## 5. 確立された教訓

### 報酬設計

- **報酬項目数**: 15-17推奨、21+は不安定（V7-V10で実証）
- **hip_posは構造的欠陥**: hip_yaw+hip_rollを同時にペナルティ。分離制御には ankle_roll を併用
- **symmetry_rangeは逆効果**: 振幅縮小（race to bottom）を引き起こす（V33）
- **hip_yaw完全自由化は逆効果**: V40で接地脚hip_yaw暴走（+30°）、Yaw悪化

### 歩行ダイナミクス

- **タップダンス vs 横揺れはトレードオフ**: swing_contact_penalty強化→長いスイング→より多くの片脚バランス→hip_roll増大
- **hip_roll mean offsetが横揺れの主因**: ang_vel_xyは角速度のみ制約し、静的offsetは制約しない
- **Yawドリフトはhip_yaw L/R非対称に起因**: RL確率性により方向は毎回変わる
- **RL確率性で同一設定でも異なる最適解**: 同じ報酬構成がlong swing/short swing等の異なる戦略に収束しうる

## 6. 未解決課題

- hip_roll mean offset（L:-18.80°, R:+17.84°）の解消
- Yawドリフトの根本解決（等変ネットワーク / ROGER等の検討）
- X速度の回復（V35の0.178 m/sに対し最近は0.11-0.12 m/s）
- ユーザー目標「ゆったり大股でしぜんに可愛く歩く」への収束
