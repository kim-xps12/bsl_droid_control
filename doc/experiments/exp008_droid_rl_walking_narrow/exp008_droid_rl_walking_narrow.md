# EXP008: BSL-Droid Simplified V2による強化学習歩容獲得実験

## 概要

BSL-Droid Simplified V2（胴体幅75%縮小モデル）に対して、exp007で確立された報酬設計を用いた強化学習歩容獲得を行う。胴体幅の縮小により両脚の間隔を狭め、横方向の制御性向上を目指す。

## 実験の進め方

実験ルール・手順は以下を参照:
- [exp008_rules.md](exp008_rules.md) - 原則・ルール（メインエントリ）
- [exp008_workflow.md](exp008_workflow.md) - ワークフロー・レポートテンプレート
- [exp008_commands.md](exp008_commands.md) - コマンドリファレンス
- [exp008_reward_design.md](exp008_reward_design.md) - 報酬設計原則・トラブルシューティング

## 背景

### exp007からの継承

exp007（40バージョンの反復チューニング）で以下が確立された:
- 17項目の報酬構成（V41案: V39ベース + ankle_roll=-0.5）
- 報酬間相互作用の知見（hip_pos構造欠陥、ang_vel_xy効果等）
- 各種課題の原因分析と対策

詳細は [exp007_summary_for_exp008.md](../exp007_droid_rl_walking_ref_unitree/exp007_summary_for_exp008.md) を参照。

### 胴体幅縮小の動機

exp007で観測された横方向並進揺れの一因は、脚間隔（hip_offset_y=0.10m）に対する胴体幅（0.18m）の大きさにある。胴体幅を75%に縮小することで:
1. 脚間隔を0.075mに縮小 → 横方向の支持基底面が狭くなるが、脚の相対位置が改善
2. 胴体の回転慣性（izz）が低下 → Yaw方向の制御が容易に
3. 全体的にコンパクトな体型 → より自然な歩行姿勢

## ロボット仕様

### BSL-Droid Simplified V2

| 項目 | V1（exp007） | V2（exp008） | 変更率 |
|------|-------------|-------------|--------|
| 胴体幅（torso_width） | 0.18 m | 0.135 m | 75% |
| 胴体奥行（torso_depth） | 0.14 m | 0.14 m | - |
| 胴体高さ（torso_height） | 0.16 m | 0.16 m | - |
| 股関節オフセット（hip_offset_y） | 0.10 m | 0.075 m | 75% |
| 胴体慣性 ixx | 0.05 | 0.05 | - |
| 胴体慣性 iyy | 0.05 | 0.038 | 76% |
| 胴体慣性 izz | 0.04 | 0.029 | 73% |
| DOF | 10 | 10 | - |
| 総質量 | 約5.8 kg | 約5.8 kg | - |
| 頭部幅（head_width） | 0.19 m | 0.19 m | - |

---

## バージョン別実験レポート

各バージョンの詳細な設計・実装・結果は以下の個別ファイルを参照：

| バージョン | ファイル | 概要 | 主な結果 |
|-----------|----------|------|----------|
| V1 | [exp008_report_v1.md](exp008_report_v1.md) | exp007 V41案の継承 + V2 URDF | X=0.190, Yaw=+16.46°, Roll std=6.72°, hip_pitch corr=-0.681 |
| V2 | [exp008_report_v2.md](exp008_report_v2.md) | hip_pos緩和 (-0.8→-0.5) | **失敗**: Yaw=+96.49°(壊滅的悪化), Roll std=5.37°(改善), X=0.156 |
| V3 | [exp008_report_v3.md](exp008_report_v3.md) | hip_pos compound penalty分離（hip_yaw_pos新設） | X=0.162, **Yaw=+18.84°(V1復元)**, **Roll std=4.89°(best)**, **Pitch std=0.98°(best)**, **hip_pitch corr=-0.805(best)** |
| V4 | [exp008_report_v4.md](exp008_report_v4.md) | 接地相限定hip_rollペナルティ（stance_hip_roll_target新設） | X=0.167, **Yaw=+7.51°(best)**, **Roll std=3.92°(best)**, Pitch std=0.98°(best維持), hip_pitch corr=-0.787, 内股offset悪化(±25°) |
| V5 | [exp008_report_v5.md](exp008_report_v5.md) | stance_hip_roll_targetスケール強化（-0.5→-2.0） | X=0.170, **Yaw=+0.69°(best)**, Roll std=6.95°(悪化), Pitch std=2.27°(悪化), **内股±4°(best, ほぼ解消)**, ペナルティ過大でgait degradation |
| V6 | [exp008_report_v6.md](exp008_report_v6.md) | 胴体鉛直性強化（orientation -0.5→-2.0） | X=0.160, Yaw=+14.05°(悪化), **Roll std=3.54°(best)**, **Pitch std=0.89°(best)**, R hip_roll offset -29.3%改善 |
| V7 | [exp008_report_v7.md](exp008_report_v7.md) | 全相足先横方向速度ペナルティ（swing_foot_lateral_velocity→foot_lateral_velocity置換） | **内股未解決**(hip_roll offset微悪化), Yaw=+1.79°(-87.3%), **Pitch std=0.70°(best)**, **penalty/positive=0.102(best)** |
| V8 | [exp008_report_v8.md](exp008_report_v8.md) | stance_hip_roll_targetターゲット角度設定（±5°外向き） | R hip_roll offset -23.3%改善, **Yaw=-14.81°(8.3x悪化)**, hip_pitch corr=-0.633(-19.2%), X=0.179(+8.5%), **内股R脚のみ部分改善・L脚不変** |
| V9 | [exp008_report_v9.md](exp008_report_v9.md) | URDF hip_roll関節リミット縮小（±25°→±12°） | **失敗**: 歩行破綻（hip_pitch corr反転+0.752）, 両足空中38.0%, Roll std=8.45°(+141%), X=0.097(-41%), **±12°が横方向バランスに過度に制約的** |
| V10 | [exp008_report_v10.md](exp008_report_v10.md) | URDF hip_roll関節リミット緩和（±25°→±18°） | **失敗**: 部分的歩行崩壊（hip_pitch corr=+0.409）, 転倒2回/10s, Yaw=+41.86°, Roll std=7.10°(+102%), Pitch std=5.80°(+729%), **±18°でも横方向バランス不足** |
| V11 | [exp008_report_v11.md](exp008_report_v11.md) | 非対称hip_rollリミット（内向き-12°制限, 外向き+25°維持） | **失敗**: 歩行破綻（hip_pitch corr=+0.835, V9同等）, R足接地9.2%, 両足空中41.6%, 歩行周波数12Hz(振動), X=0.106, **Genesis関節リミットはソフト制約（新知見）** |
| V12 | [exp008_report_v12.md](exp008_report_v12.md) | 接地足横方向位置ペナルティ（stance_hip_roll_target→stance_foot_lateral_position置換） | X=0.185(+12.1%), **L hip_roll offset -27.5%**, **hip_pitch非対称 -87.1%**, **lateral sway -47.7%**, Yaw=+3.64°(微悪化), Roll std=3.34°(-4.8%), Pitch std=0.75° |
| V13 | [exp008_report_v13.md](exp008_report_v13.md) | hip_roll PDターゲットクランプ（ハード制約、-0.20 rad） | X=0.200(+8.1%), **hip_roll range -57%**, **penalty/positive 0.070(best ever)**, 歩行パターン完全維持(0.88Hz), 内股±25°→±22°(3°改善のみ、オーバーシュート10.6°), Roll std +13.8%, hip_pitch corr -13.1% |
| V14 | [exp008_report_v14.md](exp008_report_v14.md) | hip_roll PDターゲットクランプ強化（-0.20→-0.05 rad） | **内股±22°→±12°(10°改善、FK参照値到達)**, **hip_pitch corr -0.800(+13.8%改善)**, X=0.158(-21.0%), Roll std=5.70°(+50.0%), Yaw=-8.12°(+44.5%悪化), penalty/positive=0.150, 歩行パターン完全維持(0.88Hz) |
| V15 | [exp008_report_v15.md](exp008_report_v15.md) | 横方向速度ペナルティ新設 (base_vel_y=-1.0) | **lateral std -17.7%(48→39.5mm)**, **lateral p2p -32.5%**, **Roll std -13.1%(4.98°)**, **残差std -48.5%**, Yaw=+15.15°(符号反転, 絶対値差+7.0°), Pitch std=1.98°(+85.0%悪化), X=0.146(-7.0%), hip_pitch corr=-0.817(+2.1%), penalty/positive=0.183 |
| V16 | [exp008_report_v16.md](exp008_report_v16.md) | 位相条件付きPDクランプ (hip_roll_clamp_stance_only) | **hip_roll range +126%(0.212→0.48rad, V4レベル回復)**, **lateral std -19.0%(44.7→36.2mm)**, **Roll→lateral増幅率 1.47x→0.75x(能動バランシング復活)**, **Action RMS -54.2%**, **Pitch std -24.5%**, stance hip_roll ±10.4°維持(内股防止継続), Roll std=8.25°(+58.7%悪化), hip_pitch corr=-0.638(-16.6%), X=0.151(-9.0%), base height=0.261m(-10.3%), penalty/positive=0.193 |
| V17 | [exp008_report_v17.md](exp008_report_v17.md) | foot_lateral_velocity削除（バランシング衝突解消） | **penalty/positive=0.153(-20.7%、健全域回復)**, **X=0.168(+11.3%)**, **Roll std=6.93°(-16.0%)**, **Yaw=+15.53°(-22.6%)**, hip_roll range爆発(L=0.827, R=0.879 rad), **遊脚外向き±26-27°(カタパルトパターン)**, lateral std=43.5mm(+20.2%悪化), Roll-lateral R²=0.038(崩壊), チャタリング13回, **ユーザー目視: 脚を外側に変に広げている** |
| V18 | [exp008_report_v18.md](exp008_report_v18.md) | 双方向PDクランプ（遊脚相外向き制限0.20 rad追加） | **カタパルトパターン解消**(swing hip_roll ±8-9°→±0.9-1.8°), **balance stroke -47~-62%**, **Roll std=5.69°(-14.4%)**, X=0.187(+8.7%), 内股±8-9°維持, **Yaw=-85.84°(最悪級, +476%)**, Pitch std=4.51°(+148%), ratio=0.202(最悪), **外向きクランプ0.20 radが厳しすぎ→hip_yaw補償でYawドリフト** |
| V19 | [exp008_report_v19.md](exp008_report_v19.md) | 目標速度引き上げ（V15ベース、lin_vel_x_range [0.15,0.25]→[0.25,0.35]） | **X=0.255(+57.7%)**, 歩行周波数0.88Hz維持, Yaw=-16.08°(符号反転,絶対値改善), 内股±9.6-10.3°維持, **hip_pitch corr=-0.619(-23.0%悪化)**, lateral std=58.0mm(+29.7%悪化), Roll std=5.95°(+15.0%), R²=0.379(-49.6%), ratio=0.187 |
| V20 | [exp008_report_v20.md](exp008_report_v20.md) | 歩行周波数引き上げ（V19ベース、gait_frequency 0.9→1.2 Hz） | **X=0.312(+22.4%)**, **歩行周波数0.88→1.25Hz(+42%)**, **hip_pitch corr=-0.877(+41.7%改善, 史上最高)**, **Yaw=+0.05°(≈0°)**, **lateral std=27.3mm(-52.9%)**, **R²=0.903(+138.6%)**, Roll std=4.18°(-29.8%), Pitch std=1.00°(-54.3%), 内股±9.3°(維持), ratio=0.079, **feet_swing_height+17.5%(唯一悪化)** |
| V21 | [exp008_report_v21.md](exp008_report_v21.md) | stance_foot_lateral_position削除（V20ベース、冗長報酬項目の除去） | X=0.307(-1.6%), Yaw=-1.00°, **lateral std=24.7mm(-9.5%)**, **R²=0.959(+6.2%)**, **残差std=5.0mm(-41.2%)**, Pitch std=0.82°(-18.0%), Roll std=4.21°(不変), 内股±9.5°(維持), ratio=0.078, **hip_pitch corr=-0.733(-16.4%悪化)**, 報酬項目17(推奨範囲到達) |
| V22 | [exp008_report_v22.md](exp008_report_v22.md) | swing_height_target微調整（0.05→0.03 m） | **feet_swing_heightペナルティ逆に+25.5%悪化**（目標引き下げが逆効果）, X=0.307(不変), Yaw=-1.21°, Roll std=4.03°(-5.0%), Pitch std=0.94°(+14.6%), hip_pitch corr=-0.787(+7.3%改善), 内股±9.5°(維持), lateral std=28.3mm(+9.3%), R²=0.787(-15.0%), ratio=0.085(+9.5%) |
| V23 | [exp008_report_v23.md](exp008_report_v23.md) | feet_swing_height scale削減(-8.0→-4.0) + swing_height_target復元(0.03→0.05 m) | **feet_swing_height -58.3%改善**(主目的達成), **ratio 0.085→0.070(V13レベル回復)**, **hip_pitch corr=-0.852(+8.3%)**, **非対称度-11.0%→-0.8%**, Pitch std -17.0%, X=0.297(-3.3%), Yaw=-4.46°, Roll std=4.21°(+4.5%), 内股±9.4°(維持), lateral std=28.9mm(+2.1%), R²=0.735(-6.6%), 歩行周波数1.25Hz(維持) |
| V24 | [exp008_report_v24.md](exp008_report_v24.md) | contact_no_vel 3D化(XY→XYZ) + scale増加(-0.1→-0.3)（接地時かかと叩き対策） | **目視でかかと叩き未改善**。ankle vel std改善はstance中のみ、swing末期は逆に+89.8%加速。**contact_no_velの構造的欠陥発見**: feet_velはリンク並進速度でankle回転を捕捉不可。かかと叩き=PDアンダーダンピング(ζ<1.0)、報酬整形では原理的に解消不可。**Yaw+11.18°**, ankle L/R非対称倍増(23.9→47.7%), hip_pitch corr=-0.719(-15%), Pitch std=1.37°(+69%), R²=0.448(-39%). 内股±9.4°・1.25Hz・ratio 0.073維持。V25推奨: ankle kd=2→5 + contact_no_vel V23復帰 |
| V25 | [exp008_report_v25.md](exp008_report_v25.md) | ankle_pitch kd増加(2→5) + contact_no_vel V23復帰(XY, -0.1) | **V24カスケード悪化完全解消**: Yaw+8.49°→+2.40°(-71.7%), hip_pitch corr=-0.708→-0.851(+20.2%), R²=0.706→0.755(+6.9%). **V23ベースライン回復**: hip_pitch corr -0.851≈-0.852, lateral std 25.7mm(-11.1% vs V23). ankle_pitch vel avg -12%(2.937→2.584). X=0.304, Roll std=4.44°(+5.5% vs V23), 内股±9.2-9.3°(維持), 1.25Hz(維持), ratio=0.074(healthy). かかと叩き視覚改善は目視評価要 |
