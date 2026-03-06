# V4: 接地相限定hip_rollペナルティ（stance_hip_roll_target）

## 概要

V3で確認されたankle_rollの構造的欠点（常時ペナルティによる接地相への制約集中度不足、内股offset ±15°の持続）を解消するため、ankle_rollをstance_hip_roll_target（接地相限定hip_roll²ペナルティ）に置換する。接地脚のみにhip_roll²ペナルティを適用することで、スイング相の自由度を完全に維持しつつ内股offset低減を目指す。

## 前バージョンからの改善事項

V3レポートの「次バージョンへの提案」推奨案に基づく変更:

> 推奨案: stance_hip_roll_target（接地相限定hip_rollペナルティ）
> - ankle_roll（常時hip_roll²ペナルティ）の構造的欠点を解消
> - 接地脚のみにhip_roll²ペナルティを適用
> - ankle_rollとの置換で報酬項目数17を維持（1変更1検証に適合）

| パラメータ | V3値 | V4値 | 変更理由 |
|-----------|------|------|---------|
| ankle_roll | -0.5 | **削除** | 常時ペナルティの構造的欠点を解消 |
| stance_hip_roll_target（新設） | - | **-0.5** | 接地脚のみhip_roll²をペナルティ化 |

## 設計詳細

訓練スクリプト: `rl_ws/biped_walking/train/droid_train_narrow_v4.py`
環境: `rl_ws/biped_walking/envs/droid_env_unitree.py`（`_reward_stance_hip_roll_target`関数を新設）

### stance_hip_roll_target報酬関数

```python
def _reward_stance_hip_roll_target(self) -> torch.Tensor:
    contacts = self._get_foot_contacts()
    if contacts is None:
        return torch.zeros(self.num_envs, dtype=gs.tc_float, device=gs.device)
    left_roll = self.dof_pos[:, self.left_hip_roll_idx]
    right_roll = self.dof_pos[:, self.right_hip_roll_idx]
    penalty = (torch.square(left_roll) * contacts[:, 0].float()
             + torch.square(right_roll) * contacts[:, 1].float())
    return penalty
```

ankle_rollの`hip_roll_L² + hip_roll_R²`（常時）から、stance_hip_roll_targetの`hip_roll_L² × contact_L + hip_roll_R² × contact_R`（接地相限定）への変更。

### ankle_roll vs stance_hip_roll_target比較

| 特性 | ankle_roll（V3） | stance_hip_roll_target（V4） |
|------|-----------------|---------------------------|
| 適用タイミング | 常時（全フレーム） | 接地脚のみ |
| スイング相の影響 | あり（自由度制約） | なし（完全自由） |
| 接地相の実効ペナルティ | スケール × hip_roll² | スケール × hip_roll² |
| スケール強化余地 | 低い（スイング相副作用） | 高い（接地相限定） |
| 実装パターン | 単純二乗和 | contact-conditioned（_reward_foot_flatと同一） |

### 報酬スケール一覧（17項目、変更1箇所のみ）

| 報酬名 | スケール | 備考 |
|--------|---------|------|
| tracking_lin_vel | 1.5 | 維持 |
| tracking_ang_vel | 0.5 | 維持 |
| swing_duration | 2.0 | 維持 |
| swing_contact_penalty | -0.7 | 維持 |
| contact | 0.4 | 維持 |
| single_foot_contact | 0.5 | 維持 |
| step_length | 0.8 | 維持 |
| lin_vel_z | -2.0 | 維持 |
| ang_vel_xy | -0.1 | 維持 |
| orientation | -0.5 | 維持 |
| base_height | -5.0 | 維持 |
| feet_swing_height | -8.0 | 維持 |
| contact_no_vel | -0.1 | 維持 |
| hip_yaw_pos | -0.8 | 維持 |
| **stance_hip_roll_target** | **-0.5** | **V4新設: 接地脚のみhip_roll²（ankle_roll置換）** |
| velocity_deficit | -0.5 | 維持 |
| action_rate | -0.005 | 維持 |
| swing_foot_lateral_velocity | -0.5 | 維持 |

### 期待される効果

- 接地脚hip_roll offset 15°→8-10°への低減を期待
- スイング相のhip_roll自由度維持によるRoll std維持を期待
- ankle_rollペナルティ（22.7%）の構造改善による報酬バランスの安定化を期待

### リスク

- 接地相のみへの制約集中が過度な場合、接地時のバランス崩壊の可能性
- 接地判定の精度に依存（ただし既存のcontact報酬で実績あり）
- 同スケール(-0.5)で適用範囲を約50%に限定するため、実効ペナルティが不足する可能性

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_narrow_v4.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-narrow-v4 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 60.43 |
| 最大報酬 | 60.43 (step 499) |
| エピソード長 | 1001 (上限到達) |
| Last 50 steps std | 1.919 |
| 収束パターン | Q1: 20.45→Q2: 36.82→Q3: 41.35→Q4: 52.37 |
| 訓練時間 | 822s |
| 総タイムステップ | 49,152,000 |

V3（58.80）を上回るexp008最高の最終報酬。上昇トレンドは継続中。

### 評価結果（V3との比較）

| 指標 | V3 | V4 | 変化 | 評価 |
|------|-----|-----|------|------|
| X速度 | 0.162 m/s | 0.167 m/s | +3.1% | 改善 |
| Y速度 | 0.000 m/s | -0.003 m/s | - | 同等 |
| **Yaw** | **+18.84°** | **+7.51°** | **-60.1%** | **大幅改善** |
| Yawドリフト速度 | 2.630 °/s | **1.105 °/s** | **-58.0%** | **大幅改善** |
| **Roll std** | **4.89°** | **3.92°** | **-19.8%** | **改善（exp008 best更新）** |
| Roll peak-to-peak | 14.83° | 12.86° | -13.3% | 改善 |
| Pitch std | 0.98° | 0.98° | ±0% | 維持（全実験best維持） |
| hip_pitch相関 | -0.805 | -0.787 | -2.2% | 微低下 |
| hip_pitch非対称度 | -6.1% | -7.0% | -0.9pt | 微悪化 |
| 両足接地率 | 1.2% | 0.8% | -0.4pt | 改善 |
| base_height mean | 0.250 m | 0.245 m | -2.0% | 微低下 |
| 歩行周波数 | 0.88 Hz | 0.88 Hz | ±0% | 同一 |

### 関節可動域（rad）

| 関節 | V4 L range | V4 R range | V4非対称度 | V3 L range | V3 R range |
|------|-----------|-----------|-----------|-----------|-----------|
| hip_yaw | 0.370 | 0.366 | +0.5% | 0.337 | 0.363 |
| hip_roll | 0.692 | 0.543 | +12.1% | 0.517 | 0.548 |
| hip_pitch | 0.605 | 0.693 | -6.8% | 0.615 | 0.684 |
| knee_pitch | 0.824 | 0.881 | -3.3% | 0.712 | 0.734 |
| ankle_pitch | 0.669 | 0.762 | -6.5% | 0.624 | 0.654 |

hip_roll L rangeが大幅増加（0.517→0.692, +33.8%）。スイング相のhip_roll自由度解放を反映。hip_roll R rangeはほぼ維持（0.548→0.543）。L/R非対称度が増加（V3: -2.9% → V4: +12.1%）。

### スイングフェーズ分析

| 指標 | V3 | V4 |
|------|-----|-----|
| Lスイング持続 | 0.494s | 0.490s |
| Rスイング持続 | 0.431s | 0.536s |
| Lスイング回数 | 10 | 10 |
| Rスイング回数 | 11 | 9 |

Rスイング持続時間が増加（0.431s→0.536s）し、L/Rの差が縮小。

### hip_roll接地時分析

| 指標 | V1 | V3 | V4 | V3→V4変化 |
|------|-----|-----|-----|-----------|
| L hip_roll（L接地時）mean | -15.46° | -23.71° | **-24.41°** | +3.0%悪化 |
| R hip_roll（R接地時）mean | +15.04° | +22.28° | **+24.93°** | +11.9%悪化 |

内股offsetはV3より悪化。同スケール(-0.5)で適用範囲を約50%に限定したため、実効ペナルティが不足。

### 横方向安定性分析

| 指標 | V3 | V4 | 変化 |
|------|-----|-----|------|
| 横方向detrended std | 20.2 mm | **14.9 mm** | **-26.2%改善** |
| 横方向drift rate | 28.0 mm/s | **1.9 mm/s** | **-93.2%改善** |

### 報酬コンポーネント分析

#### 正の報酬（6項目）

| 項目 | V3 | V4 | 変化 | 比率(V4) |
|------|-----|-----|------|----------|
| tracking_lin_vel | 1.3144 | 1.3883 | +5.6% | 40.9% |
| contact | 0.7111 | 0.7418 | +4.3% | 21.8% |
| single_foot_contact | 0.4621 | 0.4831 | +4.5% | 14.2% |
| swing_duration | 0.3624 | 0.3889 | +7.3% | 11.5% |
| tracking_ang_vel | 0.2979 | 0.3361 | +12.8% | 9.9% |
| step_length | 0.0611 | 0.0570 | -6.7% | 1.7% |

#### ペナルティ（12項目）

| 項目 | V3 | V4 | 変化 | 比率(V4) |
|------|-----|-----|------|----------|
| stance_hip_roll_target | -0.0839* | -0.0770 | +8.2%改善 | 22.6% |
| ang_vel_xy | -0.0881 | -0.0748 | +15.1%改善 | 22.0% |
| swing_foot_lat_vel | -0.0466 | -0.0468 | -0.4% | 13.8% |
| action_rate | -0.0435 | -0.0421 | +3.2%改善 | 12.4% |
| feet_swing_height | -0.0326 | -0.0373 | -14.4%悪化 | 11.0% |
| lin_vel_z | -0.0302 | -0.0291 | +3.6%改善 | 8.6% |
| hip_yaw_pos | -0.0241 | -0.0269 | -11.6%悪化 | 7.9% |
| base_height | -0.0135 | -0.0128 | +5.2%改善 | 3.8% |
| swing_contact | -0.0107 | -0.0096 | +10.3%改善 | 2.8% |
| orientation | -0.0080 | -0.0064 | +20.0%改善 | 1.9% |
| contact_no_vel | -0.0041 | -0.0040 | +2.4%改善 | 1.2% |
| velocity_deficit | -0.0028 | -0.0031 | -10.7% | 0.9% |

*V3はankle_rollの値

#### 報酬バランス

| 指標 | V1 | V3 | V4 |
|------|-----|-----|-----|
| 正の報酬合計 | 3.143 | 3.209 | 3.395 |
| ペナルティ合計 | -0.445 | -0.369 | -0.340 |
| ペナルティ/正報酬比率 | 0.142 | 0.115 | **0.100** |
| 報酬項目数 | 17 | 17 | 17 |

V4のペナルティ/正報酬比率0.100はexp008全体best。

## 考察と改善案

### 成功点

1. **Yawドリフト大幅改善**: +18.84°→+7.51°（-60.1%）。exp008全体bestを大幅更新。ドリフト速度も2.630→1.105 °/s（-58.0%）。予想外の大改善
2. **Roll std exp008 best更新**: 4.89°→3.92°（-19.8%）。接地相限定ペナルティによりスイング相のhip_roll自由度が確保され、バランス維持に有効活用
3. **横方向安定性の大幅改善**: detrended std 20.2→14.9mm（-26.2%）、drift rate 28.0→1.9mm/s（-93.2%）。直進性が劇的に向上
4. **X速度微改善**: 0.162→0.167 m/s（+3.1%）。スイング相自由度確保が歩行効率に寄与
5. **報酬バランスexp008全体best**: ペナルティ/正報酬比率0.100。正報酬合計も3.395でexp008最高
6. **Pitch std維持**: 0.98°（全実験best維持）

### 課題

1. **内股offsetが悪化**: L=-24.41°, R=+24.93°（V3: L=-23.71°, R=+22.28°）。接地相限定ペナルティのスケール-0.5では実効ペナルティが不足。同スケールで適用範囲を約50%に限定したため、全体ペナルティ量がankle_roll比で-8.2%減少
2. **hip_roll L/R非対称度増加**: 12.1%（V3: -2.9%）。L range 0.692 vs R range 0.543。スイング相自由度拡大による非対称性
3. **hip_pitch相関微低下**: -0.787（V3: -0.805, -2.2%）。依然として高い交互歩行品質を維持
4. **base_height微低下**: 0.245m（V3: 0.250m）。hip_roll自由度拡大による重心低下

### 因果メカニズム分析

#### stance_hip_roll_target置換の効果ツリー

```
ankle_roll（常時hip_roll²）→ stance_hip_roll_target（接地相限定hip_roll²）に置換
 ├─→ [効果A] スイング相hip_roll完全解放
 │    ├─→ hip_roll L range大幅増加（+33.8%）
 │    ├─→ スイング中のバランス維持能力向上
 │    │    ├─→ Roll std改善（-19.8%）
 │    │    ├─→ 横方向detrended std改善（-26.2%）
 │    │    └─→ X速度微改善（+3.1%）
 │    └─→ 接地時impact時の体幹角運動量伝達減少
 │         └─→ Yawドリフト大幅改善（-60.1%）
 ├─→ [効果B] 実効ペナルティ低下（適用範囲約50%化）
 │    ├─→ stance_hip_roll_targetペナルティ減少（-8.2%）
 │    └─→ 内股offset悪化（L: +3.0%, R: +11.9%）
 └─→ [効果C] 報酬間相互作用の改善
      ├─→ ang_vel_xy改善（+15.1%）= 体幹角速度の実質低下
      ├─→ orientation改善（+20.0%）
      └─→ 報酬バランス健全化（比率0.100）
```

#### Yaw大幅改善の説明

V4で最も予想外だったYaw改善（-60.1%）のメカニズム:

1. スイング相でのhip_roll自由度解放により、スイング脚がバランス維持のためにhip_rollを自由に使える
2. これにより、接地切替時の衝撃で体幹に伝達される角運動量が減少
3. hip_yaw非対称度はV3（-3.7%）とV4（+0.5%）で大きく変化しておらず、hip_yaw自体の制約変更ではない
4. ankle_rollの常時ペナルティがhip_rollのバランス利用を妨げ、間接的にYawドリフトを悪化させていたことが判明

#### 内股offset悪化の根本原因

1. stance_hip_roll_targetスケール-0.5は、ankle_roll-0.5と同値だが適用範囲が約50%
2. 接地率は約49-50%（片足あたり）なので、実効ペナルティは`-0.5 × 0.5 = -0.25`相当
3. ankle_rollの実効ペナルティは`-0.5 × 1.0 = -0.5`相当
4. つまりhip_rollへの制約力が半減 → offsetが増加
5. スケール強化（-1.0以上）で実効ペナルティをankle_roll水準に戻す必要がある

### サーベイ知見との照合

1. **Section 7.2.1（Periodic Reward Composition）**: 一致。接地相限定ペナルティにより歩行フェーズに応じた報酬設計が機能。ただし同スケールでの単純置換では実効ペナルティが不足することも確認
2. **Section 7.2（報酬スケール変更の影響伝搬）**: 一致。ankle_roll→stance_hip_roll_target置換という「実質1変更」で、Yaw/Roll/lateral/報酬バランスの4指標が同時改善。波及効果の大きさはV3のcompound penalty分離と同程度
3. **Section 7.4.2（hip_yaw非対称→Yawドリフト）**: 部分一致。hip_yaw非対称度自体はV3/V4で大きく変化しないが、Yawが大幅改善。hip_rollの自由度変更がYawに間接的に影響するメカニズムはサーベイの予測範囲外
4. **Section 8.1（物理構造の影響）**: 関連。接地相限定ペナルティは「報酬の時間構造」を変更するもので、物理構造変更（URDF）に次ぐ効果的なアプローチであることを示唆

### 次バージョンへの提案

#### 推奨案: stance_hip_roll_targetスケール強化（-0.5→-1.0）

| パラメータ | V4値 | V5値 | 変更理由 |
|-----------|------|------|---------|
| stance_hip_roll_target | -0.5 | **-1.0** | 接地相hip_rollペナルティ強化（内股offset低減） |

**根拠**:
1. V4で内股offsetが悪化した主因は実効ペナルティ不足（接地相限定により約50%に低下）
2. -1.0にすることで実効ペナルティを`-1.0 × 0.5 ≈ -0.5`に回復 = ankle_roll水準
3. 接地相限定のため、スケール強化してもスイング相への副作用がない（ankle_rollとの本質的差異）
4. 予想ペナルティ: -0.0770×2 ≈ -0.154。ペナルティ/正報酬比率は~0.13程度（V3の0.115よりやや大きいが許容範囲）
5. V4で確認されたYaw/Roll/lateral改善を維持しつつ、内股問題に集中的にアプローチ

**期待される効果**:
- 接地脚hip_roll offset 25°→15°以下への低減を期待
- スイング相自由度維持によるYaw/Roll改善の継続を期待
- ユーザ目標「内股着地の解消」への前進を期待

**リスク**:
- ペナルティ強化により接地時のバランス維持が阻害される可能性（ただし接地相限定のためリスクは限定的）

#### 代替案: stance_hip_roll_targetにターゲット角度導入

0°ではなく5°程度の外股角度をターゲットとして設定。ただしターゲット角度の最適値が不明なため、まずスケール強化（推奨案）を先行させる。

## 目視確認結果

ユーザによる目視確認の結果:

- **Yawドリフト**: 大きく改善された（定量結果と一致）
- **内股問題**: 全く改善されていない。脚が内側に入り込む問題が顕著に残存

定量分析では内股offset L=-24.41°, R=+24.93°（V3比悪化）であり、目視での「全く改善されていない」という評価と一致する。V5では内股問題の改善を最優先とする。

## まとめ

exp008 V4は、V3のankle_roll（常時hip_roll²ペナルティ）をstance_hip_roll_target（接地相限定hip_roll²ペナルティ）に置換した。

**成果**: Yawドリフト exp008 best大幅更新（+7.51°, V3比-60.1%）、Roll std exp008 best更新（3.92°, -19.8%）、横方向drift rate -93.2%改善、X速度微改善（+3.1%）、報酬バランスexp008全体best（比率0.100）。Pitch std 0.98°（全実験best維持）。接地相限定ペナルティによるスイング相自由度解放が、予想外にYaw・Roll・横方向安定性を同時改善した。

**未解決課題**: 内股offset悪化（L: -24.41°, R: +24.93°, V3比+3-12%）。目視でも全く改善が確認されず。同スケール-0.5で適用範囲を50%に限定したため、実効ペナルティが不足。

**次のV5**: stance_hip_roll_targetスケール強化（-0.5→-1.0）により、実効ペナルティをankle_roll水準に回復させ、内股offset低減を最優先で目指す。

## 備考

### 分析コマンド

```bash
cd rl_ws

# 基本評価
uv run python biped_walking/biped_eval.py -e droid-walking-narrow-v4 --no-viewer --duration 10

# CSV評価
uv run python biped_walking/biped_eval.py -e droid-walking-narrow-v4 --no-viewer --duration 10 --csv

# 訓練ログ分析
uv run python scripts/analyze_training_log.py droid-walking-narrow-v4

# 報酬コンポーネント比較（V3比較）
uv run python scripts/show_reward_components.py droid-walking-narrow-v3 droid-walking-narrow-v4

# CSV時系列分析（V4 vs V3）
uv run python scripts/analyze_eval_csv.py 4 3 --prefix droid-walking-narrow-v
```
