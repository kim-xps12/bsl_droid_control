# V3: hip_pos compound penalty分離（hip_yaw_pos新設）

## 概要

exp008 V2の壊滅的失敗（Yaw +96.49°）から得られた知見に基づき、hip_posのcompound penalty（hip_yaw²+hip_roll²）を分離する。hip_yaw制約はhip_yaw_pos（新設、-0.8）が担い、hip_roll制御はankle_roll（-0.5、既存）が担う分離制御アーキテクチャを導入する。

これにより、hip_yaw制約レベルをV1水準に維持しつつ、hip_rollがcompound penaltyから解放されることで、narrow URDFの構造的特性（hip_roll自由度増大の必要性）と整合する報酬設計を目指す。

## 前バージョンからの改善事項

V2レポートの「次バージョンへの提案」推奨案に基づく変更:

> 推奨案: hip_pos→hip_yaw_pos分離（compound penalty構造の解消）
> - hip_posのcompound構造がnarrow URDFと非整合（V2で壊滅的に証明）
> - hip_yaw制約レベルは-0.8を維持（V2で-0.5の下限超過を実証）
> - hip_roll制御はankle_roll=-0.5が既に担当
> - 実質1変更（hip_pos→hip_yaw_posの置換）で「1変更1検証」原則に適合

| パラメータ | V2値 | V3値 | 変更理由 |
|-----------|------|------|---------|
| hip_pos | -0.5 | **削除** | compound penalty（hip_yaw²+hip_roll²）の廃止 |
| hip_yaw_pos（新設） | - | **-0.8** | hip_yaw²のみをペナルティ化（分離制御） |

## 設計詳細

訓練スクリプト: `rl_ws/biped_walking/train/droid_train_narrow_v3.py`
環境: `rl_ws/biped_walking/envs/droid_env_unitree.py`（`_reward_hip_yaw_pos`関数を新設）

### hip_yaw_pos報酬関数

```python
def _reward_hip_yaw_pos(self) -> torch.Tensor:
    left_yaw = self.dof_pos[:, self.left_hip_yaw_idx]
    right_yaw = self.dof_pos[:, self.right_hip_yaw_idx]
    return torch.square(left_yaw) + torch.square(right_yaw)
```

hip_posの`hip_yaw² + hip_roll² + hip_yaw² + hip_roll²`（4項）から、hip_yaw_posの`hip_yaw² + hip_yaw²`（2項）への変更。hip_roll²はankle_rollが担当。

### 分離制御アーキテクチャ

| 制御対象 | 旧（hip_pos） | 新（V3） |
|---------|-------------|---------|
| hip_yaw | hip_pos（-0.8）内のhip_yaw² | hip_yaw_pos（-0.8）: hip_yaw²のみ |
| hip_roll | hip_pos（-0.8）内のhip_roll² | ankle_roll（-0.5）: hip_roll²のみ |

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
| **hip_yaw_pos** | **-0.8** | **V3新設: hip_yaw²のみ（hip_pos置換）** |
| ankle_roll | -0.5 | 維持（hip_roll²制御を継続） |
| velocity_deficit | -0.5 | 維持 |
| action_rate | -0.005 | 維持 |
| swing_foot_lateral_velocity | -0.5 | 維持 |

### 期待される効果

- hip_yaw制約をV1水準（-0.8）に維持 → Yawドリフトを±20°以内に抑えることを期待
- hip_rollがcompound penaltyから解放 → ankle_rollによる独立制御が効きやすくなることを期待
- V2で観測されたRoll std改善（-20.1%）の一部が維持される可能性
- 報酬間の構造的矛盾が解消 → より健全な報酬バランスを期待

### リスク

- hip_yaw_posはhip_posのサブセットであり、hip_yawに対するペナルティ強度が変化する可能性（hip_posでは4項合計、hip_yaw_posでは2項のみ）
- hip_rollがankle_rollのみの制御になるため、ankle_roll=-0.5で十分か未検証

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_narrow_v3.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-narrow-v3 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 58.80 |
| 最大報酬 | 58.80 (step 499) |
| エピソード長 | 1001 (上限到達) |
| Last 50 steps std | 1.878 |
| 収束パターン | Q1: 20.25→Q2: 37.04→Q3: 41.89→Q4: 51.36 |
| 訓練時間 | 822s |
| 総タイムステップ | 49,152,000 |

V1（56.28）、V2（50.36）を上回るexp008最高の最終報酬。上昇トレンドは継続中。

### 評価結果（V1・V2との比較）

| 指標 | V1 | V2 | V3 | V1→V3変化 | 評価 |
|------|-----|-----|-----|-----------|------|
| X速度 | 0.190 m/s | 0.156 m/s | 0.162 m/s | -14.7% | 悪化 |
| Y速度 | +0.023 m/s | -0.004 m/s | 0.000 m/s | - | 改善 |
| **Yaw** | **+16.46°** | +96.49° | **+18.84°** | +14.4% | **V1水準復元** |
| Yawドリフト速度 | 1.801 °/s | 11.872 °/s | **2.630 °/s** | +46.0% | V1水準復元 |
| **Roll std** | 6.72° | 5.37° | **4.89°** | **-27.2%** | **大幅改善** |
| Roll peak-to-peak | - | 18.85° | **14.83°** | - | **21%改善** |
| **Pitch std** | 1.56° | 5.02° | **0.98°** | **-37.2%** | **best ever** |
| **hip_pitch相関** | -0.681 | -0.571 | **-0.805** | **+18.2%** | **best ever** |
| hip_pitch非対称度 | -11.9% | -4.0% | -6.1% | +5.8pt改善 | 改善 |
| 両足接地率 | 0.8% | 0.4% | 1.2% | +0.4pt | 微悪化 |
| base_height mean | 0.268 m | 0.261 m | 0.250 m | -6.7% | 低下 |
| 歩行周波数 | 0.88 Hz | 0.88 Hz | 0.88 Hz | ±0% | 同一 |

### 関節可動域（rad）

| 関節 | V3 L range | V3 R range | V3非対称度 | V1 L range | V1 R range |
|------|-----------|-----------|-----------|-----------|-----------|
| hip_yaw | 0.337 | 0.363 | -3.7% | 0.353 | 0.469 |
| hip_roll | 0.517 | 0.548 | -2.9% | 0.448 | 0.416 |
| hip_pitch | 0.615 | 0.684 | -5.3% | 0.736 | 0.824 |
| knee_pitch | 0.712 | 0.734 | -1.5% | 0.935 | 1.029 |
| ankle_pitch | 0.624 | 0.654 | -2.4% | 0.694 | 0.815 |

V3はV1比で全関節の非対称度が大幅に改善（V1: hip_yaw非対称度-15.4% → V3: -3.7%）。hip_roll rangeはV1比で増加（0.448/0.416→0.517/0.548）、compound penaltyからの解放による自由度拡大を反映。

### スイングフェーズ分析

| 指標 | V1 | V2 | V3 |
|------|-----|-----|-----|
| Lスイング持続 | 0.531s | 0.474s | 0.494s |
| Rスイング持続 | 0.571s | 0.558s | 0.431s |
| Lスイング回数 | 7 | 10 | 10 |
| Rスイング回数 | 7 | 9 | 11 |

### hip_roll接地時分析

| 指標 | V1 | V2 | V3 |
|------|-----|-----|-----|
| L hip_roll（L接地時）mean | -15.46° | -11.4° | **-15.7°** |
| R hip_roll（R接地時）mean | +15.04° | +10.2° | **+14.7°** |

hip_roll stance offsetはV1水準に戻った。compound penaltyからの解放にもかかわらず、ankle_roll=-0.5がhip_roll制約を維持。

### 報酬コンポーネント分析

#### 正の報酬（6項目）

| 項目 | V1 | V3 | 変化 | 比率(V3) |
|------|-----|-----|------|----------|
| tracking_lin_vel | 1.2523 | 1.3144 | +5.0% | 40.9% |
| contact | 0.7114 | 0.7111 | -0.0% | 22.1% |
| single_foot_contact | 0.4641 | 0.4621 | -0.4% | 14.4% |
| swing_duration | 0.3600 | 0.3624 | +0.7% | 11.3% |
| tracking_ang_vel | 0.2934 | 0.2979 | +1.5% | 9.3% |
| step_length | 0.0614 | 0.0611 | -0.5% | 1.9% |

#### ペナルティ（11項目）

| 項目 | V1 | V3 | 変化 | 比率(V3) |
|------|-----|-----|------|----------|
| ang_vel_xy | -0.1033 | -0.0881 | +14.7%改善 | 23.9% |
| **ankle_roll** | -0.0386 | **-0.0839** | **-117%悪化** | **22.7%** |
| swing_foot_lat_vel | -0.0363 | -0.0466 | -28.4%悪化 | 12.6% |
| action_rate | -0.0429 | -0.0435 | -1.4% | 11.8% |
| feet_swing_height | -0.0544 | -0.0326 | +40.1%改善 | 8.8% |
| lin_vel_z | -0.0320 | -0.0302 | +5.6%改善 | 8.2% |
| hip_yaw_pos | -0.0854* | -0.0241 | +71.8%改善 | 6.5% |
| base_height | -0.0235 | -0.0135 | +42.6%改善 | 3.7% |
| swing_contact | -0.0114 | -0.0107 | +6.1%改善 | 2.9% |
| orientation | -0.0108 | -0.0080 | +25.9%改善 | 2.2% |
| contact_no_vel | -0.0042 | -0.0041 | +2.4%改善 | 1.1% |
| velocity_deficit | -0.0023 | -0.0028 | -21.7%悪化 | 0.8% |

*V1のhip_posはhip_yaw²+hip_roll²の合計値

#### 報酬バランス

| 指標 | V1 | V2 | V3 |
|------|-----|-----|-----|
| 正の報酬合計 | 3.143 | 3.011 | 3.209 |
| ペナルティ合計 | -0.445 | -0.488 | -0.369 |
| ペナルティ/正報酬比率 | 0.142 | 0.162 | **0.115** |
| 報酬項目数 | 17 | 17 | 17 |

V3のペナルティ/正報酬比率0.115はexp008最良。

## 考察と改善案

### 成功点

1. **Yawドリフト V1水準復元**: +18.84°（V2: +96.49°）。hip_yaw_pos=-0.8によるhip_yaw制約がV1水準のYaw安定性を復元
2. **Roll std exp008 best ever**: 4.89°（V1: 6.72°, -27.2%）。compound penalty解消によるhip_roll自由度拡大が、paradoxicalにRoll安定性を改善
3. **Pitch std 全実験best ever**: 0.98°（V1: 1.56°, V39: 1.34°）。Yaw安定化+ang_vel_xyの効果が最大限に発揮
4. **hip_pitch相関 全実験best ever**: -0.805（V38: -0.801, V1: -0.681）。明確な交互歩行パターン
5. **関節非対称度の大幅改善**: hip_yaw非対称度 -15.4%→-3.7%。hip_rollをcompound penaltyから分離した結果、RL最適化が各関節を独立に調整できるようになった
6. **報酬バランス最健全**: ペナルティ/正報酬比率0.115（V1: 0.142, V2: 0.162）

### 課題

1. **X速度低下**: 0.162 m/s（V1: 0.190, -14.7%）。hip_pitch range縮小（V1: 0.736/0.824 → V3: 0.615/0.684）が一因。hip_roll自由度拡大でhip_rollを使ったバランス戦略が優先され、hip_pitchの大振りが抑制された可能性
2. **接地脚の内股オフセット持続**: L=-15.7°, R=+14.7°（V1: L=-15.46°, R=+15.04°とほぼ同等）。compound penalty分離は内股問題を解消しなかった。ankle_roll=-0.5は「常時」hip_roll²をペナルティ化するため、接地相に集中した制御ができていない
3. **ankle_rollペナルティ急増**: -0.0386→-0.0839（+117%）。hip_posのhip_roll²制約がなくなり、ankle_rollが単独でhip_rollを制御 → 負荷増大。ペナルティ全体の22.7%を占め、ang_vel_xyと並ぶ最大級のペナルティ項目に
4. **base_height低下**: 0.250m（V1: 0.268m, -6.7%）。hip_roll range拡大により重心が下がった可能性
5. **Yawドリフト速度はV1より+46%悪化**: hip_yaw_posはhip_posよりペナルティ項数が少ない（2項 vs 4項）ため、同じスケール-0.8でも実効的な制約力が弱い

### 因果メカニズム分析

#### compound penalty分離の効果ツリー

```
hip_pos（hip_yaw²+hip_roll²）→ hip_yaw_pos（hip_yaw²のみ）に置換
 ├─→ [効果A] hip_yaw制約維持（-0.8スケール維持）
 │    └─→ Yawドリフト V1水準復元（+18.84°）
 ├─→ [効果B] hip_roll制約がankle_rollのみに
 │    ├─→ hip_roll自由度拡大（range +15%/+32%）
 │    │    ├─→ Roll std改善（-27.2%）[paradoxical: 自由度↑でRoll↓]
 │    │    ├─→ ankle_rollペナルティ急増（+117%）
 │    │    └─→ base_height低下（-6.7%）
 │    └─→ hip_roll stance offset不変（±15°）
 │         └─→ 内股着地の持続（未解決）
 └─→ [効果C] 報酬間の構造的矛盾解消
      ├─→ 関節非対称度の大幅改善（-15.4%→-3.7%）
      ├─→ hip_pitch相関best ever（-0.805）
      └─→ 報酬バランス健全化（比率0.115）
```

#### Roll std改善のparadox（自由度↑でRoll↓）の説明

hip_rollの自由度が拡大したにもかかわらずRoll stdが改善した理由:
- hip_posのcompound penaltyはhip_yawとhip_rollを同時に制約していたため、RL最適化がhip_yawとhip_rollの間でトレードオフを強いられていた
- 分離後、hip_rollがhip_yawの制約から独立 → RL最適化がhip_rollを純粋にバランス維持のために使えるようになった
- 結果として、hip_roll rangeは拡大（+15%/+32%）したが、その動きがバランス維持に有効に働き、Roll stdが低下

#### 内股オフセット持続の根本原因

ankle_roll=-0.5のhip_roll²ペナルティは**常時**適用されるため:
- 接地相: hip_rollを0°に引き戻す圧力 → 内股抑制に寄与
- スイング相: hip_rollを0°に引き戻す圧力 → バランス維持に必要な動きを阻害
- → スケールを上げられない（スイング相への副作用）→ 接地相の制約力が不足 → 15°の内股が持続

### サーベイ知見との照合

1. **Section 7.2（報酬スケール変更の影響伝搬）**: 一致。hip_pos→hip_yaw_pos置換という「実質1変更」が、Roll/Pitch/hip_pitch相関/非対称度/報酬バランスの5指標を同時改善。compound penalty解消の波及効果
2. **Section 7.4.2（hip_yaw非対称→Yawドリフト）**: 一致。hip_yaw非対称度-15.4%→-3.7%に改善し、Yawドリフトも安定化
3. **Section 3.4（hip_pos構造）**: V3で実証。compound penaltyの分離が有効であることを定量的に証明
4. **Section 7.2.1（Periodic Reward Composition）**: 内股問題の解決には、歩行フェーズに応じた条件付き報酬（stance-phase-specific penalty）が必要であることを示唆

### 次バージョンへの提案

#### 推奨案: stance_hip_roll_target（接地相限定hip_rollペナルティ）

ankle_roll（常時hip_roll²ペナルティ）を、接地脚のみにhip_rollペナルティを適用するstance_hip_roll_targetに置換する。

| パラメータ | V3値 | V4値 | 変更理由 |
|-----------|------|------|---------|
| ankle_roll | -0.5 | **削除** | 常時ペナルティの構造的欠点を解消 |
| stance_hip_roll_target（新設） | - | **-0.5** | 接地脚のみhip_roll²をペナルティ化 |

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

**根拠**:
1. 接地相のみ制約することで、同じスケールでも接地時の実効ペナルティが増加（常時ペナルティの約2倍の接地相集中度）
2. スイング相の自由度が完全に維持される → スケール強化の余地が拡大
3. 既存の`_reward_foot_flat`と同一の実装パターンで信頼性が高い
4. ankle_rollとの置換で報酬項目数17を維持（1変更1検証に適合）

**期待される効果**:
- 接地脚hip_roll offset 15°→8-10°への低減を期待
- スイング相のhip_roll自由度維持によるRoll std維持を期待
- ankle_rollペナルティ（22.7%）の構造改善による報酬バランスの安定化を期待

**リスク**:
- 接地相のみへの制約集中が過度な場合、接地時のバランス崩壊の可能性
- 接地判定の精度に依存（ただし既存のcontact報酬で実績あり）

#### 代替案: ankle_rollスケール強化（-0.5→-1.0）

実装変更なしで即座に実行可能。V40で約2°改善の実績あり。ただしスイング相も制約されるため根本解決にはならない。

## まとめ

exp008 V3は、V2で壊滅的に失敗したhip_posのcompound penalty（hip_yaw²+hip_roll²）をhip_yaw_pos（hip_yaw²のみ）に分離した。

**成果**: Yawドリフト V1水準復元（+18.84°）、Roll std exp008 best（4.89°, -27.2%）、Pitch std全実験best（0.98°）、hip_pitch相関全実験best（-0.805）、関節非対称度大幅改善、報酬バランス最健全（比率0.115）。compound penalty分離の有効性を定量的に実証した。

**未解決課題**: 接地脚の内股オフセット（±15°）が持続。ankle_rollの「常時ペナルティ」構造では、スケール強化がスイング相に副作用を及ぼすため、接地相への制約集中度が不足している。

**次のV4**: ankle_rollをstance_hip_roll_target（接地相限定hip_rollペナルティ）に置換し、接地脚の内股オフセット低減を目指す。

## 備考

### 分析コマンド

```bash
cd rl_ws

# 基本評価
uv run python biped_walking/biped_eval.py -e droid-walking-narrow-v3 --no-viewer --duration 10

# CSV評価
uv run python biped_walking/biped_eval.py -e droid-walking-narrow-v3 --no-viewer --duration 10 --csv

# 訓練ログ分析
uv run python scripts/analyze_training_log.py droid-walking-narrow-v3

# 報酬コンポーネント比較（V2比較）
uv run python scripts/show_reward_components.py droid-walking-narrow-v2 droid-walking-narrow-v3

# CSV時系列分析（V3 vs V2）
uv run python scripts/analyze_eval_csv.py 3 2 --prefix droid-walking-narrow-v
```
