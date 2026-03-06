# V22: Contact Sensor復帰 + A案（ankle_pitch_rangeペナルティ）

## 概要

V21で失敗したZ座標閾値ベースからContact Sensorに戻し、V20で挙げた「つま先突き動作の解消」のA案（ankle_pitch_rangeペナルティ）を実施する。

**注**: V20でContact Sensorは既に成功しているため、ベースライン確認のみの実験は省略し、V22で直接A案を実施する。

**段階的試行計画**:

| バージョン | 内容 | 目的 |
|-----------|------|------|
| **V22** | Contact Sensor復帰 + A案 | つま先突き抑制（本レポート） |
| **V23** | V22 + B案（air_time_offset短縮） | 足上げ動機維持 |
| **V24** | V23 + C案（contact_no_vel強化） | 滑らかな着地 [条件付き] |

## 前バージョンからの改善事項

### V20/V21の結果サマリー

| バージョン | 接地検出方式 | 片足接地率 | 主な課題 |
|-----------|------------|----------|---------|
| V20 | Contact Sensor | 91.4% | **つま先突き動作**（ankle_pitch可動域大） |
| V21 | Z座標閾値(0.10m) | 0.2% | **すり足に退行**（閾値が高すぎた） |

### V20で挙げられた「つま先突き動作の解消」試行案

V20レポートの「次バージョンへの提案」セクションで、以下の3つの試行案が提案された：

| # | 方策 | 内容 | 期待効果 |
|---|------|------|---------|
| A | ankle_pitch_rangeペナルティ追加 | 遊脚時のankle_pitch角度を制限 | つま先が下を向く動きを抑制 |
| B | air_time_offset短縮 | 0.3s → 0.2s に変更 | feet_air_time報酬を正に変え、足上げ動機を維持 |
| C | contact_no_vel強化 | -0.1 → -0.2 に強化 | 接地時の足速度をより厳しく制限 |

**推奨戦略**: A + Bの組み合わせ

### V22での変更

| パラメータ | V21値 | V22値 | 変更理由 |
|-----------|-------|-------|---------|
| `use_contact_sensor` | False | **True** | Contact Sensorに復帰 |
| `ankle_pitch_range` | なし | **-0.3** | 遊脚時のankle_pitch角度を制限（A案） |
| `ankle_pitch_limit` | なし | **0.3 rad** | ankle_pitchの許容範囲（A案） |

## 設計詳細

### 1. Contact Sensor復帰

V20で成功したContact Sensorを再導入する。

```python
env_cfg = {
    "use_contact_sensor": True,  # Contact Sensorに復帰
}
```

### 2. A案: ankle_pitch_rangeペナルティ

**設計原理**:
- V20課題: つま先を地面に突く動作が発生
- 原因: ankle_pitchの可動域が大きい（L: 0.484 rad, R: 0.369 rad）
- 対策: 遊脚（非接地）時のankle_pitch角度を制限

つま先が下を向く（ankle_pitchが正方向に大きくなる）動きを抑制することで、足を下ろす際の姿勢を改善する。

**報酬関数**: `_reward_ankle_pitch_range`（droid_env_unitree.pyに追加済み）

```python
def _reward_ankle_pitch_range(self):
    """遊脚時のankle_pitch角度制限ペナルティ（V22追加）"""
    contacts = self._get_foot_contacts()
    ankle_pitch_limit = self.reward_cfg.get("ankle_pitch_limit", 0.3)  # rad

    # ankle_pitchのデフォルトからの偏差
    left_ankle_dev = torch.abs(self.dof_pos[:, self.left_ankle_idx] - self.default_dof_pos[self.left_ankle_idx])
    right_ankle_dev = torch.abs(self.dof_pos[:, self.right_ankle_idx] - self.default_dof_pos[self.right_ankle_idx])

    # 許容範囲を超えた分をペナルティ
    left_excess = torch.clamp(left_ankle_dev - ankle_pitch_limit, min=0)
    right_excess = torch.clamp(right_ankle_dev - ankle_pitch_limit, min=0)

    # 遊脚（非接地）時のみペナルティ
    left_penalty = torch.square(left_excess) * (~contacts[:, 0]).float()
    right_penalty = torch.square(right_excess) * (~contacts[:, 1]).float()

    return left_penalty + right_penalty
```

### 3. 報酬設計（18項目）

```python
reward_cfg = {
    "tracking_sigma": 0.25,
    "base_height_target": 0.20,
    "swing_height_target": 0.05,
    "gait_frequency": 1.2,
    "contact_threshold": 0.05,  # フォールバック用
    "air_time_offset": 0.3,
    "dof_vel_limits": 44.0,
    "soft_dof_vel_limit": 0.9,
    "ankle_pitch_limit": 0.3,   # V22追加: A案

    "reward_scales": {
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

        # === V22追加: A案 ===
        "ankle_pitch_range": -0.3,  # 遊脚時のankle_pitch角度制限

        # === 関節速度制限 ===
        "dof_vel_limits": -0.3,

        # === エネルギー効率ペナルティ ===
        "torques": -1e-5,
        "action_rate": -0.01,
        "dof_acc": -2.5e-7,
    },
}
```

## 期待される効果

| 指標 | V20値 | V22目標 | 測定方法 |
|------|-------|---------|---------|
| 片足接地率 | 91.4% | > 80% | `biped_eval.py` Contact Pattern |
| ankle_pitch可動域 L | 0.484 rad | < 0.35 rad | DOF range分析 |
| ankle_pitch可動域 R | 0.369 rad | < 0.35 rad | DOF range分析 |
| つま先突き動作 | あり | なし/軽減 | 目視評価 |
| X速度 | 0.204 m/s | 0.15-0.25 m/s | `biped_eval.py` |
| hip_pitch相関 | -0.550 | < 0（交互維持） | `biped_eval.py` |

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_unitree_v22.py --max_iterations 500
```

## 評価コマンド

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-unitree-v22 --no-viewer --duration 10
```

## 結果

### 訓練結果

| 指標 | 値 |
|------|-----|
| 最終報酬 | 46.78 |
| エピソード長 | 1001（フルエピソード達成） |
| 収束ステップ | ~100（step 100でエピソード長1001に到達） |
| 訓練時間 | 約14分 |

### 評価結果

| 指標 | V20値 | V22値 | 変化 | 評価 |
|------|-------|-------|------|------|
| X速度 | 0.204 m/s | 0.209 m/s | +2.5% | ✓ 目標範囲内、改善 |
| Y速度 | - | 0.005 m/s | - | ✓ 直進維持 |
| 片足接地率 | 91.4% | 89.6% | -2.0% | ✓ 80%以上維持 |
| 両足接地率 | 6.4% | 8.8% | +2.4% | △ やや増加 |
| 両足空中率 | 2.2% | 1.6% | -0.6% | ✓ 減少 |
| hip_pitch相関 | -0.550 | -0.524 | - | ✓ 交互歩行維持 |
| ankle_pitch可動域 L | 0.484 rad | 0.451 rad | -6.8% | △ A案効果あり |
| ankle_pitch可動域 R | 0.369 rad | 0.487 rad | +32.0% | ✗ A案無効（悪化） |
| 胴体高さ | - | 0.262±0.007 m | - | ✓ 安定 |
| Yawドリフト | - | +1.96° | - | ✓ 微小 |

### ユーザーフィードバック（目視評価）

- **良い点**: ゆったり大股の歩容に近づいてきた
- **課題**: 歩容一周期の間に片足で断続的に2回地面を叩く（「タタン，タタン」＝**タップダンス歩容**）

### 報酬成分比較（V20 vs V22）

| 報酬項目 | V20 | V22 | 変化 | 評価 |
|----------|-----|-----|------|------|
| tracking_lin_vel | 1.0811 | 0.6602 | -38.9% | ⚠️ 低下 |
| single_foot_contact | 0.2058 | 0.1243 | -39.6% | ⚠️ 低下 |
| contact | 0.2395 | 0.1493 | -37.7% | ⚠️ 低下 |
| feet_air_time | -0.0325 | -0.0202 | +37.8% | ✓ 改善（負が小さく） |
| ankle_pitch_range | N/A | -0.0006 | - | ⚠️ 効果小 |

## 考察と改善案

### タップダンス歩容の原因分析

#### 主因: feet_air_time報酬が依然として負

- V20: -0.0325、V22: -0.0202（両方とも**負**）
- `air_time_offset=0.3s`が実際の遊脚時間より長い
- 足を長く上げておく動機が不足
- **結果**: 一度上げた足がすぐに接地し、再度上げるパターン（タップダンス）が発生

歩行周波数1.2Hzでは1周期=0.833s、片足の遊脚期は約0.42s。しかし、0.3s以上の連続滞空が報酬の閾値となっているため、0.3sに満たない短い足上げが繰り返される傾向がある。

#### 副因: A案（ankle_pitch_rangeペナルティ）の効果不足

- ペナルティ値: **-0.0006**（非常に小さい、ほぼ発動していない）
- 左足: 0.484→0.451 rad（-6.8%、やや減少 → **効果あり**）
- 右足: 0.369→0.487 rad（+32.0%、**増加** → **逆効果**）

ペナルティ係数-0.3が弱すぎる、または報酬関数の設計（遊脚判定のタイミング）に問題がある可能性。

#### 結果: 片足接地パターンの不安定化

- `single_foot_contact`報酬: 0.2058→0.1243（-39.6%低下）
- 両足接地率: 6.4%→8.8%（+2.4%増加）
- 「タタン」という二重接地パターンの発生

### 成功点

- **X速度向上**: 0.209 m/s（V20の0.204より+2.5%向上、目標範囲0.15-0.25 m/s内）
- **片足接地率維持**: 89.6%（80%以上の目標達成）
- **交互歩行維持**: hip_pitch相関 -0.524（負の値を維持）
- **安定した訓練**: フルエピソード達成、報酬収束（std: 0.043）
- **胴体安定性**: 高さ0.262±0.007m、Roll/Pitch変動小

### 課題

- **タップダンス歩容**: 歩容一周期の間に片足で断続的に2回地面を叩く
- **A案の効果不足**: ankle_pitch_rangeペナルティがほとんど発動していない（-0.0006）
- **feet_air_time負**: 足上げ時間が`air_time_offset=0.3s`を下回っている
- **右足のankle_pitch増加**: 0.369→0.487 rad（A案が右足で逆効果）
- **tracking_lin_vel低下**: 1.0811→0.6602（-38.9%）

### 次バージョン（V23）への提案

段階的試行計画に基づき、**B案（air_time_offset短縮）を優先して実施**する。

| 優先度 | 変更項目 | 変更内容 | 根拠 |
|--------|----------|----------|------|
| 1 | `air_time_offset` | 0.3 → **0.2** | **B案**: feet_air_time報酬を正に転換、タップダンス軽減 |
| 2 | A案の見直し | V24で検討 | 右足で効果が出ていない、設計見直しが必要 |

**推奨戦略**: V23ではB案のみを実施し、以下を確認する：
1. `feet_air_time`報酬が正に転じるか
2. タップダンス歩容が軽減されるか
3. 片足接地率が維持されるか

A案（ankle_pitch_range）は効果が限定的だったが、悪影響も小さいため現状維持とする。本格的な見直しはV24で検討。

---

## 後続バージョンの計画

### V23: B案追加（air_time_offset短縮）

**目的**: 足上げ動機を維持しつつ、feet_air_time報酬を正の値にする

| パラメータ | V22値 | V23値 | 変更理由 |
|-----------|---------|---------|---------|
| A案の設定 | 有効 | **維持** | A案の効果を継続 |
| `air_time_offset` | 0.3s | **0.2s** | feet_air_time報酬を正に |

**成功基準**:
- feet_air_time報酬 > 0（V20は-0.033で負だった）
- 足上げ動作の維持
- 片足接地率の維持（> 80%）

### V24: C案追加（contact_no_vel強化）[条件付き]

**注**: V22, V23で十分な効果が得られた場合はスキップ可能

| パラメータ | V23値 | V24値 | 変更理由 |
|-----------|---------|---------|---------|
| A案 + B案の設定 | 有効 | **維持** | 効果を継続 |
| `contact_no_vel` | -0.1 | **-0.2** | 接地時の足速度をより厳しく制限 |

---

## 報酬項目数の確認

| バージョン | 報酬項目数 | 備考 |
|-----------|-----------|------|
| V20 | 17 | ベースライン |
| V22 | **18** | +ankle_pitch_range |
| V23 | **18** | V22と同一（パラメータ変更のみ） |
| V24 | **18** | V23と同一（パラメータ変更のみ） |

exp007_reward_design.mdの推奨範囲（15-17項目）を若干超えるが、1項目の追加のため許容範囲内。

## まとめ

V22はContact Sensor復帰とA案（ankle_pitch_rangeペナルティ）を組み合わせた実験である。

**達成事項**:
- Contact Sensorへの復帰に成功（V21の失敗から回復）
- X速度0.209 m/s、片足接地率89.6%を達成
- 交互歩行を維持（hip_pitch相関 -0.524）

**未達成事項**:
- **タップダンス歩容**の発生（片足で2回地面を叩く動作）
- A案（ankle_pitch_rangeペナルティ）の効果が限定的（右足で逆効果）
- feet_air_time報酬が依然として負（-0.0202）

**原因分析**:
- タップダンス歩容の主因は`air_time_offset=0.3s`が長すぎること
- 足を長く上げておく動機が不足し、短い接地が繰り返される

**次バージョン（V23）への方針**:
- B案（air_time_offset短縮: 0.3→0.2）を実施
- feet_air_time報酬を正に転換し、タップダンス歩容を軽減する

## 備考

- 学習スクリプト: `rl_ws/biped_walking/train/droid_train_unitree_v22.py`
- 環境クラス: `rl_ws/biped_walking/envs/droid_env_unitree.py`
- 新規報酬関数: `_reward_ankle_pitch_range`（環境クラスに追加済み）
- 参照: [exp007_report_v20.md](exp007_report_v20.md)（つま先突き動作の課題、次バージョンへの提案A）
- 参照: [exp007_report_v21.md](exp007_report_v21.md)（Z座標閾値ベースの失敗分析）
