# V7: 関節グループ別 Kp/Kd の導入（Unitree G1参考）

## 概要

全関節一律のPD制御ゲイン（Kp=35.0, Kd=2.0）を、Unitree G1 RL Gymの関節グループ別ゲイン比率を参考にBSL-Droid向けにスケーリングした値に置き換える。膝は最大負荷関節として剛性を高め（Kp=50.0）、足首は柔軟性を重視して剛性を下げる（Kp=20.0）ことで、物理的に合理的なPD制御を実現することを目指す。

## 前バージョンからの改善事項

exp009_task_joint_group_kpkd.md に基づく環境パラメータ変更。報酬構成はV6から変更なし。

| パラメータ | V6値 | V7値 | 変更理由 |
|-----------|------|------|---------|
| kp (hip系) | 35.0（全関節一律） | 35.0（変更なし） | G1基準として維持 |
| kp (knee) | 35.0 | **50.0** | G1ではhipの1.5倍。膝は最大負荷のため高剛性 |
| kp (ankle) | 35.0 | **20.0** | G1ではhipの0.4倍。足首は柔軟性重視 |
| kd (knee) | 2.0 | **3.0** | G1ではhipの2倍。膝の高負荷に対応 |
| kd (ankle) | 5.0 | 5.0（変更なし） | exp008 V25から継承済み |

## 設計詳細

### Unitree G1 参考値とBSL-Droidへのスケーリング

Unitree G1 RL Gymの関節グループ別ゲイン:

| 関節グループ | G1 Kp [N·m/rad] | G1 Kd [N·m·s/rad] | G1 Kp/hip比 |
|---|---|---|---|
| hip (yaw/roll/pitch) | 100 | 2 | 1.0x |
| knee | 150 | 4 | 1.5x |
| ankle | 40 | 2 | 0.4x |

BSL-Droidは総質量約5.8kgでG1より大幅に小さいため、G1の絶対値ではなく**比率**を適用。hip基準のKp=35.0に対して:

| 関節グループ | BSL-Droid Kp | BSL-Droid Kd | 根拠 |
|---|---|---|---|
| hip (yaw/roll/pitch) | 35.0 | 2.0 | 現行値維持（G1基準） |
| knee_pitch | 50.0 | 3.0 | G1比1.5倍。膝は最大負荷 |
| ankle_pitch | 20.0 | 5.0 | G1比0.4倍。Kdはexp008 V25から継承 |

### 実装方式

環境クラス（`droid_env.py`, `droid_env_unitree.py`）に`kp_overrides`機構を追加。既存の`kd_overrides`（V25追加）と同一パターンで実装。デフォルトKp/Kdで全関節を初期化した後、overridesで指定関節のみ上書きする。

### 報酬構成

16項目（V6と完全同一、報酬変更なし）。

## 実行コマンド

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_omni_v7.py --max_iterations 4000
```

## 評価コマンド

```bash
cd rl_ws
# ランダムコマンド評価（従来）
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v7 --no-viewer --duration 10
# 方向別評価（Forward例、4方向x3試行の詳細はexp009_commands.md Section 6参照）
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v7 --no-viewer --duration 20 --csv --command 0.3 0.0 0.0 --seed 1
```

## 結果

（訓練完了後に記入）

### 訓練結果

| 指標 | V7 | V6 | 変化 |
|------|-----|-----|------|
| 最終報酬 | - | - | - |
| エピソード長 | - | - | - |
| Last 50 std | - | - | - |

### 評価結果（ランダムコマンド）

（訓練完了後に記入）

### 方向別評価結果

（訓練完了後に記入）

### 方向間バランス

（訓練完了後に記入）

## 考察と改善案

（訓練完了後に記入）

### 成功点
- （訓練完了後に記入）

### 課題
- （訓練完了後に記入）

### 次バージョンへの提案
- （訓練完了後に記入）

## まとめ

（訓練完了後に記入）

## 備考

- 本バージョンの変更はPD制御ゲインのみ（kp_overrides/kd_overrides）。報酬構成・コマンド設定はV6と完全同一
- exp009_task_joint_group_kpkd.md に基づく実装
- 環境クラスに`kp_overrides`機構を新規追加（`droid_env.py`, `droid_env_unitree.py`）
- 1変更1検証原則に基づき、PD制御ゲインのみを変更（報酬パラメータ変更を含む場合は2パラメータ同時変更となりexp008 V24の教訓に反する）
