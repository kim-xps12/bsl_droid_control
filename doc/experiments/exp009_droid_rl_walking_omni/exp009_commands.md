# EXP009: コマンドリファレンス

**前提**: 全コマンドは `cd rl_ws` 実行後に使用すること（詳細は `AGENTS.md`「コマンド実行形式」参照）。

**重要**: `cd` と後続コマンドを `&&` で連結してはならない。`settings.json` の allow パターンはコマンド先頭からマッチするため、`cd ... && cmd` 形式では許可ダイアログが発生する。`cd` と後続コマンドはbashツール呼び出しを区切って行うこと。bashツールの仕様として別に実行しても前のコマンドの結果は状態として保持されているためこれが可能である。

## 1. トレーニング実行

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_omni_v{VERSION}.py --max_iterations 500
```

オプション:
- `--num_envs`: 並列環境数（デフォルト: 4096）
- `--max_iterations`: 最大イテレーション数（デフォルト: 500）
- `-e`, `--exp_name`: 実験名（デフォルト: `droid-walking-omni-v{VERSION}`）

バックグラウンド実行 + tmux 監視の手順は `/exp009-new-version` スキル（Phase 2）を参照。

## 2. ポリシー評価（GUI付き）

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{VERSION}
```

オプション:
- `--ckpt`: チェックポイント番号（デフォルト: 最新）
- `--duration`: 評価時間（秒）
- `--command VX VY VYAW`: 速度コマンドを固定（方向別評価用）
- `--seed N`: Genesis乱数シード（デフォルト: 1）

## 3. ポリシー評価（ヘッドレス、定量評価用）

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{VERSION} --no-viewer --duration 10
```

このコマンドは以下の定量指標を出力する:
- X/Y移動距離、平均速度
- 姿勢（Roll/Pitch/Yaw）
- hip_pitch相関（交互歩行の指標）
- DOF range sum（動きの大きさ）
- 接地パターン

オプション（Section 2 と共通）:
- `--command VX VY VYAW`: 速度コマンドを固定（方向別評価用）
- `--seed N`: Genesis乱数シード（デフォルト: 1）

このコマンドは実行されて結果がコンソールに現れるまでに読み込み時間がかかるものであるため，あなたが焦って再実行することを禁じる．

## 4. TensorBoard監視

```bash
cd rl_ws
uv run python -m tensorboard.main --logdir logs
# ブラウザで http://localhost:6006 を開く
```

## 5. 分析スクリプト

### 5.1 学習ログ分析 (`analyze_training_log.py`)

```
rl_ws/scripts/analyze_training_log.py
```

```bash
cd rl_ws
uv run python scripts/analyze_training_log.py droid-walking-omni-v{VERSION}
```

出力内容:
1. **利用可能なスカラータグ一覧**: TensorBoardに記録されている全指標
2. **主要指標の推移**:
   - `Train/mean_reward`: 平均報酬
   - `Train/mean_episode_length`: 平均エピソード長
   - `Loss/value_function`: Value関数の損失
   - `Loss/surrogate`: PPOのSurrogate Loss
3. **報酬トレンド分析**:
   - 4分割ごとの平均報酬
   - 最大/最小報酬とそのステップ
   - 収束判定（Last 50 steps のstd）

### 5.2 報酬コンポーネント分析 (`show_reward_components.py`)

```
rl_ws/scripts/show_reward_components.py
```

```bash
cd rl_ws
uv run python scripts/show_reward_components.py droid-walking-omni-v{N-1} droid-walking-omni-v{N}
```

- 複数実験名を引数に並べることで比較表示が可能
- TensorBoardの`Episode/rew_*`タグから各報酬項目の最終ステップ値を表示
- 負の値には⚠️マークが付与される

### 5.3 CSV付き評価（時系列データ生成）

通常の評価コマンドに`--csv`オプションを追加することで、50Hz（0.02s間隔）の時系列CSVを生成する。

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 10 --csv
```

- **出力先**: `logs/droid-walking-omni-v{N}/eval_{ckpt_num}.csv`
- **CSV列構成（45列）**:
  - 時刻: `timestamp`
  - 胴体位置: `base_pos_x`, `base_pos_y`, `base_pos_z`
  - 胴体速度: `base_vel_x`, `base_vel_y`, `base_vel_z`
  - 姿勢角: `roll_deg`, `pitch_deg`, `yaw_deg`
  - 関節角度（10列）: `dof_pos_{L,R}_{hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch}`
  - 関節角速度（10列）: `dof_vel_{L,R}_{同上}`
  - アクション（10列）: `action_0` 〜 `action_9`
  - 接地状態: `contact_left`, `contact_right`（1.0=接地, 0.0=空中）
  - コマンド速度: `cmd_vel_x`, `cmd_vel_y`, `cmd_vel_yaw`

### 5.4 CSV時系列分析 (`analyze_eval_csv.py`)

```
rl_ws/scripts/analyze_eval_csv.py
```

```bash
cd rl_ws
uv run python scripts/analyze_eval_csv.py {N} {N-1} --prefix droid-walking-omni-v
```

- 位置引数: 比較するバージョン番号（2つ以上）。先頭が新バージョン
- `--prefix`: 実験名プレフィックス（**exp009では `droid-walking-omni-v` を指定**、デフォルトは `droid-walking-unitree-v`）
- `--epoch`: 評価エポック番号（デフォルト: 499）

分析内容:
- **歩行周期FFT分析**: hip_pitch/knee_pitchの主周波数、上位3ピーク
- **接地パターン分析**: 両足接地/片足/空中の割合、タップダンスイベント検出、スイング区間統計
- **左右位相関係分析**: L/R hip_pitch相関、ローリング相関、相互相関ラグ、非対称度
- **Yawドリフト分析**: 1秒ごとの推移、定常状態ドリフト速度
- **速度プロファイル分析**: vx/vy/vz統計、高さ安定性
- **関節ダイナミクス分析**: 角速度RMS、L/R比、アクションRMS

定常状態の分析にはt>2sのデータを使用する。

### 5.5 横方向揺れ詳細分析 (`analyze_lateral_sway.py`)

```
rl_ws/scripts/analyze_lateral_sway.py
```

```bash
cd rl_ws
uv run python scripts/analyze_lateral_sway.py {N} {N-1} --prefix droid-walking-omni-v
```

- 位置引数・オプションは `analyze_eval_csv.py` と同一（版番号2つ以上、`--prefix`、`--epoch`）
- `analyze_eval_csv.py` の Section 7 (Roll/Lateral Sway) を補完する**深い分析**に特化

分析内容:
- **Roll周期性分析**: 自己相関による周期性定量化（歩行周期との同期度）、Roll角速度RMS
- **横方向並進変位分析**: 線形ドリフト除去後のstd/peak-to-peak（mm単位）、Y速度std
- **Roll-横方向相関分析**: R²（決定係数）、回帰slope vs 幾何学的予測slope、増幅率、残差分析、位相遅れ
- **接地相/遊脚相別hip_roll分析**: 接地相/遊脚相のhip_roll平均、「バランスストローク」（接地-遊脚デルタ）、外向き修正の利用状況
- **比較サマリテーブル**: 全バージョンの主要指標一覧 + 先頭2バージョンの変化率

横方向並進揺れが課題となった際に特に有用。定常状態の分析にはt>2sのデータを使用する。

### 5.6 振動・高周波分析 (`analyze_vibration.py`)

```
rl_ws/scripts/analyze_vibration.py
```

```bash
cd rl_ws
uv run python scripts/analyze_vibration.py {N} {N-1} --prefix droid-walking-omni-v
```

- 位置引数・オプションは `analyze_eval_csv.py` と同一（版番号2つ以上、`--prefix`、`--epoch`）
- 追加オプション: `--freq-threshold` で高周波成分の閾値周波数を指定（デフォルト: 2.0 Hz）
- `analyze_eval_csv.py` の関節ダイナミクス分析を補完する**振動特化の深い分析**

分析内容:
- **関節角FFT分析**: 各関節の主周波数と高周波成分比率（>閾値Hz のエネルギー比）
- **アクション変化率分析**: 各アクションの時間微分RMS（制御入力の滑らかさ指標）
- **DOF速度ホットスポット分析**: 各関節の角速度RMSとピーク値、L/R比
- **接触切替頻度分析**: 遷移回数・レート、マイクロスタンス/スイング（<0.1s）の検出
- **2バージョン比較サマリ**: 全指標の V/V比一覧。1.3x超の変化に `**` マーク

目視で「ジタバタ」「高周波振動」が指摘された場合に特に有用。`/exp009-analyze` スキルのデータ収集で使用。

## 6. 方向別評価（exp009固有）

全方向歩行の性能を方向ごとに分離評価する。詳細なプロトコルは `exp009_rules.md` Section 5 を参照。

### 6.1 単方向評価

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.3 0.0 0.0
```

`--command VX VY VYAW` は速度コマンドを固定する。環境の `_resample_commands()` が常に指定値を返すよう `command_cfg` のrangeを `[val, val]` に上書きする仕組み。

### 6.2 4方向 x 3試行の一括評価

```bash
cd rl_ws

# Forward (3 trials)
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.3 0.0 0.0 --seed 1
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.3 0.0 0.0 --seed 2
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.3 0.0 0.0 --seed 3

# Backward (3 trials)
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command -0.3 0.0 0.0 --seed 1
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command -0.3 0.0 0.0 --seed 2
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command -0.3 0.0 0.0 --seed 3

# Left (3 trials)
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.0 0.3 0.0 --seed 1
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.0 0.3 0.0 --seed 2
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.0 0.3 0.0 --seed 3

# Right (3 trials)
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.0 -0.3 0.0 --seed 1
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.0 -0.3 0.0 --seed 2
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.0 -0.3 0.0 --seed 3
```

### 6.3 CSV出力先の命名規則

`--command` + `--csv`（パス未指定）の場合、自動命名される:

```
logs/droid-walking-omni-v{N}/eval_{ckpt}_cmd_{vx}_{vy}_{vyaw}_s{seed}.csv
```

例:
- Forward, seed 1: `eval_499_cmd_0.30_0.00_0.00_s1.csv`
- Backward, seed 2: `eval_499_cmd_-0.30_0.00_0.00_s2.csv`
- Left, seed 3: `eval_499_cmd_0.00_0.30_0.00_s3.csv`
