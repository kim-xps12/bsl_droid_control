# EXP008: コマンドリファレンス

**前提**: 全コマンドは `cd rl_ws` 実行後に使用すること（詳細は `AGENTS.md`「コマンド実行形式」参照）。

## 1. トレーニング実行

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_narrow_v{VERSION}.py --max_iterations 500
```

オプション:
- `--num_envs`: 並列環境数（デフォルト: 4096）
- `--max_iterations`: 最大イテレーション数（デフォルト: 500）
- `-e`, `--exp_name`: 実験名（デフォルト: `droid-walking-narrow-v{VERSION}`）

バックグラウンド実行 + tmux 監視の手順は `/exp008-new-version` スキル（Phase 2）を参照。

## 2. ポリシー評価（GUI付き）

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-narrow-v{VERSION}
```

オプション:
- `--ckpt`: チェックポイント番号（デフォルト: 最新）
- `--duration`: 評価時間（秒）

## 3. ポリシー評価（ヘッドレス、定量評価用）

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-narrow-v{VERSION} --no-viewer --duration 10
```

このコマンドは以下の定量指標を出力する:
- X/Y移動距離、平均速度
- 姿勢（Roll/Pitch/Yaw）
- hip_pitch相関（交互歩行の指標）
- DOF range sum（動きの大きさ）
- 接地パターン

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
uv run python scripts/analyze_training_log.py droid-walking-narrow-v{VERSION}
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
uv run python scripts/show_reward_components.py droid-walking-narrow-v{N-1} droid-walking-narrow-v{N}
```

- 複数実験名を引数に並べることで比較表示が可能
- TensorBoardの`Episode/rew_*`タグから各報酬項目の最終ステップ値を表示
- 負の値には⚠️マークが付与される

### 5.3 CSV付き評価（時系列データ生成）

通常の評価コマンドに`--csv`オプションを追加することで、50Hz（0.02s間隔）の時系列CSVを生成する。

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-narrow-v{N} --no-viewer --duration 10 --csv
```

- **出力先**: `logs/droid-walking-narrow-v{N}/eval_{ckpt_num}.csv`
- **CSV列構成（42列）**:
  - 時刻: `timestamp`
  - 胴体位置: `base_pos_x`, `base_pos_y`, `base_pos_z`
  - 胴体速度: `base_vel_x`, `base_vel_y`, `base_vel_z`
  - 姿勢角: `roll_deg`, `pitch_deg`, `yaw_deg`
  - 関節角度（10列）: `dof_pos_{L,R}_{hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch}`
  - 関節角速度（10列）: `dof_vel_{L,R}_{同上}`
  - アクション（10列）: `action_0` 〜 `action_9`
  - 接地状態: `contact_left`, `contact_right`（1.0=接地, 0.0=空中）

### 5.4 CSV時系列分析 (`analyze_eval_csv.py`)

```
rl_ws/scripts/analyze_eval_csv.py
```

```bash
cd rl_ws
uv run python scripts/analyze_eval_csv.py {N} {N-1} --prefix droid-walking-narrow-v
```

- 位置引数: 比較するバージョン番号（2つ以上）。先頭が新バージョン
- `--prefix`: 実験名プレフィックス（**exp008では `droid-walking-narrow-v` を指定**、デフォルトは `droid-walking-unitree-v`）
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
uv run python scripts/analyze_lateral_sway.py {N} {N-1} --prefix droid-walking-narrow-v
```

- 位置引数・オプションは `analyze_eval_csv.py` と同一（版番号2つ以上、`--prefix`、`--epoch`）
- `analyze_eval_csv.py` の Section 7 (Roll/Lateral Sway) を補完する**深い分析**に特化

分析内容:
- **Roll周期性分析**: 自己相関による周期性定量化（歩行周期との同期度）、Roll角速度RMS
- **横方向並進変位分析**: 線形ドリフト除去後のstd/peak-to-peak（mm単位）、Y速度std
- **Roll-横方向相関分析**: R²（決定係数）、回帰slope vs 幾何学的予測slope、増幅率、残差分析、位相遅れ
- **接地相/遊脚相別hip_roll分析**: 接地相/遊脚相のhip_roll平均、「バランスストローク」（接地-遊脚デルタ）、外向き修正の利用状況
- **比較サマリテーブル**: 全バージョンの主要指標一覧 + 先頭2バージョンの変化率

横方向並進揺れが課題となったV14以降で特に有用。定常状態の分析にはt>2sのデータを使用する。
