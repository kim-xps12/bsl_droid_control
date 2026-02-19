# EXP009: コマンドリファレンス

**前提**: 全コマンドは `cd rl_ws` 実行後に使用すること（詳細は `AGENTS.md`「コマンド実行形式」参照）。

**重要**: `cd` と後続コマンドを `&&` で連結してはならない。`settings.json` の allow パターンはコマンド先頭からマッチするため、`cd ... && cmd` 形式では許可ダイアログが発生する。`cd` と後続コマンドはbashツール呼び出しを区切って行うこと。bashツールの仕様として別に実行しても前のコマンドの結果は状態として保持されているためこれが可能である。

## 1. トレーニング実行

```bash
uv run python biped_walking/train/droid_train_omni_v{VERSION}.py --max_iterations 500
```

オプション:
- `--num_envs`: 並列環境数（デフォルト: 4096）
- `--max_iterations`: 最大イテレーション数（デフォルト: 500）
- `-e`, `--exp_name`: 実験名（デフォルト: `droid-walking-omni-v{VERSION}`）

バックグラウンド実行 + tmux 監視の手順は `/exp009-new-version` スキル（Phase 2）を参照。

## 2. ポリシー評価（GUI付き）

```bash
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{VERSION}
```

オプション:
- `--ckpt`: チェックポイント番号（デフォルト: 最新）
- `--duration`: 評価時間（秒）
- `--command VX VY VYAW`: 速度コマンドを固定（方向別評価用）
- `--seed N`: Genesis乱数シード（デフォルト: 1）

## 3. ポリシー評価（ヘッドレス、定量評価用）

```bash
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{VERSION} --no-viewer --duration 10
```

出力指標: X/Y移動距離、平均速度、姿勢（Roll/Pitch/Yaw）、hip_pitch相関、DOF range sum、接地パターン

オプションは Section 2 と共通。このコマンドは読み込み時間がかかるため、焦って再実行することを禁じる。

## 4. TensorBoard監視

```bash
uv run python -m tensorboard.main --logdir logs
# ブラウザで http://localhost:6006 を開く
```

## 5. 分析スクリプト

追加分析でスクリプトを使いたい場合、まず本セクションから用途に合うものを探し、ない場合のみ `scripts/` 直下に汎用引数インターフェースで新規作成すること。

5.2〜5.8 の共通インターフェース:
- 位置引数: バージョン番号（2つ以上、先頭が新バージョン）
- `--prefix droid-walking-omni-v`（exp009では必須指定）
- `--epoch`: 評価エポック番号（デフォルト: 499）

### 5.1 学習ログ分析 (`analyze_training_log.py`)

**使用場面**: 学習完了後の収束確認

```bash
uv run python scripts/analyze_training_log.py droid-walking-omni-v{N}
```

出力: スカラータグ一覧、主要指標推移（mean_reward, episode_length, loss）、4分割トレンド、収束判定（Last 50 steps のstd）

### 5.2 報酬コンポーネント比較 (`show_reward_components.py`)

**使用場面**: 報酬設計変更の効果確認、ペナルティ過剰の検出

```bash
uv run python scripts/show_reward_components.py droid-walking-omni-v{N-1} droid-walking-omni-v{N}
```

TensorBoardの`Episode/rew_*`タグから各報酬項目の最終値を比較表示。3つ以上のバージョン比較可能。負値に⚠️マーク付与。

### 5.3 CSV付き評価（時系列データ生成）

通常の評価コマンドに`--csv`を追加で50Hz（0.02s間隔）の時系列CSVを生成する。

```bash
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 10 --csv
```

- **出力先**: `logs/droid-walking-omni-v{N}/eval_{ckpt_num}.csv`
- **CSV列構成（45列）**: timestamp, base_pos/vel_{x,y,z}, roll/pitch/yaw_deg, dof_pos/vel_{L,R}_{hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch}, action_{0-9}, contact_{left,right}, cmd_vel_{x,y,yaw}

### 5.4 CSV時系列総合分析 (`analyze_eval_csv.py`)

**使用場面**: 毎バージョンの標準分析（必須）。最も包括的なスクリプト。

```bash
uv run python scripts/analyze_eval_csv.py {N} {N-1} --prefix droid-walking-omni-v
```

出力: 歩行周期FFT、接地パターン（タップダンス検出含む）、左右位相関係（相関・非対称度）、Yawドリフト（コマンド切替検出付き）、速度プロファイル、関節ダイナミクス（角速度RMS・L/R比）。定常状態(t>2s)を使用。

### 5.5 横方向揺れ深掘り (`analyze_lateral_sway.py`)

**使用場面**: 目視で横揺れ・Roll角の大きさが観察された場合

```bash
uv run python scripts/analyze_lateral_sway.py {N} {N-1} --prefix droid-walking-omni-v
```

5.4のRoll/Lateral Swayセクションを補完する深い分析。出力: Roll周期性（自己相関）、横方向変位(mm)、Roll-横方向相関(R²・増幅率・位相遅れ)、接地相/遊脚相別hip_roll（バランスストローク）、比較サマリテーブル。

### 5.6 振動・高周波分析 (`analyze_vibration.py`)

**使用場面**: 目視でジッター・震え・高周波振動が観察された場合

```bash
uv run python scripts/analyze_vibration.py {N} {N-1} --prefix droid-walking-omni-v
```

追加オプション: `--freq-threshold`（デフォルト: 2.0 Hz）。出力: 関節角FFT（高周波成分比率）、アクション変化率RMS、DOF速度ホットスポット、接触切替頻度（マイクロスタンス検出）。1.3x超変化に`**`マーク。

### 5.7 クロスバージョントレンド (`analyze_cross_version_trend.py`)

**使用場面**: 3バージョン以上の横断比較、中長期的な改善/悪化トレンドの確認

```bash
uv run python scripts/analyze_cross_version_trend.py {N-2} {N-1} {N} --prefix droid-walking-omni-v
```

追加オプション: `--include-rewards`。出力: 標準化指標のMarkdownトレンドテーブル（歩行品質・安定性・報酬の横断比較）。

### 5.8 Hip Yaw非対称・位相分析 (`analyze_hip_yaw_and_phase.py`)

**使用場面**: Yaw非対称の目視観察、hip_yawアクションの左右差が疑われる場合
**前提**: 固定コマンドCSV（`eval_*_cmd_*.csv`）が必要

```bash
uv run python scripts/analyze_hip_yaw_and_phase.py {N} {N-1} --prefix droid-walking-omni-v
```

出力: hip_yaw L/R action・位置の接地相/遊脚相別分析、hip_roll FFT、Roll-接地位相関係、高周波前半/後半比較（カスケードリスク評価）。

### 5.9 バージョン固有スクリプト（参考）

以下はパスがハードコードされており直接再利用不可。新規スクリプト作成時の分析ロジックの参考用。

| スクリプト | 対象 | 分析内容 |
|-----------|------|---------|
| `analyze_contact_chattering.py` | exp008 narrow-v1/v2 | 接地チャタリング検出 |
| `analyze_contact_chattering_v2.py` | exp008 narrow-v1/v2 | 接地中ankle振動の深層分析 |
| `analyze_knee_asymmetry_detail.py` | omni-v8/v9 | 膝トルク非対称度・体高Yaw相関 |
| `analyze_yaw_drift_v2.py` | exp008固有 | Yawドリフト分析 |
| `analyze_yaw_drift_v2_deep.py` | exp008固有 | Yawドリフト深掘り |
| `analyze_omni_v4_v3.py` | omni-v3/v4 | 全方向モデル比較 |
| `jitter_analysis_v4_v3.py` | v3/v4 | 関節ジッター比較 |
| `stance_analysis_v1v2.py` | v1/v2 | スタンス相分析 |

### 5.10 バッチ評価（並列実行）(`batch_eval.py`)

**使用場面**: ランダム評価 + 方向別評価（4方向 × 3シード）を並列実行して高速化

```bash
# 全評価（random + directional）を3並列で実行
uv run python scripts/batch_eval.py \
    -e droid-walking-omni-v{N} droid-walking-omni-v{N-1} \
    --vx-max 0.3 --vy-max 0.15 --workers 3

# 方向別評価のみ
uv run python scripts/batch_eval.py \
    -e droid-walking-omni-v{N} --vx-max 0.3 --vy-max 0.15 --no-random

# 特定の固定コマンドのみ追加（V{N-1}のYaw評価用FWDなど）
uv run python scripts/batch_eval.py \
    -e droid-walking-omni-v{N-1} --no-random --no-directional \
    --extra-cmd "0.3 0.0 0.0"
```

オプション:
- `--workers N`: 最大並列プロセス数（デフォルト: 3、GPU VRAM ~1GB/プロセス）
- `--vx-max`, `--vy-max`: 方向別評価の速度上限（`--no-directional` 時は不要）
- `--seeds N [N ...]`: 方向別評価のシード（デフォルト: 1 2 3）
- `--no-random`: ランダムコマンド評価をスキップ
- `--no-directional`: 方向別評価をスキップ
- `--extra-cmd "VX VY VYAW"`: 追加の固定コマンド評価（複数回指定可）
- `--force`: 既存CSVがあっても再実行
- `--dry-run`: 実行計画のみ表示

既存CSVは自動スキップ。全ジョブ完了後にサマリ（成功数・失敗数・実行時間・速度向上倍率）を表示。

### 5.11 方向別評価集計 (`analyze_directional_eval.py`)

**使用場面**: 固定コマンドCSV（4方向 × 3 seed）から方向別の評価指標を集計・比較

```bash
# 1バージョンの方向別集計
uv run python scripts/analyze_directional_eval.py {N} --prefix droid-walking-omni-v

# 複数バージョン比較
uv run python scripts/analyze_directional_eval.py {N} {N-1} --prefix droid-walking-omni-v

# 速度上限をカスタマイズ
uv run python scripts/analyze_directional_eval.py {N} --prefix droid-walking-omni-v --vx-max 0.30 --vy-max 0.20
```

オプション:
- `--epoch`: 評価エポック番号（デフォルト: 3999）
- `--prefix`: 実験名プレフィックス（デフォルト: `droid-walking-omni-v`）
- `--vx-max`: FWD/BWD方向のコマンド速度上限（デフォルト: 0.30）
- `--vy-max`: LFT/RGT方向のコマンド速度上限（デフォルト: 0.30）

出力: 各方向の cmd_vel, ortho_vel, yaw_final, roll_std, hip_pitch_corr, tracking_rate, contact_ratio L/R, hip_pitch_asym の mean ± std、および方向間バランス（追従率の全方向平均・ばらつき・FWD/BWD差・LFT/RGT差）。

### 5.12 ランダム評価サマリ抽出 (`extract_eval_summary.py`)

**使用場面**: ランダムコマンド評価CSVからレポートテーブル用の補足指標を抽出

```bash
uv run python scripts/extract_eval_summary.py {N} {N-1} --prefix droid-walking-omni-v
```

オプション:
- `--epoch`: 評価エポック番号（デフォルト: 3999）
- `--prefix`: 実験名プレフィックス（デフォルト: `droid-walking-omni-v`）

出力: Pitch std（定常状態 t>2s）、接地時間比 L/R。`analyze_eval_csv.py` が出力しない指標を補完する。

## 6. 方向別評価（exp009固有）

全方向歩行の性能を方向ごとに分離評価する。詳細なプロトコルは `exp009_rules.md` Section 5 を参照。

### 6.1 単方向評価

```bash
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.3 0.0 0.0
```

`--command VX VY VYAW` は速度コマンドを固定する。環境の `_resample_commands()` が常に指定値を返すよう `command_cfg` のrangeを `[val, val]` に上書きする仕組み。

### 6.2 4方向 x 3試行の一括評価

各方向 `--command {VX} {VY} 0.0` × `--seed {1,2,3}` の組み合わせで実行:

| 方向 | VX | VY |
|------|-----|-----|
| Forward | 0.3 | 0.0 |
| Backward | -0.3 | 0.0 |
| Left | 0.0 | 0.3 |
| Right | 0.0 | -0.3 |

```bash
# 例: Forward, seed 1
uv run python biped_walking/biped_eval.py -e droid-walking-omni-v{N} --no-viewer --duration 20 --csv --command 0.3 0.0 0.0 --seed 1
```

全12コマンド（4方向 × 3シード）を順次実行する。

### 6.3 CSV出力先の命名規則

`--command` + `--csv`（パス未指定）の場合、自動命名される:

```
logs/droid-walking-omni-v{N}/eval_{ckpt}_cmd_{vx}_{vy}_{vyaw}_s{seed}.csv
```

例: Forward seed 1 → `eval_499_cmd_0.30_0.00_0.00_s1.csv`, Backward seed 2 → `eval_499_cmd_-0.30_0.00_0.00_s2.csv`
