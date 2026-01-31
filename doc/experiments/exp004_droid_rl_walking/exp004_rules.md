# EXP004: 実験の進め方とルール

本ドキュメントは、exp004（BSL-Droid Simplified二脚ロボットの強化学習歩容獲得実験）を進める上での手順、コマンド、スクリプト、ルールを記載する。

## 1. バージョン管理とコード信頼性

強化学習環境とトレーニングスクリプトのバージョン管理において、以下の原則を厳守する（exp002から継承）:

### 1.1 完全コピー原則

新バージョン（V(n+1)）を作成する際は、前バージョン（Vn）のスクリプトを完全にコピーしてから編集を開始する。

```bash
# 例: V19からV20を作成する場合
cd rl_ws/biped_walking
cp train/droid_train_v19.py train/droid_train_v20.py
```

### 1.2 最小変更原則

コピー後は、改善目的に必要な箇所**のみ**を編集する:
- クラス名・docstring・報酬関数・報酬スケールなど、変更が必要な部分だけを特定
- 既存の動作検証済みコードは可能な限り保持
- 不必要な再実装や構造変更を避ける

### 1.3 段階的検証

各バージョンで以下を確認:
- 構文エラーがないこと（実行前に検証）
- 依存パラメータ（command_cfg等）の整合性
- 初期化処理の完全性（qpos次元数等）

### 1.4 理由

- ゼロから再実装すると、細かい設定ミス（qpos次元、command_cfg、feet_indices初期化等）が混入しやすい
- 既存の動作確認済みコードをベースにすることで、新規追加部分のデバッグに集中できる
- 信頼性の高い反復的改善が可能

---

## 2. 実験記録のルールと責務の分離

本実験における記録ファイルの責務分離は、`AGENTS.md` の「実験記録の責務分離」ルールに準拠する。

| ファイル | 責務 |
|----------|------|
| `exp004_droid_rl_walking.md` | 各バージョンの目的・変更内容・結果を累積（失敗も含む） |
| `exp004_rules.md` | 本ドキュメント（実験ルール・手順・コマンド） |
| `exp004_reward_design_survey.md` | 報酬設計に関する先行研究調査**のみ** |

---

## 3. 主要コマンド

### 3.1 トレーニング実行

```bash
cd rl_ws
uv run python biped_walking/train/droid_train_v{VERSION}.py --max_iterations 500
```

オプション:
- `--num_envs`: 並列環境数（デフォルト: 4096）
- `--max_iterations`: 最大イテレーション数（デフォルト: 500）
- `-e`, `--exp_name`: 実験名（デフォルト: `droid-walking-v{VERSION}`）

### 3.2 ポリシー評価（GUI付き）

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-v{VERSION}
```

オプション:
- `--ckpt`: チェックポイント番号（デフォルト: 最新）
- `--duration`: 評価時間（秒）

### 3.3 ポリシー評価（ヘッドレス、定量評価用）

```bash
cd rl_ws
uv run python biped_walking/biped_eval.py -e droid-walking-v{VERSION} --no-viewer --duration 10
```

このコマンドは以下の定量指標を出力する:
- X/Y移動距離、平均速度
- 姿勢（Roll/Pitch/Yaw）
- hip_pitch相関（交互歩行の指標）
- DOF range sum（動きの大きさ）
- 接地パターン

このコマンドは実行されて結果がコンソールに現れるまでに読み込み時間がかかるものであるため，あなたが焦って再実行することを禁じる．

### 3.4 TensorBoard監視

```bash
cd rl_ws
uv run tensorboard --logdir logs
# ブラウザで http://localhost:6006 を開く
```

---

## 4. 学習ログ分析スクリプト

### 4.1 スクリプトの場所

```
rl_ws/scripts/analyze_training_log.py
```

### 4.2 使い方

```bash
cd rl_ws
uv run python scripts/analyze_training_log.py droid-walking-v{VERSION}
```

### 4.3 出力内容

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

### 4.4 データの解釈

#### 収束判定

| Last 50 steps std | 判定 | 意味 |
|-------------------|------|------|
| < 0.01 | 収束 | 学習が安定、追加学習の効果は限定的 |
| 0.01 〜 0.1 | ほぼ収束 | 微調整レベル |
| > 0.1 | 変動中 | まだ学習途中、または不安定 |

#### 報酬トレンドの解釈

- **単調増加 → 横ばい**: 正常な収束パターン
- **単調増加（500stepでも）**: まだ学習途中、追加学習の余地あり
- **変動が激しい**: 報酬設計に問題がある可能性
- **低い値で収束**: 局所最適解に陥った可能性

#### エピソード長の解釈

- **1001（上限）に到達**: 終了条件に引っかかっていない（良好）
- **上限より低い**: 早期終了が発生している（終了条件が厳しい可能性）

#### 個別報酬項目の確認

専用スクリプトで個別報酬を確認:

```bash
cd rl_ws
uv run python scripts/show_reward_components.py droid-walking-v{VERSION}

# 複数バージョンを比較する場合
uv run python scripts/show_reward_components.py droid-walking-v18 droid-walking-v19
```

**注意点**:
- 報酬項目が**負の値**になっている場合、設計意図と逆の学習が行われている可能性
- 主報酬（tracking_lin_vel）がペナルティ合計より小さい場合、「動かない」が最適解になりやすい

---

## 5. ファイル構成

```
doc/experiments/exp004_droid_rl_walking/
├── exp004_droid_rl_walking.md      # 実験結果と知見（本体）
├── exp004_reward_design_survey.md  # 先行研究調査
└── exp004_rules.md                 # 本ドキュメント（実験ルール）

rl_ws/
├── biped_walking/
│   ├── envs/
│   │   └── droid_env.py            # 環境定義（統一版）
│   ├── train/
│   │   ├── droid_train_v18.py      # 各バージョンのトレーニングスクリプト
│   │   ├── droid_train_v19.py
│   │   └── ...
│   └── biped_eval.py               # 評価スクリプト（統一版）
├── scripts/
│   ├── analyze_training_log.py     # 学習ログ分析スクリプト
│   └── show_reward_components.py   # 個別報酬項目表示スクリプト
└── logs/
    ├── droid-walking-v18/          # 各バージョンの学習ログ
    ├── droid-walking-v19/
    └── ...
```

---

## 6. 報酬設計の原則

V9〜V19の実験から得られた原則:

### 6.1 報酬バランス

```
主報酬（前進） > ペナルティ合計
```

- 主報酬がペナルティを下回ると「動かない」が最適解になる
- 詳細は `exp004_reward_design_survey.md` のセクション6-7を参照

### 6.2 段階的改善

1. まずタスク達成（前進）を確立
2. 次に主要な制約（姿勢、転倒防止）を追加
3. 最後に歩行品質（歩容、エネルギー効率）を改善

**一度に複数の問題を解決しようとしない**

### 6.3 終了条件

- 厳しすぎると「動かない」が最適解になる
- 緩すぎると不自然な姿勢を許容してしまう
- V18レベル（roll/pitch 30°、height 0.10m）が基準

---

## 7. トラブルシューティング

### 7.1 「動かない」ポリシーが学習された

原因候補:
1. 終了条件が厳しすぎる
2. ペナルティが主報酬を上回っている
3. 報酬項目が設計意図と逆に機能している

対処:
1. 学習ログで個別報酬を確認
2. 終了条件を緩和
3. ペナルティスケールを下げる

### 7.2 学習が収束しない（std > 0.1が続く）

原因候補:
1. 報酬設計に矛盾がある
2. 探索が不十分
3. 学習率が不適切

対処:
1. 報酬項目間の競合を確認
2. entropy_coefを上げて探索を促進
3. learning_rateを調整

### 7.3 TypeError: missing required argument

原因:
- `get_cfgs()`の戻り値と`DroidEnv.__init__`の引数が不一致

対処:
- `command_cfg`が正しく渡されているか確認
- 既存の動作するバージョンと比較

---

## 8. 参考リンク

- [Genesis公式ドキュメント](https://genesis-world.readthedocs.io/)
- [exp004_reward_design_survey.md](exp004_reward_design_survey.md) - 報酬設計の先行研究
- [exp004_droid_rl_walking.md](exp004_droid_rl_walking.md) - 実験結果
