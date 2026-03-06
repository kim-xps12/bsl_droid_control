# Phase 1: 準備チェックリスト

新バージョン V{N} の実装を開始する前に、以下を**この順番で**実行すること。

## チェックリスト

- [ ] **1. 前バージョンのレポートを参照して精査する**
  - [ ] `doc/experiments/exp008_droid_rl_walking_narrow/exp008_report_v{N-1}.md` を読み込む
  - [ ] 「次バージョンへの提案」セクションの内容を確認する
  - [ ] 課題と改善策を把握する

- [ ] **2. 必要に応じて追加評価を実行する**（任意）
  - [ ] 前バージョンのレポートに不足情報がある場合のみ
  - [ ] 評価: `uv run python biped_walking/biped_eval.py -e droid-walking-narrow-v{N-1} --no-viewer --duration 10`
  - [ ] ログ分析: `uv run python scripts/analyze_training_log.py droid-walking-narrow-v{N-1}`

- [ ] **3. 改善策に基づいてトレーニングスクリプトを作成する**
  - [ ] コピー（`cd rl_ws` 済みの状態で実行。`&&` で連結しないこと）:
    `cp biped_walking/train/droid_train_narrow_v{N-1}.py biped_walking/train/droid_train_narrow_v{N}.py`
  - [ ] 必要な箇所のみ編集（最小変更原則）
  - [ ] 必要に応じて `droid_env_unitree.py` に新しい報酬関数を追加
  - [ ] コード品質チェック（`AGENTS.md`「コード品質ルール【必須】」準拠）:
    - [ ] ruff lint: `uv run ruff check biped_walking/ scripts/`
    - [ ] ruff format: `uv run ruff format biped_walking/ scripts/`
    - [ ] mypy 型チェック: `uv run mypy biped_walking/ scripts/`
    - [ ] 新規関数には型ヒントを付与すること
    - [ ] エラーがあれば修正してから次のステップに進む

- [ ] **4. レポートスケルトンを作成する**
  - [ ] `doc/experiments/exp008_droid_rl_walking_narrow/exp008_report_v{N}.md` を新規作成
  - [ ] `report-template.md` のテンプレートに従って記述
  - [ ] 「結果」セクションは訓練完了後に記入する旨を明記
  - [ ] 設計詳細・パラメータ変更を記載

- [ ] **5. 主レポートにバージョン参照を追加する**
  - [ ] `doc/experiments/exp008_droid_rl_walking_narrow/exp008_droid_rl_walking_narrow.md` のバージョン一覧表に追加

## 禁止事項

- ❌ 前バージョンのレポートを確認せずにスクリプトを作成する
- ❌ レポートを作成せずにスクリプトのみを提供する
- ❌ 主レポートへの参照追加を忘れる
