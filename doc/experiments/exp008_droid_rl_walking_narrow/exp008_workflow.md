# EXP008: ワークフロー概要

本ドキュメントは、新バージョン作成から訓練後分析までの End-to-End ワークフローの概要を記載する。

**Coding Agent は `/exp008-new-version` スキルを使用して E2E ワークフローを実行すること。** 本ドキュメントは人間向けの概要であり、エージェント向けの詳細手順はスキル側（`.claude/skills/exp008-new-version/`）に一元管理されている。

## フロー概要

```
Phase 1: 準備
    - 前バージョンのレポート参照・精査
    - トレーニングスクリプト作成（コピー + 最小変更）
    - lint・型チェック
    - レポートスケルトン作成
    → 詳細: .claude/skills/exp008-new-version/phase1-prepare.md
    ↓
Phase 2: 訓練（バックグラウンド実行 + tmux 監視）
    - Bash ツールの run_in_background: true で訓練を実行
    - tmux 監視ペインを作成してリアルタイム監視を可能にする
    - 訓練完了後に tmux 監視ペインを閉じてから Phase 3 へ
    → 詳細: .claude/skills/exp008-new-version/SKILL.md Phase 2
    ↓
Phase 3: 訓練後分析（Agent Teams で並列化）
    - Step A: データ収集（評価・ログ分析・CSV分析・報酬コンポーネント比較）
    - Step B: 定量分析（比較表作成・報酬バランス評価）
    - Step C: サーベイ知見との照合
    - Step D: 独立考察（因果分析・改善案設計）【最重要・省略禁止】
    - Step E: レポート執筆（結果=事実、考察=分析）
    → 詳細: .claude/skills/exp008-new-version/phase3-analyze.md
    ↓
Phase 4: 完了報告
    - 結果サマリをユーザに提示
    - 次バージョンの推奨案を提示
```

## レポートテンプレート

→ `.claude/skills/exp008-new-version/report-template.md` を参照

## コマンドリファレンス

→ [exp008_commands.md](exp008_commands.md) を参照

## 中断・再開

- 各 Phase は独立しており、途中で中断・再開が可能
- 訓練が失敗した場合は Phase 1 に戻りスクリプトを修正
- ユーザが追加の目視評価を行う場合、Phase 3 の Step A にユーザー目視観察を追加
