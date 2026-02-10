---
name: exp007
description: Rules and workflow for exp007 RL walking experiments. Applies when working on droid_env_unitree.py, droid_train_unitree_v*.py, exp007 reports, or any biped_walking RL training/evaluation tasks.
user-invocable: false
---

# EXP007 実験ルール

exp007（BSL-Droid Simplified 二脚ロボットの RL 歩容獲得実験）に関連する作業を行う際は、以下のドキュメントを読み込み、記載されたルールに従うこと。

## 必読ドキュメント

作業内容に応じて、以下を事前に読み込む:

| ドキュメント | いつ読むか |
|-------------|-----------|
| `doc/experiments/exp007_droid_rl_walking_ref_unitree/exp007_rules.md` | **常に最初に読む**（バージョン管理原則・実験記録ルール） |
| `doc/experiments/exp007_droid_rl_walking_ref_unitree/exp007_workflow.md` | 新バージョン作成時・訓練後分析時 |
| `doc/experiments/exp007_droid_rl_walking_ref_unitree/exp007_commands.md` | コマンド実行時 |
| `doc/experiments/exp007_droid_rl_walking_ref_unitree/exp007_reward_design.md` | 報酬設計変更時・トラブルシューティング時 |
| `doc/experiments/exp007_droid_rl_walking_ref_unitree/exp007_unitree_rl_gym_survey.md` | 訓練後の考察・改善案設計時（Step C） |

## 核心ルール（要約）

1. **統一環境クラス原則**: `droid_env_unitree.py` のみ。バージョン別の env ファイル作成は禁止
2. **コピー原則**: 新バージョンはトレーニングスクリプトのコピーから開始
3. **最小変更原則**: 1変更1検証。一度に変更するのは1-2項目まで
4. **報酬項目数**: 15-17項目を推奨（21以上は非推奨、V7-V10で不安定化実証済み）
5. **ペナルティ制約**: ペナルティ絶対値合計が主報酬を上回らないこと
6. **1バージョン = 1ファイル**: `exp007_report_v{N}.md` に自己完結的に記録
7. **失敗も記録**: 失敗実験のレポート省略は禁止

## コマンド実行の前提

全コマンドは `cd rl_ws` 実行後に使用すること。詳細は `AGENTS.md` 参照。
