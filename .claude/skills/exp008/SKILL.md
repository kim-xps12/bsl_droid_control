---
name: exp008
description: Rules and workflow for exp008 RL walking experiments with narrow torso model. Applies when working on droid_env_unitree.py, droid_train_narrow_v*.py, exp008 reports, or any biped_walking RL training/evaluation tasks with the narrow torso model.
user-invocable: false
---

# EXP008 実験ルール

exp008（BSL-Droid Simplified V2 二脚ロボットの強化学習歩容獲得実験）に関連する作業を行う際は、以下のドキュメントを読み込み、記載されたルールに従うこと。

## 必読ドキュメント

作業内容に応じて、以下を事前に読み込む:

| ドキュメント | いつ読むか |
|-------------|-----------|
| `doc/experiments/exp008_droid_rl_walking_narrow/exp008_rules.md` | **常に最初に読む**（バージョン管理原則・実験記録ルール） |
| `doc/experiments/exp008_droid_rl_walking_narrow/exp008_workflow.md` | ワークフロー概要の確認時（詳細手順は `/exp008-new-version` スキルに一元管理） |
| `doc/experiments/exp008_droid_rl_walking_narrow/exp008_commands.md` | コマンド実行時 |
| `doc/experiments/exp008_droid_rl_walking_narrow/exp008_reward_design.md` | 報酬設計変更時・トラブルシューティング時 |
| `doc/experiments/exp007_droid_rl_walking_ref_unitree/exp007_unitree_rl_gym_survey.md` | 訓練後の考察・改善案設計時（Step C） |
| `doc/experiments/exp007_droid_rl_walking_ref_unitree/exp007_summary_for_exp008.md` | exp007の知見を確認したいとき |

## 核心ルール

核心ルールは以下のドキュメントを直接参照すること（要約の二重管理を避けるため、ここでは繰り返さない）:
- バージョン管理・コード信頼性・実験記録: `exp008_rules.md`
- 報酬項目数・ペナルティ制約・報酬設計原則: `exp008_reward_design.md`

## コマンド実行の前提

全コマンドは `cd rl_ws` 実行後に使用すること。詳細は `AGENTS.md` 参照。
