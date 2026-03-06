# EXP008: 実験の進め方とルール

本ドキュメントは、exp008（BSL-Droid Simplified V2による強化学習歩容獲得実験）を進める上での原則とルールを記載する。

exp007のルール体系を継承し、URDF変更（胴体幅75%縮小）に伴う差分のみを更新している。

手順・コマンド・報酬設計については関連ドキュメント（Section 3）を参照。

## 1. バージョン管理とコード信頼性

強化学習環境とトレーニングスクリプトのバージョン管理において、以下の原則を厳守する（exp007から継承）:

### 1.1 統一環境クラス原則（重要）

**環境クラスファイルは1個のみ**とし、新しい報酬関数や機能は既存の統一ファイルに累積させる。

```
rl_ws/biped_walking/envs/droid_env_unitree.py  # exp007と共用
```

**禁止事項**:
- ❌ バージョンごとに環境クラスファイルを作成する
- ❌ ファイル数が爆発的に増加する構成

**正しい運用**:
- ✅ 全ての報酬関数を `droid_env_unitree.py` に実装
- ✅ トレーニングスクリプト側の `reward_scales` で報酬の有効/無効を制御
- ✅ `reward_scales` に含めない、または値を0に設定すれば報酬は無効化される

### 1.2 報酬関数の命名規則

exp007と同一。`_reward_{name}` 形式でdocstringに追加バージョン・設計原理を記載。

### 1.3 トレーニングスクリプトのコピー原則

新バージョン（V(n+1)）を作成する際は、**トレーニングスクリプトのみ**を完全コピーしてから編集を開始する。

```bash
# 例: V1からV2を作成する場合
cd rl_ws/biped_walking
cp train/droid_train_narrow_v1.py train/droid_train_narrow_v2.py
# 注意: envファイルはコピーしない（統一環境クラスを使用）
```

### 1.4 最小変更原則

コピー後は、改善目的に必要な箇所**のみ**を編集する:
- docstring・報酬スケール（`reward_scales`）・コマンド設定など、変更が必要な部分だけを特定
- 既存の動作検証済みコードは可能な限り保持
- 不必要な再実装や構造変更を避ける

### 1.5 段階的検証

各バージョンで以下を確認:
- 構文エラーがないこと（実行前に検証）
- 依存パラメータ（command_cfg等）の整合性
- 初期化処理の完全性（qpos次元数等）

---

## 2. 実験記録のルール

### 2.1 ファイル責務

本実験の記録ファイルの責務分離は `AGENTS.md`「実験記録の責務分離」に準拠する。exp008固有の追加ドキュメントは以下の通り:

| ファイル | 責務 |
|----------|------|
| `exp008_workflow.md` | ワークフロー概要（人間向け。詳細手順はスキル側に一元管理） |
| `exp008_commands.md` | コマンドリファレンス・分析スクリプト |
| `exp008_reward_design.md` | 報酬設計原則・トラブルシューティング |
| `.claude/skills/exp008-new-version/` | E2Eワークフロー詳細（エージェント向け手順の唯一の権威） |

### 2.2 バージョン別レポート原則

`AGENTS.md`「バージョン別レポート原則」に準拠する。**1バージョン = 1ファイル**（`exp008_report_v{N}.md`）とし、主レポートにはバージョン一覧とリンクのみを記載すること。

### 2.3 失敗実験の記録方針

**失敗も重要な知見である**。失敗実験のレポートには以下を必ず記載すること:

1. **何が起きたか**: 観察された問題を客観的に記述
2. **なぜ起きたか**: 原因の仮説を記述
3. **何を学んだか**: 次バージョンへの教訓を記述

失敗実験のレポート省略および主レポートのバージョン一覧からの除外は禁止する。

---

## 3. 関連ドキュメント

### exp008固有ドキュメント

| ドキュメント | 内容 |
|-------------|------|
| [exp008_workflow.md](exp008_workflow.md) | ワークフロー概要（詳細手順は `/exp008-new-version` スキルを参照） |
| [exp008_commands.md](exp008_commands.md) | トレーニング・評価・分析のコマンドリファレンス |
| [exp008_reward_design.md](exp008_reward_design.md) | 報酬項目追加ルール・報酬項目数目安・階層的設計原則・トラブルシューティング |

### exp007からの引き継ぎ資料

| ドキュメント | 内容 |
|-------------|------|
| [exp007_summary_for_exp008.md](../exp007_droid_rl_walking_ref_unitree/exp007_summary_for_exp008.md) | exp007の40バージョンの知見総括 |
| [exp007_unitree_rl_gym_survey.md](../exp007_droid_rl_walking_ref_unitree/exp007_unitree_rl_gym_survey.md) | Unitree RL Gymの報酬設計に関する調査 |

### 外部参考リンク

- [Unitree RL Gym (GitHub)](https://github.com/unitreerobotics/unitree_rl_gym)
- [Legged Gym (ETH)](https://github.com/leggedrobotics/legged_gym)
- [Genesis公式ドキュメント](https://genesis-world.readthedocs.io/)

## 4. exp007との主な差分

| 項目 | exp007 | exp008 |
|------|--------|--------|
| URDF | bsl_droid_simplified | bsl_droid_simplified_v2 |
| torso_width | 0.18m | 0.135m（75%） |
| hip_offset_y | 0.10m | 0.075m（75%） |
| 実験名 | droid-walking-unitree-v{N} | droid-walking-narrow-v{N} |
| 訓練スクリプト | droid_train_unitree_v{N}.py | droid_train_narrow_v{N}.py |
| 環境クラス | droid_env_unitree.py（共用） | droid_env_unitree.py（共用） |
