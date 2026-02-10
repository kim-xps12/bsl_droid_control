# EXP007: 実験の進め方とルール

本ドキュメントは、exp007（BSL-Droid Simplified二脚ロボットのUnitree RL Gym参考実装による強化学習歩容獲得実験）を進める上での原則とルールを記載する。

手順・コマンド・報酬設計については関連ドキュメント（Section 3）を参照。

## 1. バージョン管理とコード信頼性

強化学習環境とトレーニングスクリプトのバージョン管理において、以下の原則を厳守する（exp002から継承）:

### 1.1 統一環境クラス原則（重要）

**環境クラスファイルは1個のみ**とし、新しい報酬関数や機能は既存の統一ファイルに累積させる。

```
rl_ws/biped_walking/envs/droid_env_unitree.py  # 唯一の環境クラスファイル
```

**禁止事項**:
- ❌ バージョンごとに環境クラスファイルを作成する（例: `droid_env_unitree_v1.py`, `v2.py`, `v3.py`）
- ❌ ファイル数が爆発的に増加する構成

**正しい運用**:
- ✅ 全ての報酬関数を `droid_env_unitree.py` に実装
- ✅ トレーニングスクリプト側の `reward_scales` で報酬の有効/無効を制御
- ✅ `reward_scales` に含めない、または値を0に設定すれば報酬は無効化される

### 1.2 報酬関数の命名規則

環境クラスに新しい報酬関数を追加する際は、以下の命名規則に従うこと。

**メソッド名**:
```python
def _reward_{name}(self):
    """報酬関数の説明（追加バージョンを明記）

    【設計原理】
    - 報酬の目的と設計意図を記述

    【参考文献】（任意）
    - 参考にした論文やリポジトリがあれば記載
    """
    # 実装
    return reward_tensor
```

**命名規約**:
- プレフィックス: `_reward_`（必須）
- 名前部分: snake_case（例: `tracking_lin_vel`, `feet_air_time`, `single_foot_contact`）
- 戻り値: `torch.Tensor` shape `(num_envs,)`

**reward_scalesでの参照**:
```python
reward_scales = {
    "{name}": 1.0,  # _reward_{name} が呼び出される
}
```

**docstringに含めるべき情報**:
1. 報酬の目的（1行）
2. 追加バージョン（例: V7追加）
3. 設計原理（なぜこの報酬が必要か）
4. 参考文献（任意）

### 1.3 トレーニングスクリプトのコピー原則

新バージョン（V(n+1)）を作成する際は、**トレーニングスクリプトのみ**を完全コピーしてから編集を開始する。

```bash
# 例: V1からV2を作成する場合
cd rl_ws
cp biped_walking/train/droid_train_unitree_v1.py biped_walking/train/droid_train_unitree_v2.py
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

### 1.6 理由

- ゼロから再実装すると、細かい設定ミス（qpos次元、command_cfg、feet_indices初期化等）が混入しやすい
- 既存の動作確認済みコードをベースにすることで、新規追加部分のデバッグに集中できる
- 信頼性の高い反復的改善が可能
- 環境クラスを統一することで、ファイル数の爆発を防ぎ、保守性を維持できる

---

## 2. 実験記録のルール

### 2.1 ファイル責務

本実験の記録ファイルの責務分離は `AGENTS.md`「実験記録の責務分離」に準拠する。exp007固有の追加ドキュメントは以下の通り:

| ファイル | 責務 |
|----------|------|
| `exp007_workflow.md` | 新バージョン作成・訓練後分析の手順・レポートテンプレート |
| `exp007_commands.md` | コマンドリファレンス・分析スクリプト |
| `exp007_reward_design.md` | 報酬設計原則・トラブルシューティング |
| `exp007_unitree_rl_gym_survey.md` | Unitree RL Gymの報酬設計に関する調査 |

### 2.2 バージョン別レポート原則

`AGENTS.md`「バージョン別レポート原則」に準拠する。**1バージョン = 1ファイル**（`exp007_report_v{N}.md`）とし、主レポートにはバージョン一覧とリンクのみを記載すること。

### 2.3 失敗実験の記録方針

**失敗も重要な知見である**。失敗実験のレポートには以下を必ず記載すること:

1. **何が起きたか**: 観察された問題を客観的に記述
2. **なぜ起きたか**: 原因の仮説を記述
3. **何を学んだか**: 次バージョンへの教訓を記述

失敗実験のレポート省略および主レポートのバージョン一覧からの除外は禁止する。

---

## 3. 関連ドキュメント

### exp007固有ドキュメント

| ドキュメント | 内容 |
|-------------|------|
| [exp007_workflow.md](exp007_workflow.md) | E2Eワークフロー・レポートテンプレート・作成手順・分析フロー |
| [exp007_commands.md](exp007_commands.md) | トレーニング・評価・分析のコマンドリファレンス |
| [exp007_reward_design.md](exp007_reward_design.md) | 報酬項目追加ルール・報酬項目数目安・階層的設計原則・トラブルシューティング |
| [exp007_unitree_rl_gym_survey.md](exp007_unitree_rl_gym_survey.md) | Unitree RL Gymの報酬設計に関する調査 |

### 外部参考リンク

- [Unitree RL Gym (GitHub)](https://github.com/unitreerobotics/unitree_rl_gym)
- [Legged Gym (ETH)](https://github.com/leggedrobotics/legged_gym)
- [Genesis公式ドキュメント](https://genesis-world.readthedocs.io/)
