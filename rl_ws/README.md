# BSL-Droid RL環境

BSL-Droid二脚ロボットの歩容獲得のための強化学習ワークスペースです。[Genesis](https://genesis-world.readthedocs.io/)物理シミュレータを使用します。

## 概要

本ワークスペースでは以下を提供します：
- Genesis物理シミュレーション環境
- rsl-rl ライブラリによるPPOトレーニング
- MacBook（Metalバックエンド）対応
- **Sim2Sim: GenesisからMuJoCoへのポリシー転送**

## セットアップ・使用方法

**実行コマンドの詳細は[プロジェクトルートのREADME.md](../README.md#強化学習環境rl_ws)を参照してください。**

```bash
cd rl_ws

# 依存関係をインストール
uv sync

# Genesisの動作確認
uv run python -c "import genesis as gs; print(f'Genesis {gs.__version__} loaded')"
```

### macOSでのビューア

macOSでは標準MuJoCoビューアを使用するために`mjpython`が必要です。

```bash
# mjpythonで標準MuJoCoビューアを使用（推奨）
uv run mjpython scripts/go2_eval_mujoco.py --duration 10

# 通常のpythonでも実行可能（OpenCVビューアにフォールバック）
uv run python scripts/go2_eval_mujoco.py --duration 10
# 'q'キーまたはESCで終了
```

`mjpython`を使用しない場合、OpenCV経由でリアルタイム表示を行います。
OpenCVがインストールされていない場合は`--no-viewer`オプションを使用してください。

## ディレクトリ構造

```
rl_ws/
├── pyproject.toml           # プロジェクト依存関係
├── .python-version          # Pythonバージョン（3.11）
├── README.md                # 本ファイル
├── assets/                  # ロボットモデル
│   ├── export_urdf.py       # 二脚URDF出力スクリプト
│   └── go2_genesis.xml      # Genesis URDF変換後のMJCF（生成）
├── genesis_official/        # Genesis公式リポジトリ（クローン済み）
│   ├── genesis/assets/urdf/go2/  # Go2 URDFモデル（Sim2Sim元データ）
│   │   ├── urdf/go2.urdf    # ロボットURDF
│   │   └── dae/             # メッシュファイル
│   └── examples/
│       └── locomotion/
│           ├── go2_env.py   # Go2環境
│           ├── go2_train.py # PPOトレーニング
│           └── go2_eval.py  # ポリシー評価
├── mujoco_menagerie/        # MuJoCo公式ロボットモデル集（比較用）
│   └── unitree_go2/         # Go2 MJCFモデル（--modelオプションで使用可）
├── scripts/                 # 評価・開発ツール
│   ├── convert_urdf_to_mjcf.py  # URDF→MJCF変換スクリプト
│   ├── go2_eval_mujoco.py       # Sim2Sim評価スクリプト
│   └── compare_models.py        # モデルパラメータ比較ツール
└── logs/                    # トレーニングログ（生成）
    └── go2-walking/         # Go2歩行ポリシー
        ├── model_*.pt       # チェックポイント
        └── cfgs.pkl         # 訓練設定
```

## デバイス選択

Genesisはプラットフォームに応じて自動的にバックエンドを選択します：
- **MacBook**: Metalバックエンド
- **Linux（NVIDIA GPU搭載）**: CUDAバックエンド
- **フォールバック**: CPUバックエンド

## Go2の歩容獲得（Genesis）

```bash
cd rl_ws

# 学習（100イテレーション、約数分）
uv run python genesis_official/examples/locomotion/go2_train.py

# 推論の評価（Genesisビューア）
uv run python genesis_official/examples/locomotion/go2_eval.py
```

## TensorBoardで学習進捗を監視

訓練中のメトリクスをリアルタイムで確認できます。

```bash
cd rl_ws

# TensorBoardを起動（ブラウザで http://localhost:6006 を開く）
uv run tensorboard --logdir logs/

# 特定の実験のみ監視
uv run tensorboard --logdir logs/go2-walking
uv run tensorboard --logdir logs/biped-walking
```

主要なメトリクス：
- `Loss/value_function`: 価値関数の損失
- `Loss/surrogate`: PPOのサロゲート損失
- `Perf/mean_reward`: 平均報酬（学習の進捗指標）
- `Perf/mean_episode_length`: 平均エピソード長（転倒までのステップ数）

## Sim2Sim: MuJoCoでの評価

GenesisでトレーニングしたポリシーをMuJoCoで評価します。
これにより、異なる物理エンジン間でのポリシー転送性を検証できます。

### 概要

| 項目 | Genesis | MuJoCo |
|------|---------|--------|
| 物理エンジン | Genesis (Metal/CUDA) | MuJoCo |
| ロボットモデル | Genesis公式 URDF (Go2) | 変換済みMJCF※ |
| 制御方式 | PD位置制御 | PD→トルク変換 |
| 関節順序 | FR→FL→RR→RL | FL→FR→RL→RR |

※ Genesis URDFを事前に変換したMJCFを使用。MuJoCo Menagerieも`--model`オプションで選択可能。

### Step 1: URDF→MJCF変換

Genesis公式URDFをMuJoCo用MJCFに変換します。この処理は**初回のみ**必要です。

```bash
cd rl_ws

# 変換実行（assets/go2_genesis.xml が生成される）
uv run python scripts/convert_urdf_to_mjcf.py

# 中間ファイルを保存してデバッグ（問題発生時）
uv run python scripts/convert_urdf_to_mjcf.py --save-intermediate
```

変換処理の内容：
1. URDFをMuJoCoでコンパイルし、MJCFとしてエクスポート
2. freejoint（ベースの自由移動）を追加
3. 関節パラメータを補完（damping, armature, frictionloss）
4. 地面と光源を追加
5. 12個のアクチュエータとhome keyframeを追加

### Step 2: 評価実行

```bash
cd rl_ws

# ビューア付きで実行
# macOSではmjpythonで標準MuJoCoビューア、pythonではOpenCVにフォールバック
# Linux/Windowsではpythonで標準MuJoCoビューアが使用可能
uv run mjpython scripts/go2_eval_mujoco.py --duration 10  # macOS推奨
uv run python scripts/go2_eval_mujoco.py --duration 10    # Linux/Windows

# ビューアなしで数値のみ確認
uv run python scripts/go2_eval_mujoco.py --no-viewer --duration 5

# MuJoCo Menagerieモデルを使用（比較用）
uv run mjpython scripts/go2_eval_mujoco.py --model mujoco_menagerie/unitree_go2/scene.xml
```

### 技術的詳細

**関節マッピング（Genesis → MuJoCo）：**
- Genesis: FR_hip, FR_thigh, FR_calf, FL_*, RR_*, RL_*
- MuJoCo: FL_hip, FL_thigh, FL_calf, FR_*, RL_*, RR_*
- 自動マッピングにより関節順序の違いを吸収

**PD制御パラメータ：**
- kp = 20.0（位置ゲイン）
- kd = 0.5（速度ゲイン）
- action_scale = 0.25

**関節物理パラメータ（MuJoCo Menagerieと同値）：**
- damping = 2.0
- armature = 0.01
- frictionloss = 0.2

## 開発ツール

### モデルパラメータ比較（compare_models.py）

Genesis変換モデルとMuJoCo Menagerieモデルの物理パラメータを比較するツールです。
URDF→MJCF変換の妥当性検証やデバッグに使用します。

```bash
cd rl_ws

# パラメータ比較を実行
uv run python scripts/compare_models.py
```

出力例：
```
=== 修正後の物理パラメータ比較 ===

--- 関節パラメータ (damping, armature, frictionloss) ---
Genesis:
  FL_hip_joint: damping=2.0000, armature=0.010000, frictionloss=0.2000
Menagerie:
  FL_hip_joint: damping=2.0000, armature=0.010000, frictionloss=0.2000

--- 足の接触パラメータ ---
Genesis:
  (FL geom not found)
Menagerie:
  FL: friction=[0.8  0.02 0.01], condim=[6]

--- keyframe ---
Genesis keyframes: 1
Menagerie keyframes: 1
```

比較項目：
- **関節パラメータ**: damping, armature, frictionloss
- **足の接触パラメータ**: friction, condim
- **keyframe数**: 初期姿勢の定義

## BSL-Droid二脚ロボットへの適用

二脚ロボット（10 DOF）の歩容獲得トレーニングと評価。

### トレーニング

```bash
cd rl_ws

# V4トレーニング（交互歩行版、推奨）
uv run python genesis_official/examples/locomotion/biped_train_v4.py --max_iterations 1000
```

### 評価

統一評価スクリプト`biped_eval.py`を使用。`-e`オプションで実験名を指定して任意のバージョンを評価できます。

```bash
cd rl_ws

# V4評価（最新チェックポイント）
uv run python genesis_official/examples/locomotion/biped_eval.py -e biped-walking-v4

# 特定チェックポイントを指定
uv run python genesis_official/examples/locomotion/biped_eval.py -e biped-walking-v4 --ckpt 400

# 過去バージョン評価
uv run python genesis_official/examples/locomotion/biped_eval.py -e biped-walking-v3
```

### バージョン履歴

| バージョン | 実験名 | 特徴 |
|-----------|--------|------|
| V1 | biped-walking | 初期実装。歩行するがジャーキー |
| V2 | biped-walking-v2 | 滑らかさペナルティ過剰。歩行せず |
| V3 | biped-walking-v3 | バランス調整版。歩行するがすり足 |
| V4 | biped-walking-v4 | 交互歩行報酬追加。足を交互に出す歩行を目指す |

## 依存パッケージ

| パッケージ | バージョン | 用途 |
|---------|---------|---------|
| genesis-world | >=0.3.0 | 物理シミュレーション |
| torch | >=2.0.0 | ニューラルネットワーク |
| rsl-rl-lib | ==2.2.4 | PPO実装 |
| tensorboard | >=2.14.0 | トレーニング可視化 |
| mujoco | latest | Sim2Sim評価 |

## 参考資料

- [Genesis公式ドキュメント](https://genesis-world.readthedocs.io/)
- [Genesis Locomotionチュートリアル](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/locomotion.html)
- [Genesis GitHubリポジトリ](https://github.com/Genesis-Embodied-AI/Genesis)
- [MuJoCo Pythonドキュメント](https://mujoco.readthedocs.io/en/stable/python.html)
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
