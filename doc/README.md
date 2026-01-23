# 二脚ロボット制御システム 設計ドキュメント

本ディレクトリには、強化学習による歩容生成を目指した二脚ロボット制御システムの設計文書が含まれています。

## システム構成

本システムは**2台のコンピュータで分散実行**する構成を取ります：

| マシン | 役割 | 主な処理 |
|--------|------|----------|
| **Jetson Orin Nano Super** | ロボット搭載制御コンピュータ | リアルタイム制御、センサ処理、モータ制御 |
| **MacBook** | 開発・可視化ワークステーション | RViz可視化、データ解析、デバッグ、開発 |

---

## ドキュメント一覧

### 🌐 分散システム設計（新規）

#### 📄 [distributed_architecture.md](distributed_architecture.md)
**分散システムアーキテクチャ設計**

Jetson Orin Nano Super と MacBook の分散構成における詳細設計を記載しています。

**主な内容:**
- ハードウェア構成（Jetson/MacBook）
- ネットワーク構成（WiFi/Ethernet、ROS 2 DDS設定）
- ノード配置（どのマシンでどのノードを実行するか）
- 開発ワークフロー（Git、SSH、VS Code Remote）
- Launch構成
- トラブルシューティング

#### 🎨 [distributed_system_architecture.drawio](distributed_system_architecture.drawio)
**分散システム全体図**

Jetson と MacBook の役割分担、ノード配置、トピック通信を視覚化した図です。

**含まれる内容:**
- Jetson側ノード（RT制御層、歩容生成層、センサ/ドライバ層）
- MacBook側ツール（RViz、Plotjuggler、開発環境）
- ROS 2 DDSネットワーク通信
- 主要トピックと通信方向
- 帯域幅目安

#### 🎨 [development_workflow.drawio](development_workflow.drawio)
**開発ワークフロー図**

MacBook での開発から Jetson での実行までの流れを視覚化した図です。

**含まれる内容:**
- コード編集 → ビルド → Git push/pull → 実機実行
- VS Code Remote SSH オプション
- 可視化・デバッグのフロー
- よく使うコマンド一覧

---

### 📋 ノード設計

#### 📄 [next_nodes_design.md](next_nodes_design.md)
**次期ノード設計の詳細文書**

現在実装されている`robstride_hardware`ノードの次に実装すべきノード群の詳細設計を記載しています。

**主な内容:**
- システムアーキテクチャの階層構造
- Phase 1（優先）: State Estimator、Safety Monitor
- Phase 2: RL Policy、Gait Pattern Generator
- Phase 3（将来）: Navigation、Task Manager
- カスタムメッセージ定義
- 実装戦略とタイムライン
- パフォーマンス要件

---

### 🎨 [system_architecture.drawio](system_architecture.drawio)
**システムアーキテクチャ図**

制御システム全体の階層構造とノード間の関係を視覚化した図です。

**含まれる内容:**
- 5つの制御層（High-Level → Gait Generation → State Estimation → RT Control & Safety → Hardware）
- 各ノードの配置と役割
- 実装フェーズの色分け
- データフローの矢印
- 凡例とフェーズ説明

**表示方法:**
- [draw.io](https://app.diagrams.net/)でファイルを開く
- VS Codeの[Draw.io Integration拡張機能](https://marketplace.visualstudio.com/items?itemName=hediet.vscode-drawio)を使用

---

### 🎨 [data_flow.drawio](data_flow.drawio)
**データフロー図**

ROS 2トピックとノード間のメッセージフローを詳細に示した図です。

**含まれる内容:**
- センサ入力（IMU、Joint States、Force Sensors）
- 各ノードの処理（State Estimator、Safety Monitor、RL Policy、Hardware Interface）
- トピック名とメッセージ型
- 通信周波数の明示
- フィードバックループ
- 緊急停止シグナルの経路
- 凡例（データフロー種別の説明）

---

### 🎨 [node_implementation_details.drawio](node_implementation_details.drawio)
**ノード実装詳細図**

Phase 1優先実装ノード（State Estimator、Safety Monitor、RL Policy）の内部設計を詳細に示した図です。

**各ノードの詳細:**
- 入力トピック（Subscribers）
- 処理アルゴリズム
- 出力トピック（Publishers）
- パラメータ設定
- 実装オプション（RL Policyの場合）

**その他の情報:**
- 推奨実装タイムライン（Week 1-6+）
- 並行作業項目
- 実装優先順位

---

## 設計の要点

### Phase 1: 基盤ノード（最優先）

1. **State Estimator Node** (Week 1-2)
   - センサフュージョンによるロボット状態推定
   - robot_localizationパッケージの活用
   - 200 Hz動作、SCHED_FIFO優先度

2. **Safety Monitor Node** (Week 3)
   - リアルタイム安全性監視
   - 緊急停止機構
   - 姿勢・関節・温度の多層チェック

### Phase 2: 歩容生成（Week 4-6+）

3. **Gait Pattern Generator Node** (Week 4-5)
   - CPGベースのシンプルな歩容
   - RLポリシーのフォールバック

4. **RL Policy Node** (Week 6+)
   - PyTorch/libtorchによる推論
   - Sim-to-Real転移
   - 50-100 Hz動作

### Phase 3: 高レベル制御（将来）

5. Navigation Integration
6. Task Manager

---

## 技術スタック

### 必須パッケージ（Phase 1）
- `robot_localization` - EKF/UKFベース状態推定
- `realtime_tools` - リアルタイムデータ構造
- `diagnostic_updater` - 診断情報管理

### Phase 2追加パッケージ
- `libtorch` (C++) または `torch` (Python) - RL推論
- `control_toolbox` - PID、フィルタ
- `angles` - 角度演算

### シミュレーション環境
- **Isaac Gym** - 大規模並列RL学習
- **MuJoCo** - 高速・高精度物理シミュレーション
- **Gazebo** - ROS 2統合シミュレータ

---

## パフォーマンス要件

| ノード | 周波数 | 最大レイテンシ | スレッド優先度 |
|--------|--------|----------------|----------------|
| robstride_hardware | 200 Hz | 1 ms | SCHED_FIFO 50 |
| state_estimator | 200 Hz | 1 ms | SCHED_FIFO 40 |
| safety_monitor | 200 Hz | 1 ms | SCHED_FIFO 45 |
| rl_policy | 50-100 Hz | 10 ms | SCHED_OTHER |

---

## 次のステップ

1. ✅ 設計文書の確認（このドキュメント）
2. 📦 カスタムメッセージパッケージの作成
3. 🔧 State Estimator Nodeの実装開始
4. 🧪 シミュレーション環境の構築（Gazebo/Isaac Gym）
5. 📊 ロギング・可視化環境の整備（RViz、Plotjuggler）

---

## 参考文献

- [MIT Cheetah Software](https://github.com/mit-biomimetics/Cheetah-Software)
- [Unitree ROS](https://github.com/unitreerobotics/unitree_ros)
- [Isaac Gym Legged Robots](https://github.com/leggedrobotics/legged_gym)
- [robot_localization](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
- [ROS 2 Control](https://control.ros.org/)

---

## ライセンス

このドキュメントは、プロジェクトのメインライセンスに従います。

---

**最終更新**: 2026年1月23日
