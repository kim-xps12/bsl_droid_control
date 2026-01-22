# RobStride Hardware Interface - ドキュメント索引

RobStride Hardware Interfaceパッケージの包括的なドキュメントへようこそ。

## ドキュメント一覧

### 📖 メインドキュメント

#### [README.md](README.md)
**対象者**: 全てのユーザー
**内容**: パッケージの概要、機能、インストール方法、基本的な使い方、トラブルシューティング

このドキュメントから始めることをお勧めします。

---

### 🚀 使い方ガイド

#### [USAGE_GUIDE.md](USAGE_GUIDE.md)
**対象者**: 初心者〜中級者
**内容**:
- 初期セットアップの詳細手順
- 基本的な使い方（モーター制御の開始）
- 設定のカスタマイズ方法
- モーション制御の具体例
- デバッグとトラブルシューティング
- 高度な使い方（MoveIt2統合、カスタムコントローラー等）

**こんな人におすすめ:**
- 初めてRobStrideモーターを使う
- ros2_controlを初めて使う
- 実践的なチュートリアルが欲しい

---

### 🔧 API リファレンス

#### [API.md](API.md)
**対象者**: 開発者、上級者
**内容**:
- `RobStrideDriver`クラスの詳細API仕様
- `RobStrideHardware`クラスの詳細API仕様
- データ構造（`MotorState`, `MitCommand`等）
- プロトコル定数
- エンコーディング・デコーディング仕様
- 実装例

**こんな人におすすめ:**
- カスタムドライバーを開発したい
- ソースコードを理解したい
- 低レベルCANプロトコルを知りたい

---

### 📐 アーキテクチャ図

#### [architecture.drawio](architecture.drawio)
**対象者**: 全てのユーザー
**形式**: Draw.io形式（XMLベース）

**内容**:
- システム全体のアーキテクチャ図
- コンポーネント間のデータフロー
- スレッド構成（RTループ、State Readerスレッド）
- レイヤー分離の可視化

**閲覧方法:**
- [draw.io](https://app.diagrams.net/)でファイルを開く
- VSCode拡張機能: [Draw.io Integration](https://marketplace.visualstudio.com/items?itemName=hediet.vscode-drawio)

---

### 🔬 技術詳細資料

#### [TECHNICAL_DETAILS.md](TECHNICAL_DETAILS.md)
**対象者**: 上級者、研究者、システムインテグレーター
**内容**:
- システムアーキテクチャの詳細解説
- CANプロトコル仕様（フレームフォーマット、エンコーディング）
- リアルタイム性能の分析
- スレッド設計とLock-freeデータ共有
- データエンコーディングの数式
- エラーハンドリング
- パフォーマンス最適化手法
- セキュリティ考慮事項

**こんな人におすすめ:**
- システムの内部動作を深く理解したい
- パフォーマンスチューニングをしたい
- 研究論文やレポートを書く
- カスタムプロトコルを実装したい

---

## 推奨される読み順

### 初心者向け

1. **[README.md](README.md)** - パッケージ全体を把握
2. **[USAGE_GUIDE.md](USAGE_GUIDE.md)** - セットアップと基本的な使い方を学ぶ
3. **[architecture.drawio](architecture.drawio)** - システム構造を視覚的に理解
4. **[README.md](README.md) のトラブルシューティング** - 問題が発生したら参照

### 中級者向け

1. **[README.md](README.md)** - クイックリファレンス
2. **[architecture.drawio](architecture.drawio)** - アーキテクチャの把握
3. **[USAGE_GUIDE.md](USAGE_GUIDE.md)** - 高度な使い方の習得
4. **[API.md](API.md)** - カスタマイズに必要なAPI理解

### 上級者・開発者向け

1. **[architecture.drawio](architecture.drawio)** - システム設計の理解
2. **[API.md](API.md)** - API仕様の詳細確認
3. **[TECHNICAL_DETAILS.md](TECHNICAL_DETAILS.md)** - 内部実装の深い理解
4. **ソースコード** - 実装の詳細確認

---

## よくある質問（FAQ）

### Q1: モーターが動かない
→ [USAGE_GUIDE.md - デバッグとトラブルシューティング](USAGE_GUIDE.md#デバッグとトラブルシューティング)

### Q2: Kp, Kdゲインの調整方法は?
→ [USAGE_GUIDE.md - 設定のカスタマイズ](USAGE_GUIDE.md#設定のカスタマイズ)

### Q3: CANプロトコルの詳細を知りたい
→ [TECHNICAL_DETAILS.md - CANプロトコル仕様](TECHNICAL_DETAILS.md#canプロトコル仕様)

### Q4: APIの使い方を知りたい
→ [API.md](API.md)

### Q5: リアルタイム性能を最適化したい
→ [TECHNICAL_DETAILS.md - パフォーマンス最適化](TECHNICAL_DETAILS.md#パフォーマンス最適化)

### Q6: マルチモーター対応は?
→ [USAGE_GUIDE.md - 高度な使い方 - マルチモーターシステム](USAGE_GUIDE.md#マルチモーターシステム)
（注意: 現在の実装は1モーターのみ対応）

### Q7: MoveIt2と統合したい
→ [USAGE_GUIDE.md - 高度な使い方 - MoveIt2との統合](USAGE_GUIDE.md#moveit2との統合)

---

## クイックリファレンス

### セットアップ

```bash
# CANインターフェース起動
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# パッケージビルド
cd ~/ros2_ws
pixi run colcon build --packages-select robstride_hardware
source install/setup.bash
```

### 起動

```bash
# システム起動
pixi run ros2 launch robstride_hardware bringup.launch.py

# 状態確認
pixi run ros2 topic echo /robstride/joint_states

# コマンド送信
pixi run ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5]"
```

### デバッグ

```bash
# CANトラフィック確認
candump can0

# ログレベル変更
pixi run ros2 launch robstride_hardware bringup.launch.py --log-level debug
```

---

## ドキュメントの構成

```
doc/
├── INDEX.md                  # このファイル（索引）
├── README.md                 # メインドキュメント
├── USAGE_GUIDE.md            # 使い方ガイド
├── API.md                    # API リファレンス
├── TECHNICAL_DETAILS.md      # 技術詳細資料
└── architecture.drawio       # アーキテクチャ図
```

---

## 外部リソース

### 公式ドキュメント

- [ROS 2 Control Documentation](https://control.ros.org/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [SocketCAN Documentation](https://www.kernel.org/doc/Documentation/networking/can.txt)

### RobStride関連

- [RobStride公式サイト](https://www.robstride.com/)
- RobStrideモーター データシート（要NDA）

### 関連ツール

- [Draw.io](https://app.diagrams.net/) - アーキテクチャ図の閲覧・編集
- [can-utils](https://github.com/linux-can/can-utils) - CANデバッグツール

---

## フィードバック・バグレポート

バグや改善提案がある場合は、以下の方法でご連絡ください:

- **GitHub Issues**: (リポジトリURLを追加)
- **Email**: kim.xps12@gmail.com
- **プルリクエスト**: 歓迎します！

---

## ライセンス

MIT License

---

## 更新履歴

| 日付 | バージョン | 更新内容 |
|------|-----------|---------|
| 2024-01-22 | 1.0.0 | 初版作成（全ドキュメント） |

---

**Happy RobStriding! 🤖**
