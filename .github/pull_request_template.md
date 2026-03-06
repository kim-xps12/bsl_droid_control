## 概要
<!-- このPRで何を変更するか、簡潔に説明してください -->


## 関連Issue
<!-- 関連するIssueがあればリンクしてください（例: Closes #123） -->


## 変更種別
<!-- 該当するものにチェックを入れてください -->
- [ ] 新機能（New Feature）
- [ ] バグ修正（Bug Fix）
- [ ] リファクタリング（Refactoring）
- [ ] ドキュメント更新（Documentation）
- [ ] ビルド/CI設定（Build/CI）
- [ ] テスト追加/修正（Tests）

## 変更内容
<!-- 具体的な変更内容を箇条書きで記述してください -->
-

## 影響範囲
<!-- 変更が影響するパッケージ・コンポーネントを選択してください -->

### ROS 2環境（ros2_ws）
- [ ] biped_description（URDF/可視化/GUI）
- [ ] biped_gait_control（歩容生成）
- [ ] robstride_hardware（ros2_control HW Interface・Jetson専用）
- [ ] pub_sub_cpp / pub_sub_python（チュートリアル）

### 強化学習環境（rl_ws）
- [ ] biped_walking/envs（環境定義）
- [ ] biped_walking/train（訓練スクリプト）
- [ ] assets（URDFモデル）

### ドキュメント
- [ ] doc/design（設計ドキュメント）
- [ ] doc/experiments（実験レポート）

### その他
- [ ] その他: 

## 動作確認
<!-- 実施した動作確認を記載してください -->

### ROS 2環境（該当する場合）
- [ ] ビルド成功（`cd ros2_ws && pixi run colcon build`）
- [ ] 動作確認済み（確認方法: ）
- [ ] macOS / Jetson 両環境で確認（該当する場合）

### 強化学習環境（該当する場合）
- [ ] 依存関係解決（`cd rl_ws && uv sync`）
- [ ] 訓練実行確認（`uv run python biped_walking/train/...`）
- [ ] 評価実行確認（`uv run python biped_walking/biped_eval.py -e {実験名}`）

---

## GitHub Copilot レビュー指示

> [!NOTE]
> 以下はGitHub Copilotによる自動レビュー用の指示です。

### レビュー言語
**日本語**でレビューしてください。

### プロジェクト概要
- 二脚ロボット（BSL-Droid）の制御に関するソフトウェア全般
- Jetson Orin Nano Super（実機・RT制御）とMacBook（可視化・机上開発）の分散構成
- **ROS 2環境**: `ros2_ws/` 配下で pixi により管理
- **強化学習環境**: `rl_ws/` 配下で uv により管理（ROS 2とは独立）

### 重点レビュー項目

#### 1. 安全性（最優先）
ロボット制御システムのため、以下を厳重にチェック：
- **関節可動範囲リミット**: 実装されているか
- **緊急停止機構**: E-Stop対応があるか
- **トルク/電流制限**: 設定されているか
- **異常時フォールバック**: 定義されているか

#### 2. リアルタイム制約（ROS 2）
制御ループ（200Hz想定）内で以下を避けているか：
- メモリ動的確保（`new`, `vector.resize()`等）
- ブロッキングI/O
- 同期ログ出力

#### 3. ROS 2ベストプラクティス
- QoS設定が用途に適切か
- トピック名が `snake_case` 命名規則に従っているか
- パラメータバリデーションが実装されているか
- ライフサイクルノードの状態遷移が正しいか

#### 4. 強化学習環境
- 観測空間・行動空間が適切に定義されているか
- 実験名が命名規則に従っているか（`biped-walking-v{N}`, `droid-walking-v{N}`）
- 対応する `exp{NNN}_rules.md` のルールに従っているか

#### 5. コーディング規約
**Python:**
- 型ヒント付与
- 命名規則（変数/関数: `snake_case`, クラス: `PascalCase`）

**C++:**
- スマートポインタ使用（生ポインタ禁止）
- `nullptr` 使用（`NULL` 禁止）
- メンバ変数: `trailing_underscore_`

#### 6. プラットフォーム考慮
- `ros2_control` はmacOSで利用不可（Jetson専用）
- プラットフォーム依存コードが適切に分離されているか

#### 7. ドキュメント
- 実験レポートは `doc/experiments/exp{NNN}_{実験名}/` の構造に従っているか
- `.drawio.svg` ファイルがDrawIO XML形式（`<mxfile>`タグ）か（純粋SVG禁止）
- 補足ファイルが主レポートから参照されているか（孤立ファイル禁止）

### セキュリティチェック
- ハードコードされた認証情報・シークレットがないか
- CAN通信のモータID等が適切に設定ファイル化されているか
