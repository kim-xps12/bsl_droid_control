---
description: 'GitHub Copilot Code Review用カスタム指示 - BSL-Droid 制御システム'
applyTo: '**'
excludeAgent: ["coding-agent"]
---

# BSL-Droid Control コードレビュー指示

このリポジトリは二脚ロボット（BSL-Droid）の制御に関するソフトウェア全般を扱います．Jetson Orin Nano Super（実機・リアルタイム制御）とMacBook（可視化・机上開発）の分散構成で動作します．

## レビュー言語

コードレビューは必ず**日本語**で回答してください．

## プロジェクト固有の重要事項

### アーキテクチャ理解

- **ROS 2環境（pixi）**: `ros2_ws/`配下でpixiにより管理
- **強化学習環境（uv）**: `rl_ws/`配下でuvにより管理（ROS 2とは独立）
- **プラットフォーム分離**: Jetson (linux-aarch64)とmacOS (osx-arm64)で利用可能な機能が異なる
- **ros2_control**: macOSでは利用不可（RoboStack Jazzyの制約）
- **リアルタイム制御**: Jetsonでは200Hz制御ループ、SCHED_FIFO優先度90推奨

### ディレクトリ構造

```
bsl_droid_control/
├── ros2_ws/                        # ROS 2ワークスペース（pixi管理）
│   └── src/
│       ├── biped_description/      # URDF/可視化/GUI
│       ├── biped_gait_control/     # 歩容生成サンプル
│       ├── robstride_hardware/     # ros2_control用IF（Jetson専用・開発中）
│       └── pub_sub_cpp|python/     # ROS 2チュートリアル
├── rl_ws/                          # 強化学習環境（uv管理）
│   ├── biped_walking/              # 二脚ロボット環境・訓練スクリプト
│   │   ├── envs/                   # 環境定義（biped_env.py, droid_env.py）
│   │   └── train/                  # 訓練スクリプト群
│   ├── assets/                     # URDFモデル
│   ├── logs/                       # 訓練ログ・チェックポイント
│   ├── genesis_official/           # Genesis物理シミュレータ（submodule）
│   └── mujoco_menagerie/           # MuJoCoモデル集（submodule）
├── doc/                            # 設計資料
│   ├── design/                     # システム設計ドキュメント
│   └── experiments/                # 実験レポート群
└── ref/                            # 参考実装・外部ライブラリ
```

---

## セキュリティ重要事項

- ハードコードされた認証情報，APIキー，シークレットがないか確認
- CAN通信関連でモータID等のハードコードが適切か確認
- 設定ファイルに機密情報が含まれていないか確認

## リアルタイム・安全性チェック（最重要）

ロボット制御システムであるため，以下を最優先で確認：

### モータ制御の安全性

- 関節可動範囲のリミットチェックが実装されているか
- 緊急停止（E-Stop）機構が適切に実装されているか
- モータ電流/トルク制限が設定されているか
- 異常検出時のフォールバック動作が定義されているか

### リアルタイム制約

- 制御ループ内でのメモリ動的確保を避けているか
- ブロッキング操作（I/O待ち等）が制御ループに含まれていないか
- ログ出力が制御ループのタイミングに影響しないか
- スレッド優先度が適切に設定されているか

```cpp
// 避けるべきパターン
void control_loop() {
    std::vector<double> data;  // ❌ 動的確保
    data.resize(10);           // ❌ 再確保の可能性
    LOG_INFO("状態: %f", state);  // ❌ ブロッキングの可能性
}

// 推奨パターン
void control_loop() {
    // メンバ変数として事前確保済みのバッファを使用
    // RT-safeなログ機構を使用（非同期キュー等）
}
```

---

## ROS 2固有のチェック項目

### ノード設計

- QoS設定が用途に適切か（リアルタイムならRELIABLE，センサデータならBEST_EFFORT等）
- コールバックグループが適切に分離されているか
- ライフサイクルノードの状態遷移が正しく実装されているか
- パラメータのバリデーションが実装されているか

### トピック/サービス

- トピック名がROS 2命名規則に従っているか（snake_case）
- メッセージ型が適切か（標準メッセージを優先）
- 周期的パブリッシュの周波数が適切か
- サービスのタイムアウト処理が実装されているか

```python
# 推奨パターン
self.publisher = self.create_publisher(
    JointState,
    'joint_states',
    qos_profile=rclpy.qos.QoSProfile(
        reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        depth=10
    )
)
```

### ros2_control（robstride_hardware）

- Hardware Interfaceの`read()`/`write()`が例外をスローしないか
- 状態インターフェースと指令インターフェースの整合性
- エラー時のリカバリー処理が実装されているか

---

## Python コーディング規約

### 命名規則

- 変数・関数: `snake_case`
- クラス: `PascalCase`
- 定数: `UPPER_SNAKE_CASE`
- プライベートメンバ: `_leading_underscore`

### 型ヒント

- 関数シグネチャには型ヒントを付与
- rclpyのコールバックも適切に型付け

```python
# 推奨
def joint_state_callback(self, msg: JointState) -> None:
    self.positions: list[float] = list(msg.position)

# 避ける
def joint_state_callback(self, msg):
    self.positions = msg.position
```

### ROS 2 Python

- `rclpy.spin()`ではなく`rclpy.spin_once()`のループを検討（シャットダウン制御）
- `destroy_node()`の呼び出しを忘れない
- `try-finally`でノードのクリーンアップを保証

---

## C++ コーディング規約

### 命名規則

- 変数・関数: `snake_case`
- クラス: `PascalCase`
- メンバ変数: `trailing_underscore_`
- 定数: `kPascalCase` または `UPPER_SNAKE_CASE`
- 名前空間: `snake_case`

### モダンC++

- `nullptr`を使用（`NULL`ではなく）
- スマートポインタを使用（`std::unique_ptr`, `std::shared_ptr`）
- 範囲for文を優先
- `auto`の適切な使用（型が明らかな場合）

```cpp
// 推奨
auto node = std::make_shared<MyNode>();
for (const auto& joint : joints_) {
    // ...
}

// 避ける
MyNode* node = new MyNode();  // ❌ 生ポインタ
for (int i = 0; i < joints_.size(); i++) {  // ❌ インデックスループ
    // ...
}
```

### ros2_control Hardware Interface

- `CallbackReturn`の戻り値を適切に使用
- `hardware_interface::HW_IF_*`定数を使用
- `rclcpp_lifecycle`の状態遷移を正しく実装

---

## URDF/Xacro チェック項目

### 構造

- リンク間の親子関係が正しいか
- 慣性パラメータ（mass, inertia）が物理的に妥当か
- 関節の可動範囲（limit）が実機の仕様と一致するか
- 原点（origin）の設定が正しいか

### 逆関節（Digitigrade）固有

- 膝関節の可動範囲が負の角度（-120°〜0°）に設定されているか
- 足首関節の角度オフセットが正しいか

```xml
<!-- 逆関節の膝: 負の角度で前方に曲がる -->
<limit lower="${-120 * pi / 180}" upper="0" effort="100" velocity="10"/>
```

---

## Launch ファイルチェック項目

- 引数のデフォルト値が適切か
- 条件分岐（`IfCondition`等）が正しく動作するか
- 名前空間の設定が意図通りか
- リマッピングが正しいか

---

## パフォーマンスチェック

- 不必要なコピーを避けているか（`const&`の使用）
- 事前確保可能なコンテナは`reserve()`を使用しているか
- 高頻度ループ内での文字列操作を避けているか
- 行列演算でEigenを適切に使用しているか（一時オブジェクトの回避）

---

## ドキュメント要件

### コード内ドキュメント

- 公開APIにはdocstring/Doxygenコメントを付与
- 複雑なアルゴリズムには説明コメントを追加
- TODO/FIXMEには担当者とIssue番号を記載

### ファイルドキュメント

- 新規パッケージには`README.md`を作成
- 主要コマンドはリポジトリルートの`README.md`に記載（パッケージ内では参照のみ）
- 図は`.drawio.svg`形式で作成（**DrawIO XML形式**，純粋SVGは不可）

### 実験レポート構造

`doc/experiments/`以下の実験レポートは以下の規則に従うこと：

```
doc/experiments/exp{NNN}_{実験名}/
├── exp{NNN}_{実験名}.md                    # 主レポート（必須）
├── exp{NNN}_rules.md                       # 実験ルール・手順（推奨）
└── exp{NNN}_{実験名}_{補足}.drawio.svg     # 図（任意）
```

- `{NNN}`: 3桁の通し番号（001, 002, ...）
- 補足ファイルは必ず主レポートから参照すること（孤立ファイル禁止）

### DrawIO図の形式【厳守】

`.drawio.svg`ファイルは**DrawIO XML形式**（`<mxfile>`タグ）で作成すること．

```xml
<!-- 正しい形式 -->
<mxfile host="65bd71144e">
    <diagram name="Diagram Name" id="diagram-id">
        <mxGraphModel ...>
            ...
        </mxGraphModel>
    </diagram>
</mxfile>
```

**禁止**: 純粋なSVG形式（`<svg>`タグ）は不可（DrawIOで編集できなくなる）

---

## テスト要件

- 新機能にはユニットテストを追加
- 安全クリティカルな機能には複数のテストケース
- パラメータ境界値のテスト
- 異常系のテスト（タイムアウト，無効入力等）

---

## レビュースタイル

- 具体的かつ実行可能なフィードバックを提供
- 問題点だけでなく「なぜ」問題なのかを説明
- 良いパターンは積極的に称賛
- 安全性とリアルタイム性の問題を最優先で指摘

---

## 強化学習環境（rl_ws）固有のチェック項目

### 環境設定

- エピソード終了条件が妥当か
- 物理パラメータ（慣性，摩擦等）が現実的か

### 訓練スクリプト

- ハイパーパラメータがハードコードされていないか
- チェックポイント保存が適切に実装されているか

### 実験管理

- 実験名が命名規則（`biped-walking-v{N}`, `droid-walking-v{N}`）に従っているか
- 対応する`exp{NNN}_rules.md`のルールに従っているか
