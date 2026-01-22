# ROS 2 をベースにした CAN 通信 QDD ロボット制御システム設計書

本資料は、ROS 2 を中核プラットフォームとして **CAN 通信 QDD（Quasi Direct Drive）アクチュエータを用いたロボットシステム**を構築する際に必要となる技術要素を、網羅的・体系的・構造的に整理した技術ドキュメントである。単なる概念解説ではなく、**リアルタイム性・周期制御・スレッド設計・CAN 通信設計・ROS 2 Executor との連携・PREEMPT_RT / OS スケジューリング最適化・Python RL との協調**まで全体的に踏み込む。要約による圧縮は行わず、重要論点はすべて明示する。

---

# 1. 全体アーキテクチャ

## 1.1 システム構成の全体像

ROS 2 ベースの CAN-QDD ロボット制御システムは、以下の 3 層を中心に構成する。

1. **リアルタイム制御層（200 Hz など）**

   * C++ による専用スレッドで絶対時刻周期制御（`sleep_until`）
   * QDD への CAN Frame 書き込み、センサリードバック処理
   * SCHED_FIFO / CPU affinity など OS RT 最適化を適用

2. **非リアルタイム ROS 2 通信層（50 Hz など）**

   * Python/C++ ノードが pub/sub により高レベル指令・状態伝播を行う
   * 強化学習ポリシー（推論 50 Hz）→ 制御層へ非同期に最新コマンドを供給

3. **クラウド・解析・上位アプリ層**

   * ロギング（MCAP）、可視化（RViz）、診断、遠隔監視
   * 強化学習・シミュレーション・計画生成

**ポイント：** ROS 2 Executor に周期制御を任せず、**制御層と ROS 層を分離**することでリアルタイム性を担保する。

## 1.2 ハードウェア構成

* Jetson Orin Nano Super（6-core ARM Cortex-A78AE）
* 内蔵 or 拡張 CAN コントローラ（socketcan ベース）
* QDD アクチュエータ：RobStride RS02/05、GIM6010 系 等
* 電源・保護回路（ヒューズ・リレー・TVS・配線）
* 別途センサ：IMU、RGB-D（RealSense）、エンコーダ付腿部リンク等

---

# 2. ROS 2 とリアルタイム性

## 2.1 ROS 2 Executor の性質

ROS 2 は **callback を Executor がスレッドで実行する**アーキテクチャであり、リアルタイム保証を提供するものではない。

* SingleThreadedExecutor：直列実行、同期性はよいがリアルタイム性は低い
* MultiThreadedExecutor：スレッドプール型、並列化はできるが RT には不向き
* CallbackGroup（Reentrant / MutuallyExclusive）：並列性制御だが RT 保証ではない

**重要：** ROS 2 Executor のスレッドは Linux の通常スケジューリングクラスで動くため、**制御周期の保証には使えない**。

## 2.2 ROS 2 でリアルタイム制御を行うための原則

* **周期制御は ROS 2 executor に委ねない**
* **専用制御スレッドを C++（std::thread/pthread）で生成**する
* **absolute-time loop (`sleep_until`)** で周期位相を維持
* **非リアルタイムのコールバックは専用 executor で分離**
* コールバックと制御ループ間のデータ受け渡しは **ロックレス or 最小ロック**
* スレッド優先度（SCHED_FIFO）と CPU affinity を設定
* PREEMPT_RT カーネルによりスケジューラレイテンシを低減

## 2.3 RT カーネル（PREEMPT_RT）の役割

PREEMPT_RT は API を増やすのではなく、**既存の pthread / scheduler をリアルタイム的に正しく動作させる**ための内部改善パッチである。

改善点：

* 割り込みの thread 化
* スピンロックの mutex 置換と優先度継承
* スケジューラプリエンプション禁止区間の縮小

効果の本質：

* **worst-case latency の低減**
* **ジッタ（周期変動）の収束**
* **高優先度スレッドの即時実行性向上**

---

# 3. 200 Hz 制御ループ設計

## 3.1 絶対時刻制御 (`sleep_until`) の重要性

* `sleep_for`：相対スリープ → **ドリフト蓄積** → 制御周期がズレる
* `sleep_until`：絶対時刻 → **位相維持** → 安定した 200 Hz 周期

周期制御の正道：

```cpp
next = steady_clock::now();
while (running) {
    compute_control();
    next += period_5ms;
    std::this_thread::sleep_until(next);
}
```

## 3.2 制御スレッドの RT 設定（必須）

* `pthread_setschedparam` による **SCHED_FIFO / 優先度 90–98**
* `pthread_setaffinity_np` による **専用 CPU コア固定**
* `mlockall(MCL_CURRENT | MCL_FUTURE)` によるページフォールト回避（任意）
* 制御スレッド内で **動的メモリ確保禁止**
* 制御スレッド内で **ROS 通信禁止**（publish/subscription/logging）

## 3.3 データ受け渡し

### 実装選択肢

* `std::atomic<T>`（固定長の軽量構造体の場合）
* `realtime_tools::RealtimeBox / RealtimeBuffer`（ros2_control 標準）
* lock-free ring buffer（Boost.Lockfree など）

### 受け渡し原則

* **RT → 非RT**：非RT側が publish 等を担当
* **非RT → RT**：最新値のみ ZOH（Zero-Order Hold）で参照

---

# 4. CAN 通信設計（socketcan）

## 4.1 SocketCAN 構造

* Linux カーネルの CAN ネットワークスタック
* QDD と 1Mbps 帯域で高頻度通信を行う

## 4.2 QDD の典型通信要件

* 制御コマンド（位置/速度/トルク）送信：200 Hz
* ステータスフレーム受信：100–500 Hz（アクチュエータ仕様による）
* 電流/温度/エンコーダ角度/エラー状態

## 4.3 C++ 実装時の注意

* 制御フレーム構築を **固定長バッファ**で行う
* エラーチェック（CAN ID / DLC / CRC 等）を最小開示
* 送信は **ノンブロッキング send()** を基本とする
* 制御周期内で書き込み完了待ちをしない

## 4.4 CAN 通信スレッド構成

推奨：

* **TX スレッド（送信）**：200 Hz 制御ループと同一スレッド
* **RX スレッド（受信）**：select/poll で受信し、最新値を共有バッファへ
* **ROS 2 publish スレッド**：非RT側

---

# 5. ROS 2 ノード構造の分離設計

## 5.1 基本構成

```
[ROS executor thread] -- subscription callback
         ↓ latest_cmd (atomic / RT buffer)
[200Hz RT control thread] -- send CAN
         ↑ latest_state (RT→nonRT buffer)
[ROS executor thread] -- publish state
```

## 5.2 資料に基づく設計原則

* RT ループは **ROS executor とは無関係に自走**させる
* ROS callback は **non-RT executor（SingleThreaded）** で処理
* executor で遅延やブロッキングが起きても RT ループは影響しない
* RT スレッドと callback 間では **mutex を共有しない**

## 5.3 QoS 設計

* 上位指令（RL from Python） → KEEP_LAST(1), BEST_EFFORT
* センサ情報（state） → KEEP_LAST(1 or N), RELIABLE
* タイムスタンプ管理 → 遅延検出 / timeout（安全停止）

---

# 6. Python 強化学習（RL）との協調

## 6.1 周波数の分離

* RL 推論：50 Hz（もしくは 10–100 Hz）
* 制御：200 Hz

**ZOH（Zero-Order Hold）** により、最新指令を 200 Hz 側が保持して使用する。

## 6.2 指令の受け渡し

* Python → ROS2 publish（非RT）
* C++ ノード（executor thread）が callback で latest_cmd を更新
* 200 Hz 制御スレッドが latest_cmd を参照

## 6.3 遅延・途絶対策

* 最新指令のタイムスタンプで **staleness** 判定
* 指令途絶（例：100 ms）で自動停止 / ソフトリセット

---

# 7. OS および Jetson Orin Nano Super の最適化

## 7.1 PREEMPT_RT

* worst-case latency 低減
* IRQ thread 化
* スピンロック置換

## 7.2 CPU コア設計

* 6 コアのうち 1–2 コアを制御専用に固定（isolcpus または affinity）
* ROS executor や Python は別コアに隔離

## 7.3 メモリ管理

* mlockall によるページフォールト防止
* 制御ループでは new/delete 禁止
* ログ書き込みは非RT側に逃がす

---

# 8. 実装テンプレート（概要）

## 8.1 制御ノード骨格

```cpp
class ControlNode : public rclcpp::Node {
  std::atomic<Command> latest_cmd;
  RealtimeStateBuffer state_buf;
  
  void control_loop() {
    set_FIFO_priority(); set_affinity();
    next = now();
    while(running) {
      Command c = latest_cmd.load();
      State s = read_can();
      state_buf.write_from_rt(s);
      send_can(c);
      next += 5ms; sleep_until(next);
    }
  }
};
```

## 8.2 ROS executor（非RT）

```cpp
SingleThreadedExecutor exec;
exec.add_node(node);
std::thread ros_thread([&](){ exec.spin(); });
```

## 8.3 RL-Python

```python
pub = create_publisher(Cmd, qos_profile_sensor_data)
rate = Rate(50)
while rclpy.ok():
    cmd = policy(obs)
    pub.publish(cmd)
    rate.sleep()
```

---

# 9. システム全体の安定性設計

## 9.1 障害・安全性

* 指令途絶時の自動トルクオフ / 減速
* 温度・電流リミット監視（CAN フレーム）
* 異常 CAN ID / フレーム破損のチェック

## 9.2 ログ・診断

* MCAP ログ収集（非RT thread）
* 周期ジッタの計測（制御スレッド内部で timestamp 差分のみ計算）
* DDS 通信遅延計測

## 9.3 デバッグ

* ROS 2 rqt、rviz
* Jetson での cyclictest による RT 性能評価

---

# 10. まとめ

本資料では、**ROS 2 をベースに CAN-QDD ロボットを制御するためのリアルタイム設計全体**を網羅的に整理した。特に、

* ROS executor と RT 制御スレッドの分離
* sleep_until ベースの周期制御
* データ受け渡しの RT-safe 化
* PREEMPT_RT / SCHED_FIFO / CPU affinity
* RL-Python と 200 Hz 制御の周波数設計
* CAN 通信の TX/RX 分離

など、実機で安定した挙動を得るために必須となる項目を明示した。

この方針に基づけば、Jetson Orin Nano Super の計算資源と QDD アクチュエータの性能を最大限に活かしつつ、ROS 2 と ML アプリケーションを安全かつリアルタイム性を損なわず統合できる。
