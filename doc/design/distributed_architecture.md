# 分散システムアーキテクチャ設計

## 1. システム概要

本リポジトリは、二脚ロボット制御システムを**2台のコンピュータで分散実行**する構成を取ります。

| マシン | 役割 | 主な処理 |
|--------|------|----------|
| **Jetson Orin Nano Super** | ロボット搭載制御コンピュータ | リアルタイム制御、センサ処理、モータ制御 |
| **MacBook** | 開発・可視化ワークステーション | RViz可視化、データ解析、デバッグ、開発 |

両マシンは**ROS 2 DDS通信**（WiFiまたはEthernet）で接続され、同一のROS 2ドメインでトピック・サービスを共有します。



## 2. ハードウェア構成

### 2.1 Jetson Orin Nano Super（ロボット搭載）

**スペック:**
- CPU: 6-core ARM Cortex-A78AE
- GPU: 1024-core NVIDIA Ampere
- RAM: 8GB LPDDR5
- OS: Ubuntu 22.04 (+ PREEMPT_RT カーネル, 将来対応)

**接続デバイス:**
- USB-CAN: RobStride RS02 x10
- USB: IMU センサ

### 2.2 MacBook（開発・可視化）

**用途:**
- RViz2 による可視化
- Plotjugglerによるリアルタイムプロット
- コード開発・デバッグ
- 物理シミュレーション

**接続:**
- ROS 2ネットワーク: WiFi または Ethernet（同一ネットワーク）
- 開発時: VSCod のRemote Explorer



## 3. ネットワーク構成

### 3.1 推奨ネットワーク構成

```
┌─────────────────────────────────────────────────────────────┐
│                    WiFi Router / Access Point               │
│                      (5GHz推奨、低レイテンシ)                  │
└─────────────────────────────────────────────────────────────┘
        │                                    │
        │ WiFi/Ethernet                      │ WiFi
        │                                    │
┌───────▼───────────┐              ┌─────────▼─────────┐
│ Jetson Orin Nano  │              │     MacBook       │
│ 192.168.x.10      │              │  192.168.x.20     │
│ (固定IP推奨)       │              │                   │
└───────────────────┘              └───────────────────┘
```

### 3.2 ROS 2 DDS設定

**ROS_DOMAIN_ID** を統一して、両マシンが同一ドメインで通信：

```bash
# 両マシンの ~/.bashrc に追加
export ROS_DOMAIN_ID=42
```

**FastDDS Discovery設定**（大規模データ転送時）:

```xml
<!-- fastdds_config.xml -->
<dds>
  <profiles>
    <participant profile_name="participant_profile" is_default_profile="true">
      <rtps>
        <builtin>
          <discovery_config>
            <leaseDuration>
              <sec>10</sec>
            </leaseDuration>
          </discovery_config>
        </builtin>
      </rtps>
    </participant>
  </profiles>
</dds>
```

### 3.3 帯域幅の考慮

| データ | 推定帯域 | 備考 |
|--------|----------|------|
| `/robot_state` (200Hz) | ~100 KB/s | 状態推定データ |
| `/joint_states` (200Hz) | ~50 KB/s | 関節状態 |
| `/imu/data` (200Hz) | ~30 KB/s | IMUデータ |
| `/camera/image` (30Hz, 640x480) | ~10 MB/s | 圧縮推奨 |
| `/tf` | ~20 KB/s | 座標変換 |

**推奨**: 画像データは`image_transport`で圧縮、または必要時のみ転送。

---

## 4. ノード配置

### 4.1 Jetson側ノード（リアルタイム・制御）

| ノード | 周波数 | 説明 |
|--------|--------|------|
| `robstride_hardware` | 200 Hz | ✅ 実装済み。ros2_control HW Interface |
| `controller_manager` | 200 Hz | ros2_control コントローラ管理 |
| `state_estimator` | 200 Hz | センサフュージョン、状態推定 |
| `safety_monitor` | 200 Hz | 安全監視、緊急停止 |
| `rl_policy` | 50-100 Hz | 強化学習ポリシー推論 |
| `gait_generator` | 50-100 Hz | 歩容パターン生成 |
| `imu_driver` | 200-1000 Hz | IMUセンサドライバ |
| `robot_state_publisher` | - | URDFからTF配信 |
| `joint_state_broadcaster` | 200 Hz | 関節状態のPublish |

### 4.2 MacBook側ノード（可視化・開発）

| ノード/ツール | 説明 |
|---------------|------|
| `rviz2` | 3D可視化（ロボットモデル、TF、センサデータ） |
| `plotjuggler` | リアルタイム時系列プロット |
| `rqt_graph` | ノード・トピック関係の可視化 |
| `rqt_console` | ログ監視 |
| `ros2 bag record` | データ記録（MCAPフォーマット） |
| `foxglove_bridge` | Webベース可視化（オプション） |

### 4.3 どちらでも実行可能なノード

| ノード | 推奨実行場所 | 備考 |
|--------|--------------|------|
| `teleop_twist_keyboard` | MacBook | 手動速度指令 |
| `joy_node` | MacBook | ゲームパッド入力 |
| `static_transform_publisher` | Jetson | 静的TF |
| シミュレータ（Gazebo等） | MacBook | 開発時のみ |

---


## 5. パッケージ構成（実装済み + 計画）

```
ros2_ws/src/
├── robstride_hardware/          # ✅ 実装済み (Jetson)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   ├── include/
│   ├── launch/
│   ├── src/
│   └── urdf/
│
├── biped_msgs/                  # 🔲 計画: カスタムメッセージ
│   ├── msg/
│   │   ├── RobotState.msg
│   │   └── SafetyStatus.msg
│   └── package.xml
│
├── biped_state_estimator/       # 🔲 計画: 状態推定 (Jetson)
│   ├── src/
│   └── launch/
│
├── biped_safety/                # 🔲 計画: 安全監視 (Jetson)
│   ├── src/
│   └── config/
│
├── biped_gait/                  # 🔲 計画: 歩容生成 (Jetson)
│   ├── src/
│   ├── models/                  # 学習済みRLポリシー
│   └── launch/
│
├── biped_description/           # 🔲 計画: URDF/可視化 (両方)
│   ├── urdf/
│   ├── meshes/
│   └── rviz/
│
└── biped_bringup/               # 🔲 計画: Launch統合
    └── launch/
```

---

## トラブルシューティング

### 8.1 ノードが見えない

```bash
# 両マシンでDOMAIN_IDを確認
echo $ROS_DOMAIN_ID

# マルチキャスト疎通確認
ros2 multicast receive  # 一方で実行
ros2 multicast send     # もう一方で実行

# ファイアウォール確認（Jetson側）
sudo ufw allow 7400:7500/udp
```

### 8.2 レイテンシが大きい

```bash
# QoS設定をBEST_EFFORTに（リアルタイムデータ）
# またはRELIABLEに（重要データ）

# WiFi → 有線Ethernetに切り替え検討
```

### 8.3 画像データが重い

```bash
# 圧縮転送を使用
ros2 run image_transport republish raw compressed \
  --ros-args -r in:=/camera/image -r out:=/camera/image/compressed
```

---

## 9. セキュリティ考慮事項

### 9.1 ネットワーク分離

- ロボット専用WiFiネットワークの使用を推奨
- 外部ネットワークからの隔離

### 9.2 DDS Security（オプション）

大会・デモ環境では、ROS 2 DDS Securityの有効化を検討：

```bash
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

---

## 10. まとめ

### 実行環境の役割分担

| 環境 | 責務 | 優先度 |
|------|------|--------|
| **Jetson** | リアルタイム制御、安全監視、センサ処理 | 最高（RT） |
| **MacBook** | 可視化、開発、デバッグ、データ解析 | 通常 |

### 通信の原則

1. **制御データはJetson内で完結**: 200Hz制御ループはネットワーク遅延の影響を受けない
2. **可視化データのみネットワーク転送**: MacBookへは状態・診断データを送信
3. **帯域幅を意識**: 画像は必要時のみ、圧縮転送を活用

### 開発の原則

1. **コードはGitで共有**: 両マシンで同一リポジトリを使用
2. **ビルドは各環境で実施**: アーキテクチャ（ARM/x86）が異なるため
3. **テストはシミュレータ → 実機**: MacBookでGazebo検証後、Jetsonで実機テスト
