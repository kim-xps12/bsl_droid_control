# BSL-Droid ハードウェア構成設計

## 1. 概要

本文書は、BSL-Droidの**ハードウェア物理レイヤー**を定義する。具体的には、使用する機材の型番・仕様、電源系統の設計、通信配線の構成、および安全設計を記載する。

ソフトウェアアーキテクチャ（ROS 2ノード構成・メッセージ定義等）は本文書のスコープ外である。

---

## 2. コンポーネント一覧

| カテゴリ | コンポーネント | 型番 | 数量 | 役割 |
|---|---|---|---|---|
| 制御コンピュータ | Jetson Orin Nano Super | NVIDIA Jetson Orin Nano Super | 1 | ロボット搭載制御コンピュータ |
| アクチュエータ | モータ | RobStride RS02 | 10 | 各関節駆動（左右5軸ずつ） |
| 通信 | USB-CANアダプタ | DSD-Tech SH-C31G | 2 | CAN通信（制御系・駆動系間を電気的に絶縁） |
| センサ | IMU | WitMotion WT901C (RS232版) | 1 | 角速度・加速度・姿勢推定 |
| 入力 | ゲームパッド | Logitech F710r | 1 | 遠隔操縦（DirectInputモード） |
| 駆動系電源 | バッテリ | マキタ BL4025 | 1 | 36V (40Vmax) モータ駆動用 |
| 制御系電源 | バッテリ | Moman Power 50 Touch | 1 | D-TAP端子 16.7V 制御系給電 |
| 安全 | パワーリレー | OMRON G9EA-1-B DC24 | 1 | 非常停止時のモータ電源遮断（NO構成） |
| 安全 | 非常停止スイッチ | Schneider Electric ZBRRA | 1 | 遠隔非常停止（24V動作） |
| 電源変換 | DC-DC昇圧コンバータ | (16.7V→24V) | 1 | ZBRRA用24V生成 |
| 配線 | WAGOワンタッチコネクタ | - | 適宜 | 制御系電源の分岐 |
| 配線 | CAN終端抵抗 | 120 ohm | 2 | 各CANバス足先側終端（XT30(2+2)経由） |

---

## 3. 電源系統

### 3.1 設計原則

- **制御系と駆動系の完全電気絶縁**: 制御系（Jetson等）と駆動系（モータ）のGNDを完全に分離する。絶縁はUSB-CANアダプタDSD-Tech SH-C31Gの内蔵アイソレータに依存する。
- **フェイルセーフ**: 制御系電源の喪失時にモータ電源が自動遮断される設計とする。

### 3.2 駆動系電源

マキタ BL4025（36V / 40Vmax リチウムイオンバッテリ）からRS02モータ群へ直接給電する。電源経路はOMRON G9EA-1-Bパワーリレーを経由し、非常停止時に遮断される。

### 3.3 制御系電源

Moman Power 50 TouchのD-TAP端子から得られる16.7Vを大本の電源とする。

- **Jetson給電**: D-TAP 16.7VをJetson Orin Nano Superに直結する。
- **24V生成**: WAGOワンタッチコネクタで16.7Vを分岐し、DC-DC昇圧コンバータ（16.7V→24V）に入力する。得られた24VをSchneider Electric ZBRRAの電源に利用する。

### 3.4 非常停止 (E-stop) 回路

| 要素 | 仕様 |
|---|---|
| 非常停止スイッチ | Schneider Electric ZBRRA（遠隔操作、24V動作） |
| パワーリレー | OMRON G9EA-1-B DC24（Normally Open構成） |
| 遮断対象 | 駆動系電源（Makita BL4025 → RS02モータ間の36V電源ライン） |

**動作原理**: ZBRRAが24Vでリレーコイルを励磁し、NO接点が閉じてモータ電源が供給される。以下のいずれかの状態でリレーコイルが消磁され、接点が開放（＝モータ電源遮断）される。

| フェイルセーフ条件 | 動作 |
|---|---|
| E-stopボタン押下 | ZBRRAがリレーコイルへの24V供給を停止 → 接点開放 |
| 制御系バッテリ枯渇 | DC-DC入力喪失 → 24V消失 → 接点開放 |
| DC-DCコンバータ故障 | 24V消失 → 接点開放 |

### 3.5 電源系統図

![電源系統図](./fig/hardware_power.drawio.svg)

---

## 4. 通信系統

### 4.1 設計原則

- **左右脚のCANバス分離**: 各脚のモータ群を独立したCANバスに接続し、配線経路をシンプルにしつつモータへの通信周期を上げる。
- **電気的絶縁**: USB-CANアダプタ（SH-C31G）が制御系と駆動系の絶縁境界を兼ねる。

### 4.2 CANバス構成

#### USB-CANアダプタ

| 項目 | 仕様 |
|---|---|
| 型番 | DSD-Tech SH-C31G |
| 互換性 | CANable 2.0互換（gs_usbドライバ） |
| 絶縁 | 内蔵アイソレータ（制御系GNDと駆動系GNDの分離） |
| ビットレート | 1 Mbps |
| デバイスノード | can1（左脚）、can2（右脚） |

#### モータID割当

| CANバス | デバイスノード | 関節名 (URDF) | モータID |
|---|---|---|---|
| 左脚 | can1 | left_hip_yaw_joint | 11 |
| 左脚 | can1 | left_hip_roll_joint | 12 |
| 左脚 | can1 | left_hip_pitch_joint | 13 |
| 左脚 | can1 | left_knee_pitch_joint | 14 |
| 左脚 | can1 | left_ankle_pitch_joint | 15 |
| 右脚 | can2 | right_hip_yaw_joint | 21 |
| 右脚 | can2 | right_hip_roll_joint | 22 |
| 右脚 | can2 | right_hip_pitch_joint | 23 |
| 右脚 | can2 | right_knee_pitch_joint | 24 |
| 右脚 | can2 | right_ankle_pitch_joint | 25 |

#### CAN終端抵抗

各CANバスの両端に120 ohmの終端抵抗を配置する（CAN規格準拠）。

| 位置 | 方式 |
|---|---|
| アダプタ側（Jetson側） | SH-C31G内蔵の120 ohm終端抵抗を使用 |
| 反対側（足先側） | 外付け120 ohm抵抗をXT30(2+2)端子経由で接続 |

RS02モータには終端抵抗が内蔵されていないため、外付けで対応する。

### 4.3 IMU接続

| 項目 | 仕様 |
|---|---|
| 型番 | WitMotion WT901C (RS232版) |
| 接続 | RS232 → USB変換 → Jetson USBポート |
| 取付位置 | 胴体中央上部 |
| 座標系 | X軸: 前方正、Y軸: 左手側正、Z軸: 鉛直上方正（ROS REP-103準拠） |

### 4.4 ゲームパッド

| 項目 | 仕様 |
|---|---|
| 型番 | Logitech F710r |
| 接続 | USBワイヤレスドングル → Jetson USBポート（または開発時MacBook） |
| モード | DirectInput（本体背面スイッチで切替） |

### 4.5 ネットワーク接続

Jetson ↔ MacBook間はWiFiまたはEthernetで接続し、ROS 2 DDS通信を行う。

### 4.6 通信系統図

![通信系統図](./fig/hardware_communication.drawio.svg)

---

## 5. インターフェース設定

### 5.1 SocketCAN設定

```bash
# 各CANバスの起動
sudo ip link set can1 up type can bitrate 1000000
sudo ip link set can2 up type can bitrate 1000000
```

### 5.2 インターフェース名の永続化（systemd.link）

SH-C31Gアダプタ2個に対して、シリアル番号に基づくインターフェース名の永続化を行う。

SH-C31Gはcandlelight（gs_usb）ファームウェアで動作するため、シリアルデバイス（`/dev/ttyUSBX`）ではなくSocketCANネットワークインターフェース（`canX`）として認識される。ネットワークインターフェースの命名はsystemd-udevdが管理するため、udevルールの `NAME=` ではなく `systemd.link` を使用する。

#### シリアル番号の確認

```bash
# アダプタ接続後、各インターフェースのシリアル番号を確認
# 注: systemd.link設定前はカーネルが自動割当した名前（can0, can1等）で確認する
udevadm info /sys/class/net/can0 | grep ID_SERIAL_SHORT
udevadm info /sys/class/net/can1 | grep ID_SERIAL_SHORT
```

#### systemd.linkファイルの作成

```ini
# /etc/systemd/network/60-can-left.link
[Match]
Property=ID_SERIAL_SHORT="<LEFT_SERIAL>"

[Link]
Name=can1
```

```ini
# /etc/systemd/network/60-can-right.link
[Match]
Property=ID_SERIAL_SHORT="<RIGHT_SERIAL>"

[Link]
Name=can2
```

`<LEFT_SERIAL>` および `<RIGHT_SERIAL>` は各アダプタのシリアル番号に置き換えること。設定反映後、アダプタを再接続するかシステムを再起動する。

> **注意**: udevルールの `NAME=` によるネットワークインターフェースのリネームはsystemd環境下で非推奨であり、動作が不安定になる場合がある。`systemd.link` の使用を推奨する（参考: [candleLight_fw #76](https://github.com/candle-usb/candleLight_fw/issues/76)）。

---

## 6. 安全設計まとめ

| 安全機構 | 種別 | 対象 | 動作 |
|---|---|---|---|
| E-stopスイッチ (ZBRRA) | ハードウェア | モータ電源 | ボタン押下でリレー開放、モータ電源遮断 |
| パワーリレー (G9EA-1-B) | ハードウェア | モータ電源 | NO構成: 制御電源喪失時に自動遮断 |
| 電源絶縁 (SH-C31G) | ハードウェア | CAN通信 | 制御系・駆動系GNDの電気的絶縁 |
| 制御系・駆動系電源分離 | ハードウェア | 電源系統全体 | 独立バッテリによる完全分離 |
| biped_safety_node | ソフトウェア | 姿勢・関節状態 | 異常検知時にソフトウェアE-stop発行 |
