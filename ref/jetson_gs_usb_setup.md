# Jetson Orin Nano Superでgs_usbファームのCANableアダプタを利用する方法

## 概要

Jetson Orin Nano Superでgs_usbファームウェアを使用したUSB-CANアダプタ（CANable、Geschwister Schneider CAN adapterなど）を利用する際、標準カーネルでは`CONFIG_CAN_GS_USB`が無効化されているため、そのままでは使用できません。本資料では、gs_usbカーネルモジュールをビルド・インストールして、USB-CANアダプタを利用可能にする手順を説明します。

## 環境情報

- **ハードウェア**: Jetson Orin Nano Super
- **OS**: Ubuntu 22.04 (JetPack 6.0)
- **カーネルバージョン**: 5.15.148-tegra
- **対象デバイス**: gs_usb対応USB-CANアダプタ (CANable, candleLight等)

## 問題の診断

### 1. デバイス認識の確認

USB-CANアダプタが接続されているか確認します。

```bash
lsusb
```

出力例:
```
Bus 001 Device 007: ID 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter
```

### 2. カーネルモジュールの確認

gs_usbモジュールが存在するか確認します。

```bash
modinfo gs_usb
```

エラーが出る場合、モジュールがインストールされていません:
```
modinfo: ERROR: Module gs_usb not found.
```

### 3. カーネルコンフィグの確認

```bash
zcat /proc/config.gz | grep CAN_GS_USB
```

出力:
```
# CONFIG_CAN_GS_USB is not set
```

この場合、カーネルでgs_usbサポートが無効化されています。

## 解決方法

### 必要なパッケージのインストール

カーネルモジュールのビルドに必要なヘッダーファイルを確認します。

```bash
dpkg -l | grep linux-headers
```

NVIDIA L4T カーネルヘッダーがインストールされていることを確認:
```
ii  nvidia-l4t-kernel-headers  5.15.148-tegra-36.4.7
```

### gs_usbモジュールのビルド

#### 1. 作業ディレクトリの作成

```bash
mkdir -p /tmp/gs_usb_build
cd /tmp/gs_usb_build
```

#### 2. ソースコードの取得

Linux kernel 5.15からgs_usb.cをダウンロードします。

```bash
curl -o gs_usb.c "https://raw.githubusercontent.com/torvalds/linux/v5.15/drivers/net/can/usb/gs_usb.c"
```

#### 3. Makefileの作成

```bash
cat > Makefile << 'EOF'
obj-m += gs_usb.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a
EOF
```

#### 4. モジュールのビルド

```bash
make
```

成功すると、`gs_usb.ko`が生成されます。

```bash
ls -lh gs_usb.ko
```

### モジュールのインストール

#### 1. インストールディレクトリの作成

```bash
sudo mkdir -p /lib/modules/$(uname -r)/kernel/drivers/net/can/usb/
```

#### 2. モジュールのコピー

```bash
sudo cp gs_usb.ko /lib/modules/$(uname -r)/kernel/drivers/net/can/usb/
```

#### 3. モジュール依存関係の更新

```bash
sudo depmod -a
```

#### 4. モジュールのロード

```bash
sudo modprobe gs_usb
```

#### 5. 確認

```bash
lsmod | grep gs_usb
```

出力例:
```
gs_usb                 24576  0
can_dev                36864  2 mttcan,gs_usb
```

## CANインターフェースの設定

### デバイスの確認

```bash
ip link show
```

USB-CANアダプタは通常`can1`として認識されます（`can0`はJetsonオンボードのmttcan）。

出力例:
```
9: can1: <NOARP,ECHO> mtu 16 qdisc noop state DOWN mode DEFAULT group default qlen 10
    link/can
```

### インターフェースの起動

ビットレートを設定してインターフェースを起動します（1コマンドで実行可能）。

```bash
sudo ip link set can1 type can bitrate 1000000 up
```

ビットレートの例:
- 1 Mbps: `1000000`
- 500 kbps: `500000`
- 250 kbps: `250000`

### 状態確認

```bash
ip -details link show can1
```

出力例:
```
9: can1: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can
    can state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 0
          bitrate 1000000 sample-point 0.875
          tq 25 prop-seg 34 phase-seg1 35 phase-seg2 10 sjw 1
          gs_usb: tseg1 1..16 tseg2 1..8 sjw 1..4 brp 1..1024 brp-inc 1
          clock 48000000
```

## 自動起動設定

システム起動時に自動的にgs_usbモジュールをロードするように設定します。

### モジュールの自動ロード

```bash
sudo bash -c 'echo "gs_usb" > /etc/modules-load.d/gs_usb.conf'
```

### CANインターフェースの自動起動（オプション）

systemdサービスを作成して、起動時にCANインターフェースを自動的に起動することも可能です。

```bash
sudo cat > /etc/systemd/system/can1-setup.service << 'EOF'
[Unit]
Description=Setup CAN1 interface
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/sbin/ip link set can1 type can bitrate 1000000 up
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
```

サービスの有効化:

```bash
sudo systemctl enable can1-setup.service
sudo systemctl start can1-setup.service
```

## 動作確認

### can-utilsのインストール

```bash
sudo apt-get update
sudo apt-get install can-utils
```

### CANメッセージの受信

```bash
candump can1
```

### CANメッセージの送信

別のターミナルで:

```bash
cansend can1 123#DEADBEEF
```

### インターフェース統計の表示

```bash
ip -s link show can1
```

## トラブルシューティング

### モジュールがロードできない

```bash
sudo dmesg | tail -20
```

エラーメッセージを確認してください。

### インターフェースが起動しない

ビットレートが設定されているか確認:

```bash
ip -details link show can1
```

デバイスが接続されているか確認:

```bash
lsusb | grep -i "can\|schneider"
```

### 複数のCANインターフェース

Jetson Orin Nanoには以下のCANインターフェースがあります:
- `can0`: mttcan（オンボード）
- `can1`: gs_usb（USB-CANアダプタ）

必要に応じて使い分けてください。

## まとめ

本手順により、Jetson Orin Nano Superでgs_usbファームウェアを使用したUSB-CANアダプタを利用できるようになります。主なステップは以下の通りです:

1. カーネルヘッダーの確認
2. gs_usbモジュールのビルド
3. モジュールのインストールとロード
4. CANインターフェースの設定と起動
5. 自動起動設定（オプション）

この設定により、CANable、candleLight、その他のgs_usb対応アダプタがJetson上で動作します。

## 参考情報

- Linux Kernel CAN drivers: https://www.kernel.org/doc/html/latest/networking/can.html
- gs_usb driver: https://github.com/torvalds/linux/blob/master/drivers/net/can/usb/gs_usb.c
- CANable: https://canable.io/
- candleLight: https://github.com/candle-usb/candleLight_fw
