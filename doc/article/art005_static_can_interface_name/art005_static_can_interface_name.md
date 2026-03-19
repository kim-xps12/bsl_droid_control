# USB-CANアダプタのインタフェース名をudevルールで固定する

## はじめに

複数のUSB-CANアダプタを使う環境では，USBの挿し順やハブのポート割り当てによってインタフェース名（`can0`, `can1`, ...）が起動ごとに変わってしまうことがあります．制御ソフトウェア側でバスを名前指定している場合，これは地味に厄介な問題です．

本記事では，Linuxの`udev`ルールを使い，特定のUSB-CANアダプタに固定のインタフェース名を割り当てる方法を紹介します．作業は1ファイルの作成だけで完了し，再起動後も永続的に有効です．

## 前提条件

- **OS**: Ubuntu 22.04（JetPack / L4T含む）またはそれに準ずるLinuxディストリビューション
- **USB-CANアダプタ**: `gs_usb`ドライバで動作するもの（canable, canable2など）
- **読者に求める知識**: Linuxのターミナル操作の基本

## 問題 — 挿し順でインタフェース名が変わる

たとえば，2つのUSB-CANアダプタを使用する環境を考えます．

```
# 1本目を挿す → can1 が割り当てられる
# 2本目を挿す → can2 が割り当てられる
```

しかし，挿す順番が変わると名前も入れ替わります．

```
# 2本目を先に挿す → can1 が割り当てられる（期待はcan2）
# 1本目を後に挿す → can2 が割り当てられる（期待はcan1）
```

`ros2 launch`のコマンドで`motors:='can2:21,22,23,24,25'`のようにバス名を指定している場合，意図しないバスにコマンドが送られてしまいます．

なお，Jetson OrinシリーズのようにオンボードのCANコントローラを持つ環境では，オンボード側が`can0`を占有しているため，USBアダプタは`can1`以降に割り当てられます．この場合でもUSBアダプタ同士の順番は不定です．

## 解決策 — udevルールでシリアル番号に基づいて名前を固定する

Linuxの`udev`は，デバイスが接続された際にルールに基づいて自動的に設定を適用する仕組みです．USB-CANアダプタは個体ごとに固有のシリアル番号を持っているため，これを識別子としてインタフェース名を固定できます．

### ステップ1: デバイス情報の取得

まず，対象のUSB-CANアダプタが現在どのインタフェース名で認識されているか確認します．

```bash
ip link show type can
```

```
2: can0: <NO-CARRIER,NOARP,UP,ECHO> mtu 16 ...    ← オンボードCAN（mttcan）
10: can1: <NOARP,UP,LOWER_UP,ECHO> mtu 16 ...     ← USB-CANアダプタ（gs_usb）
```

ドライバを表示して，対象がUSB接続であることを確認します．

```bash
ls -la /sys/class/net/can1/device/driver
# → ../../drivers/gs_usb   ← USB-CANアダプタ
```

次に，`udevadm`でこのデバイスのUSB属性を取得します．

```bash
udevadm info -a -p /sys/class/net/can1 \
  | grep -E '(ATTR|ATTRS)\{(serial|idVendor|idProduct|manufacturer|product)\}'
```

```
ATTRS{idProduct}=="606f"
ATTRS{idVendor}=="1d50"
ATTRS{manufacturer}=="canable.io"
ATTRS{product}=="canable2 gs_usb"
ATTRS{serial}=="000011112222333344445555"
```

ここで重要なのは以下の3つです．

| 属性 | 値 | 説明 |
|------|-----|------|
| `idVendor` | `1d50` | USBベンダーID |
| `idProduct` | `606f` | USBプロダクトID |
| `serial` | `000011112222333344445555` | 個体固有のシリアル番号 |

`serial`はアダプタの個体ごとに異なるため，これをudevルールの識別子として使用します．

### ステップ2: udevルールの作成

取得した情報をもとに，udevルールファイルを作成します．

```bash
sudo bash -c 'cat > /etc/udev/rules.d/90-can-usb.rules << EOF
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="000011112222333344445555", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", NAME="can2"
EOF'
```

このルールは「シリアル番号が`000011112222333344445555`であるUSB-CANアダプタが接続されたら，インタフェース名を`can2`に設定する」という意味です．

複数のアダプタを固定したい場合は，同じファイルに行を追加します．

```
# /etc/udev/rules.d/90-can-usb.rules
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="000011112222333344445555", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", NAME="can2"
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="XXXXXXXXXXXXXXXXXXXX", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", NAME="can3"
```

### ステップ3: ルールの反映

作成したルールをudevに読み込ませます．

```bash
sudo udevadm control --reload-rules
```

このコマンドでルールファイルがリロードされますが，**既に接続済みのデバイスには即時適用されません**．反映するには以下のいずれかを行います．

- USB-CANアダプタを抜き差しする
- システムを再起動する

### ステップ4: 動作確認

アダプタを挿し直した後，インタフェース名を確認します．

```bash
ip link show can2
```

```
10: can2: <NOARP,ECHO> mtu 16 qdisc pfifo_fast state DOWN mode DEFAULT group default qlen 10
    link/can
```

`can2`として認識されていれば成功です．他のUSB-CANアダプタの挿し順を変えても，このアダプタは常に`can2`になります．

## udevルールの各フィールド解説

ルールの各フィールドが何を意味しているかを整理します．

| フィールド | 種類 | 説明 |
|-----------|------|------|
| `SUBSYSTEM=="net"` | マッチ条件 | ネットワークサブシステムのデバイスに限定 |
| `ACTION=="add"` | マッチ条件 | デバイス追加時のみ発火 |
| `ATTRS{serial}=="..."` | マッチ条件 | 親デバイス（USBデバイス）のシリアル番号で個体を特定 |
| `ATTRS{idVendor}=="..."` | マッチ条件 | USBベンダーIDで機種を限定 |
| `ATTRS{idProduct}=="..."` | マッチ条件 | USBプロダクトIDで機種を限定 |
| `NAME="can2"` | アクション | インタフェース名を指定した値に設定 |

`ATTRS{}`（末尾にSが付く）は親デバイスの属性を参照するための構文です．USB-CANアダプタの場合，ネットワークインタフェースの親にあたるUSBデバイスにシリアル番号が格納されているため，`ATTRS`を使用しています．

## 注意点

### 名前の衝突に注意

オンボードCANが`can0`を使用している環境で`NAME="can0"`を指定すると，名前が衝突します．既存のインタフェース名と重複しない名前を選んでください．

### 同一機種・シリアル番号なしのアダプタ

安価なUSB-CANアダプタの中には，シリアル番号を持たない（または全個体で同一の値を返す）ものがあります．その場合はシリアル番号での識別ができないため，USBポートのパス（`KERNELS`属性）で区別する方法を検討してください．ただしこの方法は物理ポートに依存するため，ポートを変えると無効になります．

### ファイル名の番号について

`/etc/udev/rules.d/`内のルールファイルは番号順に処理されます．`90-can-usb.rules`という名前は，他のシステムルール（通常`99-`系）より先に処理されるため，安全な位置付けです．

## まとめ

- USB-CANアダプタのインタフェース名は挿し順によって変わるため，複数アダプタ環境では名前の固定が必要
- `udevadm info`でアダプタ固有のシリアル番号を取得し，`udev`ルールで`NAME`を指定すれば固定できる
- 作業は`/etc/udev/rules.d/`に1ファイルを作成するだけで完了し，コード変更は不要
- 反映にはアダプタの抜き差しまたは再起動が必要

## おわりに

デバイス名の固定は地味な作業ですが，複数バスを使う制御システムでは安定運用の基盤になります．特にロボットのように多数のモータをCAN経由で制御する場合，バスの取り違えは意図しない動作に直結するため，早めに設定しておくことをおすすめします．

本記事の内容はBSL-Droidプロジェクトにおいて，Jetson Orin Nano Super上でcaanble2アダプタを使用した環境で確認しています．
