# Setup ROS 2 with pixi on Jetson

## Install

```bash
pixi init ros2_ws -c robostack-jazzy -c conda-forge
cd ros2_ws
pixi add ros-jazzy-desktop ros-jazzy-ros2run
# Pythonパッケージのビルド用
pixi add colcon-common-extensions
# C++パッケージのビルド用（C++開発をする場合）
pixi add ros-jazzy-ament-cmake-auto compilers pkg-config cmake ninja
```

### Example1: Python publisher / subscriber

```bash
pixi run ros2 pkg create --build-type ament_python --destination-directory src --node-name talker pub_sub_python
pixi run colcon build
```

### Example2: C++ pub/sub

```bash
pixi run ros2 pkg create --build-type ament_cmake --destination-directory src --node-name talker pub_sub_cpp
```

---

## RobStride RS02 モーター制御

RobStride RS02アクチュエータをros2_control準拠で制御する。

> **ハードウェア要件**: USB-to-CANインターフェース（Canableなど）が必要  
> 参考: http://canable.io/

> **アーキテクチャ**: [arch_ros2_control.md](arch_ros2_control.md) を参照  
> **実装詳細**: [robstride_hardware_impl.md](robstride_hardware_impl.md) を参照

---

### 1. 依存パッケージのインストール

```bash
cd ros2_ws

# ros2_control関連
pixi add ros-jazzy-ros2-control ros-jazzy-ros2-controllers
pixi add ros-jazzy-controller-manager ros-jazzy-hardware-interface
pixi add ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller
ixi add ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-broadcaster ros-jazzy-forward-command-controller

# システムツール
sudo apt install net-tools can-utils

# C++ビルド用
pixi add compilers pkg-config cmake ninja
```

---

### 2. RobStride_Controlライブラリの取り込み

[Seeed-Projects/RobStride_Control](https://github.com/Seeed-Projects/RobStride_Control) をgit submoduleとして取り込む。

```bash
cd ~/ros2_ws

# submoduleとして追加
git submodule add https://github.com/Seeed-Projects/RobStride_Control.git src/RobStride_Control

# 他の開発者向け（リポジトリクローン後）
git submodule update --init --recursive

# バージョン固定（推奨）
cd src/RobStride_Control
git checkout <commit-hash>
cd ../..
git add src/RobStride_Control
git commit -m "Pin RobStride_Control version"
```

---

### 3. 空パッケージの作成

```bash
cd ~/ros2_ws

# ament_cmake パッケージを作成
pixi run ros2 pkg create --build-type ament_cmake --destination-directory src robstride_hardware

# ディレクトリ構造を追加
mkdir -p src/robstride_hardware/include/robstride_hardware
mkdir -p src/robstride_hardware/src
mkdir -p src/robstride_hardware/urdf
mkdir -p src/robstride_hardware/config
mkdir -p src/robstride_hardware/launch
```

**ディレクトリ構成**:
```
ros2_ws/src/
├── RobStride_Control/          # git submodule
│   └── cpp/
│       ├── include/
│       └── src/
└── robstride_hardware/         # 作成したパッケージ
    ├── CMakeLists.txt
    ├── package.xml
    ├── include/robstride_hardware/
    ├── src/
    ├── urdf/
    ├── config/
    └── launch/
```

---

### 4. package.xml の編集

`src/robstride_hardware/package.xml` に以下の依存を追加:

- `hardware_interface`
- `pluginlib`
- `rclcpp`
- `rclcpp_lifecycle`

---

### 5. CMakeLists.txt の編集

`src/robstride_hardware/CMakeLists.txt` で以下を設定:

- submoduleのソースパス: `../RobStride_Control/cpp`
- `target_include_directories` でsubmoduleのヘッダを追加
- `pluginlib_export_plugin_description_file` でplugin登録

---

### 6. ソースファイル作成

`src/robstride_hardware/src/robstride_hardware.cpp` を作成。

`hardware_interface::SystemInterface` を継承し、以下を実装:
- `on_init()`
- `export_state_interfaces()`
- `export_command_interfaces()`
- `read()`
- `write()`

---

### 7. Plugin定義ファイル作成

`src/robstride_hardware/robstride_hardware_plugin.xml` を作成し、Hardware Interfaceをpluginとして登録。

---

### 8. ビルド確認

```bash
cd ~/ros2_ws
pixi run colcon build --packages-select robstride_hardware
```

成功すれば `Summary: 1 package finished` と表示される。

---

### CANインターフェースのセットアップ

```bash
# カーネルモジュールのロード
sudo modprobe can can_raw can_dev

# can0の設定（1Mbps）
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ifconfig can0 txqueuelen 100

# 確認
ip link show can0
```

**起動時の自動設定**: `~/setup_can.sh` を作成して上記コマンドをまとめる。

---

### トラブルシューティング

| 症状 | 確認方法 |
|------|----------|
| submoduleが空 | `git submodule update --init --recursive` |
| ヘッダが見つからない | CMakeLists.txtのROBSTRIDE_DIRパス確認 |
| CANインターフェースがない | `ip link show` でcan0確認 |
| Permission denied | `sudo usermod -aG dialout $USER` |
