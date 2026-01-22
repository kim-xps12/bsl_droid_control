# pixiによるMacbookでのROS 2開発

本記事では、パッケージマネージャー「pixi」を使用して、macOS（Apple Silicon）上でROS 2の開発環境を構築し、実際にロボットアプリケーションを開発する方法を解説します。

## 対象読者

- macOSでROS 2を使いたいITエンジニア
- ロボット開発に興味があるが、経験がない方
- Dockerを使わずにネイティブでROS 2を動かしたい方

## ROS 2とは

**ROS 2（Robot Operating System 2）** は、ロボットアプリケーション開発のためのオープンソースのミドルウェアです。「Operating System」と名前がついていますが、LinuxやWindowsのようなOSではなく、ロボット開発に必要な以下の機能を提供するフレームワークです：

- **ノード間通信**: ロボットの各コンポーネント（センサー、モーター制御、画像処理など）を独立したプロセス（ノード）として実装し、相互に通信
- **トピック（Pub/Sub）**: 非同期メッセージング。センサーデータの配信などに使用
- **サービス（Request/Response）**: 同期的なリクエスト・レスポンス通信
- **アクション**: 長時間かかる処理の非同期実行とフィードバック
- **パラメータ**: ノードの設定値を動的に変更
- **可視化ツール**: RViz2による3D可視化、rqtによるGUIツール群

### なぜROS 2なのか

ROS 1との主な違い：
- **リアルタイム対応**: 産業用ロボットで求められるリアルタイム制御に対応
- **マルチプラットフォーム**: Linux、Windows、macOSをサポート
- **DDS（Data Distribution Service）**: 業界標準の通信プロトコルを採用
- **セキュリティ**: 通信の暗号化、認証機能を標準サポート

## pixiとは

**pixi**は、prefix.dev社が開発したRust製のパッケージマネージャーです。conda/mambaと互換性があり、以下の特徴があります：

- **高速**: Rustで実装されており、依存関係解決が非常に高速
- **プロジェクト単位の環境管理**: `pixi.toml`ファイルで依存関係を管理
- **再現性**: ロックファイルにより、どの環境でも同じ依存関係を再現
- **タスクランナー**: npm scriptsのようなタスク定義が可能
- **クロスプラットフォーム**: Linux、macOS、Windowsをサポート

### なぜpixiでROS 2なのか

従来、macOSでROS 2を使うには以下の方法がありました：

1. **ソースからビルド**: 非常に時間がかかり、依存関係の解決が困難
2. **Docker**: 動作するが、GUIアプリケーション（RViz2など）の表示が面倒
3. **仮想マシン**: オーバーヘッドが大きい

**RoboStack**プロジェクトにより、ROS 2がcondaパッケージとして提供されるようになり、pixiを使って簡単にインストールできるようになりました。

## 環境

本記事の動作確認環境：

- **OS**: macOS（Apple Silicon / M1, M2, M3, M4）
- **pixi**: 最新版
- **ROS 2**: Jazzy Jalisco（2024年5月リリースの最新LTS）

## セットアップ

### 1. pixiのインストール

ターミナルで以下のコマンドを実行します：

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

インストール後、ターミナルを再起動するか、以下を実行します：

```bash
source ~/.bashrc  # または ~/.zshrc
```

インストール確認：

```bash
pixi --version
```

### 2. ROS 2プロジェクトの作成

新しいROS 2プロジェクトを作成します：

```bash
pixi init my_ros2_project -c robostack-jazzy -c conda-forge
cd my_ros2_project
```

このコマンドにより、以下の構造が作成されます：

```
my_ros2_project/
├── .gitattributes
├── .gitignore
└── pixi.toml
```

`-c`オプションで指定しているのはcondaチャンネルです：
- `robostack-jazzy`: ROS 2 Jazzyのパッケージを提供
- `conda-forge`: 一般的なPython/C++パッケージを提供

### 3. ROS 2パッケージのインストール

基本的なROS 2パッケージをインストールします：

```bash
pixi add ros-jazzy-desktop ros-jazzy-turtlesim
```

これにより、以下がインストールされます：
- `ros-jazzy-desktop`: ROS 2のデスクトップ環境（RViz2、rqtなど含む）
- `ros-jazzy-turtlesim`: 学習用のシンプルなシミュレータ

> **Note**: 初回インストールは数分かかります。ROS 2の全パッケージがダウンロードされます。

## 動作確認：turtlesimを動かす

ROS 2が正しくインストールされたことを確認するために、turtlesimを起動してみましょう。

### turtlesimノードの起動

ターミナル1で以下を実行：

```bash
pixi run ros2 run turtlesim turtlesim_node
```

亀のいるウィンドウが表示されれば成功です！

### 亀を操作する

ターミナル2を開き、以下を実行：

```bash
pixi run ros2 run turtlesim turtle_teleop_key
```

このターミナルにフォーカスを合わせた状態で、矢印キーを押すと亀が動きます。

### 何が起きているか

この例では、2つの**ノード**が動いています：

1. **turtlesim_node**: 亀のシミュレーションと描画を担当
2. **turtle_teleop_key**: キーボード入力を受け取り、移動コマンドを送信

これらは`/turtle1/cmd_vel`という**トピック**を介して通信しています。`turtle_teleop_key`がキー入力に応じて速度コマンドを**publish（配信）**し、`turtlesim_node`がそれを**subscribe（購読）**して亀を動かしています。

## macOS固有の警告について

macOSでROS 2を実行すると、以下のような警告が表示されることがあります：

```
[SYSTEM Error] Problem to set affinity of thread with id [...] to value 0. 
Error 'Protocol family not supported' -> Function configure_current_thread_affinity
```

**これは完全に無視して問題ありません。**

### 原因

ROS 2が使用するDDS（CycloneDDS）が、スレッドアフィニティ（CPUコアへのスレッド割り当て）を設定しようとしていますが、macOSではLinuxと同じAPIがサポートされていないため発生します。

### 影響

- ✅ ROS 2の全機能は正常に動作します
- ✅ パフォーマンスへの影響はほとんどありません
- ❌ 設定ファイルで完全に消すことはできません（ログレベルを変更すると重要なエラーも消えてしまうため）

## カスタムノードの開発

ここからは、自分でROS 2ノードを開発する方法を解説します。

### 開発ツールのインストール

カスタムパッケージをビルドするために、追加のツールが必要です：

```bash
# Pythonパッケージのビルド用
pixi add colcon-common-extensions

# C++パッケージのビルド用（C++開発をする場合）
pixi add ros-jazzy-ament-cmake-auto compilers pkg-config cmake ninja
```

### pixi.tomlの設定

`pixi.toml`にビルド設定を追加します：

```toml
[activation]
scripts = ["install/setup.sh"]

[tasks]
build = { cmd = "colcon build --symlink-install", inputs = ["src"] }
```

- `activation.scripts`: ビルド成果物のセットアップスクリプトを自動で読み込む
- `tasks.build`: `pixi run build`でビルドを実行できるようにする

## Python版 Publisher/Subscriber

### パッケージの作成

```bash
pixi run ros2 pkg create --build-type ament_python \
  --destination-directory src \
  --node-name talker \
  pub_sub_python
```

これにより、以下の構造が作成されます：

```
src/pub_sub_python/
├── package.xml           # パッケージのメタデータ
├── setup.py              # Pythonパッケージの設定
├── setup.cfg
├── resource/
├── test/
└── pub_sub_python/
    ├── __init__.py
    └── talker.py         # ノードの実装
```

### Publisherノードの実装

`src/pub_sub_python/pub_sub_python/talker.py`を以下の内容に書き換えます：

```python
"""Publisher node for pub_sub_python package."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """A minimal ROS 2 publisher node that publishes String messages."""

    def __init__(self):
        """Initialize the publisher node."""
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Publisher node has been started')

    def timer_callback(self):
        """Publish a message at regular intervals."""
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    """Entry point for the publisher node."""
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        minimal_publisher.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        minimal_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### コードの解説

1. **`rclpy`**: ROS 2のPythonクライアントライブラリ
2. **`Node`クラスの継承**: すべてのROS 2ノードは`Node`クラスを継承
3. **`create_publisher()`**: トピックにメッセージを配信するpublisherを作成
   - 第1引数: メッセージ型（`String`）
   - 第2引数: トピック名（`'topic'`）
   - 第3引数: QoS（Quality of Service）のキューサイズ
4. **`create_timer()`**: 定期的にコールバックを呼び出すタイマーを作成
5. **`rclpy.spin()`**: ノードを実行し、コールバックを処理

### Subscriberノードの実装

`src/pub_sub_python/pub_sub_python/listener.py`を作成します：

```python
"""Subscriber node for pub_sub_python package."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """A minimal ROS 2 subscriber node that subscribes to String messages."""

    def __init__(self):
        """Initialize the subscriber node."""
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Subscriber node has been started')

    def listener_callback(self, msg):
        """Process received messages."""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """Entry point for the subscriber node."""
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        minimal_subscriber.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        minimal_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### パッケージ設定の更新

#### package.xml

依存関係を追加します：

```xml
<?xml version="1.0"?>
<package format="3">
  <name>pub_sub_python</name>
  <version>0.0.0</version>
  <description>A simple ROS 2 pub/sub example in Python</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### setup.py

エントリーポイントにlistenerを追加します：

```python
entry_points={
    'console_scripts': [
        'talker = pub_sub_python.talker:main',
        'listener = pub_sub_python.listener:main',
    ],
},
```

### ビルドと実行

```bash
# colconビルドツールを使ってパッケージをビルド
pixi run colcon build

# ターミナル1: Publisher起動
pixi run ros2 run pub_sub_python talker

# ターミナル2: Subscriber起動
pixi run ros2 run pub_sub_python listener
```

> **重要**: `pixi.toml`の`[activation]`セクションに`scripts = ["install/setup.sh"]`が設定されているため、`pixi run`を実行するたびにビルド成果物が自動的に読み込まれます。手動で`source`コマンドを実行する必要はありません。

Publisherが0.5秒ごとに「Hello World: N」を配信し、Subscriberがそれを受信してログに出力します。

> **Note**: ROS 2では `colcon` がメタビルドツールとして機能し、各パッケージのビルドタイプ（`ament_python`、`ament_cmake`など）に応じて適切なビルドシステムを呼び出します。

## C++版 Publisher/Subscriber

### パッケージの作成

```bash
pixi run ros2 pkg create --build-type ament_cmake \
  --destination-directory src \
  --node-name talker \
  pub_sub_cpp
```

### Publisherノードの実装

`src/pub_sub_cpp/src/talker.cpp`を以下の内容に書き換えます：

```cpp
/**
 * @file talker.cpp
 * @brief A minimal ROS 2 publisher node that publishes String messages.
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
    RCLCPP_INFO(this->get_logger(), "Publisher node has been started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

### Subscriberノードの実装

`src/pub_sub_cpp/src/listener.cpp`を作成します：

```cpp
/**
 * @file listener.cpp
 * @brief A minimal ROS 2 subscriber node that subscribes to String messages.
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    auto topic_callback =
      [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
    subscription_ =
      this->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback);
    RCLCPP_INFO(this->get_logger(), "Subscriber node has been started");
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

### CMakeLists.txtの更新

```cmake
cmake_minimum_required(VERSION 3.8)
project(pub_sub_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Publisher executable
add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# Subscriber executable
add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

### package.xmlの更新

```xml
<?xml version="1.0"?>
<package format="3">
  <name>pub_sub_cpp</name>
  <version>0.0.0</version>
  <description>A simple ROS 2 pub/sub example in C++</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### ビルドと実行

```bash
# Ninjaで高速ビルド
pixi run colcon build --symlink-install --cmake-args -G Ninja

# または、pixi.tomlで定義したタスクを使用
pixi run build --cmake-args -G Ninja

# ターミナル1: C++ Publisher起動
pixi run ros2 run pub_sub_cpp talker

# ターミナル2: C++ Subscriber起動
pixi run ros2 run pub_sub_cpp listener
```

> **Tips**: Python版とC++版は同じ`topic`トピックを使用しているため、Python talker → C++ listenerのような異言語間通信も可能です！

## pixi.tomlの完成形

最終的な`pixi.toml`は以下のようになります：

```toml
[workspace]
authors = ["Your Name <your@email.com>"]
channels = ["robostack-jazzy", "conda-forge"]
name = "my_ros2_project"
platforms = ["osx-arm64"]
version = "0.1.0"

[activation]
scripts = ["install/setup.sh"]

[tasks]
build = { cmd = "colcon build --symlink-install", inputs = ["src"] }
# Python pub/sub
talker = { cmd = "ros2 run pub_sub_python talker", depends-on = ["build"] }
listener = { cmd = "ros2 run pub_sub_python listener", depends-on = ["build"] }
# C++ pub/sub
talker_cpp = { cmd = "ros2 run pub_sub_cpp talker", depends-on = ["build"] }
listener_cpp = { cmd = "ros2 run pub_sub_cpp listener", depends-on = ["build"] }

[dependencies]
ros-jazzy-ros2run = ">=0.32.6,<0.33"
ros-jazzy-desktop = ">=0.11.0,<0.12"
ros-jazzy-turtlesim = ">=1.8.3,<2"
colcon-common-extensions = ">=0.3.0,<0.4"
ros-jazzy-ament-cmake-auto = ">=2.5.4,<3"
compilers = ">=1.11.0,<2"
pkg-config = ">=0.29.2,<0.30"
cmake = ">=4.2.1,<5"
ninja = ">=1.13.2,<2"
```

### タスクの便利な使い方

```bash
# ビルドのみ
pixi run build

# ビルド＆実行（depends-onにより自動でビルドされる）
pixi run talker
pixi run listener

# ros2コマンドを直接実行
pixi run ros2 topic list
pixi run ros2 node list
```

## プロジェクト構造のまとめ

最終的なプロジェクト構造：

```
my_ros2_project/
├── pixi.toml                    # pixi設定ファイル
├── pixi.lock                    # ロックファイル（自動生成）
├── .pixi/                       # pixi環境（自動生成）
├── src/                         # ソースコード
│   ├── pub_sub_python/          # Pythonパッケージ
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── pub_sub_python/
│   │       ├── talker.py
│   │       └── listener.py
│   └── pub_sub_cpp/             # C++パッケージ
│       ├── package.xml
│       ├── CMakeLists.txt
│       └── src/
│           ├── talker.cpp
│           └── listener.cpp
├── build/                       # ビルド成果物（自動生成）
├── install/                     # インストール先（自動生成）
└── log/                         # ビルドログ（自動生成）
```

## トラブルシューティング

### RViz2を閉じたときのエラー

```
libc++abi: terminating due to uncaught exception of type std::__1::system_error: 
mutex lock failed: Invalid argument
```

**対処法**: これはmacOS上のRViz2の既知の問題で、次回起動に影響はありません。無視して問題ありません。

### "Stereo is NOT SUPPORTED" メッセージ

```
[INFO] [rviz2]: Stereo is NOT SUPPORTED
```

**対処法**: これはVRヘッドセット用のステレオ3D機能が使えないという情報メッセージです。通常の3D表示には影響しません。

### ビルドエラー: setuptools

古いチュートリアルでは`setuptools<=58.2.0`の制限がありますが、最新のRoboStackパッケージでは不要です。制限なしでインストールしてください。

## 次のステップ

本記事ではROS 2の基本的なPub/Sub通信を実装しました。さらに学習を進めるには：

1. **サービス通信**: リクエスト・レスポンス型の同期通信
2. **アクション**: 長時間処理の非同期実行
3. **Launch files**: 複数ノードの一括起動
4. **パラメータ**: ノードの設定を動的に変更
5. **tf2**: 座標変換の管理
6. **Nav2**: 自律移動ナビゲーション
7. **MoveIt2**: マニピュレータの経路計画

## 参考リンク

- [ROS 2 Documentation (Jazzy)](https://docs.ros.org/en/jazzy/)
- [pixi Documentation](https://pixi.sh/)
- [RoboStack](https://robostack.github.io/)
- [pixi ROS 2 Tutorial](https://pixi.sh/latest/tutorials/ros2/)

## まとめ

本記事では、pixiを使用してmacOS上にROS 2開発環境を構築し、PythonとC++でPub/Subノードを実装する方法を解説しました。

pixiの利点：
- ✅ **簡単セットアップ**: コマンド数行でROS 2環境が構築可能
- ✅ **再現性**: `pixi.toml`と`pixi.lock`でチーム間で同じ環境を共有
- ✅ **高速**: 依存関係解決とビルドが高速
- ✅ **ネイティブ実行**: Dockerなしでmacのネイティブ環境でROS 2を実行

macOSでのROS 2開発は、以前は困難でしたが、RoboStackとpixiの組み合わせにより、非常に手軽になりました。ぜひこの環境を活用して、ロボット開発を始めてみてください！
