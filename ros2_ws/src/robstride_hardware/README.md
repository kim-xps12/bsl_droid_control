# robstride_hardware

`robstride_hardware` は RobStride RS02 を `ros2_control` から扱うための Linux 専用パッケージです。現在の実装は「BSL-Droid 全身制御」ではなく、`joint1` という単関節の単体検証に焦点を当てています。

## 現在のスコープ

- SocketCAN ベースの RobStride ドライバ
- `ros2_control` SystemInterface
- 200 Hz の state reader thread
- 単関節 `joint1` を対象にした bringup / 正弦波デモ

## 入口文書

- `doc/README.md`

起動コマンドはプロジェクトルートの `README.md` を参照してください。
