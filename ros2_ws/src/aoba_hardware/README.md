# robstride_hardware

RobStride RS02 モータを `ros2_control` から駆動するための Linux 専用パッケージ。BSL-Droid の全 10 関節（左右各 5 軸）をデュアル CAN バス（can1/can2）で制御する。

## 特徴

- 同期送受信パターン: `write()` 内で全モータへのコマンド送信と応答受信を完結（別スレッド不要）
- 多モータ・多バス対応: URDF の関節ごとに CAN バス・モータ ID・ゲインを設定
- タイミング診断: 1 秒ごとに送受信所要時間・未応答数をログ出力

## 設計ドキュメント

- 詳細設計: `doc/design/ros2_walking/modules/robstride_hardware.md`
- クイックリファレンス（CAN マッピング等）: `doc/README.md`

起動コマンドはプロジェクトルートの `README.md` を参照してください。
