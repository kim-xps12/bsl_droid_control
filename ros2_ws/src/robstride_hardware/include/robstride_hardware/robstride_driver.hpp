// Copyright (c) 2024-2025, Yutaro KIMURA (B-SKY Lab)
// SPDX-License-Identifier: MIT

/**
 * @file robstride_driver.hpp
 * @brief RobStrideモータ CAN通信ドライバ（ROS 2非依存）
 *
 * MITコントロールプロトコルを使用してRobStrideモータとの
 * 低レベルCAN通信を処理するドライバ。
 * ros2_controlとは独立して、テストやデバッグにも使用可能。
 */

#pragma once

#include <linux/can.h>
#include <cstdint>
#include <string>
#include <utility>

namespace robstride_driver {

/**
 * @brief モータ状態を保持する構造体
 *
 * モータから受信したフィードバック値を格納する。
 * validフラグがtrueの場合のみ値が有効。
 */
struct MotorState {
  double position = 0.0;  // 現在位置 [rad]
  double velocity = 0.0;  // 現在速度 [rad/s]
  double torque = 0.0;    // 現在トルク [Nm]
  bool valid = false;     // 受信データが有効かどうか
};

/**
 * @brief MIT制御コマンド構造体
 *
 * MIT制御モードで送信するコマンドパラメータを格納する。
 * トルク出力 = kp * (position - 現在位置) + kd * (velocity - 現在速度) + torque_ff
 */
struct MitCommand {
  double position = 0.0;   // 目標位置 [rad]
  double velocity = 0.0;   // 目標速度 [rad/s]
  double kp = 30.0;        // 位置ゲイン [Nm/rad]
  double kd = 1.0;         // ダンピングゲイン [Nm/(rad/s)]
  double torque_ff = 0.0;  // フィードフォワードトルク [Nm]
};

/**
 * @brief RobStrideモータの制御モード
 */
enum class ControlMode : int8_t {
  MIT = 0,       // MIT制御モード（位置・速度・トルクの複合制御）
  POSITION = 1,  // 位置制御モード
  VELOCITY = 2,  // 速度制御モード
  TORQUE = 3     // トルク制御モード
};

/**
 * @brief RobStrideモータドライバクラス
 *
 * RobStrideモータ（RS-01, RS-02, RS-03）とのCAN通信を管理する。
 * ROS 2に依存せず、単体テストやデバッグにも使用可能。
 */
class RobStrideDriver {
public:
  RobStrideDriver() = default;
  ~RobStrideDriver();

  // コピー禁止（ソケットリソースの二重解放を防止）
  RobStrideDriver(const RobStrideDriver&) = delete;
  RobStrideDriver& operator=(const RobStrideDriver&) = delete;

  /**
   * @brief CANインターフェースに接続する
   * @param interface CANインターフェース名（例: "can0"）
   * @return 成功時true
   */
  bool connect(const std::string& interface);

  /**
   * @brief CANインターフェースから切断する
   */
  void disconnect();

  /**
   * @brief CANインターフェースに接続済みか確認する
   */
  bool is_connected() const { return can_socket_ >= 0; }

  /**
   * @brief モータを有効化する
   * @param motor_id モータのCAN ID
   * @return 成功時true
   */
  bool enable(int motor_id);

  /**
   * @brief モータを無効化する
   * @param motor_id モータのCAN ID
   * @return 成功時true
   */
  bool disable(int motor_id);

  /**
   * @brief 制御モードを設定する
   * @param motor_id モータのCAN ID
   * @param mode 制御モード
   * @return 成功時true
   */
  bool set_mode(int motor_id, ControlMode mode);

  /**
   * @brief 速度リミットを設定する
   * @param motor_id モータのCAN ID
   * @param limit 速度リミット [rad/s]
   * @return 成功時true
   */
  bool set_velocity_limit(int motor_id, float limit);

  /**
   * @brief トルクリミットを設定する
   * @param motor_id モータのCAN ID
   * @param limit トルクリミット [Nm]
   * @return 成功時true
   */
  bool set_torque_limit(int motor_id, float limit);

  /**
   * @brief MIT制御コマンドを送信する
   * @param motor_id モータのCAN ID
   * @param cmd MIT制御コマンド構造体
   * @return 成功時true
   */
  bool send_command(int motor_id, const MitCommand& cmd);

  /**
   * @brief モータ状態を読み取る
   * @param motor_id モータのCAN ID（フィルタリングに使用）
   * @return モータ状態（.validフラグで有効性を確認すること）
   */
  MotorState read_state(int motor_id);

  /**
   * @brief CANバスからOPERATION_STATUSレスポンスを1件読み取る
   *
   * poll()ベースのタイムアウトでフレームを読み取る。
   * ステータス以外のフレームはスキップする。
   * ハードウェアインターフェースの同期送受信パターンで使用される。
   *
   * @param timeout_ms poll()のタイムアウト [ms]
   * @return (motor_id, MotorState)のペア。タイムアウト時はmotor_id = -1
   */
  std::pair<int, MotorState> read_one_response(int timeout_ms = 2);

  /**
   * @brief モータの自動レポートを無効化する（通信タイプ24, F_CMD=0x00）
   *
   * モータからの定期的な非要求フィードバックフレームを明示的に停止する。
   * @param motor_id モータのCAN ID
   * @return 送信成功時true
   */
  bool disable_auto_report(int motor_id);

  /**
   * @brief 受信バッファをドレインする（古いフレームを破棄）
   */
  void drain_rx_buffer();

  /**
   * @brief ソケットのファイルディスクリプタを取得する
   * @return ソケットfd（未接続時は-1）
   */
  int socket_fd() const { return can_socket_; }

  /**
   * @brief CAN通信用のホストIDを設定する
   * @param host_id ホストID（デフォルト: 0xFF）
   */
  void set_host_id(int host_id) { host_id_ = host_id; }

private:
  int can_socket_ = -1;  // CANソケットのファイルディスクリプタ
  int host_id_ = 0xFF;   // CAN通信におけるホスト側ID

  // CAN通信ヘルパー関数
  bool send_frame(uint32_t can_id, const uint8_t* data, uint8_t dlc);
  bool read_frame(struct can_frame* frame, int timeout_ms = 10);

  // プロトコルヘルパー関数（パラメータ書き込み）
  bool write_parameter(int motor_id, uint16_t param_id, float value);
  bool write_parameter(int motor_id, uint16_t param_id, int8_t value);
};

// ============================================================================
// RobStride RS-02モータのスケールファクタ
// 仕様: 最大速度 = 44 rad/s, 最大トルク = 17 Nm
// MIT制御フレームの物理値⇔整数値変換に使用する
// ============================================================================
namespace scale {
constexpr double POSITION = 4.0 * 3.14159265358979323846;  // 位置範囲 ±4π [rad]
constexpr double VELOCITY = 44.0;                          // 速度範囲 ±44 [rad/s]（RS-02仕様）
constexpr double TORQUE = 17.0;                            // トルク範囲 ±17 [Nm]（RS-02仕様）
constexpr double KP = 500.0;  // 位置ゲイン範囲 0〜500 [Nm/rad]（RS-02仕様）
constexpr double KD = 5.0;    // ダンピングゲイン範囲 0〜5 [Nm/(rad/s)]（RS-02仕様）
}  // namespace scale

// ============================================================================
// CANプロトコル定数
// RobStride独自のCAN拡張フレームプロトコルで使用する定数群
// ============================================================================
namespace protocol {
// 通信タイプ（CAN拡張IDのビット[28:24]に格納）
namespace comm_type {
constexpr uint32_t GET_DEVICE_ID = 0;         // デバイスID取得
constexpr uint32_t OPERATION_CONTROL = 1;     // MIT制御コマンド送信
constexpr uint32_t OPERATION_STATUS = 2;      // モータ状態フィードバック（応答）
constexpr uint32_t ENABLE = 3;                // モータ有効化
constexpr uint32_t DISABLE = 4;               // モータ無効化
constexpr uint32_t SET_ZERO = 6;              // ゼロ位置設定
constexpr uint32_t WRITE_PARAMETER = 18;      // パラメータ書き込み
constexpr uint32_t DISABLE_AUTO_REPORT = 24;  // 自動レポート無効化
}  // namespace comm_type

// パラメータID（WRITE_PARAMETERコマンドで使用）
namespace param_id {
constexpr uint16_t MODE = 0x7005;            // 制御モード
constexpr uint16_t VELOCITY_LIMIT = 0x7017;  // 速度リミット
constexpr uint16_t TORQUE_LIMIT = 0x700B;    // トルクリミット
}  // namespace param_id
}  // namespace protocol

}  // namespace robstride_driver
