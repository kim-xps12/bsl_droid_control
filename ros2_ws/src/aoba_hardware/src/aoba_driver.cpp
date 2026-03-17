// Copyright (c) 2024-2025, Yutaro KIMURA (B-SKY Lab)
// SPDX-License-Identifier: MIT

/**
 * @file aoba_driver.cpp
 * @brief RobStrideモータ CAN通信ドライバの実装
 */

#include "aoba_hardware/aoba_driver.hpp"

#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace aoba_driver {

// ============================================================================
// ユーティリティ関数（バイトパッキング・アンパッキング）
// ============================================================================

namespace {

/// 16ビット値をリトルエンディアンでバッファに格納する
inline void pack_u16_le(uint8_t* buf, uint16_t val) {
  buf[0] = val & 0xFF;
  buf[1] = (val >> 8) & 0xFF;
}

/// 16ビット値をビッグエンディアンでバッファに格納する
inline void pack_u16_be(uint8_t* buf, uint16_t val) {
  buf[0] = (val >> 8) & 0xFF;
  buf[1] = val & 0xFF;
}

/// float値をリトルエンディアンでバッファに格納する
inline void pack_float_le(uint8_t* buf, float val) {
  std::memcpy(buf, &val, sizeof(float));
}

/// バッファからビッグエンディアンの16ビット値を読み取る
inline uint16_t unpack_u16_be(const uint8_t* buf) {
  return (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
}

}  // anonymous namespace

// ============================================================================
// AobaDriverの実装
// ============================================================================

AobaDriver::~AobaDriver() {
  disconnect();
}

bool AobaDriver::connect(const std::string& interface) {
  // 既存の接続があれば切断する
  if (can_socket_ >= 0) {
    disconnect();
  }

  // CANソケットを作成する
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    return false;
  }

  // CANインターフェース名からインデックスを取得する
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(can_socket_);
    can_socket_ = -1;
    return false;
  }

  // ソケットをCANインターフェースにバインドする
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(can_socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(can_socket_);
    can_socket_ = -1;
    return false;
  }

  // ソケットはブロッキングモード。タイムアウトはread_frame()内のpoll()で処理する。
  return true;
}

void AobaDriver::disconnect() {
  if (can_socket_ >= 0) {
    ::close(can_socket_);
    can_socket_ = -1;
  }
}

/// CANフレームを送信する
bool AobaDriver::send_frame(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
  if (can_socket_ < 0) {
    return false;
  }

  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id | CAN_EFF_FLAG;  // 拡張フレーム（29ビットID）
  frame.can_dlc = dlc;

  if (data && dlc > 0) {
    std::memcpy(frame.data, data, std::min(dlc, static_cast<uint8_t>(8)));
  }

  ssize_t nbytes = ::write(can_socket_, &frame, sizeof(frame));
  return nbytes == sizeof(frame);
}

/// poll()ベースのタイムアウト付きでCANフレームを1件受信する
bool AobaDriver::read_frame(struct can_frame* frame, int timeout_ms) {
  if (can_socket_ < 0 || !frame) {
    return false;
  }

  // poll()でデータ到着を待機する
  struct pollfd pfd;
  pfd.fd = can_socket_;
  pfd.events = POLLIN;
  int ret = ::poll(&pfd, 1, timeout_ms);
  if (ret <= 0) {
    return false;  // タイムアウトまたはエラー
  }

  ssize_t nbytes = ::read(can_socket_, frame, sizeof(*frame));
  return nbytes == sizeof(*frame);
}

/// モータを有効化する（拡張IDにENABLEコマンドを格納して送信）
bool AobaDriver::enable(int motor_id) {
  // 拡張ID構成: [28:24]=通信タイプ, [15:8]=ホストID, [7:0]=モータID
  uint32_t ext_id = (protocol::comm_type::ENABLE << 24) | (static_cast<uint32_t>(host_id_) << 8) |
                    static_cast<uint32_t>(motor_id);
  return send_frame(ext_id, nullptr, 0);
}

/// モータを無効化する（拡張IDにDISABLEコマンドを格納して送信）
bool AobaDriver::disable(int motor_id) {
  uint32_t ext_id = (protocol::comm_type::DISABLE << 24) | (static_cast<uint32_t>(host_id_) << 8) |
                    static_cast<uint32_t>(motor_id);
  return send_frame(ext_id, nullptr, 0);
}

bool AobaDriver::set_mode(int motor_id, ControlMode mode) {
  return write_parameter(motor_id, protocol::param_id::MODE, static_cast<int8_t>(mode));
}

bool AobaDriver::set_velocity_limit(int motor_id, float limit) {
  return write_parameter(motor_id, protocol::param_id::VELOCITY_LIMIT, limit);
}

bool AobaDriver::set_torque_limit(int motor_id, float limit) {
  return write_parameter(motor_id, protocol::param_id::TORQUE_LIMIT, limit);
}

/// パラメータ書き込み（float値）: データ部にパラメータIDと値を格納して送信
bool AobaDriver::write_parameter(int motor_id, uint16_t param_id, float value) {
  uint32_t ext_id = (protocol::comm_type::WRITE_PARAMETER << 24) |
                    (static_cast<uint32_t>(host_id_) << 8) | static_cast<uint32_t>(motor_id);

  uint8_t data[8] = {0};
  pack_u16_le(&data[0], param_id);  // データ[0:1] = パラメータID（リトルエンディアン）
  pack_float_le(&data[4], value);   // データ[4:7] = 値（リトルエンディアン float）

  return send_frame(ext_id, data, 8);
}

/// パラメータ書き込み（int8_t値）: 制御モード設定等に使用
bool AobaDriver::write_parameter(int motor_id, uint16_t param_id, int8_t value) {
  uint32_t ext_id = (protocol::comm_type::WRITE_PARAMETER << 24) |
                    (static_cast<uint32_t>(host_id_) << 8) | static_cast<uint32_t>(motor_id);

  uint8_t data[8] = {0};
  pack_u16_le(&data[0], param_id);        // データ[0:1] = パラメータID
  data[4] = static_cast<uint8_t>(value);  // データ[4] = 値（1バイト）

  return send_frame(ext_id, data, 8);
}

/// MIT制御コマンドを送信する
bool AobaDriver::send_command(int motor_id, const MitCommand& cmd) {
  // 各値をスケールファクタの有効範囲にクランプする
  double pos_clamped = std::clamp(cmd.position, -scale::POSITION, scale::POSITION);
  double vel_clamped = std::clamp(cmd.velocity, -scale::VELOCITY, scale::VELOCITY);
  double kp_clamped = std::clamp(cmd.kp, 0.0, scale::KP);
  double kd_clamped = std::clamp(cmd.kd, 0.0, scale::KD);
  double tq_clamped = std::clamp(cmd.torque_ff, -scale::TORQUE, scale::TORQUE);

  // 物理値を16ビット整数に変換する（MITモードではビッグエンディアン）
  // 位置・速度・トルクは [-1, 1] → [0, 0xFFFE] にマッピング
  // kp・kdは [0, 1] → [0, 0xFFFF] にマッピング
  uint16_t pos_u16 = static_cast<uint16_t>(((pos_clamped / scale::POSITION) + 1.0) * 0x7FFF);
  uint16_t vel_u16 = static_cast<uint16_t>(((vel_clamped / scale::VELOCITY) + 1.0) * 0x7FFF);
  uint16_t kp_u16 = static_cast<uint16_t>((kp_clamped / scale::KP) * 0xFFFF);
  uint16_t kd_u16 = static_cast<uint16_t>((kd_clamped / scale::KD) * 0xFFFF);
  uint16_t torque_u16 = static_cast<uint16_t>(((tq_clamped / scale::TORQUE) + 1.0) * 0x7FFF);

  // データ部にパック（ビッグエンディアン）: [位置, 速度, kp, kd]
  uint8_t data[8];
  pack_u16_be(&data[0], pos_u16);
  pack_u16_be(&data[2], vel_u16);
  pack_u16_be(&data[4], kp_u16);
  pack_u16_be(&data[6], kd_u16);

  // 拡張IDにトルクFFを埋め込む: [28:24]=OPERATION_CONTROL, [23:8]=torque, [7:0]=motor_id
  uint32_t ext_id = (protocol::comm_type::OPERATION_CONTROL << 24) |
                    (static_cast<uint32_t>(torque_u16) << 8) | static_cast<uint32_t>(motor_id);

  return send_frame(ext_id, data, 8);
}

/// モータ状態を読み取る（バッファ内の全フレームを読み、最新のステータスを返す）
MotorState AobaDriver::read_state(int motor_id) {
  MotorState state;
  struct can_frame frame;

  // バッファ内の全フレームを読み取り、最新のステータスを保持する
  // （モータはコマンド受信毎にステータスを返す）
  for (int attempts = 0; attempts < 20; ++attempts) {
    if (!read_frame(&frame)) {
      break;
    }

    // 拡張フレームのみ処理する
    if (!(frame.can_id & CAN_EFF_FLAG)) {
      continue;
    }

    uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
    uint32_t comm_type = (raw_id >> 24) & 0x1F;
    // RobStrideレスポンスフレームのID構成:
    // ビット[28:24] = 通信タイプ（5ビット）
    // ビット[23:16] = ステータスフラグ / エラーコード
    // ビット[15:8]  = モータID
    // ビット[7:0]   = ホストID
    uint32_t recv_motor_id = (raw_id >> 8) & 0xFF;

    // 対象モータからのOPERATION_STATUSか確認する
    if (comm_type == protocol::comm_type::OPERATION_STATUS &&
        recv_motor_id == static_cast<uint32_t>(motor_id)) {
      // MITステータスフレームをデコードする（ビッグエンディアン）
      uint16_t pos_u16 = unpack_u16_be(&frame.data[0]);
      uint16_t vel_u16 = unpack_u16_be(&frame.data[2]);
      uint16_t torque_u16 = unpack_u16_be(&frame.data[4]);

      // 整数値を物理単位に変換する
      state.position = ((static_cast<double>(pos_u16) / 0x7FFF) - 1.0) * scale::POSITION;
      state.velocity = ((static_cast<double>(vel_u16) / 0x7FFF) - 1.0) * scale::VELOCITY;
      state.torque = ((static_cast<double>(torque_u16) / 0x7FFF) - 1.0) * scale::TORQUE;
      state.valid = true;

      return state;
    }
  }

  return state;  // valid = false（レスポンスなし）
}

/// OPERATION_STATUSレスポンスを1件読み取る
/// ステータス以外のフレームはスキップし、残りのバッファをドレインする
std::pair<int, MotorState> AobaDriver::read_one_response(int timeout_ms) {
  struct can_frame frame;

  while (read_frame(&frame, timeout_ms)) {
    // 標準フレームはスキップ（拡張フレームのみ使用）
    if (!(frame.can_id & CAN_EFF_FLAG)) {
      timeout_ms = 0;  // 残りのバッファのみドレインする
      continue;
    }

    uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
    uint32_t comm_type = (raw_id >> 24) & 0x1F;
    int recv_motor_id = static_cast<int>((raw_id >> 8) & 0xFF);

    if (comm_type == protocol::comm_type::OPERATION_STATUS) {
      // ステータスフレームをデコードして返す
      MotorState state;
      uint16_t pos_u16 = unpack_u16_be(&frame.data[0]);
      uint16_t vel_u16 = unpack_u16_be(&frame.data[2]);
      uint16_t torque_u16 = unpack_u16_be(&frame.data[4]);

      state.position = ((static_cast<double>(pos_u16) / 0x7FFF) - 1.0) * scale::POSITION;
      state.velocity = ((static_cast<double>(vel_u16) / 0x7FFF) - 1.0) * scale::VELOCITY;
      state.torque = ((static_cast<double>(torque_u16) / 0x7FFF) - 1.0) * scale::TORQUE;
      state.valid = true;

      return {recv_motor_id, state};
    }

    // OPERATION_STATUS以外のフレーム: スキップして残りのバッファをドレインする
    timeout_ms = 0;
  }

  return {-1, MotorState{}};  // タイムアウト
}

/// モータの自動レポート（定期的な非要求フィードバック）を無効化する
bool AobaDriver::disable_auto_report(int motor_id) {
  uint32_t ext_id = (protocol::comm_type::DISABLE_AUTO_REPORT << 24) |
                    (static_cast<uint32_t>(host_id_) << 8) | static_cast<uint32_t>(motor_id);

  // F_CMD=0x00: 自動レポート無効化
  uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00};
  return send_frame(ext_id, data, 7);
}

/// 受信バッファ内の全フレームを読み捨てる（古いデータのクリア用）
void AobaDriver::drain_rx_buffer() {
  struct can_frame frame;
  while (read_frame(&frame, /*timeout_ms=*/0)) {
    // 破棄
  }
}

}  // namespace aoba_driver
