// Copyright (c) 2024-2025, Yutaro KIMURA (B-SKY Lab)
// SPDX-License-Identifier: MIT

/**
 * @file robstride_hardware.hpp
 * @brief RobStrideモータ用 ros2_control ハードウェアインターフェース
 *
 * 同期送受信パターン: 全CAN I/Oはwrite()内で実行され、
 * read()は内部バッファからのコピーのみ（リアルタイム安全）。
 * 複数CANバスにまたがる複数モータをサポートする。
 */

#pragma once

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "robstride_hardware/robstride_driver.hpp"

namespace robstride_hardware {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// ジョイント毎の設定（on_init()でURDFから読み込み）
struct JointConfig {
  std::string name;           // ジョイント名
  std::string can_interface;  // CANインターフェース名（例: "can1"）
  int motor_id = 0;           // モータのCAN ID
  double kp = 30.0;           // 位置ゲイン [Nm/rad]
  double kd = 1.0;            // ダンピングゲイン [Nm/(rad/s)]
};

/// CANバス毎のコンテキスト
struct BusContext {
  robstride_driver::RobStrideDriver driver;  // CANドライバインスタンス
  std::vector<size_t> joint_indices;         ///< joints_ベクタへのインデックス
};

/// バス毎のタイミング内訳（診断用）
struct BusTimingStats {
  double send_us = 0.0;     // 送信にかかった時間 [us]
  double receive_us = 0.0;  // 受信にかかった時間 [us]
  int received = 0;         // 受信できたレスポンス数
  int expected = 0;         // 期待されるレスポンス数
};

/// write()のタイミング統計（診断用）
struct WriteTimingStats {
  double send_us = 0.0;        ///< フェーズ1: 全送信時間 [us]
  double receive_us = 0.0;     ///< フェーズ2: 全受信時間 [us]
  double total_us = 0.0;       ///< write()の合計時間 [us]
  int responses_received = 0;  // 受信したレスポンス数
  int responses_expected = 0;  // 期待されるレスポンス数
};

/**
 * @brief RobStrideモータ用 ros2_control ハードウェアインターフェースクラス
 *
 * Controller Managerから200Hzで呼び出されるread()/write()を実装する。
 * write()内で全モータへのコマンド送信とレスポンス受信を同期的に行い、
 * read()ではwrite()で更新済みの内部バッファを返すだけ（リアルタイム安全）。
 */
class RobStrideHardware : public hardware_interface::SystemInterface {
public:
  // ライフサイクルコールバック
  CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  // 状態・コマンドインターフェースのエクスポート
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // リアルタイム制御ループ（Controller Managerから200Hzで呼び出される）
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

private:
  // ジョイント設定（URDFから読み込み）
  std::vector<JointConfig> joints_;

  // CANバスコンテキスト（キー = インターフェース名、例: "can1"）
  std::unordered_map<std::string, BusContext> buses_;

  // 状態ストレージ（write()で更新、StateInterfaceポインタ経由でコントローラが読み取る）
  std::vector<double> hw_positions_;   // 現在位置 [rad]
  std::vector<double> hw_velocities_;  // 現在速度 [rad/s]
  std::vector<double> hw_efforts_;     // 現在トルク [Nm]

  // コマンドストレージ（コントローラがCommandInterfaceポインタ経由で書き込む）
  std::vector<double> hw_commands_position_;  // 目標位置 [rad]

  // サイクル毎のレスポンス追跡（事前確保済み、RT中のメモリ割り当てなし）
  std::vector<bool> response_received_;

  // ジョイント毎のレスポンス欠落カウンタ（診断用）
  std::vector<int> missed_response_count_;
  static constexpr int kMissedResponseWarnThreshold = 10;  // 200Hzで50ms相当

  // === タイミング診断 ===
  WriteTimingStats last_timing_;                                // 直近サイクルのタイミング
  std::unordered_map<std::string, BusTimingStats> bus_timing_;  // バス毎のタイミング

  // 集約統計（kTimingLogIntervalサイクル毎にログ出力）
  int timing_log_counter_ = 0;
  static constexpr int kTimingLogInterval = 200;  // 200Hzで1秒相当
  double total_us_min_ = 1e9;                     // write()合計時間の最小値
  double total_us_max_ = 0.0;                     // write()合計時間の最大値
  double total_us_sum_ = 0.0;                     // write()合計時間の累積和
  double total_us_sum_sq_ = 0.0;                  // 標準偏差計算用の二乗和
  double send_us_min_ = 1e9;                      // 送信時間の最小値
  double send_us_max_ = 0.0;                      // 送信時間の最大値
  double send_us_sum_ = 0.0;                      // 送信時間の累積和
  double recv_us_min_ = 1e9;                      // 受信時間の最小値
  double recv_us_max_ = 0.0;                      // 受信時間の最大値
  double recv_us_sum_ = 0.0;                      // 受信時間の累積和
  int total_missed_sum_ = 0;                      // レスポンス欠落の累積数
};

}  // namespace robstride_hardware
