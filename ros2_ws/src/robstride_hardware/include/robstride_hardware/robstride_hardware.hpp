/**
 * @file robstride_hardware.hpp
 * @brief ros2_control Hardware Interface for RobStride motors
 *
 * Synchronous send/receive pattern: all CAN I/O happens in write(),
 * read() just copies from internal buffers (RT-safe).
 * Supports multiple motors across multiple CAN buses.
 */

#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>
#include <string>
#include <unordered_map>
#include <chrono>

#include "robstride_hardware/robstride_driver.hpp"

namespace robstride_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// Per-joint configuration (populated from URDF in on_init())
struct JointConfig {
  std::string name;
  std::string can_interface;
  int motor_id = 0;
  double kp = 30.0;
  double kd = 1.0;
};

/// Per-CAN-bus context
struct BusContext {
  robstride_driver::RobStrideDriver driver;
  std::vector<size_t> joint_indices;  ///< Indices into joints_ vector
};

/// Per-bus timing breakdown (for diagnostics)
struct BusTimingStats {
  double send_us = 0.0;
  double receive_us = 0.0;
  int received = 0;
  int expected = 0;
};

/// write() timing statistics (for diagnostics)
struct WriteTimingStats {
  double send_us = 0.0;       ///< Phase1: total send time [us]
  double receive_us = 0.0;    ///< Phase2: total receive time [us]
  double total_us = 0.0;      ///< write() total time [us]
  int responses_received = 0;
  int responses_expected = 0;
};

class RobStrideHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Joint configuration (from URDF)
  std::vector<JointConfig> joints_;

  // CAN bus contexts (key = interface name, e.g. "can1")
  std::unordered_map<std::string, BusContext> buses_;

  // State storage (updated in write(), read by controllers via StateInterface pointers)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Command storage (written by controllers via CommandInterface pointers)
  std::vector<double> hw_commands_position_;

  // Per-cycle response tracking (pre-allocated, no RT allocation)
  std::vector<bool> response_received_;

  // Missed response counter per joint (for diagnostics)
  std::vector<int> missed_response_count_;
  static constexpr int kMissedResponseWarnThreshold = 10;  // 50ms @ 200Hz

  // === Timing diagnostics ===
  WriteTimingStats last_timing_;
  std::unordered_map<std::string, BusTimingStats> bus_timing_;

  // Aggregate statistics (logged every kTimingLogInterval cycles)
  int timing_log_counter_ = 0;
  static constexpr int kTimingLogInterval = 200;  // 1 second @ 200Hz
  double total_us_min_ = 1e9;
  double total_us_max_ = 0.0;
  double total_us_sum_ = 0.0;
  double total_us_sum_sq_ = 0.0;  // for stddev
  double send_us_min_ = 1e9;
  double send_us_max_ = 0.0;
  double send_us_sum_ = 0.0;
  double recv_us_min_ = 1e9;
  double recv_us_max_ = 0.0;
  double recv_us_sum_ = 0.0;
  int total_missed_sum_ = 0;
};

}  // namespace robstride_hardware
