/**
 * @file robstride_hardware.cpp
 * @brief ros2_control Hardware Interface implementation for RobStride motors
 *
 * Synchronous send/receive: all CAN I/O in write(), read() is a no-op.
 * Supports multiple motors across multiple CAN buses (e.g. can1, can2).
 */

#include "robstride_hardware/robstride_hardware.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <chrono>
#include <algorithm>

namespace robstride_hardware
{

// ============================================================================
// Lifecycle callbacks
// ============================================================================

CallbackReturn RobStrideHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const size_t n = info_.joints.size();
  joints_.resize(n);
  hw_positions_.resize(n, 0.0);
  hw_velocities_.resize(n, 0.0);
  hw_efforts_.resize(n, 0.0);
  hw_commands_position_.resize(n, 0.0);
  response_received_.resize(n, false);
  missed_response_count_.resize(n, 0);

  for (size_t i = 0; i < n; ++i) {
    auto & jcfg = joints_[i];
    const auto & params = info_.joints[i].parameters;

    jcfg.name = info_.joints[i].name;
    jcfg.can_interface = params.count("can_interface")
      ? params.at("can_interface") : "can0";
    jcfg.motor_id = params.count("motor_id")
      ? std::stoi(params.at("motor_id")) : 11;
    jcfg.kp = params.count("kp")
      ? std::stod(params.at("kp")) : 30.0;
    jcfg.kd = params.count("kd")
      ? std::stod(params.at("kd")) : 1.0;

    // Group joints by CAN bus
    buses_[jcfg.can_interface].joint_indices.push_back(i);

    RCLCPP_INFO(
      rclcpp::get_logger("RobStrideHardware"),
      "Joint[%zu]: name=%s, bus=%s, motor_id=%d, kp=%.1f, kd=%.2f",
      i, jcfg.name.c_str(), jcfg.can_interface.c_str(),
      jcfg.motor_id, jcfg.kp, jcfg.kd);
  }

  // Pre-allocate bus timing map
  for (const auto & [bus_name, _] : buses_) {
    bus_timing_[bus_name] = BusTimingStats{};
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RobStrideHardware"),
    "Initialized: %zu joints on %zu CAN bus(es)",
    n, buses_.size());

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobStrideHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"), "Configuring...");

  for (auto & [bus_name, bus] : buses_) {
    if (!bus.driver.connect(bus_name)) {
      RCLCPP_ERROR(rclcpp::get_logger("RobStrideHardware"),
        "Failed to connect to CAN interface: %s", bus_name.c_str());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"),
      "Connected to CAN interface: %s (%zu motors)",
      bus_name.c_str(), bus.joint_indices.size());
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobStrideHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"), "Activating...");

  // Enable each motor and set MIT mode
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto & jcfg = joints_[i];
    auto & driver = buses_.at(jcfg.can_interface).driver;

    if (!driver.enable(jcfg.motor_id)) {
      RCLCPP_ERROR(rclcpp::get_logger("RobStrideHardware"),
        "Failed to enable motor %d on %s",
        jcfg.motor_id, jcfg.can_interface.c_str());
      return CallbackReturn::ERROR;
    }

    if (!driver.set_mode(jcfg.motor_id, robstride_driver::ControlMode::MIT)) {
      RCLCPP_ERROR(rclcpp::get_logger("RobStrideHardware"),
        "Failed to set MIT mode for motor %d on %s",
        jcfg.motor_id, jcfg.can_interface.c_str());
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"),
      "Motor %d on %s: enabled, MIT mode set",
      jcfg.motor_id, jcfg.can_interface.c_str());
  }

  // Drain receive buffers (consume enable/set_mode responses)
  for (auto & [bus_name, bus] : buses_) {
    bus.driver.drain_rx_buffer();
  }

  // Initialize commands to current positions
  for (size_t i = 0; i < hw_commands_position_.size(); ++i) {
    hw_commands_position_[i] = hw_positions_[i];
  }

  // Reset timing statistics
  timing_log_counter_ = 0;
  total_us_max_ = 0.0;
  total_us_sum_ = 0.0;
  total_missed_sum_ = 0;

  RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"),
    "Activated: %zu motors on %zu bus(es), synchronous send/receive mode",
    joints_.size(), buses_.size());

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobStrideHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"), "Deactivating...");

  // Send zero-torque command (safe state) and disable each motor
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto & jcfg = joints_[i];
    auto & driver = buses_.at(jcfg.can_interface).driver;

    robstride_driver::MitCommand safe_cmd;
    safe_cmd.position = hw_positions_[i];
    safe_cmd.velocity = 0.0;
    safe_cmd.kp = 0.0;
    safe_cmd.kd = 0.0;
    safe_cmd.torque_ff = 0.0;
    driver.send_command(jcfg.motor_id, safe_cmd);
    driver.disable(jcfg.motor_id);

    RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"),
      "Motor %d on %s: deactivated", jcfg.motor_id, jcfg.can_interface.c_str());
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobStrideHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"), "Cleaning up...");
  for (auto & [_, bus] : buses_) {
    bus.driver.disconnect();
  }
  return CallbackReturn::SUCCESS;
}

// ============================================================================
// Interface export
// ============================================================================

std::vector<hardware_interface::StateInterface> RobStrideHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobStrideHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_commands_position_[i]));
  }
  return command_interfaces;
}

// ============================================================================
// RT control loop (called at 200Hz by Controller Manager)
// ============================================================================

hardware_interface::return_type RobStrideHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // hw_positions_ / hw_velocities_ / hw_efforts_ were already updated
  // by the previous write() call. Nothing to do here.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobStrideHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  using clock = std::chrono::steady_clock;
  const auto t_start = clock::now();

  // === Phase 1: Send commands to all buses (burst) ===
  for (auto & [bus_name, bus] : buses_) {
    const auto t_bus_send_start = clock::now();

    for (size_t ji : bus.joint_indices) {
      robstride_driver::MitCommand cmd;
      cmd.position = hw_commands_position_[ji];
      cmd.velocity = 0.0;
      cmd.kp = joints_[ji].kp;
      cmd.kd = joints_[ji].kd;
      cmd.torque_ff = 0.0;
      bus.driver.send_command(joints_[ji].motor_id, cmd);
    }

    bus_timing_[bus_name].send_us = std::chrono::duration<double, std::micro>(
      clock::now() - t_bus_send_start).count();
  }

  const auto t_send_done = clock::now();

  // === Phase 2: Read responses from all buses ===
  std::fill(response_received_.begin(), response_received_.end(), false);
  int total_received = 0;

  for (auto & [bus_name, bus] : buses_) {
    const auto t_bus_recv_start = clock::now();
    int remaining = static_cast<int>(bus.joint_indices.size());
    int bus_received = 0;
    int timeout_ms = 2;  // Full timeout for first read

    while (remaining > 0) {
      auto [motor_id, state] = bus.driver.read_one_response(timeout_ms);
      if (motor_id < 0) {
        break;  // Timeout
      }

      for (size_t ji : bus.joint_indices) {
        if (joints_[ji].motor_id == motor_id && !response_received_[ji]) {
          hw_positions_[ji] = state.position;
          hw_velocities_[ji] = state.velocity;
          hw_efforts_[ji] = state.torque;
          response_received_[ji] = true;
          missed_response_count_[ji] = 0;
          remaining--;
          bus_received++;
          break;
        }
      }

      timeout_ms = 1;  // Shorter timeout for subsequent reads
    }

    auto & bt = bus_timing_[bus_name];
    bt.receive_us = std::chrono::duration<double, std::micro>(
      clock::now() - t_bus_recv_start).count();
    bt.received = bus_received;
    bt.expected = static_cast<int>(bus.joint_indices.size());
    total_received += bus_received;
  }

  const auto t_recv_done = clock::now();

  // === Phase 3: Handle missed responses ===
  int total_missed = 0;
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (!response_received_[i]) {
      missed_response_count_[i]++;
      total_missed++;
      if (missed_response_count_[i] == kMissedResponseWarnThreshold) {
        RCLCPP_WARN(rclcpp::get_logger("RobStrideHardware"),
          "Motor %d on %s: %d consecutive missed responses",
          joints_[i].motor_id, joints_[i].can_interface.c_str(),
          missed_response_count_[i]);
      }
      // Keep last-known state (do not update)
    }
  }

  // === Phase 4: Timing diagnostics ===
  last_timing_.send_us = std::chrono::duration<double, std::micro>(
    t_send_done - t_start).count();
  last_timing_.receive_us = std::chrono::duration<double, std::micro>(
    t_recv_done - t_send_done).count();
  last_timing_.total_us = std::chrono::duration<double, std::micro>(
    t_recv_done - t_start).count();
  last_timing_.responses_received = total_received;
  last_timing_.responses_expected = static_cast<int>(joints_.size());

  total_us_sum_ += last_timing_.total_us;
  total_us_max_ = std::max(total_us_max_, last_timing_.total_us);
  total_missed_sum_ += total_missed;
  timing_log_counter_++;

  if (timing_log_counter_ >= kTimingLogInterval) {
    double avg_us = total_us_sum_ / timing_log_counter_;
    RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"),
      "[Timing] write() avg=%.0fus, max=%.0fus, missed=%d/%d",
      avg_us, total_us_max_, total_missed_sum_,
      timing_log_counter_ * static_cast<int>(joints_.size()));

    for (const auto & [bus_name, bt] : bus_timing_) {
      RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"),
        "[Timing]   %s: send=%.0fus, recv=%.0fus, ok=%d/%d",
        bus_name.c_str(), bt.send_us, bt.receive_us,
        bt.received, bt.expected);
    }

    // Reset aggregates
    timing_log_counter_ = 0;
    total_us_sum_ = 0.0;
    total_us_max_ = 0.0;
    total_missed_sum_ = 0;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace robstride_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robstride_hardware::RobStrideHardware,
  hardware_interface::SystemInterface)
