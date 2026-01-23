/**
 * @file robstride_hardware.hpp
 * @brief ros2_control Hardware Interface for RobStride motors
 */

#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>
#include <thread>
#include <atomic>

#include "robstride_hardware/robstride_driver.hpp"

namespace robstride_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RobStrideHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
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
  // State reader thread (runs at state_reader_rate_ Hz, separate from RT loop)
  void state_reader_loop();
  
  // RobStride driver (ROS 2 independent)
  robstride_driver::RobStrideDriver driver_;
  
  // Configuration from URDF
  std::string can_interface_;
  int motor_id_ = 11;
  double kp_ = 30.0;
  double kd_ = 1.0;

  // State reader thread configuration (from URDF).
  // These in-class initializers act as fallback defaults and may be overridden in on_init()
  // if corresponding parameters are provided; if parameters are missing, these values apply.
  int state_reader_rate_ = 200;           // Fallback default: Hz (200Hz to match RT loop)
  int state_reader_cpu_affinity_ = 3;     // Fallback default: bind to CPU 3 (RT loop uses CPU 2)
  int state_reader_priority_ = 80;        // Fallback default: SCHED_FIFO priority (lower than RT loop's 90)
  
  // State storage (updated in read())
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  
  // Command storage (used in write())
  std::vector<double> hw_commands_position_;
  
  // Asynchronous state reader (does not affect RT loop)
  std::thread state_reader_thread_;
  std::atomic<bool> state_reader_running_{false};
  std::atomic<double> latest_position_{0.0};
  std::atomic<double> latest_velocity_{0.0};
  std::atomic<double> latest_torque_{0.0};
  
  // ROS 2 publisher for joint states (used by state reader thread only)
  rclcpp::Node::SharedPtr pub_node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::string joint_name_;
};

}  // namespace robstride_hardware
