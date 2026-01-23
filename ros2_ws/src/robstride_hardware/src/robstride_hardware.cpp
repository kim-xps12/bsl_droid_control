/**
 * @file robstride_hardware.cpp
 * @brief ros2_control Hardware Interface implementation for RobStride motors
 */

#include "robstride_hardware/robstride_hardware.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <chrono>
#include <pthread.h>
#include <sched.h>
#include <cstring>

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
  
  // Read parameters from URDF
  can_interface_ = info_.hardware_parameters.count("can_interface")
    ? info_.hardware_parameters.at("can_interface") : "can0";
  motor_id_ = info_.hardware_parameters.count("motor_id")
    ? std::stoi(info_.hardware_parameters.at("motor_id")) : 11;
  kp_ = info_.hardware_parameters.count("kp")
    ? std::stod(info_.hardware_parameters.at("kp")) : 30.0;
  kd_ = info_.hardware_parameters.count("kd")
    ? std::stod(info_.hardware_parameters.at("kd")) : 1.0;

  // Read state reader thread configuration from URDF
  state_reader_rate_ = info_.hardware_parameters.count("state_reader_rate")
    ? std::stoi(info_.hardware_parameters.at("state_reader_rate")) : 200;
  state_reader_cpu_affinity_ = info_.hardware_parameters.count("state_reader_cpu_affinity")
    ? std::stoi(info_.hardware_parameters.at("state_reader_cpu_affinity")) : 3;
  state_reader_priority_ = info_.hardware_parameters.count("state_reader_priority")
    ? std::stoi(info_.hardware_parameters.at("state_reader_priority")) : 80;
  
  // Initialize storage for each joint
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_efforts_.resize(info_.joints.size(), 0.0);
  hw_commands_position_.resize(info_.joints.size(), 0.0);
  
  // Store joint name for publisher
  if (!info_.joints.empty()) {
    joint_name_ = info_.joints[0].name;
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("RobStrideHardware"),
    "Initialized: can=%s, motor_id=%d, kp=%.1f, kd=%.1f, joints=%zu",
    can_interface_.c_str(), motor_id_, kp_, kd_, info_.joints.size());
  RCLCPP_INFO(
    rclcpp::get_logger("RobStrideHardware"),
    "State reader config: rate=%dHz, cpu=%d, priority=%d",
    state_reader_rate_, state_reader_cpu_affinity_, state_reader_priority_);
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobStrideHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"), "Configuring...");
  
  // Create node and publisher for joint states (used by state reader thread)
  pub_node_ = rclcpp::Node::make_shared("robstride_state_publisher");
  joint_state_pub_ = pub_node_->create_publisher<sensor_msgs::msg::JointState>(
    "/robstride/joint_states", rclcpp::QoS(10).best_effort());
  RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"), 
    "Created publisher: /robstride/joint_states");
  
  // Connect to CAN interface
  if (!driver_.connect(can_interface_)) {
    RCLCPP_ERROR(rclcpp::get_logger("RobStrideHardware"), 
      "Failed to connect to CAN interface: %s", can_interface_.c_str());
    return CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"), 
    "Connected to CAN interface: %s", can_interface_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobStrideHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"), "Activating...");
  
  // Enable motor
  if (!driver_.enable(motor_id_)) {
    RCLCPP_ERROR(rclcpp::get_logger("RobStrideHardware"), 
      "Failed to enable motor %d", motor_id_);
    return CallbackReturn::ERROR;
  }
  
  // Set MIT mode
  if (!driver_.set_mode(motor_id_, robstride_driver::ControlMode::MIT)) {
    RCLCPP_ERROR(rclcpp::get_logger("RobStrideHardware"), 
      "Failed to set MIT mode for motor %d", motor_id_);
    return CallbackReturn::ERROR;
  }
  
  // Initialize command to current position
  for (size_t i = 0; i < hw_commands_position_.size(); ++i) {
    hw_commands_position_[i] = hw_positions_[i];
  }
  
  // Start state reader thread (separate from RT loop)
  state_reader_running_ = true;
  state_reader_thread_ = std::thread(&RobStrideHardware::state_reader_loop, this);

  // Set CPU affinity for state reader thread (bind to specific CPU core)
  if (state_reader_cpu_affinity_ >= 0) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(state_reader_cpu_affinity_, &cpuset);
    int ret = pthread_setaffinity_np(state_reader_thread_.native_handle(),
                                      sizeof(cpu_set_t), &cpuset);
    if (ret != 0) {
      RCLCPP_WARN(rclcpp::get_logger("RobStrideHardware"),
        "Failed to set CPU affinity to CPU %d: %s",
        state_reader_cpu_affinity_, std::strerror(ret));
    } else {
      RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"),
        "State reader thread bound to CPU %d", state_reader_cpu_affinity_);
    }
  }

  // Set real-time scheduling priority (SCHED_FIFO)
  if (state_reader_priority_ > 0) {
    struct sched_param param;
    std::memset(&param, 0, sizeof(param));
    param.sched_priority = state_reader_priority_;
    int ret = pthread_setschedparam(state_reader_thread_.native_handle(),
                                     SCHED_FIFO, &param);
    if (ret != 0) {
      RCLCPP_WARN(rclcpp::get_logger("RobStrideHardware"),
        "Failed to set RT priority %d: %s (may need sudo or rtprio limits configured)",
        state_reader_priority_, std::strerror(ret));
    } else {
      RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"),
        "State reader thread priority set to %d (SCHED_FIFO)", state_reader_priority_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"),
    "Motor %d activated in MIT mode, state reader thread started (%dHz)",
    motor_id_, state_reader_rate_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobStrideHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"), "Deactivating...");
  
  // Stop state reader thread first
  state_reader_running_ = false;
  if (state_reader_thread_.joinable()) {
    state_reader_thread_.join();
  }
  RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"), "State reader thread stopped");
  
  // Send zero torque command (safe state)
  robstride_driver::MitCommand safe_cmd;
  safe_cmd.position = hw_positions_[0];
  safe_cmd.velocity = 0.0;
  safe_cmd.kp = 0.0;
  safe_cmd.kd = 0.0;
  safe_cmd.torque_ff = 0.0;
  driver_.send_command(motor_id_, safe_cmd);
  
  // Disable motor
  driver_.disable(motor_id_);
  
  RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"), 
    "Motor %d deactivated", motor_id_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobStrideHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"), "Cleaning up...");
  driver_.disconnect();
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
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
  }
  return command_interfaces;
}

// ============================================================================
// RT control loop (called at 200Hz by Controller Manager)
// ============================================================================

hardware_interface::return_type RobStrideHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Ultra-lightweight: just copy from atomic variables (updated by separate thread)
  // This does NOT perform any CAN I/O, so RT loop is not affected
  hw_positions_[0] = latest_position_.load(std::memory_order_relaxed);
  hw_velocities_[0] = latest_velocity_.load(std::memory_order_relaxed);
  hw_efforts_[0] = latest_torque_.load(std::memory_order_relaxed);
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobStrideHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Build and send MIT command
  robstride_driver::MitCommand cmd;
  cmd.position = hw_commands_position_[0];
  cmd.velocity = 0.0;
  cmd.kp = kp_;
  cmd.kd = kd_;
  cmd.torque_ff = 0.0;
  
  driver_.send_command(motor_id_, cmd);
  
  return hardware_interface::return_type::OK;
}

// ============================================================================
// State reader thread (configurable Hz, completely separate from RT control loop)
// ============================================================================

void RobStrideHardware::state_reader_loop()
{
  RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"),
    "State reader thread started (%dHz) for motor %d", state_reader_rate_, motor_id_);

  // Calculate period from state_reader_rate_ (200Hz = 5ms period)
  const auto period = std::chrono::microseconds(1000000 / state_reader_rate_);
  auto next_time = std::chrono::steady_clock::now();
  
  // Pre-allocate message to avoid allocation in loop
  sensor_msgs::msg::JointState msg;
  msg.name.push_back(joint_name_);
  msg.position.resize(1);
  msg.velocity.resize(1);
  msg.effort.resize(1);
  
  int read_count = 0;
  int valid_count = 0;
  
  while (state_reader_running_.load(std::memory_order_acquire)) {
    // Read motor state (this is the only thread doing CAN reads)
    auto state = driver_.read_state(motor_id_);
    
    if (state.valid) {
      // Update atomic variables (for RT loop's read())
      latest_position_.store(state.position, std::memory_order_relaxed);
      latest_velocity_.store(state.velocity, std::memory_order_relaxed);
      latest_torque_.store(state.torque, std::memory_order_relaxed);
      
      // Publish joint state (this thread only, no RT impact)
      msg.header.stamp = pub_node_->now();
      msg.position[0] = state.position;
      msg.velocity[0] = state.velocity;
      msg.effort[0] = state.torque;
      joint_state_pub_->publish(msg);
      
      valid_count++;
    }
    read_count++;
    
    // Log every 1 second (state_reader_rate_ cycles)
    if (read_count % state_reader_rate_ == 0) {
      RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"),
        "[StateReader] pos: %.3f rad, vel: %.3f rad/s, torque: %.3f, valid: %d/%d (%.1f%%)",
        latest_position_.load(), latest_velocity_.load(), latest_torque_.load(),
        valid_count, read_count, (100.0 * valid_count / read_count));
      read_count = 0;
      valid_count = 0;
    }
    
    // Sleep until next period (fixed 100Hz)
    next_time += period;
    std::this_thread::sleep_until(next_time);
  }
  
  RCLCPP_INFO(rclcpp::get_logger("RobStrideHardware"), 
    "State reader thread stopped for motor %d", motor_id_);
}

}  // namespace robstride_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robstride_hardware::RobStrideHardware,
  hardware_interface::SystemInterface)
