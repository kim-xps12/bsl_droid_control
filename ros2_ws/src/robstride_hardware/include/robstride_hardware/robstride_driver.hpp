/**
 * @file robstride_driver.hpp
 * @brief RobStride motor CAN communication driver (ROS 2 independent)
 * 
 * This driver handles low-level CAN communication with RobStride motors
 * using the MIT control protocol. It can be used independently of ros2_control
 * for testing and debugging.
 */

#pragma once

#include <cstdint>
#include <string>
#include <linux/can.h>

namespace robstride_driver
{

/**
 * @brief Motor state structure
 */
struct MotorState {
  double position = 0.0;   // [rad]
  double velocity = 0.0;   // [rad/s]
  double torque = 0.0;     // [Nm]
  bool valid = false;
};

/**
 * @brief MIT control command structure
 */
struct MitCommand {
  double position = 0.0;   // Target position [rad]
  double velocity = 0.0;   // Target velocity [rad/s]
  double kp = 30.0;        // Position gain [Nm/rad]
  double kd = 1.0;         // Damping gain [Nm/(rad/s)]
  double torque_ff = 0.0;  // Feedforward torque [Nm]
};

/**
 * @brief Control modes for RobStride motors
 */
enum class ControlMode : int8_t {
  MIT = 0,
  POSITION = 1,
  VELOCITY = 2,
  TORQUE = 3
};

/**
 * @brief RobStride motor driver class
 * 
 * Handles CAN communication with RobStride motors (RS-01, RS-02, RS-03).
 * This class is ROS 2 independent and can be used standalone for testing.
 */
class RobStrideDriver
{
public:
  RobStrideDriver() = default;
  ~RobStrideDriver();
  
  // Disable copy
  RobStrideDriver(const RobStrideDriver&) = delete;
  RobStrideDriver& operator=(const RobStrideDriver&) = delete;
  
  /**
   * @brief Connect to CAN interface
   * @param interface CAN interface name (e.g., "can0")
   * @return true if successful
   */
  bool connect(const std::string& interface);
  
  /**
   * @brief Disconnect from CAN interface
   */
  void disconnect();
  
  /**
   * @brief Check if connected to CAN interface
   */
  bool is_connected() const { return can_socket_ >= 0; }
  
  /**
   * @brief Enable motor
   * @param motor_id Motor CAN ID
   * @return true if successful
   */
  bool enable(int motor_id);
  
  /**
   * @brief Disable motor
   * @param motor_id Motor CAN ID
   * @return true if successful
   */
  bool disable(int motor_id);
  
  /**
   * @brief Set control mode
   * @param motor_id Motor CAN ID
   * @param mode Control mode
   * @return true if successful
   */
  bool set_mode(int motor_id, ControlMode mode);
  
  /**
   * @brief Set velocity limit
   * @param motor_id Motor CAN ID
   * @param limit Velocity limit [rad/s]
   * @return true if successful
   */
  bool set_velocity_limit(int motor_id, float limit);
  
  /**
   * @brief Set torque limit
   * @param motor_id Motor CAN ID
   * @param limit Torque limit [Nm]
   * @return true if successful
   */
  bool set_torque_limit(int motor_id, float limit);
  
  /**
   * @brief Send MIT control command
   * @param motor_id Motor CAN ID
   * @param cmd MIT command structure
   * @return true if successful
   */
  bool send_command(int motor_id, const MitCommand& cmd);
  
  /**
   * @brief Read motor state
   * @param motor_id Motor CAN ID (used for filtering)
   * @return Motor state (check .valid flag)
   */
  MotorState read_state(int motor_id);
  
  /**
   * @brief Set host ID for CAN communication
   * @param host_id Host ID (default: 0xFF)
   */
  void set_host_id(int host_id) { host_id_ = host_id; }

private:
  int can_socket_ = -1;
  int host_id_ = 0xFF;
  
  // CAN communication helpers
  bool send_frame(uint32_t can_id, const uint8_t* data, uint8_t dlc);
  bool read_frame(struct can_frame* frame, int timeout_ms = 10);
  
  // Protocol helpers
  bool write_parameter(int motor_id, uint16_t param_id, float value);
  bool write_parameter(int motor_id, uint16_t param_id, int8_t value);
};

// ============================================================================
// Scale factors for RobStride RS-02 motor
// Spec: Max velocity = 44 rad/s, Max torque = 17 Nm
// ============================================================================
namespace scale
{
  constexpr double POSITION = 4.0 * 3.14159265358979323846;  // ±4π rad
  constexpr double VELOCITY = 44.0;   // ±44 rad/s (RS-02 spec)
  constexpr double TORQUE = 17.0;     // ±17 Nm (RS-02 spec)
  constexpr double KP = 500.0;        // 0-500 Nm/rad (RS-02 spec)
  constexpr double KD = 5.0;          // 0-5 Nm/(rad/s) (RS-02 spec)
}

// ============================================================================
// Protocol constants
// ============================================================================
namespace protocol
{
  // Communication types
  namespace comm_type
  {
    constexpr uint32_t GET_DEVICE_ID = 0;
    constexpr uint32_t OPERATION_CONTROL = 1;
    constexpr uint32_t OPERATION_STATUS = 2;
    constexpr uint32_t ENABLE = 3;
    constexpr uint32_t DISABLE = 4;
    constexpr uint32_t SET_ZERO = 6;
    constexpr uint32_t WRITE_PARAMETER = 18;
  }
  
  // Parameter IDs
  namespace param_id
  {
    constexpr uint16_t MODE = 0x7005;
    constexpr uint16_t VELOCITY_LIMIT = 0x7017;
    constexpr uint16_t TORQUE_LIMIT = 0x700B;
  }
}

}  // namespace robstride_driver
