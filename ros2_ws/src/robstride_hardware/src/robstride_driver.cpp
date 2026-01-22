/**
 * @file robstride_driver.cpp
 * @brief RobStride motor CAN communication driver implementation
 */

#include "robstride_hardware/robstride_driver.hpp"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <linux/can/raw.h>

namespace robstride_driver
{

// ============================================================================
// Utility functions
// ============================================================================

namespace
{

inline void pack_u16_le(uint8_t* buf, uint16_t val)
{
  buf[0] = val & 0xFF;
  buf[1] = (val >> 8) & 0xFF;
}

inline void pack_u16_be(uint8_t* buf, uint16_t val)
{
  buf[0] = (val >> 8) & 0xFF;
  buf[1] = val & 0xFF;
}

inline void pack_float_le(uint8_t* buf, float val)
{
  std::memcpy(buf, &val, sizeof(float));
}

inline uint16_t unpack_u16_be(const uint8_t* buf)
{
  return (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
}

}  // anonymous namespace

// ============================================================================
// RobStrideDriver implementation
// ============================================================================

RobStrideDriver::~RobStrideDriver()
{
  disconnect();
}

bool RobStrideDriver::connect(const std::string& interface)
{
  if (can_socket_ >= 0) {
    disconnect();
  }
  
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    return false;
  }
  
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  
  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(can_socket_);
    can_socket_ = -1;
    return false;
  }
  
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  
  if (bind(can_socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(can_socket_);
    can_socket_ = -1;
    return false;
  }
  
  // Set non-blocking read (no timeout)
  int flags = fcntl(can_socket_, F_GETFL, 0);
  fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
  
  return true;
}

void RobStrideDriver::disconnect()
{
  if (can_socket_ >= 0) {
    ::close(can_socket_);
    can_socket_ = -1;
  }
}

bool RobStrideDriver::send_frame(uint32_t can_id, const uint8_t* data, uint8_t dlc)
{
  if (can_socket_ < 0) {
    return false;
  }
  
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id | CAN_EFF_FLAG;  // Extended frame (29-bit ID)
  frame.can_dlc = dlc;
  
  if (data && dlc > 0) {
    std::memcpy(frame.data, data, std::min(dlc, static_cast<uint8_t>(8)));
  }
  
  ssize_t nbytes = ::write(can_socket_, &frame, sizeof(frame));
  return nbytes == sizeof(frame);
}

bool RobStrideDriver::read_frame(struct can_frame* frame, int /*timeout_ms*/)
{
  if (can_socket_ < 0 || !frame) {
    return false;
  }
  
  ssize_t nbytes = ::read(can_socket_, frame, sizeof(*frame));
  return nbytes == sizeof(*frame);
}

bool RobStrideDriver::enable(int motor_id)
{
  uint32_t ext_id = (protocol::comm_type::ENABLE << 24) | 
                    (static_cast<uint32_t>(host_id_) << 8) | 
                    static_cast<uint32_t>(motor_id);
  return send_frame(ext_id, nullptr, 0);
}

bool RobStrideDriver::disable(int motor_id)
{
  uint32_t ext_id = (protocol::comm_type::DISABLE << 24) | 
                    (static_cast<uint32_t>(host_id_) << 8) | 
                    static_cast<uint32_t>(motor_id);
  return send_frame(ext_id, nullptr, 0);
}

bool RobStrideDriver::set_mode(int motor_id, ControlMode mode)
{
  return write_parameter(motor_id, protocol::param_id::MODE, static_cast<int8_t>(mode));
}

bool RobStrideDriver::set_velocity_limit(int motor_id, float limit)
{
  return write_parameter(motor_id, protocol::param_id::VELOCITY_LIMIT, limit);
}

bool RobStrideDriver::set_torque_limit(int motor_id, float limit)
{
  return write_parameter(motor_id, protocol::param_id::TORQUE_LIMIT, limit);
}

bool RobStrideDriver::write_parameter(int motor_id, uint16_t param_id, float value)
{
  uint32_t ext_id = (protocol::comm_type::WRITE_PARAMETER << 24) | 
                    (static_cast<uint32_t>(host_id_) << 8) | 
                    static_cast<uint32_t>(motor_id);
  
  uint8_t data[8] = {0};
  pack_u16_le(&data[0], param_id);
  pack_float_le(&data[4], value);
  
  return send_frame(ext_id, data, 8);
}

bool RobStrideDriver::write_parameter(int motor_id, uint16_t param_id, int8_t value)
{
  uint32_t ext_id = (protocol::comm_type::WRITE_PARAMETER << 24) | 
                    (static_cast<uint32_t>(host_id_) << 8) | 
                    static_cast<uint32_t>(motor_id);
  
  uint8_t data[8] = {0};
  pack_u16_le(&data[0], param_id);
  data[4] = static_cast<uint8_t>(value);
  
  return send_frame(ext_id, data, 8);
}

bool RobStrideDriver::send_command(int motor_id, const MitCommand& cmd)
{
  // Clamp values to valid ranges
  double pos_clamped = std::clamp(cmd.position, -scale::POSITION, scale::POSITION);
  double vel_clamped = std::clamp(cmd.velocity, -scale::VELOCITY, scale::VELOCITY);
  double kp_clamped = std::clamp(cmd.kp, 0.0, scale::KP);
  double kd_clamped = std::clamp(cmd.kd, 0.0, scale::KD);
  double tq_clamped = std::clamp(cmd.torque_ff, -scale::TORQUE, scale::TORQUE);
  
  // Convert to uint16 (big-endian for MIT mode)
  uint16_t pos_u16 = static_cast<uint16_t>(((pos_clamped / scale::POSITION) + 1.0) * 0x7FFF);
  uint16_t vel_u16 = static_cast<uint16_t>(((vel_clamped / scale::VELOCITY) + 1.0) * 0x7FFF);
  uint16_t kp_u16 = static_cast<uint16_t>((kp_clamped / scale::KP) * 0xFFFF);
  uint16_t kd_u16 = static_cast<uint16_t>((kd_clamped / scale::KD) * 0xFFFF);
  uint16_t torque_u16 = static_cast<uint16_t>(((tq_clamped / scale::TORQUE) + 1.0) * 0x7FFF);
  
  // Pack data (big-endian)
  uint8_t data[8];
  pack_u16_be(&data[0], pos_u16);
  pack_u16_be(&data[2], vel_u16);
  pack_u16_be(&data[4], kp_u16);
  pack_u16_be(&data[6], kd_u16);
  
  // CAN ID includes torque feedforward
  uint32_t ext_id = (protocol::comm_type::OPERATION_CONTROL << 24) | 
                    (static_cast<uint32_t>(torque_u16) << 8) | 
                    static_cast<uint32_t>(motor_id);
  
  return send_frame(ext_id, data, 8);
}

MotorState RobStrideDriver::read_state(int motor_id)
{
  MotorState state;
  struct can_frame frame;
  
  // Read all available frames to clear buffer, keep latest status
  // (Motor sends status after every command)
  for (int attempts = 0; attempts < 20; ++attempts) {
    if (!read_frame(&frame)) {
      break;
    }
    
    // Check for extended frame
    if (!(frame.can_id & CAN_EFF_FLAG)) {
      continue;
    }
    
    uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
    uint32_t comm_type = (raw_id >> 24) & 0x1F;
    // In response frame format for RobStride:
    // bits[28:24] = comm_type (5 bits)
    // bits[23:16] = status flags / error code
    // bits[15:8] = motor_id
    // bits[7:0] = host_id
    uint32_t recv_motor_id = (raw_id >> 8) & 0xFF;
    
    // Check for operation status from target motor
    if (comm_type == protocol::comm_type::OPERATION_STATUS && 
        recv_motor_id == static_cast<uint32_t>(motor_id)) {
      
      // Decode MIT status frame (big-endian)
      uint16_t pos_u16 = unpack_u16_be(&frame.data[0]);
      uint16_t vel_u16 = unpack_u16_be(&frame.data[2]);
      uint16_t torque_u16 = unpack_u16_be(&frame.data[4]);
      
      // Convert to physical units
      state.position = ((static_cast<double>(pos_u16) / 0x7FFF) - 1.0) * scale::POSITION;
      state.velocity = ((static_cast<double>(vel_u16) / 0x7FFF) - 1.0) * scale::VELOCITY;
      state.torque = ((static_cast<double>(torque_u16) / 0x7FFF) - 1.0) * scale::TORQUE;
      state.valid = true;
      
      return state;
    }
  }
  
  return state;  // valid = false
}

}  // namespace robstride_driver
