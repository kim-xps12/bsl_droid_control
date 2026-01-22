# RobStride Hardware Interface 実装リファレンス

ros2_control準拠のHardware Interface実装例。

## 参考リポジトリ

- **Seeed RobStride_Control**: https://github.com/Seeed-Projects/RobStride_Control
  - CANプロトコル・MITモード制御の参考実装（`ref/RobStride_Control/` にコピーあり）

## ファイル構成

```
robstride_hardware/
├── CMakeLists.txt
├── package.xml
├── robstride_hardware_plugin.xml
├── urdf/
│   └── robstride_rs02.urdf.xacro
├── config/
│   └── controllers.yaml
├── launch/
│   └── robstride_control.launch.py
├── include/robstride_hardware/
│   └── robstride_hardware.hpp
└── src/
    └── robstride_hardware.cpp
```

## robstride_hardware.hpp

```cpp
#pragma once
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

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

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // RT control loop (Controller Managerから200Hzで呼ばれる)
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  int can_socket_ = -1;
  std::string can_interface_;
  std::vector<int> motor_ids_;
  
  // State interfaces
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  
  // Command interfaces
  std::vector<double> hw_commands_position_;
  
  double kp_ = 30.0;
  double kd_ = 0.5;
};

}  // namespace robstride_hardware
```

## robstride_hardware.cpp

```cpp
#include "robstride_hardware/robstride_hardware.hpp"
#include "robstride_control/can_interface.h"
#include "robstride_control/protocol.h"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace robstride_hardware
{

CallbackReturn RobStrideHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  
  can_interface_ = info_.hardware_parameters["can_interface"];
  
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_efforts_.resize(info_.joints.size(), 0.0);
  hw_commands_position_.resize(info_.joints.size(), 0.0);
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobStrideHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  can_socket_ = init_can(can_interface_.c_str());
  if (can_socket_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("RobStrideHardware"), "Failed to init CAN");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobStrideHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    enable_motor(can_socket_, motor_ids_[i]);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobStrideHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    write_operation_frame(can_socket_, motor_ids_[i], 0.0, 0.0, 0.0);
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> 
RobStrideHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
RobStrideHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type RobStrideHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    auto [pos, vel, torque] = read_operation_frame(can_socket_, motor_ids_[i]);
    hw_positions_[i] = pos;
    hw_velocities_[i] = vel;
    hw_efforts_[i] = torque;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobStrideHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    write_operation_frame(can_socket_, motor_ids_[i], hw_commands_position_[i], kp_, kd_);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace robstride_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robstride_hardware::RobStrideHardware,
  hardware_interface::SystemInterface)
```

## CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(robstride_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# RobStride_Control (git submodule)
set(ROBSTRIDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../RobStride_Control/cpp)

add_library(${PROJECT_NAME} SHARED
  src/robstride_hardware.cpp
  ${ROBSTRIDE_DIR}/src/can_interface.cpp
  ${ROBSTRIDE_DIR}/src/protocol.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${ROBSTRIDE_DIR}/include
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface pluginlib rclcpp rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface robstride_hardware_plugin.xml)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY urdf config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
```

## robstride_hardware_plugin.xml

```xml
<library path="robstride_hardware">
  <class name="robstride_hardware/RobStrideHardware"
         type="robstride_hardware::RobStrideHardware"
         base_class_type="hardware_interface::SystemInterface">
    <description>RobStride RS02 Hardware Interface for ros2_control</description>
  </class>
</library>
```

## urdf/robstride_rs02.urdf.xacro

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robstride_rs02">
  <ros2_control name="RobStrideSystem" type="system">
    <hardware>
      <plugin>robstride_hardware/RobStrideHardware</plugin>
      <param name="can_interface">can0</param>
      <param name="motor_ids">[1]</param>
      <param name="kp">30.0</param>
      <param name="kd">0.5</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
</robot>
```

## config/controllers.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 200  # Hz
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    forward_position_controller:
      type: forward_command_controller/ForwardCommandController

forward_position_controller:
  ros__parameters:
    joints: [joint1]
    interface_name: position
```

## launch/robstride_control.launch.py

```python
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('robstride_hardware')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'robstride_rs02.urdf.xacro'])
    controller_config = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])
    
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': Command(['xacro ', urdf_file])},
                controller_config,
                {'lock_memory': True, 'cpu_affinity': [0], 'thread_priority': 50}
            ],
            output='both',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['forward_position_controller'],
        ),
    ])
```
