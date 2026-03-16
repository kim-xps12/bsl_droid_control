cmake_minimum_required(VERSION 3.14)
project(robstride_hardware)

# CMP0167: FindBoost removed in CMake 3.30+.
# realtime_tools (dependency of hardware_interface) still triggers FindBoost.
# Use the NEW behaviour (use BoostConfig.cmake) to suppress the dev warning.
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 共有ライブラリを作成（driver + hardware interface）
add_library(${PROJECT_NAME} SHARED
  src/robstride_driver.cpp
  src/robstride_hardware.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface pluginlib rclcpp rclcpp_lifecycle
)

# Plugin export（ros2_control用）
pluginlib_export_plugin_description_file(
  hardware_interface robstride_hardware_plugin.xml)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY urdf config launch DESTINATION share/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/single_motor_test_commander.py
  scripts/multi_motor_zero_commander.py
  scripts/analyze_timing_log.py
  DESTINATION lib/${PROJECT_NAME}
)

# Standalone CAN latency test (no ROS 2 dependency)
add_executable(can_latency_test
  test/can_latency_test.cpp
  src/robstride_driver.cpp
)
target_include_directories(can_latency_test PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(can_latency_test PRIVATE cxx_std_17)
install(TARGETS can_latency_test DESTINATION lib/${PROJECT_NAME})

# Multi-bus CAN latency test (no ROS 2 dependency)
add_executable(multi_bus_latency_test
  test/multi_bus_latency_test.cpp
  src/robstride_driver.cpp
)
target_include_directories(multi_bus_latency_test PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(multi_bus_latency_test PRIVATE cxx_std_17)
install(TARGETS multi_bus_latency_test DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
