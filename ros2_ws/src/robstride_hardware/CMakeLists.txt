cmake_minimum_required(VERSION 3.8)
project(robstride_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

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
  hardware_interface pluginlib rclcpp rclcpp_lifecycle sensor_msgs
)

# Plugin export（ros2_control用）
pluginlib_export_plugin_description_file(
  hardware_interface robstride_hardware_plugin.xml)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY urdf config launch DESTINATION share/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/sinusoidal_motion_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
