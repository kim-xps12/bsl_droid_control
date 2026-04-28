# Copyright (c) 2024-2025, Yutaro KIMURA (B-SKY Lab)
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.14)
project(sysid_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# AobaDriver: ROS非依存、相対パスで直接参照
set(AOBA_DRIVER_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../ros2_ws/src/aoba_hardware"
    CACHE PATH "Path to aoba_hardware package root")

add_executable(sysid_recorder
  src/main.cpp
  "${AOBA_DRIVER_ROOT}/src/aoba_driver.cpp")

target_include_directories(sysid_recorder PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${AOBA_DRIVER_ROOT}/include")

target_compile_features(sysid_recorder PRIVATE cxx_std_17)

# SCHED_FIFO / mlockall / clock_nanosleep はすべて glibc に含まれる
# Linux >= 2.6 では pthread のみでよい
target_link_libraries(sysid_recorder PRIVATE pthread)

install(TARGETS sysid_recorder DESTINATION bin)
