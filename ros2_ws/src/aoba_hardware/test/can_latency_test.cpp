// Copyright (c) 2024-2025, Yutaro KIMURA (B-SKY Lab)
// SPDX-License-Identifier: MIT

/**
 * @file can_latency_test.cpp
 * @brief CAN通信レイテンシ測定テスト
 *
 * 単一モータ(ID=127)に対してコマンド送信→レスポンス受信の往復時間を計測
 *
 * Build:
 *   g++ -std=c++17 -O2 -I../include can_latency_test.cpp ../src/robstride_driver.cpp -o
 * can_latency_test
 *
 * Usage:
 *   sudo ./can_latency_test [iterations]
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>
#include "robstride_hardware/robstride_driver.hpp"

namespace {
volatile sig_atomic_t g_running = 1;

void signal_handler(int) {
  g_running = 0;
}
}  // namespace

int main(int argc, char** argv) {
  // Usage: can_latency_test [interface] [motor_id] [iterations]
  const std::string can_interface = (argc > 1) ? argv[1] : "can0";
  const int motor_id = (argc > 2) ? std::stoi(argv[2]) : 127;
  const int iterations = (argc > 3) ? std::stoi(argv[3]) : 1000;

  std::cout << "=== CAN Latency Test ===" << std::endl;
  std::cout << "Interface: " << can_interface << std::endl;
  std::cout << "Motor ID: " << motor_id << std::endl;
  std::cout << "Iterations: " << iterations << std::endl;
  std::cout << std::endl;

  // シグナルハンドラ設定
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // ドライバ接続
  robstride_driver::RobStrideDriver driver;
  if (!driver.connect(can_interface)) {
    std::cerr << "Error: Failed to connect to " << can_interface << std::endl;
    return 1;
  }
  std::cout << "Connected to " << can_interface << std::endl;

  // モータ有効化
  std::cout << "Enabling motor " << motor_id << "..." << std::endl;
  if (!driver.enable(motor_id)) {
    std::cerr << "Error: Failed to send enable command" << std::endl;
    return 1;
  }

  // MITモード設定
  if (!driver.set_mode(motor_id, robstride_driver::ControlMode::MIT)) {
    std::cerr << "Error: Failed to send set_mode command" << std::endl;
    return 1;
  }

  // enable/set_mode応答を待機してドレイン
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  driver.drain_rx_buffer();

  // モータ通信確認: プローブコマンド送信
  std::cout << "Verifying motor communication..." << std::endl;
  {
    robstride_driver::MitCommand probe;
    probe.position = 0.0;
    probe.velocity = 0.0;
    probe.kp = 0.0;
    probe.kd = 0.0;
    probe.torque_ff = 0.0;

    driver.send_command(motor_id, probe);
    auto [recv_id, state] = driver.read_one_response(50);

    if (recv_id == motor_id && state.valid) {
      std::cout << "  Motor responded: pos=" << state.position << " rad, vel=" << state.velocity
                << " rad/s" << std::endl;
    } else {
      std::cerr << "  WARNING: Motor did not respond to probe command!" << std::endl;
      std::cerr << "  Check: motor power, CAN wiring, CAN bitrate (1Mbps), motor ID (" << motor_id
                << ")" << std::endl;
      std::cerr << "  Continuing anyway (responses will likely timeout)..." << std::endl;
    }
  }

  std::cout << "Starting latency test..." << std::endl;
  std::cout << std::endl;

  // レイテンシ計測
  std::vector<double> latencies_us;
  latencies_us.reserve(iterations);

  int success_count = 0;
  int timeout_count = 0;

  robstride_driver::MitCommand cmd;
  cmd.position = 0.0;  // 現在位置維持
  cmd.velocity = 0.0;
  cmd.kp = 0.0;  // kp=0でトルク出さない（安全）
  cmd.kd = 0.5;  // 軽いダンピング
  cmd.torque_ff = 0.0;

  using clock = std::chrono::steady_clock;

  for (int i = 0; i < iterations && g_running; ++i) {
    auto t_start = clock::now();

    // コマンド送信
    if (!driver.send_command(motor_id, cmd)) {
      std::cerr << "Send failed at iteration " << i << std::endl;
      continue;
    }

    // レスポンス受信（タイムアウト5ms）
    auto [recv_id, state] = driver.read_one_response(5);

    auto t_end = clock::now();
    double latency_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();

    if (recv_id == motor_id && state.valid) {
      latencies_us.push_back(latency_us);
      success_count++;

      // 進捗表示（100回ごと）
      if ((i + 1) % 100 == 0) {
        std::cout << "  [" << (i + 1) << "/" << iterations << "] "
                  << "last=" << latency_us << "us" << std::endl;
      }
    } else {
      timeout_count++;
      if (timeout_count <= 10) {
        std::cout << "  Timeout at iteration " << i << std::endl;
      }
    }

    // 200Hz相当の間隔（5ms）で次を送信
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // モータ停止
  std::cout << std::endl;
  std::cout << "Disabling motor..." << std::endl;
  driver.disable(motor_id);
  driver.disconnect();

  // 結果集計
  std::cout << std::endl;
  std::cout << "=== Results ===" << std::endl;
  std::cout << "Success: " << success_count << "/" << iterations << std::endl;
  std::cout << "Timeout: " << timeout_count << std::endl;

  if (latencies_us.empty()) {
    std::cerr << "No successful measurements!" << std::endl;
    return 1;
  }

  // 統計計算
  std::sort(latencies_us.begin(), latencies_us.end());

  double sum = std::accumulate(latencies_us.begin(), latencies_us.end(), 0.0);
  double mean = sum / latencies_us.size();

  double sq_sum = 0.0;
  for (double v : latencies_us) {
    sq_sum += (v - mean) * (v - mean);
  }
  double stddev = std::sqrt(sq_sum / latencies_us.size());

  double min_val = latencies_us.front();
  double max_val = latencies_us.back();
  double median = latencies_us[latencies_us.size() / 2];
  double p95 = latencies_us[static_cast<size_t>(latencies_us.size() * 0.95)];
  double p99 = latencies_us[static_cast<size_t>(latencies_us.size() * 0.99)];

  std::cout << std::endl;
  std::cout << "Latency Statistics (us):" << std::endl;
  std::cout << "  Min:    " << min_val << std::endl;
  std::cout << "  Max:    " << max_val << std::endl;
  std::cout << "  Mean:   " << mean << std::endl;
  std::cout << "  Median: " << median << std::endl;
  std::cout << "  Stddev: " << stddev << std::endl;
  std::cout << "  P95:    " << p95 << std::endl;
  std::cout << "  P99:    " << p99 << std::endl;

  // 200Hz制御可否判定
  std::cout << std::endl;
  if (p99 < 5000.0) {
    std::cout << "✓ 200Hz control is feasible (P99 < 5ms)" << std::endl;
  } else {
    std::cout << "✗ 200Hz control may be challenging (P99 >= 5ms)" << std::endl;
  }

  return 0;
}
