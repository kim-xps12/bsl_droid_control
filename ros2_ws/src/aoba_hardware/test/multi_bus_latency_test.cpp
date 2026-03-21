// Copyright (c) 2024-2025, Yutaro KIMURA (B-SKY Lab)
// SPDX-License-Identifier: MIT

/**
 * @file multi_bus_latency_test.cpp
 * @brief 多モータ・多バス CAN レイテンシ計測テスト
 *
 * 本番 write() と同一のバースト送受信パターンで RT 品質を計測する。
 * Phase 1: 全バスの全モータへコマンドをバースト送信
 * Phase 2: 全バスから全応答を poll() で受信（タイムアウト 2ms）
 *
 * Usage:
 *   multi_bus_latency_test <bus:id,...> [<bus:id,...> ...] [--iterations=N]
 *
 * Example:
 *   multi_bus_latency_test can1:11,12,13,14,15 can2:21,22,23,24,25
 *   multi_bus_latency_test can1:11,12 --iterations=2000
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <map>
#include <numeric>
#include <sstream>
#include <thread>
#include <vector>
#include "aoba_hardware/aoba_driver.hpp"

namespace {
volatile sig_atomic_t g_running = 1;

void signal_handler(int) {
  g_running = 0;
}
}  // namespace

/// バス設定（インターフェース名とモータIDのリスト）
struct BusConfig {
  std::string interface;       // CANインターフェース名（例: "can1"）
  std::vector<int> motor_ids;  // そのバス上のモータIDリスト
};

/// バス毎の計測統計
struct BusStats {
  std::string interface;        // CANインターフェース名
  std::vector<double> send_us;  // 各サイクルの送信時間 [us]
  std::vector<double> recv_us;  // 各サイクルの受信時間 [us]
  int ok = 0;                   // 受信成功数
  int expected = 0;             // 期待される受信数
};

/// コマンドライン引数 "can1:11,12,13" をパースしてBusConfigに変換する
static bool parse_bus_arg(const std::string& arg, BusConfig& out) {
  auto colon = arg.find(':');
  if (colon == std::string::npos)
    return false;
  out.interface = arg.substr(0, colon);
  std::string ids_str = arg.substr(colon + 1);
  std::stringstream ss(ids_str);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    if (!tok.empty()) {
      out.motor_ids.push_back(std::stoi(tok));
    }
  }
  return !out.motor_ids.empty();
}

static double percentile(std::vector<double>& sorted_vec, double p) {
  if (sorted_vec.empty())
    return 0.0;
  size_t idx = static_cast<size_t>(sorted_vec.size() * p);
  if (idx >= sorted_vec.size())
    idx = sorted_vec.size() - 1;
  return sorted_vec[idx];
}

int main(int argc, char** argv) {
  // コマンドライン引数をパースする
  std::vector<BusConfig> bus_configs;
  int iterations = 1000;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg.substr(0, 13) == "--iterations=") {
      iterations = std::stoi(arg.substr(13));
    } else {
      BusConfig cfg;
      if (parse_bus_arg(arg, cfg)) {
        bus_configs.push_back(cfg);
      } else {
        std::cerr << "Error: invalid argument '" << arg << "'" << std::endl;
        std::cerr << "Usage: multi_bus_latency_test <bus:id,...> [...] [--iterations=N]"
                  << std::endl;
        return 1;
      }
    }
  }

  if (bus_configs.empty()) {
    std::cerr << "Usage: multi_bus_latency_test <bus:id,...> [...] [--iterations=N]" << std::endl;
    std::cerr << "Example: multi_bus_latency_test can1:11,12,13,14,15 can2:21,22,23,24,25"
              << std::endl;
    return 1;
  }

  // テスト設定を表示する
  std::cout << "=== Multi-Bus CAN Latency Test ===" << std::endl;
  int total_motors = 0;
  for (const auto& cfg : bus_configs) {
    std::cout << "Bus: " << cfg.interface << "  Motors:";
    for (int id : cfg.motor_ids)
      std::cout << " " << id;
    std::cout << std::endl;
    total_motors += static_cast<int>(cfg.motor_ids.size());
  }
  std::cout << "Total motors: " << total_motors << std::endl;
  std::cout << "Iterations: " << iterations << std::endl;
  std::cout << std::endl;

  // シグナルハンドラ設定
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // 各バスのドライバに接続する
  std::vector<aoba_driver::AobaDriver> drivers(bus_configs.size());
  for (size_t b = 0; b < bus_configs.size(); ++b) {
    if (!drivers[b].connect(bus_configs[b].interface)) {
      std::cerr << "Error: Failed to connect to " << bus_configs[b].interface << std::endl;
      return 1;
    }
    std::cout << "Connected to " << bus_configs[b].interface << std::endl;
  }

  // モータ初期化（自動レポート無効化 → 有効化 → MITモード設定）
  std::cout << "Initializing motors..." << std::endl;
  for (size_t b = 0; b < bus_configs.size(); ++b) {
    for (int id : bus_configs[b].motor_ids) {
      drivers[b].disable_auto_report(id);
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  for (size_t b = 0; b < bus_configs.size(); ++b) {
    for (int id : bus_configs[b].motor_ids) {
      if (!drivers[b].enable(id)) {
        std::cerr << "Warning: Failed to send enable to motor " << id << " on "
                  << bus_configs[b].interface << std::endl;
      }
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  for (size_t b = 0; b < bus_configs.size(); ++b) {
    for (int id : bus_configs[b].motor_ids) {
      drivers[b].set_mode(id, aoba_driver::ControlMode::MIT);
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // 古いフレームをドレインする
  for (size_t b = 0; b < bus_configs.size(); ++b) {
    drivers[b].drain_rx_buffer();
  }

  // プローブ（ゼロトルク）で通信を確認する
  std::cout << "Probing motors..." << std::endl;
  {
    aoba_driver::MitCommand probe;
    probe.position = 0.0;
    probe.velocity = 0.0;
    probe.kp = 0.0;
    probe.kd = 0.5;
    probe.torque_ff = 0.0;

    for (size_t b = 0; b < bus_configs.size(); ++b) {
      for (int id : bus_configs[b].motor_ids) {
        drivers[b].send_command(id, probe);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    for (size_t b = 0; b < bus_configs.size(); ++b) {
      for (int k = 0; k < static_cast<int>(bus_configs[b].motor_ids.size()); ++k) {
        auto [recv_id, state] = drivers[b].read_one_response(10);
        if (recv_id >= 0 && state.valid) {
          std::cout << "  " << bus_configs[b].interface << " motor " << recv_id
                    << ": pos=" << state.position << " rad" << std::endl;
        }
      }
    }
  }

  std::cout << std::endl << "Starting latency measurement..." << std::endl << std::endl;

  // 計測用ベクタを事前確保する
  std::vector<double> cycle_us;
  cycle_us.reserve(iterations);

  std::vector<BusStats> bus_stats(bus_configs.size());
  for (size_t b = 0; b < bus_configs.size(); ++b) {
    bus_stats[b].interface = bus_configs[b].interface;
    bus_stats[b].send_us.reserve(iterations);
    bus_stats[b].recv_us.reserve(iterations);
    bus_stats[b].expected = static_cast<int>(bus_configs[b].motor_ids.size()) * iterations;
  }

  int total_missed = 0;
  using clock = std::chrono::steady_clock;

  // 安全コマンド: kp=0, kd=0.5, torque_ff=0（トルク出力なし）
  aoba_driver::MitCommand cmd;
  cmd.position = 0.0;
  cmd.velocity = 0.0;
  cmd.kp = 0.0;
  cmd.kd = 0.5;
  cmd.torque_ff = 0.0;

  for (int iter = 0; iter < iterations && g_running; ++iter) {
    auto t_cycle_start = clock::now();

    // フェーズ1: 全バスへコマンドをバースト送信する
    for (size_t b = 0; b < bus_configs.size(); ++b) {
      auto t_send_start = clock::now();
      for (int id : bus_configs[b].motor_ids) {
        drivers[b].send_command(id, cmd);
      }
      auto t_send_end = clock::now();
      bus_stats[b].send_us.push_back(
          std::chrono::duration<double, std::micro>(t_send_end - t_send_start).count());
    }

    // フェーズ2: 全バスからレスポンスをバースト受信する
    for (size_t b = 0; b < bus_configs.size(); ++b) {
      auto t_recv_start = clock::now();
      int received = 0;
      int expected = static_cast<int>(bus_configs[b].motor_ids.size());
      for (int k = 0; k < expected; ++k) {
        auto [recv_id, state] = drivers[b].read_one_response(2);
        if (recv_id >= 0 && state.valid) {
          received++;
        }
      }
      auto t_recv_end = clock::now();
      bus_stats[b].recv_us.push_back(
          std::chrono::duration<double, std::micro>(t_recv_end - t_recv_start).count());
      bus_stats[b].ok += received;
      total_missed += (expected - received);
    }

    auto t_cycle_end = clock::now();
    cycle_us.push_back(
        std::chrono::duration<double, std::micro>(t_cycle_end - t_cycle_start).count());

    // 100イテレーション毎に進捗を表示する
    if ((iter + 1) % 100 == 0) {
      double last = cycle_us.back();
      std::cout << "  [" << (iter + 1) << "/" << iterations << "] "
                << "cycle=" << last << "us" << std::endl;
    }

    // 200Hz相当の間隔（5msスリープ）
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // 安全にシャットダウンする
  std::cout << std::endl << "Disabling motors..." << std::endl;
  for (size_t b = 0; b < bus_configs.size(); ++b) {
    for (int id : bus_configs[b].motor_ids) {
      drivers[b].disable(id);
    }
    drivers[b].disconnect();
  }

  // 統計計算
  if (cycle_us.empty()) {
    std::cerr << "No measurements recorded!" << std::endl;
    return 1;
  }

  std::sort(cycle_us.begin(), cycle_us.end());
  double sum = std::accumulate(cycle_us.begin(), cycle_us.end(), 0.0);
  double mean = sum / cycle_us.size();
  double sq_sum = 0.0;
  for (double v : cycle_us)
    sq_sum += (v - mean) * (v - mean);
  double stddev = std::sqrt(sq_sum / cycle_us.size());

  std::cout << std::endl;
  std::cout << "=== Results ===" << std::endl;
  std::cout << "Cycles measured: " << cycle_us.size() << "/" << iterations << std::endl;
  std::cout << "Total missed responses: " << total_missed << "/"
            << (total_motors * static_cast<int>(cycle_us.size())) << std::endl;
  std::cout << std::endl;

  std::cout << "--- Cycle Latency [us] ---" << std::endl;
  std::cout << "  Min:    " << cycle_us.front() << std::endl;
  std::cout << "  Avg:    " << mean << std::endl;
  std::cout << "  Max:    " << cycle_us.back() << std::endl;
  std::cout << "  Stddev: " << stddev << std::endl;
  std::cout << "  P95:    " << percentile(cycle_us, 0.95) << std::endl;
  std::cout << "  P99:    " << percentile(cycle_us, 0.99) << std::endl;
  std::cout << std::endl;

  std::cout << "--- Per-Bus Breakdown ---" << std::endl;
  for (size_t b = 0; b < bus_configs.size(); ++b) {
    auto& bs = bus_stats[b];
    if (bs.send_us.empty())
      continue;

    double send_sum = std::accumulate(bs.send_us.begin(), bs.send_us.end(), 0.0);
    double recv_sum = std::accumulate(bs.recv_us.begin(), bs.recv_us.end(), 0.0);
    double send_avg = send_sum / bs.send_us.size();
    double recv_avg = recv_sum / bs.recv_us.size();
    std::sort(bs.recv_us.begin(), bs.recv_us.end());
    double recv_max = bs.recv_us.back();

    std::cout << "  " << bs.interface << ":  send_avg=" << send_avg << "us"
              << "  recv_avg=" << recv_avg << "us"
              << "  recv_max=" << recv_max << "us"
              << "  ok=" << bs.ok << "/" << bs.expected << std::endl;
  }
  std::cout << std::endl;

  // 200Hz制御可否判定
  double p99 = percentile(cycle_us, 0.99);
  if (p99 < 5000.0) {
    std::cout << "OK  200Hz control is feasible (P99=" << p99 << "us < 5ms)" << std::endl;
  } else {
    std::cout << "NG  200Hz control may be challenging (P99=" << p99 << "us >= 5ms)" << std::endl;
  }

  return 0;
}
