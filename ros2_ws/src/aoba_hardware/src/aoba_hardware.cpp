// Copyright (c) 2024-2025, Yutaro KIMURA (B-SKY Lab)
// SPDX-License-Identifier: MIT

/**
 * @file aoba_hardware.cpp
 * @brief RobStrideモータ用 ros2_control ハードウェアインターフェースの実装
 *
 * 同期送受信パターン: 全CAN I/Oはwrite()内で実行し、read()は何もしない。
 * 複数CANバス（例: can1, can2）にまたがる複数モータをサポートする。
 */

#include "aoba_hardware/aoba_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace aoba_hardware {

// ============================================================================
// ライフサイクルコールバック
// ============================================================================

/// 初期化: URDFからジョイント設定を読み込み、内部バッファを確保する
CallbackReturn AobaHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // ジョイント数に応じてバッファを事前確保する
  const size_t n = info_.joints.size();
  joints_.resize(n);
  hw_positions_.resize(n, 0.0);
  hw_velocities_.resize(n, 0.0);
  hw_efforts_.resize(n, 0.0);
  hw_commands_position_.resize(n, 0.0);
  response_received_.resize(n, false);
  missed_response_count_.resize(n, 0);

  // URDFのjointパラメータからジョイント設定を読み込む
  for (size_t i = 0; i < n; ++i) {
    auto& jcfg = joints_[i];
    const auto& params = info_.joints[i].parameters;

    jcfg.name = info_.joints[i].name;
    jcfg.can_interface = params.count("can_interface") ? params.at("can_interface") : "can0";
    jcfg.motor_id = params.count("motor_id") ? std::stoi(params.at("motor_id")) : 11;
    jcfg.kp = params.count("kp") ? std::stod(params.at("kp")) : 30.0;
    jcfg.kd = params.count("kd") ? std::stod(params.at("kd")) : 1.0;

    // ジョイントをCANバス毎にグループ化する
    buses_[jcfg.can_interface].joint_indices.push_back(i);

    RCLCPP_INFO(rclcpp::get_logger("AobaHardware"),
                "Joint[%zu]: name=%s, bus=%s, motor_id=%d, kp=%.1f, kd=%.2f", i, jcfg.name.c_str(),
                jcfg.can_interface.c_str(), jcfg.motor_id, jcfg.kp, jcfg.kd);
  }

  // バス毎のタイミングマップを事前確保する
  for (const auto& [bus_name, _] : buses_) {
    bus_timing_[bus_name] = BusTimingStats{};
  }

  RCLCPP_INFO(rclcpp::get_logger("AobaHardware"), "Initialized: %zu joints on %zu CAN bus(es)", n,
              buses_.size());

  return CallbackReturn::SUCCESS;
}

/// 設定: 全CANバスへの接続を確立する
CallbackReturn AobaHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("AobaHardware"), "Configuring...");

  // 各CANバスに接続する
  for (auto& [bus_name, bus] : buses_) {
    if (!bus.driver.connect(bus_name)) {
      RCLCPP_ERROR(rclcpp::get_logger("AobaHardware"), "Failed to connect to CAN interface: %s",
                   bus_name.c_str());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("AobaHardware"), "Connected to CAN interface: %s (%zu motors)",
                bus_name.c_str(), bus.joint_indices.size());
  }

  return CallbackReturn::SUCCESS;
}

/// アクティベート: 各モータの有効化、MITモード設定、通信確認を行う
CallbackReturn AobaHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("AobaHardware"), "Activating...");

  // 各モータを1台ずつ初期化する
  // CAN応答の衝突を避けるため、各操作の間にドレインを挟む
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto& jcfg = joints_[i];
    auto& driver = buses_.at(jcfg.can_interface).driver;

    // 自動レポートを無効化する（非要求フレームの送信を停止）
    driver.disable_auto_report(jcfg.motor_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    driver.drain_rx_buffer();

    // モータを有効化する
    if (!driver.enable(jcfg.motor_id)) {
      RCLCPP_ERROR(rclcpp::get_logger("AobaHardware"), "Failed to enable motor %d on %s",
                   jcfg.motor_id, jcfg.can_interface.c_str());
      return CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    driver.drain_rx_buffer();

    // MIT制御モードを設定する
    if (!driver.set_mode(jcfg.motor_id, aoba_driver::ControlMode::MIT)) {
      RCLCPP_ERROR(rclcpp::get_logger("AobaHardware"), "Failed to set MIT mode for motor %d on %s",
                   jcfg.motor_id, jcfg.can_interface.c_str());
      return CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    driver.drain_rx_buffer();

    RCLCPP_INFO(rclcpp::get_logger("AobaHardware"), "Motor %d on %s: enabled, MIT mode set",
                jcfg.motor_id, jcfg.can_interface.c_str());

    // プローブ: ゼロトルクのMITコマンドを送信してモータの応答を確認する
    aoba_driver::MitCommand probe;
    probe.position = 0.0;
    probe.velocity = 0.0;
    probe.kp = 0.0;
    probe.kd = 0.0;
    probe.torque_ff = 0.0;
    driver.send_command(jcfg.motor_id, probe);

    auto [recv_id, state] = driver.read_one_response(50);
    if (recv_id == jcfg.motor_id && state.valid) {
      hw_positions_[i] = state.position;
      RCLCPP_INFO(rclcpp::get_logger("AobaHardware"), "Motor %d on %s: probe OK (pos=%.3f rad)",
                  jcfg.motor_id, jcfg.can_interface.c_str(), state.position);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("AobaHardware"),
                   "Motor %d on %s: no response to probe command. "
                   "Check: motor power, CAN wiring, CAN bitrate (1Mbps), motor ID",
                   jcfg.motor_id, jcfg.can_interface.c_str());
      return CallbackReturn::ERROR;
    }
  }

  // コマンドの初期値を現在位置に設定する（急な動作を防止）
  for (size_t i = 0; i < hw_commands_position_.size(); ++i) {
    hw_commands_position_[i] = hw_positions_[i];
  }

  // タイミング統計をリセットする
  timing_log_counter_ = 0;
  total_us_min_ = 1e9;
  total_us_max_ = 0.0;
  total_us_sum_ = 0.0;
  total_us_sum_sq_ = 0.0;
  send_us_min_ = 1e9;
  send_us_max_ = 0.0;
  send_us_sum_ = 0.0;
  recv_us_min_ = 1e9;
  recv_us_max_ = 0.0;
  recv_us_sum_ = 0.0;
  total_missed_sum_ = 0;

  // 非RTログスレッドを起動する
  log_thread_stop_.store(false, std::memory_order_relaxed);
  snapshot_ready_.store(false, std::memory_order_relaxed);
  log_thread_ = std::thread(&AobaHardware::diagnostic_log_thread_func, this);

  RCLCPP_INFO(rclcpp::get_logger("AobaHardware"),
              "Activated: %zu motors on %zu bus(es), synchronous send/receive mode", joints_.size(),
              buses_.size());

  return CallbackReturn::SUCCESS;
}

/// ディアクティベート: 安全なトルクゼロ状態にしてからモータを無効化する
CallbackReturn AobaHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("AobaHardware"), "Deactivating...");

  // 非RTログスレッドを停止する
  log_thread_stop_.store(true, std::memory_order_relaxed);
  if (log_thread_.joinable()) {
    log_thread_.join();
  }

  // ゼロトルクコマンド（安全状態）を送信してから各モータを無効化する
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto& jcfg = joints_[i];
    auto& driver = buses_.at(jcfg.can_interface).driver;

    aoba_driver::MitCommand safe_cmd;
    safe_cmd.position = hw_positions_[i];
    safe_cmd.velocity = 0.0;
    safe_cmd.kp = 0.0;
    safe_cmd.kd = 0.0;
    safe_cmd.torque_ff = 0.0;
    driver.send_command(jcfg.motor_id, safe_cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    driver.drain_rx_buffer();

    driver.disable(jcfg.motor_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    driver.drain_rx_buffer();

    RCLCPP_INFO(rclcpp::get_logger("AobaHardware"), "Motor %d on %s: deactivated", jcfg.motor_id,
                jcfg.can_interface.c_str());
  }

  return CallbackReturn::SUCCESS;
}

/// クリーンアップ: 全CANバスの接続を切断する
CallbackReturn AobaHardware::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("AobaHardware"), "Cleaning up...");
  for (auto& [_, bus] : buses_) {
    bus.driver.disconnect();
  }
  return CallbackReturn::SUCCESS;
}

// ============================================================================
// インターフェースのエクスポート
// ============================================================================

/// 状態インターフェースをエクスポートする（位置・速度・トルク）
std::vector<hardware_interface::StateInterface> AobaHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // 各ジョイントについて位置・速度・トルクの3つのインターフェースを登録する
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }
  return state_interfaces;
}

/// コマンドインターフェースをエクスポートする（位置指令）
std::vector<hardware_interface::CommandInterface> AobaHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
  }
  return command_interfaces;
}

// ============================================================================
// リアルタイム制御ループ（Controller Managerから200Hzで呼び出される）
// ============================================================================

/// read(): write()で既に更新済みの内部バッファを返すだけ（何もしない）
hardware_interface::return_type AobaHardware::read(const rclcpp::Time& /*time*/,
                                                   const rclcpp::Duration& /*period*/) {
  // hw_positions_ / hw_velocities_ / hw_efforts_ は直前のwrite()で
  // 既に更新済みのため、ここでは何もしない
  return hardware_interface::return_type::OK;
}

/// write(): 全モータへのコマンド送信とレスポンス受信を同期的に行う
hardware_interface::return_type AobaHardware::write(const rclcpp::Time& /*time*/,
                                                    const rclcpp::Duration& /*period*/) {
  using clock = std::chrono::steady_clock;
  const auto t_start = clock::now();

  // === フェーズ1: 全バスへコマンドをバースト送信する ===
  for (auto& [bus_name, bus] : buses_) {
    const auto t_bus_send_start = clock::now();

    // バス上の全モータにMIT制御コマンドを送信する
    for (size_t ji : bus.joint_indices) {
      aoba_driver::MitCommand cmd;
      cmd.position = hw_commands_position_[ji];
      cmd.velocity = 0.0;
      cmd.kp = joints_[ji].kp;
      cmd.kd = joints_[ji].kd;
      cmd.torque_ff = 0.0;
      bus.driver.send_command(joints_[ji].motor_id, cmd);
    }

    bus_timing_[bus_name].send_us =
        std::chrono::duration<double, std::micro>(clock::now() - t_bus_send_start).count();
  }

  const auto t_send_done = clock::now();

  // === フェーズ2: 全バスからレスポンスを受信する（モータIDマッチング） ===
  std::fill(response_received_.begin(), response_received_.end(), false);
  int total_received = 0;
  int total_missed = 0;

  for (auto& [bus_name, bus] : buses_) {
    const auto t_bus_recv_start = clock::now();
    int remaining = static_cast<int>(bus.joint_indices.size());
    int bus_received = 0;
    int timeout_ms = 3;  // 全バーストレスポンスを待機（5モータで約1.5ms）

    while (remaining > 0) {
      auto [motor_id, state] = bus.driver.read_one_response(timeout_ms);
      if (motor_id < 0) {
        break;  // タイムアウト
      }

      // 受信したモータIDと一致するジョイントを探して状態を更新する
      for (size_t ji : bus.joint_indices) {
        if (joints_[ji].motor_id == motor_id && !response_received_[ji]) {
          hw_positions_[ji] = state.position;
          hw_velocities_[ji] = state.velocity;
          hw_efforts_[ji] = state.torque;
          response_received_[ji] = true;
          missed_response_count_[ji] = 0;
          remaining--;
          bus_received++;
          break;
        }
      }

      timeout_ms = 1;  // 2件目以降は短いタイムアウトで読み取る
    }

    auto& bt = bus_timing_[bus_name];
    bt.receive_us =
        std::chrono::duration<double, std::micro>(clock::now() - t_bus_recv_start).count();
    bt.received = bus_received;
    bt.expected = static_cast<int>(bus.joint_indices.size());
    total_received += bus_received;
  }

  const auto t_recv_done = clock::now();

  // === フェーズ3: レスポンス欠落を処理する ===
  // ミスレスポンス警告をスナップショットに蓄積するためのカウンタ
  int warn_count = 0;
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (!response_received_[i]) {
      missed_response_count_[i]++;
      total_missed++;
      if (missed_response_count_[i] == kMissedResponseWarnThreshold &&
          warn_count < DiagnosticSnapshot::kMaxWarnSlots) {
        // 警告データをスナップショットバッファに記録（ログ出力は非RTスレッドで行う）
        auto& w = diag_snapshot_.missed_warns[warn_count];
        w.motor_id = joints_[i].motor_id;
        std::strncpy(w.can_interface, joints_[i].can_interface.c_str(),
                     sizeof(w.can_interface) - 1);
        w.can_interface[sizeof(w.can_interface) - 1] = '\0';
        w.count = missed_response_count_[i];
        warn_count++;
      }
    }
  }

  // === タイミング診断 ===
  last_timing_.send_us = std::chrono::duration<double, std::micro>(t_send_done - t_start).count();
  last_timing_.receive_us =
      std::chrono::duration<double, std::micro>(t_recv_done - t_send_done).count();
  last_timing_.total_us = std::chrono::duration<double, std::micro>(t_recv_done - t_start).count();
  last_timing_.responses_received = total_received;
  last_timing_.responses_expected = static_cast<int>(joints_.size());

  total_us_sum_ += last_timing_.total_us;
  total_us_sum_sq_ += last_timing_.total_us * last_timing_.total_us;
  total_us_min_ = std::min(total_us_min_, last_timing_.total_us);
  total_us_max_ = std::max(total_us_max_, last_timing_.total_us);
  send_us_sum_ += last_timing_.send_us;
  send_us_min_ = std::min(send_us_min_, last_timing_.send_us);
  send_us_max_ = std::max(send_us_max_, last_timing_.send_us);
  recv_us_sum_ += last_timing_.receive_us;
  recv_us_min_ = std::min(recv_us_min_, last_timing_.receive_us);
  recv_us_max_ = std::max(recv_us_max_, last_timing_.receive_us);
  total_missed_sum_ += total_missed;
  timing_log_counter_++;

  // スナップショットにミスレスポンス警告数を記録（毎サイクル更新）
  diag_snapshot_.num_missed_warns = warn_count;

  if (timing_log_counter_ >= kTimingLogInterval) {
    // ログスレッドが前回のスナップショットを消費済みの場合のみ書き込む
    // 未消費の場合はこのインターバルのログをドロップする（RT安全性優先）
    if (!snapshot_ready_.load(std::memory_order_acquire)) {
      const double n = static_cast<double>(timing_log_counter_);
      const double avg_us = total_us_sum_ / n;
      const double variance = (total_us_sum_sq_ / n) - (avg_us * avg_us);
      const double stddev_us = std::sqrt(std::max(0.0, variance));
      const double send_avg = send_us_sum_ / n;
      const double recv_avg = recv_us_sum_ / n;

      // スナップショットにタイミング統計をコピーする
      diag_snapshot_.total_us_min = total_us_min_;
      diag_snapshot_.total_us_max = total_us_max_;
      diag_snapshot_.total_us_avg = avg_us;
      diag_snapshot_.total_us_stddev = stddev_us;
      diag_snapshot_.send_us_min = send_us_min_;
      diag_snapshot_.send_us_max = send_us_max_;
      diag_snapshot_.send_us_avg = send_avg;
      diag_snapshot_.recv_us_min = recv_us_min_;
      diag_snapshot_.recv_us_max = recv_us_max_;
      diag_snapshot_.recv_us_avg = recv_avg;
      diag_snapshot_.total_missed_sum = total_missed_sum_;
      diag_snapshot_.total_cycles = timing_log_counter_;
      diag_snapshot_.num_joints = static_cast<int>(joints_.size());

      // バス毎タイミングをコピーする
      int bus_idx = 0;
      for (const auto& [bus_name, bt] : bus_timing_) {
        if (bus_idx >= DiagnosticSnapshot::kMaxBuses)
          break;
        auto& bs = diag_snapshot_.buses[bus_idx];
        std::strncpy(bs.name, bus_name.c_str(), sizeof(bs.name) - 1);
        bs.name[sizeof(bs.name) - 1] = '\0';
        bs.send_us = bt.send_us;
        bs.receive_us = bt.receive_us;
        bs.received = bt.received;
        bs.expected = bt.expected;
        bus_idx++;
      }
      diag_snapshot_.num_buses = bus_idx;

      // モータ状態スナップショットをコピーする
      for (size_t i = 0; i < joints_.size() && static_cast<int>(i) < DiagnosticSnapshot::kMaxJoints;
           ++i) {
        auto& js = diag_snapshot_.joints[i];
        std::strncpy(js.name, joints_[i].name.c_str(), sizeof(js.name) - 1);
        js.name[sizeof(js.name) - 1] = '\0';
        js.motor_id = joints_[i].motor_id;
        js.cmd_pos = hw_commands_position_[i];
        js.pos = hw_positions_[i];
        js.vel = hw_velocities_[i];
        js.effort = hw_efforts_[i];
      }

      // スナップショット準備完了を通知する（release順序でデータの可視性を保証）
      snapshot_ready_.store(true, std::memory_order_release);
    }

    // 集約統計をリセットする
    timing_log_counter_ = 0;
    total_us_min_ = 1e9;
    total_us_sum_ = 0.0;
    total_us_sum_sq_ = 0.0;
    total_us_max_ = 0.0;
    send_us_min_ = 1e9;
    send_us_max_ = 0.0;
    send_us_sum_ = 0.0;
    recv_us_min_ = 1e9;
    recv_us_max_ = 0.0;
    recv_us_sum_ = 0.0;
    total_missed_sum_ = 0;
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// 非RT診断ログスレッド
// ============================================================================

/// 診断ログ出力スレッド: RTスレッドが準備したスナップショットを非RTコンテキストで出力する
void AobaHardware::diagnostic_log_thread_func() {
  auto logger = rclcpp::get_logger("AobaHardware");

  while (!log_thread_stop_.load(std::memory_order_relaxed)) {
    // スナップショットが準備されるまで待機する
    if (snapshot_ready_.load(std::memory_order_acquire)) {
      // スナップショットのローカルコピーを取得する（RTスレッドの書き込みブロックを最小化）
      const DiagnosticSnapshot snap = diag_snapshot_;

      // スナップショット消費完了を通知する（RTスレッドが次回書き込み可能になる）
      snapshot_ready_.store(false, std::memory_order_release);

      // ミスレスポンス警告を出力する
      for (int i = 0; i < snap.num_missed_warns; ++i) {
        const auto& w = snap.missed_warns[i];
        RCLCPP_WARN(logger, "Motor %d on %s: %d consecutive missed responses", w.motor_id,
                    w.can_interface, w.count);
      }

      // タイミングサマリを出力する
      RCLCPP_INFO(logger,
                  "[Timing] write() min=%.0fus avg=%.0fus max=%.0fus stddev=%.1fus | "
                  "send min=%.0f avg=%.0f max=%.0fus | recv min=%.0f avg=%.0f max=%.0fus | "
                  "missed=%d/%d",
                  snap.total_us_min, snap.total_us_avg, snap.total_us_max, snap.total_us_stddev,
                  snap.send_us_min, snap.send_us_avg, snap.send_us_max, snap.recv_us_min,
                  snap.recv_us_avg, snap.recv_us_max, snap.total_missed_sum,
                  snap.total_cycles * snap.num_joints);

      // バス毎タイミングを出力する
      for (int i = 0; i < snap.num_buses; ++i) {
        const auto& bs = snap.buses[i];
        RCLCPP_INFO(logger, "[Timing]   %s: send=%.0fus, recv=%.0fus, ok=%d/%d", bs.name,
                    bs.send_us, bs.receive_us, bs.received, bs.expected);
      }

      // モータ状態スナップショットを出力する
      for (int i = 0; i < snap.num_joints; ++i) {
        const auto& js = snap.joints[i];
        RCLCPP_INFO(logger,
                    "[State] %s (id=%d): cmd=%.4f rad | pos=%.4f rad, vel=%.4f rad/s, "
                    "effort=%.4f Nm",
                    js.name, js.motor_id, js.cmd_pos, js.pos, js.vel, js.effort);
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}  // namespace aoba_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(aoba_hardware::AobaHardware, hardware_interface::SystemInterface)
