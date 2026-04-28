// Copyright (c) 2024-2025, Yutaro KIMURA (B-SKY Lab)
// SPDX-License-Identifier: MIT

/**
 * @file main.cpp
 * @brief RS-02 QDD モータ用システム同定データ記録ツール（1kHz）
 *
 * 2つの動作モード:
 *   --sysid   (デフォルト): マルチサイントルク励振 → CSVに記録
 *   --validate             : PDコントローラ（ランダム位置目標）→ CSVに記録
 *
 * CSVスキーマ:
 *   timestamp,cmd_torque,target_position,position,velocity,estimated_torque,valid
 *
 * ビルド & 実行:
 *   cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j4
 *   sudo build/sysid_recorder --interface can0 --motor-id 1 --freq 4.5 --amp 2.5
 *   (SCHED_FIFOにはsudoまたはCAP_SYS_NICE+CAP_IPC_LOCK権限が必要)
 */

#include <linux/can.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <getopt.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "aoba_hardware/aoba_driver.hpp"
#include "sysid_recorder/excitation.hpp"

// ============================================================================
// グローバルシグナルフラグ（volatile sig_atomic_t: async-signal-safe）
// ============================================================================

static volatile sig_atomic_t g_running = 1;

static void signal_handler(int) {
  g_running = 0;
}

// ============================================================================
// サンプル構造体
// ============================================================================

struct Sample {
  double timestamp;       // ループ開始からの経過時間 [s]
  double cmd_torque;      // 送信トルクコマンド [Nm]
  double target_position; // PDモード時の目標位置 [rad]、sysidモード時は0
  double position;        // モータ実測位置 [rad]
  double velocity;        // モータ実測速度 [rad/s]
  double estimated_torque; // モータ推定トルク [Nm]
  int valid;              // 1=応答あり、0=タイムアウト
};

// ============================================================================
// コンフィグ
// ============================================================================

struct Config {
  std::string interface = "can0";
  int motor_id = 1;
  double freq = 4.5;    // 基本周波数 [Hz]（sysidモード）
  double amp = 2.5;     // 励振振幅 [Nm]（sysidモード）
  double duration = 10.0; // 記録時間 [s]
  std::string output = "data/recording.csv";
  bool validate_mode = false;
  // PDコントローラゲイン（validateモード）
  double kp = 8.0;
  double kd = 0.5;
  // PDモードのランダム目標位置の範囲 [rad]
  double target_range = 3.14159265358979323846; // ±π
  // ランダム目標の更新間隔 [s]
  double target_interval = 1.0;
};

static void print_usage(const char* prog) {
  std::cerr << "Usage: " << prog << " [OPTIONS]\n"
            << "  --interface  <iface>  CAN interface (default: can0)\n"
            << "  --motor-id   <id>     Motor CAN ID (default: 1)\n"
            << "  --freq       <Hz>     Base frequency for multi-sine (default: 4.5)\n"
            << "  --amp        <Nm>     Excitation amplitude (default: 2.5)\n"
            << "  --duration   <s>      Recording duration (default: 10.0)\n"
            << "  --output     <path>   CSV output path (default: data/recording.csv)\n"
            << "  --validate            Enable PD validation mode\n"
            << "  --kp         <val>    PD position gain (default: 8.0, validate mode only)\n"
            << "  --kd         <val>    PD damping gain (default: 0.5, validate mode only)\n"
            << "  --help\n";
}

static Config parse_args(int argc, char** argv) {
  Config cfg;
  static const struct option long_opts[] = {
    {"interface",  required_argument, nullptr, 'i'},
    {"motor-id",   required_argument, nullptr, 'm'},
    {"freq",       required_argument, nullptr, 'f'},
    {"amp",        required_argument, nullptr, 'a'},
    {"duration",   required_argument, nullptr, 'd'},
    {"output",     required_argument, nullptr, 'o'},
    {"validate",   no_argument,       nullptr, 'V'},
    {"kp",         required_argument, nullptr, 'p'},
    {"kd",         required_argument, nullptr, 'D'},
    {"help",       no_argument,       nullptr, 'h'},
    {nullptr, 0, nullptr, 0},
  };

  int c;
  while ((c = getopt_long(argc, argv, "", long_opts, nullptr)) != -1) {
    switch (c) {
      case 'i': cfg.interface = optarg; break;
      case 'm': cfg.motor_id = std::stoi(optarg); break;
      case 'f': cfg.freq = std::stod(optarg); break;
      case 'a': cfg.amp = std::stod(optarg); break;
      case 'd': cfg.duration = std::stod(optarg); break;
      case 'o': cfg.output = optarg; break;
      case 'V': cfg.validate_mode = true; break;
      case 'p': cfg.kp = std::stod(optarg); break;
      case 'D': cfg.kd = std::stod(optarg); break;
      case 'h':
      default:
        print_usage(argv[0]);
        std::exit((c == 'h') ? 0 : 1);
    }
  }
  return cfg;
}

// ============================================================================
// リアルタイム設定
// ============================================================================

static void setup_realtime() {
  struct sched_param param;
  param.sched_priority = 80;
  if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
    std::cerr << "[WARN] sched_setscheduler(SCHED_FIFO) failed: " << strerror(errno)
              << " — jitter may increase (try sudo or setcap)\n";
  }
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::cerr << "[WARN] mlockall() failed: " << strerror(errno) << "\n";
  }
}

// ============================================================================
// 1ms 絶対時刻タイマー進め方
// ============================================================================

static inline void advance_1ms(struct timespec& ts) {
  ts.tv_nsec += 1'000'000L;
  if (ts.tv_nsec >= 1'000'000'000L) {
    ts.tv_nsec -= 1'000'000'000L;
    ts.tv_sec++;
  }
}

// ============================================================================
// CSV フラッシュ
// ============================================================================

static bool flush_csv(const std::string& path, const std::vector<Sample>& samples) {
  // 親ディレクトリが存在しない場合の簡易作成
  const auto slash = path.rfind('/');
  if (slash != std::string::npos) {
    const std::string dir = path.substr(0, slash);
    std::system(("mkdir -p " + dir).c_str());
  }

  std::ofstream ofs(path);
  if (!ofs) {
    std::cerr << "[ERROR] Cannot open output file: " << path << "\n";
    return false;
  }

  ofs << "timestamp,cmd_torque,target_position,position,velocity,estimated_torque,valid\n";
  ofs << std::fixed << std::setprecision(6);
  for (const auto& s : samples) {
    ofs << s.timestamp << ','
        << s.cmd_torque << ','
        << s.target_position << ','
        << s.position << ','
        << s.velocity << ','
        << s.estimated_torque << ','
        << s.valid << '\n';
  }
  return true;
}

// ============================================================================
// メイン
// ============================================================================

int main(int argc, char** argv) {
  std::signal(SIGINT,  signal_handler);
  std::signal(SIGTERM, signal_handler);

  const Config cfg = parse_args(argc, argv);

  const char* mode_str = cfg.validate_mode ? "VALIDATE (PD)" : "SYSID (multi-sine)";
  std::cout << "=== sysid_recorder ===\n"
            << "  Mode:      " << mode_str << "\n"
            << "  Interface: " << cfg.interface << "\n"
            << "  Motor ID:  " << cfg.motor_id << "\n";
  if (!cfg.validate_mode) {
    std::cout << "  Freq:      " << cfg.freq << " Hz\n"
              << "  Amp:       " << cfg.amp << " Nm\n";
  } else {
    std::cout << "  kp=" << cfg.kp << "  kd=" << cfg.kd << "\n";
  }
  std::cout << "  Duration:  " << cfg.duration << " s\n"
            << "  Output:    " << cfg.output << "\n\n";

  // ── モータ初期化 ─────────────────────────────────────────────────────────
  aoba_driver::AobaDriver driver;

  if (!driver.connect(cfg.interface)) {
    std::cerr << "[ERROR] Failed to connect to " << cfg.interface << "\n";
    return 1;
  }
  std::cout << "Connected to " << cfg.interface << "\n";

  driver.disable_auto_report(cfg.motor_id);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  driver.drain_rx_buffer();

  if (!driver.enable(cfg.motor_id)) {
    std::cerr << "[ERROR] Failed to enable motor " << cfg.motor_id << "\n";
    driver.disconnect();
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  if (!driver.set_mode(cfg.motor_id, aoba_driver::ControlMode::MIT)) {
    std::cerr << "[ERROR] Failed to set MIT mode\n";
    driver.disconnect();
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  driver.drain_rx_buffer();

  // プローブ: ゼロトルクコマンドで通信確認
  {
    aoba_driver::MitCommand probe{};
    driver.send_command(cfg.motor_id, probe);
    auto [id, state] = driver.read_one_response(50);
    if (id == cfg.motor_id && state.valid) {
      std::cout << "Motor probe OK: pos=" << state.position
                << " rad, vel=" << state.velocity << " rad/s\n\n";
    } else {
      std::cerr << "[WARN] Motor did not respond to probe — continuing anyway\n"
                << "  Check: motor power, CAN wiring, bitrate (1Mbps), motor ID ("
                << cfg.motor_id << ")\n\n";
    }
  }

  // ── バッファ確保 ──────────────────────────────────────────────────────────
  const int n_total = static_cast<int>(cfg.duration * 1000.0) + 100;
  std::vector<Sample> samples;
  samples.reserve(n_total);

  // ── RT設定（socket確立後に行う）─────────────────────────────────────────
  setup_realtime();

  // ── タイミング基点 ────────────────────────────────────────────────────────
  struct timespec next_wake;
  clock_gettime(CLOCK_MONOTONIC, &next_wake);
  const struct timespec t_loop_start = next_wake;

  auto elapsed_seconds = [&t_loop_start]() -> double {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - t_loop_start.tv_sec)
           + (now.tv_nsec - t_loop_start.tv_nsec) * 1e-9;
  };

  int tick = 0;
  int missed = 0;

  // validateモード用: ランダム目標位置の管理
  double target_pos = 0.0;
  int next_target_tick = 0;
  std::srand(42);
  auto random_target = [&]() -> double {
    return cfg.target_range * (2.0 * (static_cast<double>(std::rand()) / RAND_MAX) - 1.0);
  };

  std::cout << "Recording... (Ctrl+C to stop early)\n";

  // ── 1kHz RTループ ─────────────────────────────────────────────────────────
  while (g_running && tick < n_total) {
    // 実測経過時間（ループ開始からの秒数）。CLOCK_MONOTONIC で単調増加保証。
    // vDSO 経由の clock_gettime は ~20ns で軽量。
    const double t = elapsed_seconds();

    // トルクコマンド計算
    double tau;
    if (!cfg.validate_mode) {
      // sysidモード: マルチサイン純トルク励振
      tau = sysid::multi_sine_torque(t, cfg.freq, cfg.amp);
      target_pos = 0.0;
    } else {
      // validateモード: PDコントローラ
      if (tick >= next_target_tick) {
        target_pos = random_target();
        next_target_tick = tick + static_cast<int>(cfg.target_interval * 1000.0);
      }
      // tau はここでは仮値（実際のフィードバックから計算）
      tau = 0.0;
    }

    // コマンド送信（kp=0, kd=0 → 純トルク / validateモードもtorque_ffは0で後で設定）
    aoba_driver::MitCommand cmd{};
    if (!cfg.validate_mode) {
      cmd.torque_ff = tau;
    }
    // validateモードはkp/kdを使う
    if (cfg.validate_mode) {
      cmd.position = target_pos;
      cmd.velocity = 0.0;
      cmd.kp = cfg.kp;
      cmd.kd = cfg.kd;
      cmd.torque_ff = 0.0;
      // cmd_torque として記録する値を計算（前回サンプルがあれば）
      if (!samples.empty() && samples.back().valid) {
        tau = cfg.kp * (target_pos - samples.back().position)
              + cfg.kd * (0.0 - samples.back().velocity);
      }
    }

    driver.send_command(cfg.motor_id, cmd);

    // レスポンス受信（タイムアウト2ms）
    auto [id, state] = driver.read_one_response(2);
    const bool got_response = (id == cfg.motor_id && state.valid);

    if (!got_response) {
      missed++;
    }

    samples.push_back({
      t, tau, target_pos,
      got_response ? state.position : 0.0,
      got_response ? state.velocity : 0.0,
      got_response ? state.torque   : 0.0,
      got_response ? 1 : 0,
    });

    // 進捗表示（100ticks=0.1s毎）
    if (tick % 100 == 0) {
      const double pos = got_response ? state.position : (samples.empty() ? 0.0 : samples.back().position);
      std::printf("\r[%5.1fs] tick=%-6d missed=%-4d pos=%7.3f rad  ", t, tick, missed, pos);
      std::fflush(stdout);
    }

    // 次の絶対起床時刻へ進める（ドリフトなし）
    advance_1ms(next_wake);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, nullptr);
    tick++;
  }

  std::cout << "\n";

  // ── シャットダウン ────────────────────────────────────────────────────────
  aoba_driver::MitCommand zero{};
  driver.send_command(cfg.motor_id, zero);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  driver.disable(cfg.motor_id);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  driver.disconnect();

  std::cout << "Recording done: " << samples.size() << " samples, "
            << missed << " missed ("
            << std::fixed << std::setprecision(1)
            << (100.0 * missed / std::max(1, (int)samples.size())) << "%)\n";

  if (!flush_csv(cfg.output, samples)) {
    return 1;
  }
  std::cout << "Saved: " << cfg.output << "\n";
  return 0;
}
