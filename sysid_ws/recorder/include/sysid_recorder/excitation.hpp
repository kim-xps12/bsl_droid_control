// Copyright (c) 2024-2025, Yutaro KIMURA (B-SKY Lab)
// SPDX-License-Identifier: MIT

#pragma once
#include <cmath>

namespace sysid {

// torque(t) = amp × (sin(2π·f·t) + 0.6·sin(2π·3.4f·t) + 0.3·sin(2π·7.4f·t))
// 周波数比 1.0:3.4:7.4 は非有理数比で相殺しない。
// RS-02 推奨: freq=4–5 Hz, amp=2–3 Nm
inline double multi_sine_torque(double t, double freq, double amp) noexcept {
  constexpr double TWO_PI = 2.0 * 3.14159265358979323846;
  return amp * (std::sin(TWO_PI * freq * t) +
                0.6 * std::sin(TWO_PI * 3.4 * freq * t) +
                0.3 * std::sin(TWO_PI * 7.4 * freq * t));
}

}  // namespace sysid
