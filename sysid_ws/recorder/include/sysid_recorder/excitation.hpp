// Copyright (c) 2024-2025, Yutaro KIMURA (B-SKY Lab)
// SPDX-License-Identifier: MIT

#pragma once
#include <cmath>

namespace sysid {

// torque(t) = amp × (0.4·cos(2π·0.3f·t) + sin(2π·f·t)
//                  + 0.6·sin(2π·3.4f·t) + 0.3·sin(2π·7.4f·t))
//
// 周波数比 0.3:1.0:3.4:7.4 は短い共通周期を持たないよう選定（実用上「非有理」近似）。
// 4 帯域の役割:
//   0.3·f (=1.35 Hz at f=4.5): 低速度域 (v ~ 1〜2 rad/s) を励起。
//                              friction-dominated 領域で μ·sign(v) を引き出す。
//   1.0·f (=4.5 Hz):           慣性/摩擦遷移帯域。
//   3.4·f / 7.4·f:             高周波で慣性 (J·θ̈) を励起。
// 高周波になるほど振幅を下げてピーク速度を制限。
//
// 低周波成分のみ cos を使うのは、有限時間録画でのドリフト回避のため:
//   sin(ωt) の積分は (1-cos)/ω で常に非負 → 第1半周期で位置が単調に
//   一方向へドリフトする。低周波（周期 740 ms）では 30 秒録画でも
//   cycles=40.5 と半整数になり、原信号設計（4.5/15.3/33.3 Hz は全て
//   T=30 で整数周期）の "ドリフトキャンセル" 効果から外れる。
//   cos(ωt) の積分は sin/ω で ±1/ω に有界振動 → 位置ドリフトなし。
//   t=0 で cos=1 は torque step だが、position/velocity は 0 から
//   連続に立ち上がるので機械的衝撃なし。
//
// RS-02 推奨: freq=4–5 Hz, amp=2–3 Nm（ピーク合計 ≤ 2.3·amp ≈ 6 Nm < 12 Nm torque_limit）
inline double multi_sine_torque(double t, double freq, double amp) noexcept {
  constexpr double TWO_PI = 2.0 * 3.14159265358979323846;
  return amp * (0.4 * std::cos(TWO_PI * 0.3 * freq * t) +
                std::sin(TWO_PI * freq * t) +
                0.6 * std::sin(TWO_PI * 3.4 * freq * t) +
                0.3 * std::sin(TWO_PI * 7.4 * freq * t));
}

}  // namespace sysid
